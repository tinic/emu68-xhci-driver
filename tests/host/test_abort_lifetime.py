#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise the real abort handler and dispatcher with a freeing pool stub."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile


PREAMBLE = r"""
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
typedef unsigned u32;
struct ExecBase { int unused; };
struct xhci_ctrl { int unused; };
struct Message { unsigned mn_Length; };
struct IORequest {
    struct Message io_Message;
    void *io_Unit;
    unsigned io_Command, io_Flags;
    int io_Error;
};
struct USBIORequest { struct IORequest req; void *data_buffer; };
struct IOStdReq { struct IORequest req; };
struct XHCIUnit {
    struct ExecBase *sysBase;
    struct xhci_ctrl *xhci_ctrl;
    void *memoryPool;
};
enum {
    CMD_FLUSH = 1, CMD_RESET, CMD_DEVICE_QUERY, CMD_DEVICE_RESET,
    CMD_DEVICE_RESUME, CMD_STOP, CMD_START, CMD_REQUEST_CONTROL,
    CMD_REQUEST_ISOCHRONOUS, CMD_REQUEST_INTERRUPT, CMD_REQUEST_BULK,
    CMD_INTERNAL_ABORT_REQUEST, NSCMD_DEVICEQUERY,
    CMD_REGISTER_ISOCHRONOUS_HOOKS, CMD_UNREGISTER_ISOCHRONOUS_HOOKS,
    CMD_START_REALTIME_ISOCHRONOUS, CMD_STOP_REALTIME_ISOCHRONOUS
};
enum { COMMAND_SCHEDULED, COMMAND_PROCESSED, IOF_QUICK = 1,
       ERR_NO_ERROR = 0, IOERR_BADLENGTH = -4, IOERR_NOCMD = -3 };
static struct ExecBase exec_base;
static struct xhci_ctrl controller;
static struct XHCIUnit unit = { &exec_base, &controller, &unit };
static unsigned freed, aborted, replied;
static u32 completion = COMMAND_PROCESSED;
static struct USBIORequest *expected_message, original;
static void xhci_td_abort_req(struct USBIORequest *io)
{ assert(io == &original); aborted++; }
static void free_message(struct ExecBase *base, void *pool, void *io)
{
    assert(base == &exec_base && pool == unit.memoryPool);
    assert(io == expected_message);
    freed++;
    free(io);
}
static void reply_message(struct ExecBase *base, struct Message *io)
{
    assert(base == &exec_base && (void *)io == expected_message);
    replied++;
}
static u32 ordinary_command(void *io) { (void)io; return completion; }
#define pool_free(pool, io) free_message(SysBase, pool, io)
#define ReplyMsg(io) reply_message(SysBase, io)
#define Kprintf(...) ((void)0)
#define Do_CMD_FLUSH ordinary_command
#define Do_CMD_RESET ordinary_command
#define Do_CMD_DEVICE_QUERY ordinary_command
#define Do_CMD_DEVICE_RESET ordinary_command
#define Do_CMD_DEVICE_RESUME ordinary_command
#define Do_CMD_STOP ordinary_command
#define Do_CMD_START ordinary_command
#define Do_CMD_XFER ordinary_command
#define Do_NSCMD_DEVICEQUERY ordinary_command
#define Do_CMD_REGISTER_ISO_HANDLER ordinary_command
#define Do_CMD_UNREGISTER_ISO_HANDLER ordinary_command
#define Do_CMD_STARTRTISO ordinary_command
#define Do_CMD_STOPRTISO ordinary_command
"""

CASES = r"""
static struct USBIORequest *request(unsigned command, unsigned flags)
{
    struct USBIORequest *io = calloc(1, sizeof(*io));
    assert(io);
    io->req.io_Message.mn_Length = sizeof(*io);
    io->req.io_Unit = &unit;
    io->req.io_Command = command;
    io->req.io_Flags = flags;
    expected_message = io;
    freed = aborted = replied = 0;
    completion = COMMAND_PROCESSED;
    return io;
}
int main(void)
{
    struct USBIORequest *io = request(CMD_INTERNAL_ABORT_REQUEST, IOF_QUICK);
    io->data_buffer = &original;
    ProcessCommand(io);
    assert(freed == 1 && aborted == 1 && replied == 0);

    io = request(CMD_INTERNAL_ABORT_REQUEST, IOF_QUICK);
    ProcessCommand(io);
    assert(freed == 1 && aborted == 0 && replied == 0);

    io = request(CMD_FLUSH, 0);
    ProcessCommand(io);
    assert(freed == 0 && aborted == 0 && replied == 1);
    free(io);

    io = request(CMD_FLUSH, IOF_QUICK);
    ProcessCommand(io);
    assert(freed == 0 && replied == 0);
    free(io);

    io = request(CMD_FLUSH, 0);
    completion = COMMAND_SCHEDULED;
    ProcessCommand(io);
    assert(freed == 0 && replied == 0);
    free(io);

    io = request(CMD_FLUSH, 0);
    io->req.io_Message.mn_Length = 0;
    ProcessCommand(io);
    assert(io->req.io_Error == IOERR_BADLENGTH && replied == 1 && freed == 0);
    free(io);

    io = request(999, 0);
    ProcessCommand(io);
    assert(io->req.io_Error == IOERR_NOCMD && replied == 1 && freed == 0);
    free(io);

    io = request(NSCMD_DEVICEQUERY, 0);
    io->req.io_Message.mn_Length = 0;
    ProcessCommand(io);
    assert(io->req.io_Error == ERR_NO_ERROR && replied == 1 && freed == 0);
    free(io);
    puts("abort lifetime: 8 cases passed");
}
"""


def function(source, signature):
    matches = re.findall(
        rf"(?ms)^{re.escape(signature)}\([^\n]*\)\n\{{.*?^\}}", source
    )
    if len(matches) != 1:
        raise ValueError(f"expected exactly one {signature}")
    return matches[0]


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path,
                        default=root / "xhci.device/src/unit_commands.c")
    parser.add_argument("--build-dir", type=Path, default=root / "build")
    args = parser.parse_args()
    source = args.source.read_text()
    code = PREAMBLE + function(source, "static inline u32 Do_CMD_INTERNAL_ABORT")
    code += "\n" + function(source, "void ProcessCommand") + CASES
    args.build_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="abort-lifetime-", dir=args.build_dir) as tmp:
        src, exe = Path(tmp) / "test.c", Path(tmp) / "test"
        src.write_text(code)
        subprocess.run([
            os.environ.get("CC", "clang"), "-std=c11", "-Wall", "-Wextra",
            "-Werror", "-O1", "-g", "-fsanitize=address,undefined",
            "-fno-omit-frame-pointer", str(src), "-o", str(exe)
        ], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
