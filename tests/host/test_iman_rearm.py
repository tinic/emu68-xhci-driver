#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Run the real IMAN gate against an IMAN model with IP write-1-to-clear."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile


PREAMBLE = r"""
#include <assert.h>
#include <stdio.h>
typedef unsigned u32;
typedef int BOOL;
struct Library { int unused; };
struct xhci_intr_reg { u32 irq_pending; };
struct xhci_ctrl { struct xhci_intr_reg *ir_set; void *pci_dev; };
struct XHCIDevice { struct Library *pcieBase; };
struct XHCIUnit {
    struct xhci_ctrl *xhci_ctrl;
    struct XHCIDevice *device;
    BOOL msi_enabled;
};
/* IMAN (xHCI 5.5.2.1): IP bit 0 RW1C, IE bit 1 RW, rest preserved */
static u32 ip, ie, writes, ip_writes, vector_masked;
static u32 mmio_read32(u32 *reg) { (void)reg; return ip | ie << 1; }
static void mmio_write32(u32 value, u32 *reg)
{
    (void)reg;
    writes++;
    if (value & 1) { ip_writes++; ip = 0; }
    ie = (value >> 1) & 1;
}
/* like the proto macros, the vector calls go through pcielibBase */
static void vector_mask(struct Library *base, void *dev, int vec, u32 masked)
{ assert(base); (void)dev; (void)vec; vector_masked = masked; }
#define MaskIntVector(dev, vec) vector_mask(pcielibBase, dev, vec, 1)
#define UnmaskIntVector(dev, vec) vector_mask(pcielibBase, dev, vec, 0)
static u32 line(void) { return ip && ie; }
"""

CASES = r"""
static struct xhci_intr_reg ir;
static struct xhci_ctrl ctrl = { &ir, 0 };
static struct Library pcie;
static struct XHCIDevice dev = { &pcie };
static struct XHCIUnit unit = { &ctrl, &dev, 0 };
static void reset(u32 pending, u32 enabled)
{ ip = pending; ie = enabled; writes = ip_writes = vector_masked = 0; }
int main(void)
{
    /* ISR gate: acks the interrupt it serves and shuts IE */
    reset(1, 1);
    xhci_irq_mask(&unit);
    assert(ip == 0 && ie == 0 && ip_writes == 1);

    /* event lands while gated: re-arm must keep IP so the line re-asserts */
    ip = 1; ip_writes = 0;
    xhci_int_rearm(&unit);
    assert(ie == 1);
    assert(ip_writes == 0 && "enable wrote IP=1 and cleared a pending interrupt");
    assert(ip == 1 && line());

    /* nothing pending: re-arm leaves the line quiet */
    reset(0, 0);
    xhci_int_rearm(&unit);
    assert(ie == 1 && ip == 0 && !line() && ip_writes == 0);

    /* MSI path leaves IMAN alone */
    unit.msi_enabled = 1;
    reset(1, 1);
    xhci_irq_mask(&unit);
    assert(vector_masked && writes == 0);
    xhci_int_rearm(&unit);
    assert(!vector_masked && writes == 0 && ip == 1);
    puts("iman rearm: 4 cases passed");
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
                        default=root / "xhci.device/src/irq.c")
    parser.add_argument("--regs", type=Path,
                        default=root / "xhci.device/include/xhci/xhci-regs.h")
    parser.add_argument("--build-dir", type=Path, default=root / "build")
    args = parser.parse_args()
    source = args.source.read_text()
    macros = re.findall(r"(?m)^#define ER_IRQ_(?:PENDING|CLEAR|ENABLE|DISABLE)\(.*$",
                        args.regs.read_text())
    if len(macros) != 4:
        raise ValueError("expected four ER_IRQ_ macros")
    code = PREAMBLE + "\n".join(macros) + "\n"
    for signature in ("static inline void xhci_irq_disable_runtime",
                      "static inline void xhci_irq_enable_runtime",
                      "static inline void xhci_irq_mask",
                      "static inline void xhci_irq_unmask",
                      "void xhci_int_rearm"):
        code += function(source, signature) + "\n"
    code += CASES
    args.build_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="iman-rearm-", dir=args.build_dir) as tmp:
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
