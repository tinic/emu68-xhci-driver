#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Run the real interrupt attach, ISR gate and re-arm against an xHC model
with IMAN.IP write-1-to-clear."""
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
typedef int s32;
typedef long LONG;
typedef unsigned long ULONG;
typedef int BOOL;
typedef signed char BYTE;
typedef void *APTR;
#define TRUE 1
#define FALSE 0
#define asm(reg)
#define EMU68_INTSERVER(name)
#define NT_INTERRUPT 2
#define PCI_IRQ_INTX 1u
#define PCI_IRQ_MSI 2u
#define PCI_IRQ_MSIX 4u
#define DEVICE_USE_MSI TRUE
#define DEVICE_USE_MSIX TRUE
#define BIT(n) (1u << (n))
#define IMAN_IP BIT(0)
#define IMAN_IE BIT(1)
#define STS_FATAL BIT(2)
#define STS_EINT BIT(3)
#define STS_PORT BIT(4)
#define CMD_EIE BIT(2)
#define CMD_HSEIE BIT(3)
#define XHCI_IRQ_ACK_MASK (STS_EINT | STS_FATAL | STS_PORT)
static void Kprintf(const char *fmt, ...) { (void)fmt; }
#define KprintfT Kprintf
struct ExecBase { int unused; };
struct Library { int unused; };
struct Task { int unused; };
struct Node { int ln_Type; const char *ln_Name; };
struct Interrupt { struct Node is_Node; APTR is_Data; APTR is_Code; };
struct pci_dev { int unused; };
struct xhci_hcor { u32 or_usbcmd, or_usbsts; };
struct xhci_intr_reg { u32 irq_pending; };
struct xhci_ctrl {
    struct xhci_hcor *hcor;
    struct xhci_intr_reg *ir_set;
    struct pci_dev *pci_dev;
    u32 iman_base;
};
struct XHCIDevice { struct Library *pcieBase; };
struct XHCIUnit {
    struct xhci_ctrl *xhci_ctrl;
    struct XHCIDevice *device;
    struct Task *task;
    struct Interrupt irq_isr;
    u32 irq_line;
    BYTE irq_signal;
    BOOL msi_enabled;
};

/* xHC model.  IMAN (xHCI 5.5.2.1): IP bit 0 RW1C, IE bit 1 RW, 31:2 RsvdP.
 * USBSTS: EINT RW1C, set when an IP goes 0->1 (5.4.2).  The line follows
 * IP && IE for INTx and the non-PCI attach (4.17.2). */
#define RSVD 0xa5a5a5a4u
static struct xhci_hcor hcor;
static struct xhci_intr_reg ir;
static u32 ip, ie, rsvd, eint, usbcmd;
static u32 iman_writes, ip_writes, rsvd_lost, vector_masked, signals;
static u32 intserver_added, attach_itype, attached;
static void post_event(void)
{
    if (!ip) eint = 1;
    ip = 1;
}
static u32 line(void) { return ip && ie; }
static u32 mmio_read32(u32 *reg)
{
    if (reg == &ir.irq_pending) return rsvd | ie << 1 | ip;
    if (reg == &hcor.or_usbsts) return eint ? STS_EINT : 0;
    if (reg == &hcor.or_usbcmd) return usbcmd;
    assert(!"read of an unmodelled register");
    return 0;
}
static void mmio_write32(u32 value, u32 *reg)
{
    if (reg == &ir.irq_pending)
    {
        iman_writes++;
        if ((value & ~3u) != rsvd) rsvd_lost++;
        if (value & IMAN_IP) { ip_writes++; ip = 0; }
        ie = (value & IMAN_IE) ? 1 : 0;
    }
    else if (reg == &hcor.or_usbsts) { if (value & STS_EINT) eint = 0; }
    else if (reg == &hcor.or_usbcmd) usbcmd = value;
    else assert(!"write of an unmodelled register");
}
/* like the proto macros, the vector calls go through pcielibBase */
static void vector_mask(struct Library *base, struct pci_dev *dev, int vec, u32 masked)
{ assert(base && dev && vec == 0); vector_masked = masked; }
#define MaskIntVector(dev, vec) vector_mask(pcielibBase, dev, vec, 1)
#define UnmaskIntVector(dev, vec) vector_mask(pcielibBase, dev, vec, 0)
/* exec calls use the SysBase local, as EXEC_BASE_NAME does */
#define Signal(task, mask) (assert(SysBase && (task) && (mask)), signals++)
static LONG AddIntServerEx(ULONG irq, int pri, BOOL edge, struct Interrupt *isr)
{ (void)irq; (void)pri; (void)edge; assert(isr->is_Code); intserver_added++; return 0; }
static LONG pci_irq_attach(struct Library *base, struct pci_dev *dev, struct Interrupt *isr,
                           ULONG flags, ULONG *itype)
{
    assert(base && dev && isr->is_Code);
    assert(flags == (PCI_IRQ_INTX | PCI_IRQ_MSI | PCI_IRQ_MSIX));
    attached++; *itype = attach_itype; return 0;
}
static const char *pcie_strerror(LONG rc) { (void)rc; return "?"; }
"""

CASES = r"""
static struct pci_dev pdev;
static struct xhci_ctrl ctrl = { &hcor, &ir, 0, 0 };
static struct Library pcie;
static struct XHCIDevice dev = { &pcie };
static struct Task task;
static struct XHCIUnit unit = { &ctrl, &dev, &task, {{0, 0}, 0, 0}, 0, 5, 0 };
static struct ExecBase sysbase;

static void attach(struct pci_dev *pd, ULONG itype)
{
    ctrl.pci_dev = pd; attach_itype = itype;
    ip = ie = eint = usbcmd = 0; rsvd = RSVD;
    iman_writes = ip_writes = rsvd_lost = vector_masked = signals = 0;
    intserver_added = attached = 0;
    assert(xhci_int_enable(&unit) == 0);
    assert(usbcmd == (CMD_EIE | CMD_HSEIE) && ie == 1 && !rsvd_lost);
    iman_writes = ip_writes = 0; /* count runtime writes only */
}

static ULONG isr(void) { return xhci_int_isr(&sysbase, &unit, 0); }

/* IMAN-gated mode: the onboard unit 0 (no pci_dev) and PCIe INTx */
static void gated_mode(const char *name)
{
    /* event: line up, ISR acks EINT and IP (deliberate IP=1), shuts IE */
    post_event();
    assert(line());
    assert(isr() == 1 && signals == 1 && !eint);
    assert(ip == 0 && ie == 0 && ip_writes == 1 && !line() && !rsvd_lost);

    /* event while gated (after the drain, before the re-arm): latches IP */
    post_event();
    assert(!line() && eint);
    ip_writes = 0;
    xhci_int_rearm(&unit);
    assert(ie == 1 && !rsvd_lost);
    assert(ip_writes == 0 && "re-arm wrote IP=1 and cleared a pending interrupt");
    assert(ip == 1 && line());
    assert(isr() == 1 && signals == 2 && ip == 0 && !line());

    /* nothing pending: re-arm leaves the line quiet */
    ip_writes = 0;
    xhci_int_rearm(&unit);
    assert(ie == 1 && ip == 0 && !line() && ip_writes == 0 && !rsvd_lost);
    printf("iman rearm: %s passed\n", name);
}

int main(void)
{
    /* unit 0, onboard /scb/xhci: no pci_dev, AddIntServerEx, never MSI */
    attach(0, PCI_IRQ_MSI);
    assert(intserver_added == 1 && attached == 0 && !unit.msi_enabled);
    gated_mode("unit 0 non-PCI (IMAN gate)");

    /* PCIe unit on INTx: same IMAN gate */
    attach(&pdev, PCI_IRQ_INTX);
    assert(attached == 1 && intserver_added == 0 && !unit.msi_enabled);
    gated_mode("PCIe INTx (IMAN gate)");

    /* not the xHC asserting: ISR reports not-handled and writes nothing */
    iman_writes = 0;
    assert(isr() == 0 && iman_writes == 0);

    /* startup enable shares the helper: a latched IP survives it */
    rsvd = RSVD; ip = 1; ie = 0; ip_writes = 0;
    ctrl.pci_dev = 0;
    assert(xhci_int_enable(&unit) == 0);
    assert(ie == 1 && ip == 1 && ip_writes == 0 && !rsvd_lost);

    /* MSI and MSI-X: the runtime gate is the root-complex vector mask,
     * IMAN is not written after the start */
    ULONG msi[2] = { PCI_IRQ_MSI, PCI_IRQ_MSIX };
    for (int i = 0; i < 2; i++)
    {
        attach(&pdev, msi[i]);
        assert(attached == 1 && unit.msi_enabled);
        post_event();
        iman_writes = 0;
        assert(isr() == 1 && signals == 1 && vector_masked && iman_writes == 0);
        xhci_int_rearm(&unit);
        assert(!vector_masked && iman_writes == 0 && ie == 1);
    }
    puts("iman rearm: MSI and MSI-X (vector mask) passed");
    puts("iman rearm: all cases passed");
}
"""

FUNCTIONS = (
    "static inline void xhci_irq_disable_runtime",
    "static inline void xhci_irq_enable_runtime",
    "static inline void xhci_irq_mask",
    "static inline void xhci_irq_unmask",
    "static inline void xhci_irq_update_cmd",
    "ULONG xhci_int_isr",
    "static inline void xhci_setup_isr",
    "static inline void xhci_irq_start",
    "void xhci_int_rearm",
    "static s32 xhci_pci_int_enable",
    "s32 xhci_int_enable",
)


def function(source, signature):
    matches = re.findall(
        rf"(?ms)^{re.escape(signature)}\([^{{;]*?\)\n\{{.*?^\}}", source
    )
    if len(matches) != 1:
        raise ValueError(f"expected exactly one {signature}")
    return matches[0]


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path,
                        default=root / "xhci.device/src/irq.c")
    parser.add_argument("--build-dir", type=Path, default=root / "build")
    args = parser.parse_args()
    source = args.source.read_text()
    code = PREAMBLE
    for signature in FUNCTIONS:
        code += function(source, signature) + "\n"
    code += CASES
    args.build_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="iman-rearm-", dir=args.build_dir) as tmp:
        src, exe = Path(tmp) / "test.c", Path(tmp) / "test"
        src.write_text(code)
        subprocess.run([
            os.environ.get("CC", "clang"), "-std=c11", "-Wall", "-Wextra",
            "-Werror", "-O1", "-g", "-fsanitize=address,undefined",
            "-fno-sanitize-recover=all", "-fno-omit-frame-pointer",
            str(src), "-o", str(exe)
        ], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
