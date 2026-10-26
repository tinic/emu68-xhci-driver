#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise cache-policy primitives and producer gates extracted from source.

The LVO decoder and CacheClearE implementation are mocks. Swapped-word mode
checks conversion arithmetic, not native big-endian ABI or physical cache lines.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CC = os.environ.get('CC', 'cc')


def source(path):
    return (ROOT / 'xhci.device' / path).read_text()


def function(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end] + '\n'


header = source('include/xhci/xhci.h')
private = source('src/xhci/xhci-ring-priv.h')
ring = source('src/xhci/xhci-ring.c')
submit = source('src/xhci/xhci-submit.c')
controller = source('src/xhci/xhci.c')
functions = [
    function(header, 'static inline BOOL xhci_cache_whole('),
    function(header, 'static inline void xhci_cache_flush_all('),
    function(private, 'static inline BOOL ring_defers_cache('),
    function(private, 'static inline int last_trb('),
    function(private, 'static inline BOOL last_trb_on_last_seg('),
    function(private, 'static inline void inc_enq('),
    function(private, 'static inline dma_addr_t xhci_ring_enqueue_trb_flags('),
    function(private, 'static inline dma_addr_t xhci_ring_enqueue_trb_ns('),
    function(private, 'static inline void prepare_ring('),
    function(submit, 'inline static void prime_first_trb('),
    function(ring, 'void xhci_ring_set_defer_cache('),
    function(controller, 'static BOOL in_kickstart_rom('),
    function(controller, 'static BOOL xhci_cache_ops_are_kickstart('),
]

prefix = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t u32;typedef unsigned long ULONG;typedef long LONG;typedef uintptr_t dma_addr_t;typedef int BOOL;
#define TRUE 1
#define FALSE 0
#define XHCI_COALESCE_CACHE_OPS 1
#define TRBS_PER_SEGMENT 2
#define TRB_CYCLE 1U
#define TRB_CHAIN 16U
#define LINK_TOGGLE 2U
#define TRB_TYPE(x) ((u32)(x)<<10)
#define TRB_LINK 6U
#define TRB_TYPE_LINK_LE32(x) (((x)&le32(63U<<10))==le32(TRB_TYPE(TRB_LINK)))
#define DMA_ReadFromRAM 1U
#define DMAF_NoSync 2U
/* Distinct mock tokens: assertions check which symbol the real wrapper uses. */
#define CACRF_ClearI 1U
#define CACRF_ClearD 2U
#define XHCI_LVO_CACHEPREDMA (-762)
#define XHCI_LVO_CACHEPOSTDMA (-768)
#if TEST_SWAPPED
#define le32(x) __builtin_bswap32((u32)(x))
#else
#define le32(x) ((u32)(x))
#endif
struct ExecBase {int unused;};
struct xhci_generic_trb {u32 field[4];};
union xhci_trb {struct xhci_generic_trb generic;struct {u32 unused[3],control;} link;};
struct xhci_segment {union xhci_trb *trbs;struct xhci_segment *next;};
struct xhci_ring {struct ExecBase *sysBase;BOOL is_event_ring;struct xhci_segment *first_seg,*enq_seg;union xhci_trb *enqueue;u32 cycle_state;BOOL defer_cache;};
struct xhci_ctrl {BOOL cache_whole;};
static union xhci_trb cpu[4],ram[4];static struct ExecBase exec;
static unsigned checks,failures,pre,clears,bad_lvo;
static void *clear_address;static ULONG clear_length,clear_flags,pre_target,post_target;
static void fake_pre(void *p,unsigned n,unsigned flags){
 (void)flags;size_t i=(union xhci_trb*)p-cpu;pre++;memcpy(&ram[i],p,n);
}
#define cache_pre_dma(p,n,f) ((void)SysBase,fake_pre((p),(n),(f)))
static void fake_clear(void *p,ULONG n,ULONG f){clear_address=p;clear_length=n;clear_flags=f;clears++;memcpy(ram,cpu,sizeof ram);}
#define CacheClearE(p,n,f) ((void)SysBase,fake_clear((p),(n),(f)))
/* The 68k JMP-slot load has target-specific alignment/word-size semantics;
   only the actual ROM bounds and pair-selection logic are under test here. */
static ULONG exec_lvo_target(struct ExecBase *s,LONG lvo){
 (void)s;if(lvo==XHCI_LVO_CACHEPREDMA)return pre_target;
 if(lvo==XHCI_LVO_CACHEPOSTDMA)return post_target;
 bad_lvo++;return 0;
}
'''

suffix = r'''
static void check(int yes,const char *why){checks++;if(!yes){failures++;printf("FAIL %s\n",why);}}
static void reset(void){memset(cpu,0,sizeof cpu);memset(ram,0,sizeof ram);pre=clears=0;}
static void producer(int defer){
 struct xhci_segment a={cpu,0},b={cpu+2,&a};a.next=&b;
 struct xhci_ring r={&exec,FALSE,&a,&a,cpu,1,FALSE};struct xhci_ctrl c={defer};
 xhci_ring_set_defer_cache(&r,&c);check(r.defer_cache==defer,"defer selector follows controller");
 reset();cpu[0].generic.field[3]=le32(0x20);
 prime_first_trb(&r,&cpu[0].generic);
 check(cpu[0].generic.field[3]==le32(0x21),"prime flips converted cycle and preserves flags");
 check(pre==(unsigned)!defer,"prime publishes only legacy ring");
 check(ram[0].generic.field[3]==(defer?0:le32(0x21)),"prime defers visibility until giveback");
 prime_first_trb(&r,&cpu[0].generic);
 check(cpu[0].generic.field[3]==le32(0x20),"second prime restores converted cycle");
 reset();cpu[1].link.control=le32(TRB_TYPE(TRB_LINK));
 dma_addr_t addr=xhci_ring_enqueue_trb_ns(&r,TRUE,77,88,99,TRB_CHAIN|TRB_CYCLE);
 check(addr==(dma_addr_t)&cpu[0]&&cpu[0].generic.field[0]==le32(77),"enqueue returns written TRB");
 check(r.enqueue==cpu+2&&r.enq_seg==&b&&r.cycle_state==1,"enqueue crosses nonfinal segment");
 check(cpu[1].link.control==le32(TRB_TYPE(TRB_LINK)|TRB_CHAIN|TRB_CYCLE),"enqueue Link chain/cycle");
 check(pre==(defer?0U:2U),"enqueue and inc_enq obey defer gate");
 check(ram[0].generic.field[0]==(defer?0:le32(77))&&ram[1].link.control==(defer?0:cpu[1].link.control),"enqueue and Link visibility obey defer gate");
 reset();r.enq_seg=&a;r.enqueue=cpu;r.cycle_state=1;cpu[1].link.control=le32(TRB_TYPE(TRB_LINK)|TRB_CHAIN);
 xhci_ring_enqueue_trb_ns(&r,FALSE,42,0,0,TRB_CYCLE);
 check(r.enqueue==cpu+1&&r.enq_seg==&a,"closed TD parks at Link");
 check(pre==(unsigned)!defer,"parked Link not published prematurely");
 prepare_ring(&r);
 check(r.enqueue==cpu+2&&r.enq_seg==&b,"prepare resumes parked Link");
 check(cpu[1].link.control==le32(TRB_TYPE(TRB_LINK)|TRB_CYCLE),"prepare clears Link chain");
 check(pre==(defer?0U:2U),"prepare obeys defer gate");
 check(ram[1].link.control==(defer?0:cpu[1].link.control),"prepare Link visibility obeys defer gate");
 reset();r.enq_seg=&b;r.enqueue=cpu+3;r.cycle_state=1;
 cpu[3].link.control=le32(TRB_TYPE(TRB_LINK)|LINK_TOGGLE|TRB_CHAIN);
 prepare_ring(&r);
 check(r.enqueue==cpu&&r.enq_seg==&a&&r.cycle_state==0,"prepare final Link wraps producer cycle");
 check(cpu[3].link.control==le32(TRB_TYPE(TRB_LINK)|LINK_TOGGLE|TRB_CYCLE),"prepare final Link preserves toggle");
 check(pre==(unsigned)!defer,"wrapped prepare obeys defer gate");
}
int main(void){
 reset();cpu[0].generic.field[0]=77;xhci_cache_flush_all(&exec);
 check(clears==1&&clear_address==NULL&&clear_length==~0UL,"flush covers all addresses");
 check(clear_flags==CACRF_ClearD,"flush requests data-cache maintenance");
 check(ram[0].generic.field[0]==77,"mock flush publishes modeled cache");
 ULONG starts[]={0xF80000UL,0xE00000UL},ends[]={0x1000000UL,0xE80000UL};
 for(unsigned i=0;i<2;i++){
  check(in_kickstart_rom(starts[i])&&in_kickstart_rom(ends[i]-1),"ROM bank includes endpoints");
  check(!in_kickstart_rom(starts[i]-1)&&!in_kickstart_rom(ends[i]),"ROM bank excludes outside endpoints");
 }
 pre_target=0xF80000UL;post_target=0xE00000UL;
 check(xhci_cache_ops_are_kickstart(&exec),"ROM pair enables coalescing");
 post_target=0x1000UL;check(!xhci_cache_ops_are_kickstart(&exec),"patched post vector disables coalescing");
 pre_target=0x1000UL;post_target=0xF80000UL;check(!xhci_cache_ops_are_kickstart(&exec),"patched pre vector disables coalescing");
 check(!bad_lvo,"probe uses correct LVO identities");
 producer(0);producer(1);
 printf("cache-policy swapped=%d checks=%u failures=%u\n",TEST_SWAPPED,checks,failures);
 return failures?1:0;
}
'''

mutants = {
    'prime-no-cycle': (9, 'start_trb->field[3] ^= le32(TRB_CYCLE);', '(void)start_trb;', 'prime flips converted cycle'),
    'prime-no-conversion': (9, '^= le32(TRB_CYCLE)', '^= TRB_CYCLE', 'prime flips converted cycle'),
    'flush-instruction-only': (1, 'CACRF_ClearD', 'CACRF_ClearI', 'flush requests data-cache'),
    'flush-zero-length': (1, '~0UL', '0UL', 'flush covers all addresses'),
    'defer-inverted': (10, '= xhci_cache_whole(ctrl)', '= !xhci_cache_whole(ctrl)', 'defer selector'),
    'probe-either-vector': (12, ' &&', ' ||', 'patched post vector'),
    'enqueue-always-publish': (6, 'if (!ring_defers_cache(ring))', 'if (TRUE)', 'enqueue and inc_enq obey defer'),
    'inc-enq-always-publish': (5, 'if (!ring_defers_cache(ring))', 'if (TRUE)', 'enqueue and inc_enq obey defer'),
    'prepare-always-publish': (8, 'if (!ring_defers_cache(ep_ring))', 'if (TRUE)', 'prepare obeys defer'),
}

with tempfile.TemporaryDirectory(prefix='xhci-policy-') as temporary:
    directory = Path(temporary)
    for swapped in [0, 1]:
        for name in ['fixed', *mutants]:
            pieces = functions.copy()
            if name != 'fixed':
                index, old, new, target = mutants[name]
                assert pieces[index].count(old) == 1, (name, old)
                pieces[index] = pieces[index].replace(old, new, 1)
            c = directory / (name + '.c')
            exe = directory / name
            c.write_text(prefix + ''.join(pieces) + suffix)
            subprocess.run([CC, '-std=c11', '-Wall', '-Wextra', '-Werror', '-O1', '-g',
                            '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                            '-DTEST_SWAPPED=' + str(swapped), str(c), '-o', str(exe)],
                           check=True, timeout=60)
            result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=10)
            # Omitting conversion is equivalent only in identity-word mode.
            negative = name != 'fixed' and not (name == 'prime-no-conversion' and swapped == 0)
            assert result.returncode == int(negative), (name, swapped, result.stdout, result.stderr)
            if negative:
                assert 'FAIL ' + target in result.stdout, (name, swapped, result.stdout)
            print(name, result.stdout, end='', flush=True)

print('PASS test_cache_policy.py')
