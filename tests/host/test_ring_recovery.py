#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Host model by codex-amiga, adapted to run from this tree (tests/host/run_all.sh).
"""Exact 44ce227 ring functions, with deterministic CPU-cache/RAM mocks.
Host control-flow regression only; not a hardware DMA or USB stress test.
"""
from pathlib import Path
import os,subprocess,sys,tempfile
X=Path(__file__).resolve().parents[2]
workspace = tempfile.TemporaryDirectory(prefix='xhci-host-')
B = Path(workspace.name)
CC=os.environ.get('CC','cc')
CFLAGS=['-std=c11','-Wall','-Wextra','-Werror','-O1','-g','-fsanitize=address,undefined','-fno-sanitize-recover=all']
def read(p):return (X/'xhci.device/src/xhci'/p).read_text()
def fun(s,sig):
 a=s.index(sig);i=s.index('{',a)+1;d=1
 while d:d+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[a:i]+'\n'
ring=read('xhci-ring.c');priv=read('xhci-ring-priv.h')
functions=[fun(priv,'static inline BOOL ring_defers_cache('),fun(priv,'static inline int last_trb('),fun(priv,'static inline BOOL last_trb_on_last_seg('),fun(ring,'inline static void inc_deq('),fun(ring,'void xhci_ring_consume_event('),fun(ring,'u32 xhci_ring_get_deq_ptr_for_trb('),fun(ring,'void xhci_ring_patch_trbs_to_noop(')]
prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t u32;typedef uintptr_t dma_addr_t;typedef int BOOL;
#define TRUE 1
#define FALSE 0
#define XHCI_COALESCE_CACHE_OPS 1
#define TRBS_PER_SEGMENT 2
#define TRB_CYCLE 1U
#define TRB_CHAIN (1U<<4)
#define LINK_TOGGLE (1U<<1)
#define TRB_TYPE(x) ((u32)(x)<<10)
#define TRB_LINK 6U
#define TRB_TR_NOOP 8U
#define TRB_TYPE_LINK_LE32(x) (((x)&(63U<<10))==TRB_TYPE(TRB_LINK))
#define DMA_ReadFromRAM 1U
#define le32(x) (x)
struct ExecBase {int x;};
union xhci_trb {struct {u32 field[4];} generic;struct {u32 unused[3],control;} link;};
struct xhci_segment {union xhci_trb *trbs;struct xhci_segment *next;};
struct xhci_ring {struct ExecBase *sysBase;BOOL is_event_ring;struct xhci_segment *first_seg,*deq_seg;union xhci_trb *dequeue;u32 cycle_state;BOOL defer_cache;};
struct xhci_ctrl {struct xhci_ring *event_ring;};
static union xhci_trb cpu[4],ram[4];static unsigned pre,post,full,checks,failures;static char trace[100];static unsigned nt;
static void note(char c){if(nt<sizeof(trace)-1)trace[nt++]=c;}
static void xhci_cache_flush_all(struct ExecBase *s){(void)s;memcpy(ram,cpu,sizeof ram);full++;note('F');}
static void fake_post(void *p,unsigned len,unsigned flags){(void)flags;size_t i=(union xhci_trb*)p-cpu;memcpy(p,&ram[i],len);post++;note('I');}
static void fake_pre(void *p,unsigned len,unsigned flags){(void)flags;size_t i=(union xhci_trb*)p-cpu;memcpy(&ram[i],p,len);pre++;note('P');}
#define cache_post_dma(p,n,f) ((void)SysBase,fake_post(p,n,f))
#define cache_pre_dma(p,n,f) ((void)SysBase,fake_pre(p,n,f))
'''
suffix=r'''
static void check(int good,const char *why){checks++;if(!good){failures++;printf("FAIL %s\n",why);}}
static void reset(void){memset(cpu,0,sizeof cpu);memset(ram,0,sizeof ram);memset(trace,0,sizeof trace);pre=post=full=nt=0;}
int main(void){
 struct ExecBase exec={0};struct xhci_segment a={cpu,0},b={cpu+2,&a};a.next=&b;
 struct xhci_ring er={&exec,TRUE,&a,&a,cpu,1,FALSE};struct xhci_ctrl ctrl={&er};
 xhci_ring_consume_event(&ctrl);check(er.dequeue==cpu+1&&er.deq_seg==&a&&er.cycle_state==1,"event step within first segment");
 xhci_ring_consume_event(&ctrl);check(er.dequeue==cpu+2&&er.deq_seg==&b&&er.cycle_state==1,"event step to next segment keeps cycle");
 xhci_ring_consume_event(&ctrl);check(er.dequeue==cpu+3&&er.cycle_state==1,"event step within final segment");
 xhci_ring_consume_event(&ctrl);check(er.dequeue==cpu&&er.deq_seg==&a&&er.cycle_state==0,"final segment wrap toggles cycle");
 for(unsigned i=0;i<4;i++)xhci_ring_consume_event(&ctrl);
 check(er.dequeue==cpu&&er.cycle_state==1,"second lap restores cycle");
 /* Transfer dequeue skips Link TRBs and does not change producer cycle. */
 reset();struct xhci_ring tx={&exec,FALSE,&a,&a,cpu,1,TRUE};cpu[1].link.control=TRB_TYPE(TRB_LINK)|LINK_TOGGLE;
 inc_deq(&tx);check(tx.dequeue==cpu+2&&tx.deq_seg==&b&&tx.cycle_state==1,"transfer consumer skips link without changing producer cycle");
 reset();tx.defer_cache=TRUE;cpu[0].generic.field[0]=0xfeed;cpu[0].generic.field[3]=TRB_CYCLE;
 u32 q=xhci_ring_get_deq_ptr_for_trb(&tx,(dma_addr_t)cpu);
 check(q==((u32)(uintptr_t)cpu|1U)&&cpu[0].generic.field[0]==0xfeed,"dequeue lookup preserves pending dirty cycle/data");
 check(full==1&&post==1&&!strcmp(trace,"FI"),"dequeue pushes dirty TRB before invalidate");
 reset();check(xhci_ring_get_deq_ptr_for_trb(&tx,0)==0&&full==0&&post==0,"null dequeue lookup has no cache work");
 reset();tx.defer_cache=FALSE;ram[0].generic.field[3]=1;cpu[0].generic.field[3]=0;
 q=xhci_ring_get_deq_ptr_for_trb(&tx,(dma_addr_t)cpu);check((q&1U)==1&&full==0&&post==1,"legacy lookup retains direct invalidate path");
 /* Dirty span crossing a Link: recovery must preserve both cycles, patch
    only selected TRBs, and clear the chain on the intervening Link. */
 reset();tx.defer_cache=TRUE;cpu[0].generic.field[0]=77;cpu[0].generic.field[3]=1|TRB_CHAIN;
 cpu[1].link.control=TRB_TYPE(TRB_LINK)|TRB_CHAIN|LINK_TOGGLE|TRB_CYCLE;cpu[2].generic.field[0]=88;cpu[2].generic.field[3]=0;
 dma_addr_t addrs[2]={(dma_addr_t)&cpu[0],(dma_addr_t)&cpu[2]};xhci_ring_patch_trbs_to_noop(&tx,addrs,2,0);
 check(full==1&&trace[0]=='F',"NoOp recovery pushes dirty ring before invalidates");
 check(ram[0].generic.field[0]==0&&ram[0].generic.field[1]==0&&ram[0].generic.field[2]==0&&ram[0].generic.field[3]==(TRB_TYPE(TRB_TR_NOOP)|1U),"NoOp first preserves pending cycle1");
 check(ram[2].generic.field[3]==TRB_TYPE(TRB_TR_NOOP),"NoOp second preserves pending cycle0");
 check(ram[1].link.control==(TRB_TYPE(TRB_LINK)|LINK_TOGGLE|TRB_CYCLE),"intervening Link loses chain but retains type/toggle/cycle");
 check(pre==3&&post==3,"NoOp recovery publishes both TRBs and intervening Link");
 reset();cpu[0].generic.field[0]=77;cpu[0].generic.field[3]=1;cpu[2].generic.field[3]=1;
 xhci_ring_patch_trbs_to_noop(&tx,addrs,2,1);check(ram[0].generic.field[0]==77&&ram[2].generic.field[3]==(TRB_TYPE(TRB_TR_NOOP)|1U),"start index leaves preceding TRB intact");
 reset();xhci_ring_patch_trbs_to_noop(&tx,addrs,2,2);xhci_ring_patch_trbs_to_noop(&tx,0,2,0);check(full==0&&pre==0&&post==0,"invalid recovery ranges do no cache work");
 printf("ring-recovery checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
results={}
for variant in ['fixed','missing-recovery-push-negative','missing-link-unchain-negative','missing-event-cycle-toggle-negative']:
 f=functions.copy()
 if variant=='missing-recovery-push-negative':
  for i in [5,6]:f[i]=f[i].replace('if (ring_defers_cache(ring))','if (0 && ring_defers_cache(ring))')
 if variant=='missing-link-unchain-negative':f[6]=f[6].replace('link->link.control &= le32(~TRB_CHAIN);','link->link.control |= le32(0);')
 if variant=='missing-event-cycle-toggle-negative':f[3]=f[3].replace('ring->cycle_state = (ring->cycle_state ? 0 : 1);','ring->cycle_state = ring->cycle_state;')
 source=prefix+''.join(f)+suffix;p=B/(variant+'.c');exe=B/variant;p.write_text(source)
 subprocess.run([CC]+CFLAGS+[str(p),'-o',str(exe)],check=True, timeout=60)
 r=subprocess.run([str(exe)],text=True,capture_output=True, timeout=10)
 assert r.returncode==(0 if variant=='fixed' else 1),(variant,r.stdout,r.stderr)
 target={'missing-recovery-push-negative':'FAIL dequeue lookup preserves pending dirty cycle/data','missing-link-unchain-negative':'FAIL intervening Link loses chain','missing-event-cycle-toggle-negative':'FAIL final segment wrap toggles cycle'}
 if variant in target:assert target[variant] in r.stdout,(variant,r.stdout)
 results[variant]={'returncode':r.returncode,'output':r.stdout};print(variant,r.stdout,flush=True)
assert 'ring-recovery checks=17 failures=0' in results['fixed']['output'], results['fixed']
print('PASS', 'test_ring_recovery.py')
