#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Host model by codex-amiga, adapted to run from this tree (tests/host/run_all.sh).
"""Exercise exact driver ordering functions with an explicit stale-cache model.

This checks control flow under the stated model, not hardware memory ordering.
"""
from pathlib import Path
import os,subprocess,sys,tempfile
X=Path(__file__).resolve().parents[2]
B=Path(tempfile.mkdtemp(prefix='xhci-host-'))
CC=os.environ.get('CC','cc')
CFLAGS=['-std=c11','-Wall','-Wextra','-Werror','-O1','-g','-fsanitize=address,undefined','-fno-sanitize-recover=all']
def read(path):return (X/'xhci.device/src/xhci'/path).read_text()
def function(text,sig):
 a=text.index(sig);i=text.index('{',a)+1;depth=1
 while depth:depth+=(text[i]=='{')-(text[i]=='}');i+=1
 return text[a:i]+'\n'
ring=read('xhci-ring.c');submit=read('xhci-submit.c');events=read('xhci-events.c')
functions=[function(ring,'union xhci_trb *xhci_ring_get_event_trb_cached('),function(ring,'void xhci_cache_sync_all('),function(submit,'inline static void prime_first_trb('),function(submit,'inline static void giveback_first_trb('),function(events,'static BOOL process_events_coalesced(')]
prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t u32;typedef unsigned long ULONG;typedef int BOOL;
#define TRUE 1
#define FALSE 0
#define XHCI_COALESCE_CACHE_OPS 1
#define TRB_CYCLE 1U
#define DMA_ReadFromRAM 1U
#define le32(x) (x)
struct ExecBase {int unused;};
struct xhci_generic_trb {u32 field[4];};
union xhci_trb {struct xhci_generic_trb generic;struct {u32 unused[3],flags;} event_cmd;};
struct xhci_ring {union xhci_trb *dequeue;u32 cycle_state;struct ExecBase *sysBase;BOOL defer_cache;};
struct xhci_ctrl {struct xhci_ring *event_ring;struct ExecBase *sysBase;BOOL evt_unacked;};
struct usb_device {struct xhci_ctrl *controller;};
static union xhci_trb ram[2],cache[2];static struct xhci_generic_trb first;
static struct ExecBase exec;static struct xhci_ring er,tx;static struct xhci_ctrl ctrl;static struct usb_device dev;
static unsigned mode,handled,ids[8],acks,doors,pub_active,pub_flushes,first_flush_valid,ram_cycle,payload_ram,payload_cpu,bad;
static unsigned checks,failures;static char trace[128];static unsigned ti;
static void note(char c){if(ti<sizeof(trace)-1)trace[ti++]=c;else bad++;}
static int ring_defers_cache(const struct xhci_ring *r){return r->defer_cache;}
static void xhci_ring_ack_events(struct xhci_ctrl *c){(void)c;acks++;note('A');}
static void xhci_cache_flush_all(struct ExecBase *s){
 (void)s;note('F');memcpy(cache,ram,sizeof(cache));
 if(pub_active){if(pub_flushes++==0)first_flush_valid=first.field[3]&1;payload_ram=payload_cpu;ram_cycle=first.field[3]&1;}
}
static void fake_pre_dma(void *p,unsigned n,unsigned d){
 (void)p;(void)n;(void)d;note('P');ram_cycle=first.field[3]&1;
 if(pub_active&&(!pub_flushes||first_flush_valid||payload_ram!=7||ram_cycle!=1))bad++;
}
#define cache_pre_dma(p,n,d) ((void)SysBase,fake_pre_dma((p),(n),(d)))
static void xhci_submit_ring_doorbell(struct usb_device *u,struct xhci_ring *r){(void)u;(void)r;note('D');doors++;if(pub_active&&(ram_cycle!=1||payload_ram!=7))bad++;}
static void xhci_ring_consume_event(struct xhci_ctrl *c){
 c->event_ring->dequeue++;
 if(c->event_ring->dequeue==cache+2){c->event_ring->dequeue=cache;c->event_ring->cycle_state^=1;}
}
static void handle_event(struct xhci_ctrl *,union xhci_trb *);
'''
suffix=r'''
static void check(int yes,const char *why){checks++;if(!yes){failures++;printf("FAIL %s\n",why);}}
static void handle_event(struct xhci_ctrl *c,union xhci_trb *e){
 unsigned id=e->generic.field[0];if(handled>=8){bad++;return;}ids[handled++]=id;
 if(id==1&&mode){ram[1].generic.field[0]=2;ram[1].event_cmd.flags=1;}
 if(id==1&&mode==2){pub_active=1;giveback_first_trb(&dev,&tx,&first);pub_active=0;}
 if(id==1&&mode==3){
  /* Both slots are now available after ERDP; simulate a controller wrapping
     and reusing the first retired slot during the callback's flush. */
  ram[0].generic.field[0]=3;ram[0].event_cmd.flags=0;xhci_cache_sync_all(c);
  if(e->generic.field[0]!=id)bad++;
 }
}
static void reset(void){
 memset(ram,0,sizeof ram);memset(cache,0,sizeof cache);memset(&first,0,sizeof first);memset(ids,0,sizeof ids);memset(trace,0,sizeof trace);
 mode=handled=acks=doors=pub_active=pub_flushes=first_flush_valid=ram_cycle=payload_ram=bad=ti=0;payload_cpu=7;
 er=(struct xhci_ring){cache,1,&exec,0};tx=(struct xhci_ring){0,1,&exec,1};ctrl=(struct xhci_ctrl){&er,&exec,0};dev.controller=&ctrl;
}
int main(void){
 reset();pub_active=1;giveback_first_trb(&dev,&tx,&first);pub_active=0;
 check(!strcmp(trace,"FPD"),"submit orders full flush then publication then doorbell");
 check(pub_flushes==1&&!first_flush_valid&&payload_ram==7&&ram_cycle==1&&!bad,"first cycle stays invalid until payload is flushed");
 reset();ram[0].generic.field[0]=1;ram[0].event_cmd.flags=1;mode=1;
 check(process_events_coalesced(&ctrl),"event drain reports activity");
 check(handled==2&&ids[0]==1&&ids[1]==2,"empty recheck finds event hidden in stale cache line");
 check(acks==2&&!ctrl.evt_unacked&&!bad,"empty recheck acknowledges retired events before exit");
 reset();ram[0].generic.field[0]=1;ram[0].event_cmd.flags=1;mode=2;process_events_coalesced(&ctrl);
 check(handled==2&&ids[1]==2&&doors==1&&!bad,"giveback in handler supplies acknowledgement and recheck flush");
 check(strstr(trace,"AFPD")!=0,"nested giveback acknowledges ERDP before flushing and publishing");
 reset();ram[0].generic.field[0]=1;ram[0].event_cmd.flags=1;mode=3;process_events_coalesced(&ctrl);
 check(handled==3&&ids[0]==1&&ids[1]==2&&ids[2]==3,"event drain handles ring cycle wrap after callback reuse");
 check(!bad,"copied event survives early retirement and cache replacement");
 check(!ctrl.evt_unacked,"wrapped drain ends with no unacknowledged event");
 printf("coalesce-order checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
results={}
for name in ['fixed','publish-before-flush-negative','no-empty-recheck-negative','retired-alias-negative']:
 f=functions.copy()
 if name=='publish-before-flush-negative':
  old='xhci_cache_sync_all(udev->controller);\n\t\tstart_trb->field[3] ^= le32(TRB_CYCLE);';assert old in f[3];f[3]=f[3].replace(old,'start_trb->field[3] ^= le32(TRB_CYCLE);\n\t\txhci_cache_sync_all(udev->controller);')
 if name=='no-empty-recheck-negative':f[4]=f[4].replace('            continue;','            break;')
 if name=='retired-alias-negative':f[4]=f[4].replace('union xhci_trb event = *trb;','union xhci_trb *event = trb;').replace('handle_event(ctrl, &event);','handle_event(ctrl, event);')
 text=prefix+''.join(f)+suffix;c=B/(name+'.c');exe=B/name;c.write_text(text)
 subprocess.run([CC]+CFLAGS+[str(c),'-o',str(exe)],check=True)
 q=subprocess.run([str(exe)],capture_output=True,text=True)
 assert q.returncode==(0 if name=='fixed' else 1),(name,q.stdout,q.stderr)
 assert 'coalesce-order checks=10 failures=' in q.stdout,(name,q.stdout)
 target={'publish-before-flush-negative':'first cycle stays invalid','no-empty-recheck-negative':'empty recheck finds event hidden','retired-alias-negative':'copied event survives'}
 if name in target:assert 'FAIL '+target[name] in q.stdout,(name,q.stdout)
 results[name]={'exit':q.returncode,'output':q.stdout};print(name,q.stdout)
assert 'coalesce-order checks=10 failures=0' in results['fixed']['output'], results['fixed']
print('PASS', 'test_coalesce_order.py')
