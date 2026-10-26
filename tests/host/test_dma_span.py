#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Host model by codex-amiga, adapted to run from this tree (tests/host/run_all.sh).
"""Source-extracted DMA span tests. Hardware/cache-line behavior is not modeled."""
from pathlib import Path
import os,subprocess,sys,tempfile
X=Path(__file__).resolve().parents[2]
workspace = tempfile.TemporaryDirectory(prefix='xhci-host-')
B = Path(workspace.name)
CC=os.environ.get('CC','cc')
CFLAGS=['-std=c11','-Wall','-Wextra','-Werror','-O1','-g','-fsanitize=address,undefined','-fno-sanitize-recover=all']
def src(path):return (X/'xhci.device'/path).read_text()
def fun(s,sig):
 a=s.index(sig);i=s.index('{',a)+1;d=1
 while d:d+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[a:i]+'\n'
s=src('src/xhci/xhci-dma.c');h=src('include/xhci/xhci-dma.h');types=h[h.index('enum xhci_dma_mapping'):h.index('/* ---- Span API')]
funcs=[fun(s,'static inline u32 split_middle_len('),fun(s,'void xhci_dma_span_sync('),fun(s,'u32 xhci_dma_span_extents('),fun(s,'APTR xhci_dma_span_link_piece('),fun(s,'static inline u32 to_copy_home('),fun(s,'static void copy_piece_home('),fun(s,'void xhci_dma_span_unmap(')]
prefix=r'''// SPDX-License-Identifier: GPL-2.0-only
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
typedef uint32_t u32;typedef uint16_t u16;typedef unsigned long ULONG;typedef unsigned char UBYTE;typedef void *APTR;typedef uintptr_t dma_addr_t;typedef int BOOL;
#define BIT(n) (1U<<(n))
#define DMA_ALIGN_MIN 32U
#define DMA_ReadFromRAM 1U
#define DMA_WriteToRAM 2U
struct ExecBase {int x;};struct slab_cache {int x;};
struct xhci_ctrl {struct ExecBase *sysBase;int whole;struct slab_cache bounce_link;};
static _Alignas(32) unsigned char user[8192],bounce[8192],linkbuf[8192];
static unsigned char uram[8192],bram[8192],lram[8192];
static unsigned checks,failures,pre,post,freed,linkfree;
static unsigned char *ram_for(void *p){uintptr_t a=(uintptr_t)p;
 if(a>=(uintptr_t)user&&a<(uintptr_t)(user+sizeof user))return uram+(a-(uintptr_t)user);
 if(a>=(uintptr_t)bounce&&a<(uintptr_t)(bounce+sizeof bounce))return bram+(a-(uintptr_t)bounce);
 if(a>=(uintptr_t)linkbuf&&a<(uintptr_t)(linkbuf+sizeof linkbuf))return lram+(a-(uintptr_t)linkbuf);
 abort();}
static int xhci_cache_whole(const struct xhci_ctrl *c){return c->whole;}
static void fake_pre(void *p,unsigned n,unsigned dir){pre++;if(dir==DMA_ReadFromRAM)memcpy(ram_for(p),p,n);else memcpy(p,ram_for(p),n);}
static void fake_post(void *p,unsigned n,unsigned flags){(void)flags;post++;memcpy(p,ram_for(p),n);}
#define cache_pre_dma(p,n,d) ((void)SysBase,fake_pre(p,n,d))
#define cache_post_dma(p,n,d) ((void)SysBase,fake_post(p,n,d))
static void *slab_alloc(struct slab_cache *s){(void)s;return linkbuf;}
static void slab_free(struct slab_cache *s,void *p){(void)s;if(p!=linkbuf)abort();linkfree++;}
'''
helpers=r'''
static void bounce_free(struct xhci_ctrl *c,void *b,enum xhci_bounce_pool pool){(void)c;(void)pool;if(b!=bounce)abort();freed++;}
'''
suffix=r'''
static void check(int good,const char *why){checks++;if(!good){failures++;printf("FAIL %s\n",why);}}
static void reset(void){memset(user,0xa5,sizeof user);memset(bounce,0xcc,sizeof bounce);memset(linkbuf,0xdd,sizeof linkbuf);memset(uram,0,sizeof uram);memset(bram,0,sizeof bram);memset(lram,0,sizeof lram);pre=post=freed=linkfree=0;}
static void publish(void){memcpy(uram,user,sizeof user);memcpy(bram,bounce,sizeof bounce);memcpy(lram,linkbuf,sizeof linkbuf);}
static unsigned char pattern(unsigned i){return (unsigned char)((i*37U+19U)%251U);}
static void run_case(int whole,enum xhci_dma_mapping mapping,int out,int linked,unsigned delivered){
 reset();struct ExecBase ex={0};struct xhci_ctrl c={&ex,whole,{0}};
 struct xhci_dma_span p={.buffer=user+64,.length=4096,.mapping=mapping,.to_device=out,.bounce_buf=mapping==XHCI_DMA_DIRECT?0:bounce,.head_len=mapping==XHCI_DMA_SPLIT?13:0,.tail_len=mapping==XHCI_DMA_SPLIT?7:0};
 if(out)for(unsigned i=0;i<p.length;i++)user[64+i]=pattern(i);
 xhci_dma_span_sync(&c,&p);
 void *slot=0;if(linked)slot=xhci_dma_span_link_piece(&c,&p,100,64);
 if(linked){check(slot==linkbuf,"link slot acquired");check(xhci_dma_span_link_piece(&c,&p,200,32)==0,"second link piece refuses without replacement");}
 if(whole){check(pre==0,"coalesced sync defers per-range cache calls");publish();}
 struct xhci_dma_extent ext[3];unsigned n=xhci_dma_span_extents(&p,ext),offset=0;
 for(unsigned j=0;j<n;j++){
  unsigned char *rp=ram_for((void*)ext[j].addr);
  for(unsigned i=0;i<ext[j].length;i++){
   if(out){if(rp[i]!=pattern(offset+i)){check(0,"OUT published payload checksum");goto after;}}
   else if(offset+i<delivered)rp[i]=pattern(offset+i);
  }
  offset+=ext[j].length;
 }
 if(out)check(offset==p.length,"OUT published payload checksum");
 if(linked){unsigned char *lp=ram_for(slot);for(unsigned i=0;i<64;i++){
  if(out){if(lp[i]!=pattern(100+i)){check(0,"OUT published link checksum");goto after;}}
  else if(100+i<delivered)lp[i]=pattern(100+i);
 }
 if(out)check(1,"OUT published link checksum");
 /* Stale original bytes in the replaced link piece must be overwritten by
    the final copy from the link slot, after the base span copy/invalidate. */
 if(!out)for(unsigned i=100;i<164;i++){
  if(mapping==XHCI_DMA_BOUNCE)bram[i]=0xee;
  else uram[64+i]=0xee;
 }
 }
 after:;
 unsigned oldpost=post;xhci_dma_span_unmap(&c,&p,delivered);
 unsigned upto=delivered<4096?delivered:4096;int good=1;
 for(unsigned i=0;i<(out?4096:upto);i++)if(user[64+i]!=pattern(i)){good=0;break;}
 check(good,out?"OUT unmap preserves caller data":"IN unmap copies current delivered payload checksum");
 check(user[63]==0xa5&&user[4160]==0xa5,"caller neighboring sentinel bytes survive");
 check(p.mapping==XHCI_DMA_NONE&&p.bounce_buf==0&&p.link.slot==0,"unmap clears ownership");
 check(freed==(mapping==XHCI_DMA_DIRECT?0U:1U)&&linkfree==(unsigned)linked,"unmap frees owned buffers exactly once");
 if(out||!delivered)check(post==oldpost,"OUT or abandoned IN does not read DMA output");
 unsigned oldfree=freed,oldlink=linkfree;xhci_dma_span_unmap(&c,&p,delivered);check(freed==oldfree&&linkfree==oldlink,"second unmap is idempotent");
}
int main(void){
 for(int whole=0;whole<=1;whole++){
  run_case(whole,XHCI_DMA_DIRECT,1,0,4096);
  run_case(whole,XHCI_DMA_BOUNCE,1,1,4096);
  for(unsigned m=XHCI_DMA_DIRECT;m<=XHCI_DMA_SPLIT;m++){
   run_case(whole,(enum xhci_dma_mapping)m,0,0,4096);
   run_case(whole,(enum xhci_dma_mapping)m,0,0,9);
   run_case(whole,(enum xhci_dma_mapping)m,0,0,0);
   if(m!=XHCI_DMA_SPLIT)run_case(whole,(enum xhci_dma_mapping)m,0,1,4096);
  }
 }
 printf("dma-span checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
results={}
for variant in ['fixed','missing-out-bounce-copy-negative','missing-in-post-invalidate-negative','missing-link-copy-negative']:
 f=funcs.copy()
 if variant=='missing-out-bounce-copy-negative':f[1]=f[1].replace('memcpy(span->bounce_buf, span->buffer, span->length);','(void)span;')
 if variant=='missing-in-post-invalidate-negative':
  f[5]=f[5].replace('cache_post_dma(device_copy, n, 0);','(void)SysBase;')
  f[6]=f[6].replace('cache_post_dma(span->buffer, span->length, 0);','(void)SysBase;').replace('cache_post_dma((UBYTE *)span->buffer + span->head_len, split_middle_len(span), 0);','(void)SysBase;')
 if variant=='missing-link-copy-negative':f[6]=f[6].replace('copy_piece_home(ctrl, span, span->link.slot, span->link.offset, span->link.length, delivered);','(void)span;')
 tail=suffix.replace("int main(void){", "int main(void){ (void)fake_post;")
 text=prefix+types+helpers+''.join(f)+tail;p=B/(variant+'.c');exe=B/variant;p.write_text(text)
 subprocess.run([CC]+CFLAGS+[str(p),'-o',str(exe)],check=True, timeout=60)
 r=subprocess.run([str(exe)],text=True,capture_output=True, timeout=10)
 assert r.returncode==(0 if variant=='fixed' else 1),(variant,r.stdout,r.stderr)
 if variant!='fixed':assert 'payload checksum' in r.stdout,(variant,r.stdout)
 results[variant]={'returncode':r.returncode,'output':r.stdout};print(variant,r.stdout,flush=True)
assert 'dma-span checks=171 failures=0' in results['fixed']['output'], results['fixed']
print('PASS', 'test_dma_span.py')
