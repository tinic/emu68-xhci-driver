#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Host model by codex-amiga, adapted to run from this tree (tests/host/run_all.sh).
"""Compile actual cache selectors and recovery functions in both macro modes.

Standalone source-extraction model. Not a full driver build or hardware proof.
"""
from pathlib import Path
import ast
import os
import shutil
import subprocess
import tempfile

X = Path(__file__).resolve().parents[2]
B = Path(tempfile.mkdtemp(prefix='xhci-host-'))
CC = os.environ.get('CC', 'cc')


def read(path):
    return (X / 'xhci.device' / path).read_text()


def function(source, signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'


header = read('include/xhci/xhci.h')
ring = read('src/xhci/xhci-ring.c')
private = read('src/xhci/xhci-ring-priv.h')
default = header[header.index('#ifndef XHCI_COALESCE_CACHE_OPS'):header.index('\nstruct usb_device;')]
selectors = function(header, 'static inline BOOL xhci_cache_whole(') + function(private, 'static inline BOOL ring_defers_cache(')
functions = ''.join([
    function(private, 'static inline int last_trb('),
    function(private, 'static inline BOOL last_trb_on_last_seg('),
    function(ring, 'inline static void inc_deq('),
    function(ring, 'void xhci_ring_consume_event('),
    function(ring, 'u32 xhci_ring_get_deq_ptr_for_trb('),
    function(ring, 'void xhci_ring_patch_trbs_to_noop('),
])
# Reuse only the old model's literal mock definitions, without running it.
tree = ast.parse((Path(__file__).resolve().parent / 'test_ring_recovery.py').read_text())
prefix = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'prefix' for t in n.targets))
prefix = prefix.replace('#define XHCI_COALESCE_CACHE_OPS 1', default)
prefix = prefix.replace('u32 cycle_state;BOOL defer_cache;', 'u32 cycle_state;\n#if XHCI_COALESCE_CACHE_OPS\nBOOL defer_cache;\n#endif\n')
prefix = prefix.replace('struct xhci_ctrl {struct xhci_ring *event_ring;};', 'struct xhci_ctrl {struct xhci_ring *event_ring;\n#if XHCI_COALESCE_CACHE_OPS\nBOOL cache_whole;\n#endif\n};')
flush = 'static void xhci_cache_flush_all(struct ExecBase *s){(void)s;memcpy(ram,cpu,sizeof ram);full++;note(\'F\');}'
assert flush in prefix
prefix = prefix.replace(flush, '#if XHCI_COALESCE_CACHE_OPS\n' + flush + '\n#endif')

suffix = r'''
static void check(int yes,const char *why){checks++;if(!yes){failures++;printf("FAIL %s\n",why);}}
static void reset(void){memset(cpu,0,sizeof cpu);memset(ram,0,sizeof ram);memset(trace,0,sizeof trace);pre=post=full=nt=0;}
static void run(int deferred){
 struct ExecBase exec={0};struct xhci_segment a={cpu,0},b={cpu+2,&a};a.next=&b;
 struct xhci_ring er={.sysBase=&exec,.is_event_ring=TRUE,.first_seg=&a,.deq_seg=&a,.dequeue=cpu,.cycle_state=1};
 struct xhci_ctrl ctrl={.event_ring=&er};struct xhci_ring tx={.sysBase=&exec,.is_event_ring=FALSE,.first_seg=&a,.deq_seg=&a,.dequeue=cpu,.cycle_state=1};
#if XHCI_COALESCE_CACHE_OPS
 ctrl.cache_whole=deferred;tx.defer_cache=deferred;
#endif
 check(XHCI_COALESCE_CACHE_OPS==EXPECTED,"compile default/override selection");
 check(xhci_cache_whole(&ctrl)==deferred&&ring_defers_cache(&tx)==deferred,"controller/ring runtime selection");
 xhci_ring_consume_event(&ctrl);check(er.dequeue==cpu+1&&er.cycle_state==1,"event first step");
 xhci_ring_consume_event(&ctrl);check(er.dequeue==cpu+2&&er.deq_seg==&b&&er.cycle_state==1,"event next segment");
 xhci_ring_consume_event(&ctrl);check(er.dequeue==cpu+3&&er.cycle_state==1,"event final segment");
 xhci_ring_consume_event(&ctrl);check(er.dequeue==cpu&&er.cycle_state==0,"event cycle wrap");
 for(unsigned i=0;i<4;i++)xhci_ring_consume_event(&ctrl);
 check(er.dequeue==cpu&&er.cycle_state==1,"event second lap");
 reset();cpu[1].link.control=TRB_TYPE(TRB_LINK)|LINK_TOGGLE;inc_deq(&tx);
 check(tx.dequeue==cpu+2&&tx.deq_seg==&b&&tx.cycle_state==1,"transfer skips Link");
 reset();cpu[0].generic.field[0]=0xfeed;cpu[0].generic.field[3]=1;
 if(!deferred)memcpy(ram,cpu,sizeof ram); /* legacy producer already published */
 u32 q=xhci_ring_get_deq_ptr_for_trb(&tx,(dma_addr_t)cpu);
 check(q==((u32)(uintptr_t)cpu|1U)&&cpu[0].generic.field[0]==0xfeed,"dequeue data/cycle preserved");
 check(full==(unsigned)deferred&&post==1&&!strcmp(trace,deferred?"FI":"I"),"dequeue cache ordering");
 reset();ram[0].generic.field[3]=1;cpu[0].generic.field[3]=0;
 if(!deferred){q=xhci_ring_get_deq_ptr_for_trb(&tx,(dma_addr_t)cpu);check((q&1U)==1&&post==1&&full==0,"legacy stale CPU cycle invalidated");}
 reset();check(xhci_ring_get_deq_ptr_for_trb(&tx,0)==0&&full==0&&post==0,"null lookup no work");
 reset();cpu[0].generic.field[0]=77;cpu[0].generic.field[3]=1|TRB_CHAIN;
 cpu[1].link.control=TRB_TYPE(TRB_LINK)|TRB_CHAIN|LINK_TOGGLE|1;cpu[2].generic.field[0]=88;
 if(!deferred)memcpy(ram,cpu,sizeof ram);
 dma_addr_t addrs[2]={(dma_addr_t)&cpu[0],(dma_addr_t)&cpu[2]};xhci_ring_patch_trbs_to_noop(&tx,addrs,2,0);
 check(full==(unsigned)deferred&&trace[0]==(deferred?'F':'I'),"NoOp cache ordering");
 check(ram[0].generic.field[0]==0&&ram[0].generic.field[3]==(TRB_TYPE(TRB_TR_NOOP)|1U),"NoOp first cycle");
 check(ram[2].generic.field[3]==TRB_TYPE(TRB_TR_NOOP),"NoOp second cycle");
 check(ram[1].link.control==(TRB_TYPE(TRB_LINK)|LINK_TOGGLE|1U),"Link unchain preserves flags");
 check(pre==3&&post==3,"NoOp publishes both TRBs and Link");
 reset();cpu[0].generic.field[0]=77;cpu[2].generic.field[3]=1;if(!deferred)memcpy(ram,cpu,sizeof ram);
 xhci_ring_patch_trbs_to_noop(&tx,addrs,2,1);
 check(ram[0].generic.field[0]==77&&ram[2].generic.field[3]==(TRB_TYPE(TRB_TR_NOOP)|1U),"start index preserves preceding TRB");
 reset();xhci_ring_patch_trbs_to_noop(&tx,addrs,2,2);xhci_ring_patch_trbs_to_noop(&tx,0,2,0);
 check(full==0&&pre==0&&post==0,"invalid patch no cache work");
}
int main(void){run(0);
#if XHCI_COALESCE_CACHE_OPS
 run(1);
#endif
 printf("config-recovery macro=%d checks=%u failures=%u\n",XHCI_COALESCE_CACHE_OPS,checks,failures);return failures?1:0;
}
'''

matrix = [
    ('range-default', [], 0),
    ('lvo-default', ['-DEMU68_FORCE_LVO_CACHE_OPS=1'], 1),
    ('lvo-disabled', ['-DEMU68_FORCE_LVO_CACHE_OPS=1', '-DXHCI_COALESCE_CACHE_OPS=0'], 0),
    ('explicit-enabled', ['-DXHCI_COALESCE_CACHE_OPS=1'], 1),
]
results = {}
for name, defines, expected in matrix:
    for negative in [False, True]:
        label = name + ('-missing-invalidate-negative' if negative else '')
        body = functions
        if negative:
            old = 'cache_post_dma(trb, sizeof(*trb), 0);'
            assert old in body
            body = body.replace(old, '(void)SysBase; (void)trb;', 1)
        source = prefix + selectors + body + suffix
        c = B / (label + '.c')
        exe = B / label
        c.write_text(source)
        command = [CC, '-std=c11', '-Wall', '-Wextra', '-Werror', '-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-DEXPECTED=' + str(expected), *defines, str(c), '-o', str(exe)]
        subprocess.run(command, check=True)
        result = subprocess.run([str(exe)], text=True, capture_output=True)
        assert result.returncode == int(negative), (label, result.stdout, result.stderr)
        if negative:
            assert 'FAIL legacy stale CPU cycle invalidated' in result.stdout, result.stdout
        else:
            assert 'failures=0' in result.stdout
        results[label] = {'defines': defines, 'returncode': result.returncode, 'output': result.stdout}
        print(label, result.stdout.strip(), flush=True)

shutil.rmtree(B)
print('PASS', 'test_config_recovery.py')
