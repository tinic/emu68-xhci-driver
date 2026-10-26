#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Model subprocess failures are bounded and leave no model temp directory."""
from pathlib import Path
import os
import signal
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
MODELS = ['test_coalesce_order.py', 'test_ring_recovery.py',
          'test_dma_span.py', 'test_config_recovery.py', 'test_cache_policy.py']


def run_model(command, env, timeout):
    # An outer deadline must also stop any compiled binary spawned by the
    # Python model. Killing only Python leaves that binary running.
    with subprocess.Popen(command, env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True,
                          start_new_session=True) as process:
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5)
            raise
        return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)

with tempfile.TemporaryDirectory(prefix='xhci-lifecycle-') as outer:
    outer = Path(outer)
    model_tmp = outer / 'model-tmp'
    model_tmp.mkdir()
    env = os.environ.copy()
    env['TMPDIR'] = str(model_tmp)
    broken_cc = outer / 'broken-cc'
    broken_cc.write_text('#!/bin/sh\nexit 42\n')
    broken_cc.chmod(0o755)
    for model in MODELS:
        failed_env = dict(env, CC=str(broken_cc))
        result = run_model([sys.executable, '-I', str(ROOT / 'tests/host' / model)],
                           env=failed_env, timeout=20)
        assert result.returncode != 0 and 'CalledProcessError' in result.stderr, (model, result)
        assert not list(model_tmp.iterdir()), (model, 'temporary directory leaked')
        print('PASS compiler failure cleanup', model)

    # Copy only the model and its three source inputs; never mutate the checkout.
    copy = outer / 'mutated-source'
    for path in ['tests/host/test_coalesce_order.py',
                 'xhci.device/src/xhci/xhci-ring.c',
                 'xhci.device/src/xhci/xhci-submit.c',
                 'xhci.device/src/xhci/xhci-events.c']:
        destination = copy / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / path, destination)
    ring = copy / 'xhci.device/src/xhci/xhci-ring.c'
    source = ring.read_text()
    start = source.index('union xhci_trb *xhci_ring_get_event_trb_cached(')
    before, function = source[:start], source[start:]
    condition = 'if ((le32(event->event_cmd.flags) & TRB_CYCLE) != ring->cycle_state)'
    assert function.count(condition) == 1
    ring.write_text(before + function.replace(condition, 'if (0)', 1))
    result = run_model([sys.executable, '-I', str(copy / 'tests/host/test_coalesce_order.py')],
                       env=env, timeout=25)
    assert result.returncode != 0 and 'TimeoutExpired' in result.stderr, result
    assert not list(model_tmp.iterdir()), 'timed-out model temporary directory leaked'
    print('PASS infinite-drain timeout and cleanup')

    producer = outer / 'mutated-producer'
    for path in ['tests/host/test_cache_policy.py',
                 'xhci.device/include/xhci/xhci.h',
                 'xhci.device/src/xhci/xhci.c',
                 'xhci.device/src/xhci/xhci-ring.c',
                 'xhci.device/src/xhci/xhci-ring-priv.h',
                 'xhci.device/src/xhci/xhci-submit.c']:
        destination = producer / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / path, destination)
    private = producer / 'xhci.device/src/xhci/xhci-ring-priv.h'
    source = private.read_text()
    condition = 'while (last_trb(ring, ring->enq_seg, next))'
    assert source.count(condition) == 1
    private.write_text(source.replace(condition, 'while (TRUE)', 1))
    result = run_model([sys.executable, '-I', str(producer / 'tests/host/test_cache_policy.py')],
                       env=env, timeout=25)
    assert result.returncode != 0 and 'TimeoutExpired' in result.stderr, result
    assert not list(model_tmp.iterdir()), 'timed-out producer temporary directory leaked'
    print('PASS infinite-producer timeout and cleanup')

    # A second communicate() timeout used to be mistaken for success here.
    # Keep a PID witness, check the original deadline, and verify that the
    # grandchild stopped. A zombie is stopped, even before init reaps it.
    witness = outer / 'sleeper-pids'
    sleeper = (
        "import os,pathlib,subprocess,sys,time; "
        "child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); "
        "pathlib.Path(sys.argv[1]).write_text(str(os.getpid())+' '+str(child.pid)); "
        "time.sleep(60)"
    )
    deadline = 2
    try:
        try:
            run_model([sys.executable, '-c', sleeper, str(witness)],
                      env=env, timeout=deadline)
        except subprocess.TimeoutExpired as error:
            assert error.timeout == deadline, ('cleanup timed out', error)
        else:
            raise AssertionError('outer timeout did not fire')
        assert witness.exists(), 'sleeper never started its grandchild'
        group, grandchild = map(int, witness.read_text().split())
        for attempt in range(20):
            state = subprocess.run(['ps', '-o', 'stat=', '-p', str(grandchild)],
                                   capture_output=True, text=True, timeout=2)
            assert state.returncode in (0, 1), state
            if not state.stdout.strip() or state.stdout.lstrip().startswith('Z'):
                break
            time.sleep(0.05)
        else:
            raise AssertionError(('grandchild survived outer timeout', grandchild,
                                  state.stdout))
        print('PASS outer timeout stops witnessed model process group')
    finally:
        # A negative control must not leak the very process it detects.
        if witness.exists():
            group, grandchild = map(int, witness.read_text().split())
            try:
                os.killpg(group, signal.SIGKILL)
            except ProcessLookupError:
                pass

print('PASS test_harness_lifecycle.py')
