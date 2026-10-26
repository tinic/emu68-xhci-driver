#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Model subprocess failures are bounded and leave no model temp directory."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
MODELS = ['test_coalesce_order.py', 'test_ring_recovery.py',
          'test_dma_span.py', 'test_config_recovery.py']

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
        result = subprocess.run([sys.executable, '-I', str(ROOT / 'tests/host' / model)],
                                env=failed_env, capture_output=True, text=True, timeout=20)
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
    result = subprocess.run([sys.executable, '-I', str(copy / 'tests/host/test_coalesce_order.py')],
                            env=env, capture_output=True, text=True, timeout=25)
    assert result.returncode != 0 and 'TimeoutExpired' in result.stderr, result
    assert not list(model_tmp.iterdir()), 'timed-out model temporary directory leaked'
    print('PASS infinite-drain timeout and cleanup')

print('PASS test_harness_lifecycle.py')
