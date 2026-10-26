#!/bin/sh
# Host models of the XHCI_COALESCE_CACHE_OPS paths: the driver functions are
# extracted from the source tree, compiled against cache/RAM mocks, and run
# fixed and with one mutation each (a mutation must fail).  CC defaults to cc.
set -e
cd "$(dirname "$0")"
for t in test_coalesce_order.py test_ring_recovery.py test_dma_span.py test_config_recovery.py test_harness_lifecycle.py; do
	python3 -I "$t"
done
