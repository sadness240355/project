#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
export RECEIVER_BUILD=receiver_runtime_final
bash "$root/board/run_rdma_receiver.sh" colorbar_rdma_60s_round001_rx 18529 14745600
bash "$root/board/run_rdma_receiver.sh" colorbar_rdma_60s_round002_rx 18529 14745600
