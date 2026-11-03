#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
bash "$root/board/run_colorbar_rdma.sh" colorbar_rdma_60s_round001_tx 60
bash "$root/board/run_colorbar_rdma.sh" colorbar_rdma_60s_round002_tx 60
