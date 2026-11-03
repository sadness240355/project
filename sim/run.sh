#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
case_id="${1:-unit_retransmit_001}"
case "$case_id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/sim/results/$case_id"
if [[ -e "$out" ]]; then echo "refusing to overwrite $out" >&2; exit 2; fi
mkdir -p "$out"
cd "$out"
iverilog -V > tool_version.txt 2>&1
sha256sum "$root/src/rtl/lrdma_qp_retransmit_ctrl.v" "$root/src/rtl/lrdma_defs.vh" "$root/sim/tb_retransmit_ctrl.sv" > inputs.sha256
iverilog -g2012 -I "$root/src/rtl" -s tb -o sim.vvp "$root/src/rtl/lrdma_qp_retransmit_ctrl.v" "$root/sim/tb_retransmit_ctrl.sv" > compile.log 2>&1
timeout 30 vvp sim.vvp > run.log 2>&1
grep -q 'UNIT_PASS cases=4' run.log
