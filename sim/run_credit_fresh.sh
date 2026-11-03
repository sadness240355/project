#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
id="${1:?new case ID}";case "$id" in *[!a-zA-Z0-9_-]*|'')exit 2;;esac
out="$root/sim/results/$id"
python3 "$root/sim/gen_credit_packet.py" "$out"
rtl="$root/src/rtl/vivado_project/reconic_zdma.srcs/sources_1/fpga/rdma_subsystem/lrdma/compat"
cp "$root/sim/gen_credit_packet.py" "$root/sim/tb_credit_config_fresh.sv" "$0" "$out/"
sha256sum "$rtl/lrdma_credit_rx.sv" "$rtl/lrdma_compat_config.sv" "$root/sim/tb_credit_config_fresh.sv" "$root/sim/gen_credit_packet.py" > "$out/inputs.sha256"
cd "$out"
iverilog -V > tool_version.txt 2>&1
iverilog -g2012 -s tb -o sim.vvp "$rtl/lrdma_credit_rx.sv" "$rtl/lrdma_compat_config.sv" "$root/sim/tb_credit_config_fresh.sv" > compile.log 2>&1
set +e
timeout 30 vvp sim.vvp > run.log 2>&1
rc=$?;printf '%s\n' "$rc" > exit_code.txt
set -e
[[ $rc == 0 ]] && grep -q 'CREDIT_UNIT_PASS cases=2' run.log
printf 'SUPPLEMENTARY_SYNTHETIC_PACKET_CONFIG_AND_CREDIT_UNIT_PASS_NOT_BOARD_RESULT\n' > result.txt
