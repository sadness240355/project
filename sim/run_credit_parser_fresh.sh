#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
id="${1:?new test ID}";variant="${2:-credit_diag_001}"
case "$id" in *[!a-zA-Z0-9_-]*|'')exit 2;;esac
case "$variant" in credit_mig_001|credit_diag_001);;*)exit 2;;esac
out="$root/sim/results/$id";[[ ! -e "$out" ]] || exit 2;mkdir -p "$out"
rtl="$root/src/rtl/vivado_project/reconic_zdma.srcs/sources_1/fpga"
cp "$root/src/rtl/baselines/${variant}_lrdma_credit_rx.sv" "$out/"
cp "$root/sim/tb_credit_parser_fresh.sv" "$out/tb.sv"
cp "$root/sim/inputs/credit_real_packet002.hex" "$out/packet.hex"
cp "$rtl/rdma_subsystem/lrdma/compat/lrdma_compat_config.sv" "$rtl/box_250mhz/packet_parser.sv" "$rtl/utility/axi_lite_slave.sv" "$rtl/utility/axi_lite_register.sv" "$out/"
cp "$0" "$out/run_used.sh"
cd "$out"
sha256sum ./*.sv packet.hex run_used.sh > inputs.sha256
iverilog -V > tool_version.txt 2>&1
iverilog -g2012 -s tb -o sim.vvp ./*.sv > compile.log 2>&1
set +e
timeout 30 vvp sim.vvp > run.log 2>&1
rc=$?;printf '%s\n' "$rc" > exit_code.txt
set -e
[[ $rc == 0 ]] && grep -q REAL_PACKET_FIXED_PASS run.log
printf 'PARSER_CONFIG_CREDIT_REAL_PACKET_UNIT_PASS_NOT_FABRIC_OR_BOARD_VALIDATION\n' > result.txt
