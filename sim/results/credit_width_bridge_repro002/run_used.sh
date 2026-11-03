#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
id="${1:?new test ID}"
case "$id" in *[!a-zA-Z0-9_-]*|'')exit 2;;esac
out="$root/sim/results/$id";[[ ! -e "$out" ]] || exit 2
mkdir -p "$out/rtl" "$out/baseline" "$out/fixed" "$out/boundary"
rtl="$root/source/design/reconic_zdma.srcs/sources_1/fpga"
cp "$root/sim/tb_credit_bridge_fresh.sv" "$out/rtl/"
cp "$root/build/candidates/credit_width_001/lrdma_credit_rx.sv" "$rtl/rdma_subsystem/lrdma/compat/lrdma_compat_config.sv" "$rtl/axi_width_converter/512_to_256_converter.v" "$rtl/dma_subsystem/dma_rtl/edev_axi_mm_pcie_bridge.v" "$out/rtl/"
cp "$rtl/rdma_subsystem/rdma_subsystem.sv" "$out/boundary/baseline.sv"
cp "$root/build/candidates/credit_width_001/rdma_subsystem.sv" "$out/boundary/fixed.sv"
cp "$0" "$out/run_used.sh"
python3 - "$out" <<'PY'
import sys
from pathlib import Path
p=Path(sys.argv[1])
for name,n in [('baseline',3),('fixed',6)]:
 s=(p/'boundary'/f'{name}.sv').read_text()
 assert f"assign m_axi_rdma_rsp_payload_awsize=3'd{n};" in s
 assert "assign m_axi_rdma_rsp_payload_wstrb=64'hff;" in s
 assert "assign m_axi_rdma_rsp_payload_awlen='0;" in s
PY
for tag in baseline fixed;do
 cp "$root/sim/inputs/credit_real_packet002.hex" "$out/$tag/packet.hex"
 if [[ "$tag" == baseline ]];then size=3;else size=6;fi
 (
 cd "$out/$tag"
 iverilog -V > tool_version.txt 2>&1
 iverilog -g2012 -s tb -Ptb.AW_SIZE="$size" -o sim.vvp ../rtl/*sv ../rtl/*.v > compile.log 2>&1
 set +e
 timeout 30 vvp sim.vvp > run.log 2>&1
 rc=$?;printf '%s\n' "$rc" > exit_code.txt
 set -e
 [[ $rc == 0 ]]
 )
done
grep -q BASELINE_BRIDGE_STALL_REPRODUCED "$out/baseline/run.log"
grep -q CREDIT_WIDTH_BRIDGE_PASS "$out/fixed/run.log"
(cd "$out" && sha256sum rtl/* boundary/* baseline/packet.hex fixed/packet.hex run_used.sh > inputs.sha256)
printf 'REAL_RTL_BASELINE_STALL_AND_MASKED_8_BYTE_FIX_VERIFIED_NOT_BOARD_RESULT\n' > "$out/result.txt"
