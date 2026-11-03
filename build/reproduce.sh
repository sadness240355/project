#!/usr/bin/env bash
# No synthesis, implementation, programming, driver loading, or board execution.
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
mode=${1:-check}; id=${2:-repro_$(date +%Y%m%d_%H%M%S)_$$}
case "$id" in *[!a-zA-Z0-9_-]*|'') echo invalid_ID >&2; exit 2;; esac
case "$mode" in
 check) exec python3 "$root/build/check_project.py";;
 sim)
  for name in run run_credit_fresh run_credit_parser_fresh run_credit_bridge_fresh; do
   bash "$root/sim/$name.sh" "${id}_$name"
  done;;
 hls) exec bash "$root/sim/run_isp_csim.sh" "$id";;
 software)
  export RUNTIME_BUILD=${id}_runtime
  bash "$root/board/build_runtime_lib.sh" "$RUNTIME_BUILD"
  bash "$root/board/build_rdma_bench.sh" sender "${id}_sender"
  bash "$root/board/build_rdma_bench.sh" receiver "${id}_receiver"
  bash "$root/board/build_colorbar_rdma.sh" "${id}_colorbar"
  bash "$root/board/build_camera_config_fresh.sh" "${id}_config"
  bash "$root/board/build_camera_capture.sh" "${id}_capture"
  bash "$root/board/build_video_transport.sh" receiver "${id}_video_receiver"
  bash "$root/board/build_video_sender_final.sh" "${id}_video_sender"
  printf 'RUNTIME_BUILD=%s\nSENDER_BUILD=%s\nRECEIVER_BUILD=%s\nCAPTURE_BUILD=%s\nCONFIG_BUILD=%s\nVIDEO_SENDER_BUILD=%s\nVIDEO_RECEIVER_BUILD=%s\n' "$RUNTIME_BUILD" "${id}_sender" "${id}_receiver" "${id}_capture" "${id}_config" "${id}_video_sender" "${id}_video_receiver";;
 vivado)
  out="$root/build/$id"
  [[ ! -e "$out" ]] || { echo 'Use a new ID' >&2; exit 2; }
  mkdir -p "$out"
  cd "$out"
  exec "${VIVADO:-vivado}" -mode batch -nojournal -log vivado.log -source "$root/build/rebuild_clean.tcl" -tclargs "$out" create "${JOBS:-2}";;
 reference) exec python3 "$root/data/generate_reference.py" "${id}_reference";;
 driver) exec bash "$root/board/build_driver.sh" "$id";;
 ui-sender|ui-receiver) exec bash "$root/build/build_ui.sh" "${mode#ui-}";;
 *) echo 'Usage: bash build/reproduce.sh {check|sim|hls|software|vivado|reference|driver|ui-sender|ui-receiver} [new_ID]' >&2; exit 2;;
esac
printf 'REPRO_STAGE_PASS mode=%s id=%s\n' "$mode" "$id"
