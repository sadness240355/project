#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new run ID}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/board/results/$id"
[[ ! -e "$out" ]] || exit 2
mkdir -p "$out"
cp "$0" "$out/run_used.sh"
printf 'CPU=2\n' > "$out/cpu_affinity.txt"
ring="${2:?new DDR ownership file}"
[[ -f "$ring" ]] || exit 2 # Owner has completed and logged DDR_RING initialization.
bin="$root/board/application_build/${VIDEO_SENDER_BUILD:-video_sender_final}/sender"
sha256sum "$bin" > "$out/binary.sha256"
set +e
VIDEO_CRC_MODE=full taskset -c 2 "$bin" --resource2 "${FPGA_RESOURCE}" --dma-dev "${DMA_DEVICE}" --ddr-ring "$ring" --netdev "${FPGA_NETDEV}" --src-ip "${FPGA_IP}" --dst-ip "${PEER_IP}" --dst-mac "${PEER_MAC}" --control-ip "${PEER_CONTROL_IP}" --tcp-port 18531 --frames 0 --idle-timeout 30 > "$out/run.stdout" 2> "$out/run.stderr"
rc=$?
printf '%s\n' "$rc" > "$out/exit_code.txt"
exit "$rc"
