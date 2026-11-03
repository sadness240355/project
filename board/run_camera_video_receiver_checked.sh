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
if ss -H -ltn 'sport = :18531' | grep -q .;then
 printf 'receiver_control_port_already_listening\n' > "$out/run.stderr"
 printf '3\n' > "$out/exit_code.txt";exit 3
fi
bin="$root/board/application_build/${VIDEO_RECEIVER_BUILD:-video_receiver_final}/receiver"
sha256sum "$bin" > "$out/binary.sha256"
cd "$out"
set +e
VIDEO_CRC_MODE=full taskset -c 2 "$bin" --boxes-channel --boxes-file ./received_boxes.bin --device "${RDMA_DEVICE}" --ib-port 1 --gid-index "${RDMA_GID_INDEX}" --listen-ip "${PEER_IP}" --netdev "${PEER_NETDEV}" --control-ip "${PEER_CONTROL_IP}" --tcp-port 18531 --max-size 16777216 --output-file ./received_frames.bin --save-last "$out/last_frame.ppm" > "$out/run.stdout" 2> "$out/run.stderr"
rc=$?
printf '%s\n' "$rc" > "$out/exit_code.txt"
exit "$rc"
