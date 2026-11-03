#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new ID}";seconds="${2:?seconds}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/board/results/$id"
[[ ! -e "$out" ]] || exit 2
mkdir -p "$out"
cp "$0" "$out/run_used.sh"
python3 "$root/board/session_preflight.py" --url "${CONTROL_URL}" > "$out/application_idle.json"
ref="$root/data/reference_v1/colorbar_2400x2048_rgb888.bin"
bin="$root/board/rdma_build/colorbar_sender_final/sender"
sha256sum "$ref" "$bin" > "$out/inputs.sha256"
printf 'CPU=2\n' > "$out/cpu_affinity.txt"
set +e
taskset -c 2 "$bin" "${PEER_CONTROL_IP}" 18529 "$seconds" "$ref" --execute > "$out/result.json" 2> "$out/run.stderr"
rc=$?
printf '%s\n' "$rc" > "$out/exit_code.txt"
exit "$rc"
