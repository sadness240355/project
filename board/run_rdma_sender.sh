#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new run ID}"; ip="${2:?receiver control IPv4}"; port="${3:?port}"; length="${4:?length}"; batch="${5:?batch}"; scope="${6:?development or programmed bitstream SHA256}"; source_mode="${7:?host or ddr}"; repeats="${8:?repeats}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
if [[ "$scope" != development && ! "$scope" =~ ^[0-9a-f]{64}$ ]]; then exit 2; fi
out="$root/board/results/$id"
[[ ! -e "$out" ]] || { echo refusing_overwrite >&2; exit 2; }
mkdir -p "$out"
cp "$0" "$out/run_used.sh"
printf '%s\n' "$scope" > "$out/bitstream_scope.txt"
python3 "$root/board/session_preflight.py" --url "${CONTROL_URL:-http://10.23.68.0:8788}" > "$out/application_idle.json"
bin="$root/board/rdma_build/${SENDER_BUILD:-sender_runtime_final}/sender"
sha256sum "$bin" > "$out/binary.sha256"
printf '%s\n' "$bin $ip $port $length $batch $source_mode $repeats --execute" > "$out/command.txt"
printf 'CPU=%s\n' "${SENDER_CPU:-2}" > "$out/cpu_affinity.txt"
set +e
taskset -c "${SENDER_CPU:-2}" "$bin" "$ip" "$port" "$length" "$batch" "$source_mode" "$repeats" --execute > "$out/sender.csv" 2> "$out/sender.stderr"
rc=$?
printf '%s\n' "$rc" > "$out/exit_code.txt"
exit "$rc"
