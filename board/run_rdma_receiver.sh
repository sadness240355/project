#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new run ID}"; port="${2:?port}"; capacity="${3:?registered bytes}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/board/results/$id"
[[ ! -e "$out" ]] || { echo refusing_overwrite >&2; exit 2; }
mkdir -p "$out"
cp "$0" "$out/run_used.sh"
bin="$root/board/rdma_build/${RECEIVER_BUILD:-receiver_runtime_final}/receiver"
sha256sum "$bin" > "$out/binary.sha256"
printf '%s\n' "$bin $port $capacity" > "$out/command.txt"
printf '%s\n' "$$" > "$out/runner.pid"
printf 'CPU=%s\n' "${RECEIVER_CPU:-2}" > "$out/cpu_affinity.txt"
set +e
taskset -c "${RECEIVER_CPU:-2}" "$bin" "$port" "$capacity" > "$out/receiver.log" 2> "$out/receiver.stderr"
rc=$?
printf '%s\n' "$rc" > "$out/exit_code.txt"
exit "$rc"
