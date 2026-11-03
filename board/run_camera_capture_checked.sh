#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new run ID}";seconds="${2:?seconds}"; ring="${3:-}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
exec 9>"$root/board/camera_capture.lock"
flock -n 9 || { echo "another campaign capture still owns the device" >&2; exit 3; }
out="$root/board/results/$id"
[[ ! -e "$out" ]] || exit 2
mkdir -p "$out"
cp "$0" "$out/run_used.sh"
python3 "$root/board/session_preflight.py" --url "${CONTROL_URL}" > "$out/application_idle.json"
bin="$root/board/application_build/${CAPTURE_BUILD:-capture_final}/capture"
sha256sum "$bin" > "$out/binary.sha256"
set +e
args=("$seconds" "$out/first_frame.ppm" "$out/frames.csv")
[[ -z "$ring" ]] || args+=("$ring")
taskset -c 2 "$bin" "${args[@]}" > "$out/result.json" 2> "$out/run.stderr"
rc=$?
printf '%s\n' "$rc" > "$out/exit_code.txt"
exit "$rc"
