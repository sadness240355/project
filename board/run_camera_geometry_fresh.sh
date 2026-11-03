#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new ID}";case "$id" in *[!a-zA-Z0-9_-]*|'')exit 2;;esac
out="$root/board/results/$id";[[ ! -e "$out" ]] || exit 2;mkdir -p "$out"
cp "$0" "$out/run_used.sh"
python3 "$root/board/credit_board_idle.py" > "$out/idle_before.json"
bin="$root/board/application_build/${CONFIG_BUILD:-camera_config_final}/configure";sha256sum "$bin" > "$out/binary.sha256"
set +e
"$bin" --fpga --cmd "w AcquisitionStop 1" --configure-bayer8 --width 2400 --height 2048 --fps 140 --readback > "$out/run.stdout" 2> "$out/run.stderr"
rc=$?;printf '%s\n' "$rc" > "$out/exit_code.txt"
exit "$rc"
