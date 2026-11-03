#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new ID}";case "$id" in *[!a-zA-Z0-9_-]*|'')exit 2;;esac
out="$root/board/application_build/$id";[[ ! -e "$out" ]] || exit 2;mkdir -p "$out/source"
python3 "$root/build/snapshot_software.py" "$out/source"
app="$out/source/application/software"
python3 "$root/board/configure_sources.py" "$out/source"
"${CC:-cc}" -O2 -Wall -Wextra -I "$app/capture_reference" "$app/capture_reference/hik_camera_cfg.c" -lm -o "$out/configure" 2> "$out/compile.log"
"${CC:-cc}" --version > "$out/compiler.txt";find "$out/source" -type f -exec sha256sum {} + > "$out/source.sha256";sha256sum "$out/configure" > "$out/binary.sha256"
printf 'FRESH_CONFIG_TOOL_BUILD_PASS_NOT_CAMERA_RESULT\n' > "$out/result.txt"
