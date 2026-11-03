#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new build ID}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/board/application_build/$id"
[[ ! -e "$out" ]] || exit 2
mkdir -p "$out/source"
python3 "$root/build/snapshot_software.py" "$out/source"
cp "$root/board/camera_capture_fresh.c" "$root/src/host_driver/edev_sgdma_driver.h" "$out/source/"
app="$out/source/application/software"
python3 "$root/board/configure_sources.py" "$out/source"
"${CC:-cc}" -O2 -Wall -Wextra -pthread -I "$out/source" -I "$app/capture_reference" -I "$app/src" -I "$app/lib" "$out/source/camera_capture_fresh.c" "$app/capture_reference/edev_camera_io.c" "$app/src/video_crc.c" -ldl -lz -lm -o "$out/capture" 2> "$out/compile.log"
"${CC:-cc}" --version > "$out/compiler.txt"
find "$out/source" -type f -exec sha256sum {} + > "$out/source.sha256"
sha256sum "$out/capture" > "$out/binary.sha256"
printf 'BUILD_PASS_NOT_CAPTURE_RESULT\n' > "$out/result.txt"
