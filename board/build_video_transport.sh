#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
role="${1:?sender or receiver}";id="${2:?new build ID}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
[[ "$role" == sender || "$role" == receiver ]] || exit 2
out="$root/board/application_build/$id"
[[ ! -e "$out" ]] || exit 2
mkdir -p "$out/source"
python3 "$root/build/snapshot_software.py" "$out/source"
app="$out/source/application/software"
if [[ "$role" == sender ]]; then
 cp "$root/src/host_driver/edev_sgdma_driver.h" "$app/capture_reference/"
 cp "$root/board/runtime_build/${RUNTIME_BUILD:-runtime_final}/librn_fresh.a" "$out/"
python3 "$root/board/configure_sources.py" "$out/source"
 "${CC:-cc}" -O2 -Wall -Wextra -pthread -I "$app/src" -I "$app/lib" "$app/src/ernic_video_sender.c" "$app/src/video_crc.c" "$out/librn_fresh.a" -ldl -lz -lm -lrt -o "$out/sender" 2> "$out/compile.log"
else
 "${CC:-cc}" -O2 -Wall -Wextra -pthread -I "$app/src" "$app/src/mlx5_video_receiver.c" "$app/src/video_crc.c" -libverbs -ldl -lz -lm -lrt -o "$out/receiver" 2> "$out/compile.log"
fi
"${CC:-cc}" --version > "$out/compiler.txt"
find "$out/source" -type f -exec sha256sum {} + > "$out/source.sha256"
sha256sum "$out/$role" > "$out/binary.sha256"
printf 'PRODUCTION_DUT_SOURCE_FRESH_BUILD_PASS_NOT_RUN_RESULT\n' > "$out/result.txt"
