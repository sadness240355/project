#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new ID}";seconds="${2:?seconds}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/board/results/$id"
[[ ! -e "$out" ]] || exit 2
mkdir -p "$out/source"
python3 "$root/board/session_preflight.py" --url "${CONTROL_URL}" > "$out/application_idle.json"
cp "$root/board/colorbar_dma.c" "$root/src/host_driver/edev_sgdma_driver.h" "$out/source/"
python3 "$root/board/configure_sources.py" "$out/source"
"${CC:-cc}" -O2 -Wall -Wextra -I "$out/source" "$out/source/colorbar_dma.c" -o "$out/colorbar_dma" 2> "$out/compile.log"
ref="$root/data/reference_v1/colorbar_2400x2048_rgb888.bin"
sha256sum "$ref" "$out/source/"* "$out/colorbar_dma" > "$out/inputs.sha256"
printf 'CPU=2\n' > "$out/cpu_affinity.txt"
cp "$0" "$out/run_used.sh"
set +e
taskset -c 2 "$out/colorbar_dma" "$ref" "$seconds" "$out/final_readback.bin" > "$out/result.json" 2> "$out/run.stderr"
rc=$?
printf '%s\n' "$rc" > "$out/exit_code.txt"
set -e
[[ $rc == 0 ]] || exit "$rc"
cmp "$ref" "$out/final_readback.bin"
printf 'STATIC_COLORBAR_DMA_FULL_READBACK_VERIFIED\n' > "$out/verified.txt"
