#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
base="${1:?reserved DDR base required}"
run_id="${2:?new run ID required}"
case "$run_id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/board/results/$run_id"
if [[ -e "$out" ]]; then echo refusing_overwrite >&2; exit 2; fi
mkdir -p "$out"
python3 "$root/board/session_preflight.py" --url "${CONTROL_URL:-http://127.0.0.1:8788}" > "$out/session_idle.json"
python3 "$root/board/preflight.py" > "$out/preflight.json"
mkdir -p "$out/source"
cp "$root/board/dma_matrix.c" "$out/source/dma_matrix.c"
cp "$root/src/host_driver/edev_sgdma_driver.h" "$out/source/edev_sgdma_driver.h"
python3 "$root/board/configure_sources.py" "$out/source"
"${CC:-cc}" -O2 -Wall -Wextra -pthread "$out/source/dma_matrix.c" -o "$out/dma_matrix" 2> "$out/compile.log"
"${CC:-cc}" --version > "$out/compiler.txt"
sha256sum "$root/board/dma_matrix.c" "$root/src/host_driver/edev_sgdma_driver.h" "$out/dma_matrix" > "$out/inputs.sha256"
"$out/dma_matrix" --selftest > "$out/pattern_1KiB.bin"
cmp "$out/pattern_1KiB.bin" "$root/data/reference_v1/dma_1KiB.bin"
for dir in h2c c2h; do
 for exponent in 12 13 14 15 16 17 18 19 20; do
  bytes=$((1 << exponent))
  for queues in 1 2 3 4; do
   stem="$out/${dir}_${bytes}_q${queues}"
   printf '%s\n' "$out/dma_matrix $base $bytes $queues $dir 100 --execute" > "$stem.command.txt"
   "$out/dma_matrix" "$base" "$bytes" "$queues" "$dir" 100 --execute > "$stem.csv" 2> "$stem.stderr"
  done
 done
done
python3 "$root/data/validate_dma_csv.py" "$out"/*.csv > "$out/csv_validation.json"
printf 'MATRIX_PROCESS_COMPLETED\n' > "$out/completed.txt"
