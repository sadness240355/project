#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
campaign="${1:?new campaign ID}"
case "$campaign" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/board/campaigns/${campaign}_receiver"
[[ ! -e "$out" ]] || exit 2
mkdir -p "$out"
cp "$0" "$out/run_used.sh"
printf 'length,batch,run_id\n' > "$out/completed.csv"
for batch in 1 50; do
 for exponent in {7..23}; do
  length=$((1 << exponent))
  id="${campaign}_${length}_b${batch}_rx"
  printf '%s\n' "$id" > "$out/current.txt"
  bash "$root/board/run_rdma_receiver.sh" "$id" 18529 "$((length*batch))"
  printf '%s,%s,%s\n' "$length" "$batch" "$id" >> "$out/completed.csv"
 done
done
printf 'RECEIVER_MATRIX_COMPLETED\n' > "$out/completed.txt"
