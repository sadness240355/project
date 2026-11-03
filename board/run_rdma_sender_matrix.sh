#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
campaign="${1:?new campaign ID}";bit="${2:?programmed bitstream SHA256}"
case "$campaign" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
[[ "$bit" =~ ^[0-9a-f]{64}$ ]] || exit 2
out="$root/board/campaigns/${campaign}_sender"
[[ ! -e "$out" ]] || exit 2
mkdir -p "$out"
cp "$0" "$out/run_used.sh"
printf '%s\n' "$bit" > "$out/bitstream.sha256"
printf 'length,batch,run_id\n' > "$out/completed.csv"
for batch in 1 50; do
 for exponent in {7..23}; do
  length=$((1 << exponent))
  id="${campaign}_${length}_b${batch}_tx"
  printf '%s\n' "$id" > "$out/current.txt"
  bash "$root/board/run_rdma_sender.sh" "$id" "${PEER_CONTROL_IP}" 18529 "$length" "$batch" "$bit" ddr 100
  printf '%s,%s,%s\n' "$length" "$batch" "$id" >> "$out/completed.csv"
 done
done
printf 'SENDER_MATRIX_COMPLETED\n' > "$out/completed.txt"
