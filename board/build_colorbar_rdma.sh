#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new build ID}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/board/rdma_build/$id"
[[ ! -e "$out" ]] || exit 2
mkdir -p "$out/source"
cp "$root/board/colorbar_rdma_sender.c" "$root/src/host_driver/edev_sgdma_driver.h" "$out/source/"
cp -r "$root/src/software/runtime_lib" "$out/source/"
cp "$root/board/runtime_build/${RUNTIME_BUILD:-runtime_final}/librn_fresh.a" "$out/"
python3 "$root/board/configure_sources.py" "$out/source"
"${CC:-cc}" -O2 -Wall -Wextra -I "$out/source" -I "$out/source/runtime_lib" "$out/source/colorbar_rdma_sender.c" "$out/librn_fresh.a" -o "$out/sender" 2> "$out/compile.log"
"${CC:-cc}" --version > "$out/compiler.txt"
find "$out/source" -type f -exec sha256sum {} + > "$out/source.sha256"
sha256sum "$out/sender" "$out/librn_fresh.a" > "$out/binary.sha256"
printf 'BUILD_PASS_NO_TRANSPORT_RESULT\n' > "$out/result.txt"
