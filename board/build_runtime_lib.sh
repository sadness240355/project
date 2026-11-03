#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new run ID required}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/board/runtime_build/$id"
[[ ! -e "$out" ]] || { echo refusing_overwrite >&2; exit 2; }
mkdir -p "$out/source" "$out/objects"
cp "$root/src/software/runtime_lib/"* "$out/source/"
"${CC:-cc}" --version > "$out/compiler.txt"
uname -a > "$out/platform.txt"
sha256sum "$out/source/"* > "$out/source.sha256"
python3 "$root/board/configure_sources.py" "$out/source"
for name in auxiliary control_api dma_pool memory_api rdma_api reconic; do
 "${CC:-cc}" -O2 -Wall -Wextra -fPIC -I "$out/source" -c "$out/source/$name.c" -o "$out/objects/$name.o" 2> "$out/$name.compile.log"
done
"${AR:-ar}" rcs "$out/librn_fresh.a" "$out/objects/"*.o
sha256sum "$out/librn_fresh.a" > "$out/binary.sha256"
printf 'FRESH_RUNTIME_LIBRARY_BUILD_PASS_NOT_TRANSPORT_TEST\n' > "$out/result.txt"
