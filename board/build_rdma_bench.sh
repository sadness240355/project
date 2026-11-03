#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
role="${1:?sender or receiver}"
id="${2:?new build ID}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
case "$role" in sender) name=rdma_write_sender;; receiver) name=rdma_verify_receiver;; *) exit 2;; esac
out="$root/board/rdma_build/$id"
[[ ! -e "$out" ]] || { echo refusing_overwrite >&2; exit 2; }
mkdir -p "$out/source"
cp "$root/board/$name.c" "$out/source/"
"${CC:-cc}" --version > "$out/compiler.txt"
uname -a > "$out/platform.txt"
if [[ "$role" == sender ]]; then
 lib="$root/board/runtime_build/${RUNTIME_BUILD:-runtime_final}/librn_fresh.a"
 cp -r "$root/src/software/runtime_lib" "$out/source/"
 cp "$lib" "$out/"
 cp "$root/src/host_driver/edev_sgdma_driver.h" "$out/source/"
 printf '%s\n' 'cc -O2 -Wall -Wextra -I source/runtime_lib -I source source/rdma_write_sender.c librn_fresh.a -o sender' > "$out/command.txt"
python3 "$root/board/configure_sources.py" "$out/source"
 "${CC:-cc}" -O2 -Wall -Wextra -I "$out/source/runtime_lib" -I "$out/source" "$out/source/$name.c" "$out/librn_fresh.a" -o "$out/sender" 2> "$out/compile.log"
else
 printf '%s\n' 'cc -O2 -Wall -Wextra source/rdma_verify_receiver.c -libverbs -o receiver' > "$out/command.txt"
 "${CC:-cc}" -O2 -Wall -Wextra "$out/source/$name.c" -libverbs -o "$out/receiver" 2> "$out/compile.log"
fi
find "$out/source" -type f -exec sha256sum {} + > "$out/source.sha256"
sha256sum "$out/$role" > "$out/binary.sha256"
printf 'BUILD_PASS_NO_TRANSPORT_RESULT\n' > "$out/result.txt"
