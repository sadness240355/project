#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new build ID required}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/board/driver_build/$id"
if [[ -e "$out" ]]; then echo refusing_overwrite >&2; exit 2; fi
mkdir -p "$out/work"
cp "$root/src/host_driver/"*.c "$root/src/host_driver/"*.h "$out/work/"
printf 'obj-m += reconic_edev.o\nreconic_edev-y := edev_sgdma_driver.o reconic_netdev.o\n' > "$out/work/Makefile"
uname -a > "$out/environment.txt"
"${CC:-cc}" --version >> "$out/environment.txt"
sha256sum "$out/work/"*.c "$out/work/"*.h > "$out/source.sha256"
set +e
make -C "${KERNEL_BUILD:-/lib/modules/$(uname -r)/build}" M="$out/work" -j2 modules > "$out/build.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$out/exit_code.txt"
if [[ "$rc" = 0 ]]; then sha256sum "$out/work/reconic_edev.ko" > "$out/module.sha256"; modinfo "$out/work/reconic_edev.ko" > "$out/module_info.txt"; fi
exit "$rc"
