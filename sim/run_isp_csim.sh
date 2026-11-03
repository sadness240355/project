#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
id="${1:?new run ID required}"
case "$id" in *[!a-zA-Z0-9_-]*|'') exit 2;; esac
out="$root/sim/results/$id"
[[ ! -e "$out" ]] || { echo refusing_overwrite >&2; exit 2; }
# Vitis HLS 2022.2 rejects spaces in project/solution paths. Stage the tiny C-sim
# inputs in an isolated system temporary directory and copy evidence back.
work=$(mktemp -d "${HLS_WORK_ROOT:-/tmp}/lrdma_csim.XXXXXXXX")
case "$work" in *[[:space:]]*) rmdir "$work"; echo 'HLS_WORK_ROOT must not contain whitespace' >&2; exit 2;; esac
trap 'rm -rf -- "$work"' EXIT
mkdir -p "$out/source" "$work/src/hls" "$work/sim" "$work/results"
cp "$root/src/hls/"* "$work/src/hls/"
cp "$root/sim/tb_isp_fresh.cpp" "$root/sim/isp_csim.tcl" "$work/sim/"
cp "$root/src/hls/"* "$root/sim/tb_isp_fresh.cpp" "$root/sim/isp_csim.tcl" "$out/source/"
(cd "$out/source" && sha256sum ./*) > "$out/inputs.sha256"
export ISP_CSIM_OUT="$work/results"
cd "$work"
set +e
"${VITIS_HLS:-vitis_hls}" -f "$work/sim/isp_csim.tcl" > "$out/console.log" 2>&1
rc=$?
set -e
cp -a "$work/results/." "$out/"
printf '%s\n' "$rc" > "$out/exit_code.txt"
[[ $rc == 0 ]] || exit "$rc"
grep -q 'ISP_CSIM_PASS,cases=78' "$out/console.log"
printf 'ISP_CSIM_PASS cases=78 C_MODEL_ONLY\n' > "$out/result.txt"
