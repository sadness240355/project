#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
config=${CAMERA_CFG_BIN:-$here/build/hik_camera_cfg}
viewer=${CAMERA_VIEWER_BIN:-$here/build/camera_viewer_bayerrg8_egl}
[[ -x "$config" && -x "$viewer" ]] || { echo 'Run video.sh build-sender first' >&2; exit 2; }
width=1920; height=1080; fps=120; pattern=0; exposure=7500; skip=0
ctrl=${DMA_DEVICE:-/dev/edev0}; args=()
while (($#)); do
 case "$1" in
  --help|-h) exec "$viewer" --help;;
  --size) width=${2%x*}; height=${2#*x}; args+=("$1" "$2"); shift 2;;
  --fps) fps=$2; args+=("$1" "$2"); shift 2;;
  --ctrl) ctrl=$2; args+=("$1" "$2"); shift 2;;
  --test-pattern)
   case "$2" in off|none) pattern=0;; colorbar|color) pattern=12;; mono) pattern=11;; checker) pattern=13;; *) pattern=$2;; esac
   shift 2;;
  --exposure-us) exposure=$2; shift 2;;
  --skip-config) skip=1; shift;;
  *) args+=("$1"); shift;;
 esac
done
[[ $width =~ ^[0-9]+$ && $height =~ ^[0-9]+$ && $exposure =~ ^[0-9]+$ ]] || exit 2
if [[ $skip == 0 ]]; then
 "$config" --ctrl "$ctrl" --cmd 'w AcquisitionStop 1' --configure-bayer8 --width "$width" --height "$height" --fps "$fps" --test-pattern "$pattern" --cmd "w ExposureTime $exposure" --readback
fi
exec "$viewer" --ctrl "$ctrl" --device "${DMA_DEVICE:-/dev/edev0}_q64" "${args[@]}"
