#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
role=${1:?sender or receiver}; ui="$root/src/software/ui"
cc=${CC:-cc}; flags=(-O2 -g -std=gnu11 -Wall -Wextra)
mkdir -p "$ui/build"
"$cc" "${flags[@]}" -fPIC -shared "$ui/src/video_crc.c" -lz -o "$ui/libvideo_crc.so"
"$cc" "${flags[@]}" -fPIC -shared "$ui/src/video_jpeg.c" -ljpeg -o "$ui/libvideo_jpeg.so"
"$cc" "${flags[@]}" -fPIC -shared "$ui/src/video_gpu.c" -lEGL -lGLESv2 -lX11 -lXext -lm -pthread -o "$ui/libvideo_gpu.so"
"$cc" "${flags[@]}" -I "$ui/src" "$ui/src/video_pattern_source.c" "$ui/src/video_crc.c" -lz -ldl -lrt -pthread -o "$ui/video_pattern_source"
case "$role" in
 base) ;;
 sender)
   id="ui_runtime_$(date +%Y%m%d_%H%M%S)_$$"
   bash "$root/board/build_runtime_lib.sh" "$id"
   RUNTIME_BUILD="$id" bash "$root/board/build_video_sender_final.sh" "${id}_video"
   cp "$root/board/application_build/${id}_video/sender" "$ui/ernic_video_sender"
   "$cc" "${flags[@]}" -I "$ui/capture_reference" "$ui/capture_reference/camera_viewer_bayerrg8_egl.c" "$ui/capture_reference/edev_camera_io.c" "$ui/src/video_crc.c" -lz -lEGL -lGLESv2 -lX11 -lm -ldl -lrt -pthread -o "$ui/build/camera_viewer_bayerrg8_egl"
   "$cc" "${flags[@]}" -I "$ui/capture_reference" "$ui/capture_reference/hik_camera_cfg.c" "$ui/capture_reference/edev_camera_io.c" "$ui/src/video_crc.c" -lz -lm -ldl -lrt -pthread -o "$ui/build/hik_camera_cfg"
   ;;
 receiver)
   "$cc" "${flags[@]}" "$ui/src/mlx5_video_receiver.c" "$ui/src/video_crc.c" -libverbs -lz -ldl -lrt -pthread -o "$ui/mlx5_video_receiver"
   ;;
 *) echo 'role must be sender or receiver' >&2; exit 2;;
esac
