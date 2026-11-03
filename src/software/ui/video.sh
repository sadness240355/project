#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$HERE"
source "$HERE/../../../board/env.sh"
mkdir -p "$HERE/.tmp"
export TMPDIR="$HERE/.tmp"
mode=${1:-station}
[[ $# == 0 ]] || shift
case "$mode" in
 build-sender) exec bash "$HERE/../../../build/build_ui.sh" sender ;;
 build-receiver) exec bash "$HERE/../../../build/build_ui.sh" receiver ;;
 capture)
   export CAMERA_HOST_PATH=direct CAMERA_YOLO_INPUT_PATH=preview
   if [[ -n ${CAMERA_DDR_RING:-} ]]; then
     export CAMERA_ANALYSIS_SHM_FPS=${CAMERA_ANALYSIS_SHM_FPS:-10}
   fi
   export VIDEO_DISPLAY_FIFO
   VIDEO_DISPLAY_FIFO=$(python3 "$HERE/video_display_memory.py")
   ISP_MASK=$(python3 "$HERE/video_isp.py")
   export CAMERA_CFG_BIN="$HERE/build/hik_camera_cfg" CAMERA_VIEWER_BIN="$HERE/build/camera_viewer_bayerrg8_egl"
   export DISPLAY=${DISPLAY:-:0} CAMERA_DMA_CUDA_PINNED=${CAMERA_DMA_CUDA_PINNED:-1}
   exec bash "${CAMERA_CAPTURE_SCRIPT:-$HERE/capture_reference/bayerrg8_app.sh}" --size 1920x1080 --fps 120 --duration 0 --buffers 8 --swap-interval 0 --no-gl-finish --fifo-display --no-display --test-pattern off --shm-name /fpga_video_input --isp-enable "$ISP_MASK" --black-level 0 --wb-r 1.01 --wb-g 1.04 --wb-b 0.95 --ccm '1.2157,-0.0329,-0.1828,-0.1984,1.3814,-0.1830,-0.0048,-0.3104,1.3151' --exposure-us 7500 --debug-csr "$@"
   ;;
 sender)
   exec "$HERE/ernic_video_sender" --resource2 "${FPGA_RESOURCE}" --dma-dev "${DMA_DEVICE}" --netdev "${FPGA_NETDEV}" --src-ip "${FPGA_IP}" --dst-ip "${PEER_IP}" --dst-mac "${PEER_MAC}" --control-ip "${PEER_CONTROL_IP}" "$@"
   ;;
 receiver)
   export VIDEO_DISPLAY_FIFO
   VIDEO_DISPLAY_FIFO=$(python3 "$HERE/video_display_memory.py")
   gid_index=$(python3 "$HERE/rdma_gid.py" --device "${RDMA_DEVICE}" --port 1 --netdev "${PEER_NETDEV}" --ipv4 "${PEER_IP}")
   exec "$HERE/mlx5_video_receiver" --output-file ./runtime/display_memory/latest_frames.bin --device "${RDMA_DEVICE}" --ib-port 1 --gid-index "$gid_index" --netdev "${PEER_NETDEV}" --listen-ip "${PEER_IP}" --control-ip "${PEER_CONTROL_IP}" "$@"
   ;;
 yolo)
   if [[ $# -gt 1 ]]; then
     printf '%s\n' 'Usage: ./video.sh yolo [start|stop]' >&2; exit 2
   fi
   case "${1:-start}" in
     start) enabled=1 ;;
     stop) enabled=0 ;;
     *) printf '%s\n' 'Usage: ./video.sh yolo [start|stop]' >&2; exit 2 ;;
   esac
   runner=(python3)
   if [[ -S "$HERE/runtime/control.sock" && ! -w "$HERE/runtime/control.sock" && $EUID -ne 0 ]]; then
     runner=(sudo python3)
   fi
   exec "${runner[@]}" - "$enabled" <<'PY_YOLO_CONTROL'
import sys
from video_console import control
try:
    result=control({'op':'yolo_toggle','enabled':sys.argv[1]=='1'})
    print(result.get('message','操作完成'))
except (OSError,RuntimeError,ValueError) as exc:
    print('YOLO 控制失败：'+str(exc),file=sys.stderr)
    print('请先执行 ./video.sh station，并启动 RGB 视频采集。',file=sys.stderr)
    sys.exit(1)
PY_YOLO_CONTROL
   ;;
 yolo-deploy)
   exec python3 "$HERE/video_yolo_deploy.py" --ensure
   ;;
 station)
   if ! python3 "$HERE/video_yolo_deploy.py" --ensure; then
     printf '%s\n' 'YOLO 模型尚未部署；视频工作站继续启动，检测开关会显示所需模型路径。' >&2
   fi
   if ! python3 - <<'PY_CONTROL_READY'
from video_console import control
try:
 result=control({'op':'status'})
 raise SystemExit(0 if result.get('ok') else 1)
except (OSError,RuntimeError,ValueError):
 raise SystemExit(1)
PY_CONTROL_READY
   then
     sudo python3 "$HERE/video_control.py" --background
   fi
   python3 "$HERE/video_web.py" --background
   exec python3 "$HERE/video_desktop.py"
   ;;
 desktop) exec python3 "$HERE/video_desktop.py" "$@" ;;
 browser) exec python3 "$HERE/open_console.py" ;;
 control) exec python3 "$HERE/video_control.py" "$@" ;;
 web) exec python3 "$HERE/video_web.py" "$@" ;;
 viewer)
   if [[ $# == 0 || ${1:-} == --self-test ]]; then exec python3 "$HERE/video_desktop.py" "$@"; fi
   exec python3 "$HERE/video_viewer.py" "$@" ;;
 *) printf '%s\n' 'Usage: ./video.sh {build-sender|build-receiver|capture|sender|receiver|yolo|yolo-deploy|viewer|web|control|station|desktop|browser} [options]';;
esac
