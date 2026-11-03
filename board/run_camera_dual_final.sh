#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd -P)
source "$root/board/env.sh"
id="${1:?new session ID}";seconds="${2:?seconds}"
case "$id" in *[!a-zA-Z0-9_-]*|'')exit 2;;esac
out="$root/board/results/${id}_orchestration";ring="$root/board/results/$id.ring"
[[ ! -e "$out" && ! -e "$ring" && ! -e "$root/board/results/${id}_owner" && ! -e "$root/board/results/${id}_tx" ]] || exit 2
mkdir -p "$out";cp "$0" "$out/run_used.sh"
python3 "$root/board/credit_board_idle.py" > "$out/idle_before.json"
bash "$root/board/run_camera_capture_checked.sh" "${id}_owner" "$seconds" "$ring" > "$out/owner_wrapper.stdout" 2> "$out/owner_wrapper.stderr" &
owner=$!;printf '%s\n' "$owner" > "$out/owner_wrapper.pid"
ready=0
for ((i=0;i<200;i++));do
 if [[ -f "$root/board/results/${id}_owner/run.stderr" ]] && grep -q 'DDR_RING ready=1' "$root/board/results/${id}_owner/run.stderr";then ready=1;break;fi
 if ! kill -0 "$owner";then break;fi
 sleep .05
done
[[ $ready == 1 ]] || { echo owner_ring_not_ready >&2;exit 3; }
printf 'OWNER_RING_INITIALIZED_BEFORE_SENDER_START\n' > "$out/startup_order.txt"
bash "$root/board/run_camera_video_sender_final.sh" "${id}_tx" "$ring" > "$out/sender_wrapper.stdout" 2> "$out/sender_wrapper.stderr" &
sender=$!;printf '%s\n' "$sender" > "$out/sender_wrapper.pid"
set +e
wait "$owner";owner_rc=$?;printf '%s\n' "$owner_rc" > "$out/owner_exit_code.txt"
wait "$sender";sender_rc=$?;printf '%s\n' "$sender_rc" > "$out/sender_exit_code.txt"
set -e
[[ $owner_rc == 0 && $sender_rc == 0 ]]
python3 "$root/board/credit_board_idle.py" > "$out/idle_after.json"
printf 'OWNER_AND_SENDER_EXITED_ZERO_CHECK_REMOTE_CRC_LOG\n' > "$out/result.txt"
