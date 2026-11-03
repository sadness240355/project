#!/usr/bin/env bash
set -euo pipefail
mode=${1:-help}; [[ $# == 0 ]] || shift
case "$mode" in
 capture|sender|jetson-web) host=${JETSON_SSH:?Set JETSON_SSH=user@host}; dir=${JETSON_PROJECT:-project}/src/software/ui;;
 receiver|viewer|web) host=${RECEIVER_SSH:?Set RECEIVER_SSH=user@host}; dir=${RECEIVER_PROJECT:-project}/src/software/ui;;
 *) echo 'Usage: remote_video.sh {capture|sender|receiver|viewer|web|jetson-web} [options]'; exit 0;;
esac
[[ "$mode" != jetson-web ]] || mode=web
privilege=(); case "$mode" in capture|sender|receiver) privilege=(sudo --preserve-env);; esac
printf -v command '%q ' "${privilege[@]}" bash "$dir/video.sh" "$mode" "$@"
exec ssh -t "$host" "$command"
