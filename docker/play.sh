#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 /absolute/path/to/policy.onnx [ROS launch arguments...]" >&2
  exit 2
fi
policy_file="$(realpath -- "$1")"
shift
[[ -f "$policy_file" ]] || { echo "Policy not found: $policy_file" >&2; exit 2; }
: "${DISPLAY:?Run from a terminal in your graphical desktop session.}"
auth_file="${XAUTHORITY:-${HOME}/.Xauthority}"
[[ -f "$auth_file" ]] || { echo "Xauthority file not found: $auth_file" >&2; exit 2; }
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
terminal_args=(-i)
if [[ -t 0 && -t 1 ]]; then terminal_args+=(-t); fi

# Older NVIDIA container toolkits omit a library required by the 570 driver.
# Mount only the matching host library, without changing the host runtime.
driver_mounts=()
driver_version="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n 1)"
gpucomp_lib="/usr/lib/x86_64-linux-gnu/libnvidia-gpucomp.so.${driver_version}"
if [[ -f "$gpucomp_lib" ]]; then
  driver_mounts+=(--mount "type=bind,src=${gpucomp_lib},dst=${gpucomp_lib},readonly")
fi

exec docker run --rm --init "${terminal_args[@]}" \
  --name "${BEYONDMIMIC_CONTAINER:-beyondmimic-sim2sim}" \
  --hostname "$(hostname)" \
  --user "$(id -u):$(id -g)" --shm-size=512m \
  --gpus "device=${BEYONDMIMIC_GPU:-1}" \
  "${driver_mounts[@]}" \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility \
  -e DISPLAY -e XAUTHORITY=/tmp/host.xauthority -e HOME=/tmp/beyondmimic \
  -e ROS_DOMAIN_ID=73 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
  --mount "type=bind,src=${auth_file},dst=/tmp/host.xauthority,readonly" \
  --mount "type=bind,src=${repo_dir},dst=/ws/src/motion_tracking_controller,readonly" \
  --mount "type=bind,src=${policy_file},dst=/policies/policy.onnx,readonly" \
  "${BEYONDMIMIC_IMAGE:-beyondmimic-sim2sim:jazzy}" \
  ros2 launch motion_tracking_controller mujoco.launch.py \
  policy_path:=/policies/policy.onnx enable_teleop:=false "$@"
