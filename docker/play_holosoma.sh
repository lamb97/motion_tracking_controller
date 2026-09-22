#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /path/to/adapted_holosoma.onnx" >&2
  exit 2
fi
policy="$(realpath -- "$1")"
pose_file="${policy%.onnx}.initial_pose.json"
[[ -f "$pose_file" ]] || { echo "Missing initial pose: $pose_file; run scripts/convert_holosoma_onnx.py first." >&2; exit 2; }
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${repo_dir}/docker/play.sh" "$policy" "initial_joint_positions:=$(cat -- "$pose_file")"
