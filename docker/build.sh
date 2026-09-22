#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec docker build --progress=plain -f "${repo_dir}/docker/Dockerfile" \
  -t "${BEYONDMIMIC_IMAGE:-beyondmimic-sim2sim:jazzy}" "$repo_dir" "$@"
