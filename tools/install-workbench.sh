#!/usr/bin/env bash
# Idempotent per-user setup; preserves existing tool configuration/calibration.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
if [[ ! -x .venv-camera/bin/python ]]; then
    /usr/bin/python3 -m venv .venv-camera
    .venv-camera/bin/python -m pip install -r requirements-preview.txt
fi
.venv-camera/bin/python -m launcher --check-config
.venv-camera/bin/python tools/install_workbench_desktop.py
