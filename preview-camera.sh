#!/usr/bin/env bash
# Independent camera environment; never starts or connects the robot application.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
if [[ ! -x .venv-camera/bin/python ]]; then
    echo "Create the camera environment: python3 -m venv .venv-camera && .venv-camera/bin/pip install -r requirements-preview.txt" >&2
    exit 1
fi
# Default is the unpacked SDK supplied for this workstation; override via env
# or --sdk-root. No system-library installation or root permissions required.
export MINDVISION_SDK_ROOT="${MINDVISION_SDK_ROOT:-$HOME/Downloads/new3dvision/library/hopencv/mindvision2/usv-multiSensor-driver-master/linuxSDK_V2.1.0.6}"
exec .venv-camera/bin/python -m pcbvision.preview "$@"
