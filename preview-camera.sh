#!/usr/bin/env bash
# Independent camera environment; never starts or connects the robot application.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
if [[ ! -x .venv-camera/bin/python ]]; then
    echo "Create the camera environment: python3 -m venv .venv-camera && .venv-camera/bin/pip install -r requirements-preview.txt" >&2
    exit 1
fi
# Use the same installed libMVSDK.so as the working GTK demo by default.
# Explicit --sdk-root, MINDVISION_SDK_ROOT or MINDVISION_SDK_LIBRARY overrides it.
# The unpacked SDK contains a different library which cannot discover this GE502M.
exec .venv-camera/bin/python -m pcbvision.preview "$@"
