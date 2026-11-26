#!/usr/bin/env bash
# Always run source in this checkout; no copy/build step after editing Python.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
if [[ ! -x .venv-camera/bin/python ]]; then
    echo "Run tools/install-workbench.sh once to install the launcher environment." >&2
    exit 1
fi
exec .venv-camera/bin/python -m launcher "$@"
