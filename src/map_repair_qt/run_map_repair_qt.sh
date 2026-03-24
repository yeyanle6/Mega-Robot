#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
ENTRYPOINT="${SCRIPT_DIR}/scripts/map_repair_qt.py"

if [[ -f "${INSTALL_SETUP}" ]]; then
  # Prefer the installed environment when the workspace has been built.
  # shellcheck disable=SC1090
  source "${INSTALL_SETUP}"
fi

exec python3 "${ENTRYPOINT}" "$@"
