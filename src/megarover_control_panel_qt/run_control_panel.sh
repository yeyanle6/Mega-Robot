#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
ENTRYPOINT="${SCRIPT_DIR}/scripts/control_panel.py"

if [[ -f "${INSTALL_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${INSTALL_SETUP}"
fi

exec python3 "${ENTRYPOINT}" "$@"
