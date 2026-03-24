#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
ENTRYPOINT="${SCRIPT_DIR}/scripts/nav_test_workbench.py"

if [[ -f "${INSTALL_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${INSTALL_SETUP}"
fi

exec python3 "${ENTRYPOINT}" "$@"
