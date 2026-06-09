#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SMUG_BIN="${SMUG_BIN:-${SCRIPT_DIR}/bin/smug}"
CONFIG="${CONFIG:-${SCRIPT_DIR}/super_mid360.smug.yml}"

if [[ ! -x "${SMUG_BIN}" ]]; then
  echo "ERROR: smug not found at ${SMUG_BIN}" >&2
  exit 1
fi

exec "${SMUG_BIN}" stop -f "${CONFIG}" "$@"
