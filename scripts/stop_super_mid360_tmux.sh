#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SMUG_BIN="${SMUG_BIN:-${SCRIPT_DIR}/bin/smug}"
CONFIG="${CONFIG:-${SCRIPT_DIR}/super_mid360.smug.yml}"
WORKSPACE="${WORKSPACE:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

if [[ ! -x "${SMUG_BIN}" ]]; then
  echo "ERROR: smug not found at ${SMUG_BIN}" >&2
  exit 1
fi

if [[ ! -f "${CONFIG}" ]]; then
  echo "ERROR: config not found at ${CONFIG}" >&2
  exit 1
fi

RENDERED_CONFIG="$(mktemp)"
trap 'rm -f "${RENDERED_CONFIG}"' EXIT

sed -e "s#/home/usrga2rl/super_ws#${WORKSPACE}#g" \
    -e "s#/home/jetson/super_ws#${WORKSPACE}#g" \
  -e "s#__WORKSPACE__#${WORKSPACE}#g" \
    "${CONFIG}" > "${RENDERED_CONFIG}"

exec "${SMUG_BIN}" stop -f "${RENDERED_CONFIG}" "$@"
