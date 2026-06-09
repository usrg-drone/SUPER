#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SMUG_BIN="${SMUG_BIN:-${SCRIPT_DIR}/bin/smug}"
CONFIG="${CONFIG:-${SCRIPT_DIR}/super_mid360.smug.yml}"
WORKSPACE="${WORKSPACE:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
ENABLE_RVIZ="${ENABLE_RVIZ:-}"
RVIZ_CONFIG="${RVIZ_CONFIG:-}"

if [[ ! -x "${SMUG_BIN}" ]]; then
  echo "ERROR: smug not found at ${SMUG_BIN}" >&2
  exit 1
fi

if [[ ! -f "${CONFIG}" ]]; then
  echo "ERROR: config not found at ${CONFIG}" >&2
  exit 1
fi

if [[ ! -d "${WORKSPACE}" ]]; then
  echo "ERROR: workspace not found at ${WORKSPACE}" >&2
  exit 1
fi

# If caller did not pass RViz options, inherit defaults from YAML env.
if [[ -z "${ENABLE_RVIZ}" ]]; then
  ENABLE_RVIZ="$(sed -n 's/^  ENABLE_RVIZ: *"\{0,1\}\([^"# ]*\)"\{0,1\}.*/\1/p' "${CONFIG}" | head -n1)"
fi
if [[ -z "${ENABLE_RVIZ}" ]]; then
  ENABLE_RVIZ="0"
fi

if [[ -z "${RVIZ_CONFIG}" ]]; then
  RVIZ_CONFIG="$(sed -n 's/^  RVIZ_CONFIG: *"\{0,1\}\([^"#]*\)"\{0,1\}.*/\1/p' "${CONFIG}" | head -n1)"
fi
if [[ -z "${RVIZ_CONFIG}" ]]; then
  RVIZ_CONFIG="${WORKSPACE}/src/super_planner/rviz/default.rviz"
fi

RENDERED_CONFIG="$(mktemp)"
trap 'rm -f "${RENDERED_CONFIG}"' EXIT

RVIZ_CONFIG_ESCAPED="${RVIZ_CONFIG//\\/\\\\}"
RVIZ_CONFIG_ESCAPED="${RVIZ_CONFIG_ESCAPED//&/\\&}"

# Keep the checked-in config generic by replacing known hardcoded workspace roots.
sed -e "s#/home/usrga2rl/super_ws#${WORKSPACE}#g" \
    -e "s#/home/jetson/super_ws#${WORKSPACE}#g" \
    -e "s#__WORKSPACE__#${WORKSPACE}#g" \
    -e "s#__ENABLE_RVIZ__#${ENABLE_RVIZ}#g" \
    -e "s#__RVIZ_CONFIG__#${RVIZ_CONFIG_ESCAPED}#g" \
    "${CONFIG}" > "${RENDERED_CONFIG}"

exec "${SMUG_BIN}" start -f "${RENDERED_CONFIG}" "$@"
