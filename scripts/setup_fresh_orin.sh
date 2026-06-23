#!/usr/bin/env bash
set -euo pipefail

# Fresh-clone setup for this SUPER + FAST_LIO_GPU + Livox MID360 workspace on Jetson Orin.
# Assumes this repository is checked out at: ${WORKSPACE}/src
# Override defaults, for example:
#   WORKSPACE=/home/jetson/super_ws ROS_DISTRO=humble ./scripts/setup_fresh_orin.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "${SRC_DIR}/.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
CUDA_ARCH="${CUDA_ARCH:-87}"
JOBS="${JOBS:-1}"
USE_ZENOH="${USE_ZENOH:-1}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "ERROR: required command not found: $1" >&2
    return 1
  fi
}

info() {
  printf '\n==> %s\n' "$*"
}

source_safe() {
  # ROS setup scripts may reference unset vars; source them with nounset disabled.
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

info "Checking prerequisites"
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || {
  echo "ERROR: ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
}
[[ -x /usr/local/cuda/bin/nvcc ]] || {
  echo "ERROR: CUDA nvcc not found at /usr/local/cuda/bin/nvcc" >&2
  exit 1
}
require_cmd git
require_cmd cmake
require_cmd colcon
require_cmd tmux

info "Workspace"
echo "WORKSPACE=${WORKSPACE}"
echo "SRC_DIR=${SRC_DIR}"
echo "ROS_DISTRO=${ROS_DISTRO}"
echo "CUDA_ARCH=${CUDA_ARCH}"
echo "JOBS=${JOBS}"
echo "USE_ZENOH=${USE_ZENOH}"

cd "${SRC_DIR}"

info "Initializing git submodules"
if [[ -e FAST_LIO_GPU/.git && -f FAST_LIO_GPU/.gitmodules ]]; then
  git -C FAST_LIO_GPU submodule update --init --recursive
fi

info "Building Livox-SDK2 locally"
[[ -d Livox-SDK2 ]] || {
  echo "ERROR: Livox-SDK2 directory is missing. Clone/copy it into ${SRC_DIR}/Livox-SDK2." >&2
  exit 1
}
cmake -S "${SRC_DIR}/Livox-SDK2" -B "${SRC_DIR}/Livox-SDK2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${SRC_DIR}/Livox-SDK2/build" -- -j"${JOBS}"

info "Preparing livox_ros_driver2 for ROS 2"
[[ -d livox_ros_driver2 ]] || {
  echo "ERROR: livox_ros_driver2 directory is missing." >&2
  exit 1
}
cp -f livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml
rm -rf livox_ros_driver2/launch
cp -a livox_ros_driver2/launch_ROS2 livox_ros_driver2/launch

info "Patching livox_ros_driver2 SDK path for this workspace"
perl -0pi -e "s#/home/[^\n\" ]*/super_ws/src/Livox-SDK2#${SRC_DIR}/Livox-SDK2#g; s#/tmp/Livox-SDK2#${SRC_DIR}/Livox-SDK2#g" livox_ros_driver2/CMakeLists.txt

if [[ "${USE_ZENOH}" == "1" ]]; then
  info "Checking rmw_zenoh_cpp (USE_ZENOH=1)"
  _zenoh_pkg="ros-${ROS_DISTRO}-rmw-zenoh-cpp"
  if dpkg -s "${_zenoh_pkg}" >/dev/null 2>&1; then
    echo "${_zenoh_pkg} already installed"
  else
    echo "Installing ${_zenoh_pkg} ..."
    sudo apt install -y "${_zenoh_pkg}"
  fi
  unset _zenoh_pkg
else
  echo "Skipping rmw_zenoh_cpp install (USE_ZENOH=0)"
fi

info "Sourcing ROS"
source_safe "/opt/ros/${ROS_DISTRO}/setup.bash"
export CUDACXX=/usr/local/cuda/bin/nvcc

info "Building core ROS packages"
cd "${WORKSPACE}"
colcon build --symlink-install \
  --packages-select mars_quadrotor_msgs rog_map super_planner livox_ros_driver2 \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS="${ROS_DISTRO}"

info "Building FAST_LIO_GPU with CUDA enabled"
source_safe "${WORKSPACE}/install/setup.bash"
CUDA_ARCH="${CUDA_ARCH}" JOBS="${JOBS}" WORKSPACE="${WORKSPACE}" "${SCRIPT_DIR}/build_fast_lio_gpu_orin.sh"

info "Verifying installed executables"
source_safe "/opt/ros/${ROS_DISTRO}/setup.bash"
source_safe "${WORKSPACE}/install/setup.bash"
# fast_lio is installed via cmake, not colcon, so it is not chained by the workspace
# setup.bash. Source its own local_setup.bash so ros2 pkg can find it.
source_safe "${WORKSPACE}/install/fast_lio/share/fast_lio/local_setup.bash"
ros2 pkg executables livox_ros_driver2
ros2 pkg executables fast_lio
ros2 pkg executables super_planner

info "Installing smug system-wide"
sudo install -m 755 "${SCRIPT_DIR}/bin/smug" /usr/local/bin/smug
echo "smug installed to /usr/local/bin/smug"

info "Installing smug bash completion"
sudo install -m 644 "${SCRIPT_DIR}/smug_completion.bash" /etc/bash_completion.d/smug
echo "completion installed to /etc/bash_completion.d/smug"

info "Rendering smug config for 'smug start superplanner'"
mkdir -p "${HOME}/.config/smug"
_rviz_config="${SRC_DIR}/super_planner/rviz/mid360_goal_path.rviz"
sed \
  -e "s#/home/usrga2rl/super_ws#${WORKSPACE}#g" \
  -e "s#/home/jetson/super_ws#${WORKSPACE}#g" \
  -e "s#/home/usrg/workspace/super_ws#${WORKSPACE}#g" \
  -e "s#__WORKSPACE__#${WORKSPACE}#g" \
  -e "s#__ENABLE_RVIZ__#1#g" \
  -e "s#__RVIZ_CONFIG__#${_rviz_config}#g" \
  "${SCRIPT_DIR}/super_mid360.smug.yml" \
  > "${HOME}/.config/smug/superplanner.yml"
unset _rviz_config
echo "config written to ${HOME}/.config/smug/superplanner.yml"
echo "run: smug start superplanner"

_zenoh_note=""
if [[ "${USE_ZENOH}" == "1" ]]; then
  _zenoh_note="
     export RMW_IMPLEMENTATION=rmw_zenoh_cpp"
fi

cat <<MSG

Fresh Orin setup complete.

Before running:
  1. Edit MID360 network settings if needed:
     ${SRC_DIR}/livox_ros_driver2/config/MID360_config.json
  2. Source the workspace:
     source ${WORKSPACE}/install/setup.bash
     source ${WORKSPACE}/install/fast_lio/share/fast_lio/local_setup.bash${_zenoh_note}
  3. Start the tmux stack:
     smug start superplanner
     (or: ${SCRIPT_DIR}/start_super_mid360_tmux.sh)

MSG
unset _zenoh_note
