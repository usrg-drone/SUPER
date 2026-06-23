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
source_safe "${WORKSPACE}/install/fast_lio/local_setup.bash"
ros2 pkg executables livox_ros_driver2
ros2 pkg executables fast_lio
ros2 pkg executables super_planner

cat <<MSG

Fresh Orin setup complete.

Before running:
  1. Edit MID360 network settings if needed:
     ${SRC_DIR}/livox_ros_driver2/config/MID360_config.json
  2. Source the workspace:
     source ${WORKSPACE}/install/setup.bash
     source ${WORKSPACE}/install/fast_lio/local_setup.bash
  3. Start the tmux stack:
     ${SCRIPT_DIR}/start_super_mid360_tmux.sh

MSG
