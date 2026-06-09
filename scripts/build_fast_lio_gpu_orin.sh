#!/usr/bin/env bash
set -euo pipefail

# Build FAST_LIO_GPU with CUDA enabled for Jetson Orin NX / Livox MID360.
# Run from anywhere inside this workspace, or pass WORKSPACE=/path/to/ws.

WORKSPACE="${WORKSPACE:-/home/usrga2rl/super_ws}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
FAST_LIO_SRC="${FAST_LIO_SRC:-${WORKSPACE}/src/FAST_LIO_GPU}"
BUILD_DIR="${BUILD_DIR:-${WORKSPACE}/build/fast_lio}"
INSTALL_PREFIX="${INSTALL_PREFIX:-${WORKSPACE}/install/fast_lio}"
CUDA_ARCH="${CUDA_ARCH:-87}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
CXX_RELEASE_FLAGS="${CXX_RELEASE_FLAGS:--O1}"
JOBS="${JOBS:-1}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found" >&2
  exit 1
fi

if [[ ! -d "${FAST_LIO_SRC}" ]]; then
  echo "ERROR: FAST_LIO_GPU source not found at ${FAST_LIO_SRC}" >&2
  exit 1
fi

if [[ ! -x /usr/local/cuda/bin/nvcc ]]; then
  echo "ERROR: nvcc not found at /usr/local/cuda/bin/nvcc" >&2
  exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source existing workspace overlays so livox_ros_driver2 and generated messages are visible.
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
  source "${WORKSPACE}/install/setup.bash"
fi

export CUDACXX=/usr/local/cuda/bin/nvcc

if [[ -e "${FAST_LIO_SRC}/.git" && -f "${FAST_LIO_SRC}/.gitmodules" ]]; then
  git -C "${FAST_LIO_SRC}" submodule update --init --recursive
fi

cmake -S "${FAST_LIO_SRC}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_CXX_FLAGS_RELEASE="${CXX_RELEASE_FLAGS}" \
  -DFASTLIO_USE_CUDA=ON \
  -DFASTLIO_REQUIRE_LIVOX=ON \
  -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCH}" \
  -DAMENT_CMAKE_SYMLINK_INSTALL=1 \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"

cmake --build "${BUILD_DIR}" --target fastlio_mapping -- -j"${JOBS}"
cmake --install "${BUILD_DIR}"

source "${WORKSPACE}/install/setup.bash"
ros2 pkg executables fast_lio

echo
echo "FAST_LIO_GPU CUDA build complete."
echo "Source this workspace before running: source ${WORKSPACE}/install/setup.bash"
