#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "$1" >&2
  exit 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INSTALL_DIR="${REPO_ROOT}/Thirdparty/opencv/4.4.0/install"

if [[ ! -d "${INSTALL_DIR}" ]]; then
  fail "OpenCV is not installed under ${INSTALL_DIR}. Run scripts/install_opencv_macos.sh first."
fi

CONFIG_FILE="$(find "${INSTALL_DIR}" -name OpenCVConfig.cmake -print -quit)"
if [[ -z "${CONFIG_FILE}" ]]; then
  fail "OpenCVConfig.cmake was not found under ${INSTALL_DIR}."
fi

export ORB_SLAM3_OPENCV_ROOT="${INSTALL_DIR}"
export OpenCV_DIR="$(cd "$(dirname "${CONFIG_FILE}")" && pwd)"
export CMAKE_PREFIX_PATH="${INSTALL_DIR}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"

LIB_DIR="$(find "${INSTALL_DIR}" -type d -name lib -print -quit)"
if [[ -n "${LIB_DIR}" ]]; then
  export DYLD_LIBRARY_PATH="${LIB_DIR}${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"
fi

echo "OpenCV_DIR=${OpenCV_DIR}"
