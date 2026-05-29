#!/usr/bin/env bash
set -euo pipefail

VERSION="4.4.0"
JOBS="${JOBS:-$(sysctl -n hw.ncpu)}"
DOWNLOAD_ONLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      VERSION="$2"
      shift 2
      ;;
    --jobs)
      JOBS="$2"
      shift 2
      ;;
    --download-only)
      DOWNLOAD_ONLY=1
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERSION_ROOT="${REPO_ROOT}/Thirdparty/opencv/${VERSION}"
DOWNLOAD_DIR="${VERSION_ROOT}/downloads"
SOURCE_PARENT="${VERSION_ROOT}/src"
SOURCE_DIR="${SOURCE_PARENT}/opencv-${VERSION}"
BUILD_DIR="${VERSION_ROOT}/build"
INSTALL_DIR="${VERSION_ROOT}/install"
ARCHIVE_PATH="${DOWNLOAD_DIR}/opencv-${VERSION}.tar.gz"
ARCHIVE_URL="https://github.com/opencv/opencv/archive/refs/tags/${VERSION}.tar.gz"

mkdir -p "${DOWNLOAD_DIR}" "${SOURCE_PARENT}"

if [[ ! -f "${ARCHIVE_PATH}" ]]; then
  curl -fL "${ARCHIVE_URL}" -o "${ARCHIVE_PATH}"
fi

rm -rf "${SOURCE_DIR}"
tar -xzf "${ARCHIVE_PATH}" -C "${SOURCE_PARENT}"

if [[ "${DOWNLOAD_ONLY}" -eq 1 ]]; then
  echo "OpenCV sources are available under ${SOURCE_DIR}"
  exit 0
fi

if ! command -v cmake >/dev/null 2>&1; then
  echo "cmake is required to build OpenCV." >&2
  exit 1
fi

cmake -S "${SOURCE_DIR}" \
  -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
  -DBUILD_LIST=calib3d,core,features2d,flann,highgui,imgcodecs,imgproc \
  -DBUILD_opencv_apps=OFF \
  -DBUILD_opencv_dnn=OFF \
  -DBUILD_opencv_gapi=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_JAVA=OFF \
  -DBUILD_opencv_java=OFF \
  -DBUILD_opencv_python2=OFF \
  -DBUILD_opencv_python3=OFF \
  -DOPENCV_PYTHON_SKIP_DETECTION=ON \
  -DOPENCV_SKIP_PYTHON_WARNING=ON \
  -DWITH_FFMPEG=OFF \
  -DWITH_IPP=OFF \
  -DWITH_WEBP=OFF \
  -DBUILD_WEBP=OFF \
  -DBUILD_PROTOBUF=OFF

cmake --build "${BUILD_DIR}" --parallel "${JOBS}"
cmake --install "${BUILD_DIR}"

echo "OpenCV installed under ${INSTALL_DIR}"
echo "Source the activation script before configuring ORB_SLAM3:"
echo "  source ./scripts/activate_opencv_macos.sh"
