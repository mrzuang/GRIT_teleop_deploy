#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${G1_BRIDGE_BUILD_DIR:-${ROOT_DIR}/build}"
BIN="${BUILD_DIR}/g1_udp_bridge"
CONFIG="${G1_BRIDGE_CONFIG:-${ROOT_DIR}/config/g1_bridge.yaml}"
NET="${G1_NET:-lo}"
LOCK_FILE="${G1_BRIDGE_LOCK_FILE:-/tmp/g1_udp_bridge.lock}"

case "$(uname -m)" in
  x86_64)
    SDK_ARCH="x86_64"
    ;;
  aarch64|arm64)
    SDK_ARCH="aarch64"
    ;;
  *)
    echo "Unsupported architecture: $(uname -m)" >&2
    exit 1
    ;;
esac

SDK_LIB_DIR="${ROOT_DIR}/third_party/unitree_sdk2/thirdparty/lib/${SDK_ARCH}"

if [[ ! -x "${BIN}" ]]; then
  echo "Bridge binary not found: ${BIN}" >&2
  echo "Run: bash ${ROOT_DIR}/scripts/build.sh" >&2
  exit 1
fi

if [[ ! -d "${SDK_LIB_DIR}" ]]; then
  echo "Unitree SDK library directory not found: ${SDK_LIB_DIR}" >&2
  exit 1
fi

# A second bridge can split UDP commands and initialize DDS twice on one robot.
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "Another G1 bridge is already running (lock: ${LOCK_FILE})" >&2
  exit 1
fi
if pgrep -x g1_udp_bridge >/dev/null; then
  echo "Another g1_udp_bridge process is already running:" >&2
  pgrep -af g1_udp_bridge >&2
  exit 1
fi

echo "[run_bridge] binary=${BIN}"
echo "[run_bridge] config=${CONFIG}"
echo "[run_bridge] network=${NET}"
echo "[run_bridge] sdk_lib=${SDK_LIB_DIR}"

# Keep the native bridge independent of Conda, ROS, CUDA, and XR service libs.
unset LD_PRELOAD
export LD_LIBRARY_PATH="${SDK_LIB_DIR}"

exec "${BIN}" --net "${NET}" --config "${CONFIG}" "$@"
