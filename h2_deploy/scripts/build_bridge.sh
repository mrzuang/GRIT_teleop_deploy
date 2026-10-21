#!/usr/bin/env bash
# Build native_bridge/build/h2_udp_bridge (unitree_sdk2, yaml-cpp and zlib are vendored).
set -euo pipefail
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/../native_bridge/scripts/build.sh" "$@"
