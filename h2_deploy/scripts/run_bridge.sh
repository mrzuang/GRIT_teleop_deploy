#!/usr/bin/env bash
# Real H2 native bridge. Usage: H2_NET=<robot-facing NIC, e.g. enp3s0> ./scripts/run_bridge.sh
set -euo pipefail
: "${H2_NET:?Set H2_NET to the robot-facing network interface, e.g. H2_NET=enp3s0}"
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/../native_bridge/scripts/run_bridge.sh" "$@"
