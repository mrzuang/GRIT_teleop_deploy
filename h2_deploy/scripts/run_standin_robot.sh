#!/usr/bin/env bash
# MuJoCo stand-in for the native bridge (no hardware). Keys in this terminal: s / a / x.
set -euo pipefail
cd "$(dirname -- "${BASH_SOURCE[0]}")/.."
exec ./scripts/python.sh -m h2_deploy.mujoco_robot "$@"
