#!/usr/bin/env bash
# MuJoCo sim2sim: robot starts on the reference's first frame. Extra args go to h2_deploy.sim2sim.
set -euo pipefail
cd "$(dirname -- "${BASH_SOURCE[0]}")/.."
exec ./scripts/python.sh -m h2_deploy.sim2sim "$@"
