#!/usr/bin/env bash
# Policy runtime for the real robot (or the stand-in). Talks UDP to the bridge on localhost.
set -euo pipefail
cd "$(dirname -- "${BASH_SOURCE[0]}")/.."
exec ./scripts/python.sh -m h2_deploy.controller "$@"
