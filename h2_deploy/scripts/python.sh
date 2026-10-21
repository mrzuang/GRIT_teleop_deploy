#!/usr/bin/env bash
# Run Python with this package on PYTHONPATH. Interpreter: $H2_PYTHON, else ./.venv (see scripts/setup_env.sh).
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
PY="${H2_PYTHON:-${ROOT}/.venv/bin/python}"
if [[ ! -x "${PY}" ]]; then
  echo "Python environment not found: ${PY}. Run ./scripts/setup_env.sh first (or set H2_PYTHON)." >&2
  exit 1
fi
if ! "${PY}" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)'; then
  echo "${PY} is $("${PY}" --version 2>&1); Python >= 3.10 is required. Run ./scripts/setup_env.sh." >&2
  exit 1
fi
exec "${PY}" "$@"
