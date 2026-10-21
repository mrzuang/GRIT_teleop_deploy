#!/usr/bin/env bash
# Create ./.venv with Python >= 3.10 and install requirements.txt.
# Uses uv when available (it downloads a suitable Python if none is installed),
# otherwise the newest python3.1x found on PATH.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
PY_VERSION="${H2_PYTHON_VERSION:-3.10}"
ok_python() { "$1" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null; }

if [[ -x .venv/bin/python ]] && ! ok_python .venv/bin/python; then
  echo "[setup] existing .venv uses $(.venv/bin/python --version 2>&1); Python >= 3.10 is required, recreating it"
  rm -rf .venv
fi

if command -v uv >/dev/null 2>&1; then
  [[ -x .venv/bin/python ]] || uv venv --python "${PY_VERSION}" .venv
  uv pip install --python .venv/bin/python -r requirements.txt
else
  if [[ ! -x .venv/bin/python ]]; then
    BASE=""
    for candidate in python3.13 python3.12 python3.11 python3.10 python3; do
      if command -v "${candidate}" >/dev/null 2>&1 && ok_python "$(command -v "${candidate}")"; then
        BASE="$(command -v "${candidate}")"; break
      fi
    done
    if [[ -z "${BASE}" ]]; then
      echo "[setup] No Python >= 3.10 found (python3 is $(python3 --version 2>&1))." >&2
      echo "        Install uv (curl -LsSf https://astral.sh/uv/install.sh | sh) and rerun, or" >&2
      echo "        conda create -n h2 python=3.10 && conda activate h2 && ./scripts/setup_env.sh" >&2
      exit 1
    fi
    echo "[setup] creating .venv with ${BASE} ($("${BASE}" --version 2>&1))"
    if ! "${BASE}" -m venv .venv; then
      rm -rf .venv
      echo "[setup] venv creation failed. Install uv (curl -LsSf https://astral.sh/uv/install.sh | sh)" >&2
      echo "        or the venv module (e.g. sudo apt install python3.10-venv), then rerun." >&2
      exit 1
    fi
  fi
  .venv/bin/python -m pip install --upgrade pip
  .venv/bin/python -m pip install -r requirements.txt
fi

.venv/bin/python -c 'import sys, numpy, mujoco, onnxruntime, yaml; print(f"[setup] ok: Python {sys.version.split()[0]}, numpy {numpy.__version__}, mujoco {mujoco.__version__}, onnxruntime {onnxruntime.__version__}")'
