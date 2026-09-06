#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HNO_VIO_VENV="${HNO_VIO_VENV:-/home/he/.venvs/hnovio}"
RTABMAP_CORE_LIB_DIR="${RTABMAP_CORE_LIB_DIR:-/opt/ros/humble/lib}"

if [[ ! -x "${HNO_VIO_VENV}/bin/python" ]]; then
  echo "HNO-VIO Python environment not found: ${HNO_VIO_VENV}" >&2
  echo "Set HNO_VIO_VENV to the environment containing rerun-sdk, evo and plyfile." >&2
  exit 1
fi

set +u
if [[ -s /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
set -u

if [[ -d "${RTABMAP_CORE_LIB_DIR}" ]]; then
  export LD_LIBRARY_PATH="${RTABMAP_CORE_LIB_DIR}:${LD_LIBRARY_PATH:-}"
fi

export PATH="${HNO_VIO_VENV}/bin:${PATH}"
exec "${HNO_VIO_VENV}/bin/python" "${SCRIPT_DIR}/export_rtabmap_rrd.py" "$@"
