#!/usr/bin/env bash
# Source this script to set up OpenNI2 environment variables.
# Usage: source scripts/setup_env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
SDK_DIR="${SCRIPT_DIR}/../third_party/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux"

export OPENNI2_INCLUDE="${SDK_DIR}/sdk/Include"
export OPENNI2_REDIST="${SDK_DIR}/sdk/libs"
export LD_LIBRARY_PATH="${OPENNI2_REDIST}:${LD_LIBRARY_PATH}"

echo "[setup_env] OPENNI2_INCLUDE=$OPENNI2_INCLUDE"
echo "[setup_env] OPENNI2_REDIST=$OPENNI2_REDIST"

