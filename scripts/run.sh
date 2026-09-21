#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
export ASTRA_DEPTH_FORMAT=ps
export MALLOC_CHECK_=0
export LD_LIBRARY_PATH="$ROOT/build:${LD_LIBRARY_PATH:-}"
export OPENNI2_DRIVERS_PATH="$ROOT/build/OpenNI2/Drivers"
exec "$ROOT/build/astra_capture" "$@"
