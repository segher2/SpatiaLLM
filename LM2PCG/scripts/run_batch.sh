#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BIN="$BUILD_DIR/pcg_room"

INPUT_ROOT="${1:-$ROOT_DIR/data/rooms}"
OUTPUT_ROOT="${2:-$ROOT_DIR/output}"
RADIUS="${3:-0.05}"
MIN_CLUSTER_SIZE="${4:-50}"

if [[ ! -x "$BIN" ]]; then
  echo "pcg_room binary not found; building..."
  mkdir -p "$BUILD_DIR"
  cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$BUILD_DIR" -j
fi

shopt -s nullglob
for dir in "$INPUT_ROOT"/*/ ; do
  room_name="$(basename "$dir")"
  out_dir="$OUTPUT_ROOT/$room_name"
  echo "Processing room: $room_name"
  "$BIN" "$dir" "$out_dir" "$RADIUS" "$MIN_CLUSTER_SIZE"
done

echo "All rooms processed. Logs under each room's results/ directory."
