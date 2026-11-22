#!/usr/bin/env bash
# Build and run the Lite flight-core host tests, then syntax-check the Arduino
# sketch against minimal mocks of the Arduino/Bluepad32 APIs it uses.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
L="$HERE/../libraries"
OUT="${TMPDIR:-/tmp}/lite_fw_tests"; mkdir -p "$OUT"
CXX="${CXX:-g++}"
FLAGS=(-std=c++17 -O2 -Wall -Wextra -Werror -I"$L/nf_core/src" -I"$L/lite_fc/src" -I"$HERE")
"$CXX" "${FLAGS[@]}" "$HERE/test_lite.cpp" "$L/lite_fc/src/lite_flight_core.cpp" \
  "$L/nf_core/src/nf_control.cpp" "$L/nf_core/src/nf_estimator.cpp" "$L/nf_core/src/nf_filters.cpp" -o "$OUT/test_lite"
"$OUT/test_lite"
# sketch: compile only (mocks have no behaviour)
"$CXX" "${FLAGS[@]}" -Wno-unused-parameter -I"$HERE/mock" -x c++ -fsyntax-only "$HERE/../lite_fc/lite_fc.ino"
echo "sketch syntax OK (against mocks; the real check is compiling in the Arduino IDE)"
