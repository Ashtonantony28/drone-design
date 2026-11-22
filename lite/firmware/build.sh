#!/usr/bin/env bash
# Real ESP32 compile of both sketches with arduino-cli (no hardware needed).
#   lite/firmware/build.sh            # installs the cores/libraries on first run (~1 GB), then compiles
# Flight board: esp32_bluepad32 4.1.0 core, "ESP32 Dev Module". Camera board: esp32 3.0.7, XIAO_ESP32S3, OPI PSRAM.
# Needs arduino-cli on PATH (or ARDUINO_CLI=/path/to/arduino-cli) and python3 with pyserial (esptool).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CLI="${ARDUINO_CLI:-arduino-cli}"
export ARDUINO_DIRECTORIES_DATA="${ARDUINO_DIRECTORIES_DATA:-$HOME/.arduino15}"
export ARDUINO_DIRECTORIES_USER="${ARDUINO_DIRECTORIES_USER:-$HOME/Arduino}"
[ -n "${HTTPS_PROXY:-}" ] && "$CLI" config set network.proxy "$HTTPS_PROXY" >/dev/null 2>&1 || true
URLS="https://espressif.github.io/arduino-esp32/package_esp32_index.json,https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json"
if ! "$CLI" core list 2>/dev/null | grep -q "esp32-bluepad32:esp32"; then
  "$CLI" core update-index --additional-urls "$URLS"
  "$CLI" core install esp32:esp32@3.0.7 esp32-bluepad32:esp32@4.1.0 --additional-urls "$URLS"
  "$CLI" lib install "Bitcraze PMW3901@1.2" "VL53L1X@1.3.1"
fi
OUT="${TMPDIR:-/tmp}/lite_fw_build"
"$CLI" compile --warnings default --fqbn esp32-bluepad32:esp32:esp32 --libraries "$HERE/libraries" \
  --output-dir "$OUT/lite_fc" "$HERE/lite_fc"
"$CLI" compile --warnings default --fqbn esp32:esp32:XIAO_ESP32S3:PSRAM=opi --libraries "$HERE/libraries" \
  --output-dir "$OUT/companion" "$HERE/companion"
"$CLI" compile --warnings default --fqbn esp32-bluepad32:esp32:esp32 --libraries "$HERE/libraries" \
  --output-dir "$OUT/policy_bench" "$HERE/policy_bench"
echo "all sketches compiled: $OUT"
