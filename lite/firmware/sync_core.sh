#!/usr/bin/env bash
# Copy the portable neuroflight-fc firmware core (math, filters, Mahony, rate PID,
# mixer) into an Arduino library, so the Lite flies the exact code the simulator
# was checked against. vehicle_params.hpp is NOT copied: gen_params.py writes the
# Lite's own from lite/out/flight.json.
#   NEUROFLIGHT_FC=../neuroflight-fc lite/firmware/sync_core.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
NF="${NEUROFLIGHT_FC:-$HERE/../../../neuroflight-fc}"
DST="$HERE/libraries/nf_core/src"
for f in math.hpp filters.hpp estimator.hpp control.hpp; do cp "$NF/fw/core/include/nf/$f" "$DST/nf/$f"; done
for f in filters.cpp estimator.cpp control.cpp; do cp "$NF/fw/core/src/$f" "$DST/nf_$f"; done
echo "$(git -C "$NF" rev-parse HEAD)" > "$DST/NEUROFLIGHT_FC_COMMIT"
echo "nf_core synced from neuroflight-fc $(cat "$DST/NEUROFLIGHT_FC_COMMIT" | cut -c1-8)"
