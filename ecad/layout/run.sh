#!/bin/sh
# Layout pipeline for nf_fc. Run from ecad/.  Usage: layout/run.sh <step>...
#   place   outline, placement, planes, keep-outs        (KiCad Python, in the container)
#   dump    geometry JSON for plot.py / check_layout.py  -> layout/build/board.json
#   plot    review pictures                              -> layout/build/view_{top,bottom}.png
#   pre     scripted copper: decoupling links, DShot escapes, plane vias, stitching (preroute.py)
#   auto    maze router for everything else (autoroute.py, pure Python, ~10 min)
#   finish  L1/L4 pours, zone refill
#   drc     kicad-cli DRC                                -> layout/build/drc.json (+ summary)
#   fab     gerbers, drill, pick-and-place, STEP         -> fab/
#   all     place dump pre auto finish dump drc plot
# KiCad 10 runs from the kicad/kicad:10.0.8 image; set KICAD_RUN to another wrapper
# (or to "env" when KiCad 10 is installed on this machine).
set -e
cd "$(dirname "$0")/.."
BOARD=boards/nf_fc/layout/nf_fc/nf_fc.kicad_pcb
OUT=layout/build
PY=${PYTHON:-python3}
mkdir -p "$OUT"
kicad() {
    if [ -n "$KICAD_RUN" ]; then $KICAD_RUN "$@"; else
        docker run --rm -u "$(id -u)" -e HOME=/tmp -v "$PWD":"$PWD" -w "$PWD" kicad/kicad:10.0.8 "$@"
    fi
}
steps="$*"
[ "$steps" = all ] && steps="place dump pre auto finish dump drc plot"
for step in $steps; do
    case "$step" in
    place) rm -f "$OUT/drc.json"; kicad python3 layout/place.py "$BOARD" ;;
    dump) kicad python3 layout/dump.py "$BOARD" "$OUT/board.json" ;;
    plot) $PY layout/plot.py "$OUT/board.json" "$OUT/view" $( [ -f "$OUT/drc.json" ] && echo "$OUT/drc.json" ) && $PY layout/overlaps.py "$OUT/board.json" ;;
    pre) $PY layout/preroute.py "$OUT/board.json" "$OUT/preroute.json" &&
        kicad python3 layout/route.py apply "$BOARD" "$OUT/preroute.json" locked &&
        kicad python3 layout/dump.py "$BOARD" "$OUT/pre.json" ;;
    auto) $PY layout/autoroute.py "$OUT/pre.json" "$OUT/autoroute.json" &&
        kicad python3 layout/route.py apply "$BOARD" "$OUT/autoroute.json" unlocked ;;
    finish) kicad python3 layout/route.py finish "$BOARD" ;;
    drc) kicad kicad-cli pcb drc --format json --severity-all --units mm -o "$OUT/drc.json" "$BOARD" >/dev/null &&
        $PY layout/drc_summary.py "$OUT/drc.json" ;;
    *) echo "unknown step $step" >&2; exit 2 ;;
    esac
done
