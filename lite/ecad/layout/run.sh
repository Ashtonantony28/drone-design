#!/bin/sh
# Layout pipeline for the Lite FC. Run from lite/ecad/.  Usage: layout/run.sh <step>...
#   new     regenerate the KiCad board from the Zener design (pcb layout, in the KiCad container)
#   place   outline, holes, parts, netclasses, mill DRC rules        (place.py, KiCad Python)
#   dump    geometry JSON                                           -> layout/build/board.json
#   route   single-sided grid router (router.py, host Python)       -> layout/build/routes.json
#   apply   tracks + bottom GND pour onto the board                 (route_apply.py, KiCad Python)
#   drc     kicad-cli DRC with the mill rules                       -> layout/build/drc.json
#   plot    review pictures                                         -> layout/build/view_{top,bottom}.png
#   fab     Gerbers, Excellon drill, previews                       -> fab/
#   all     new place dump route apply dump drc plot
# KiCad 10 runs from the kicad/kicad:10.0.8 image (set KICAD_RUN to override).
set -e
cd "$(dirname "$0")/.."
BOARD=boards/lite_fc/layout/lite_fc/lite_fc.kicad_pcb
OUT=layout/build
PY=${PYTHON:-python3}
mkdir -p "$OUT"
kicad() {
    if [ -n "$KICAD_RUN" ]; then $KICAD_RUN "$@"; else
        docker run --rm -u "$(id -u)" -e HOME=/tmp -v "$PWD":"$PWD" -w "$PWD" kicad/kicad:10.0.8 "$@"
    fi
}
steps="$*"
[ "$steps" = all ] && steps="new place dump route apply dump drc plot"
for step in $steps; do
    case "$step" in
    new) docker run --rm -u "$(id -u)" -e HOME="$HOME" -v "$HOME/.local:$HOME/.local" -v "$PWD":"$PWD" -w "$PWD" \
            kicad/kicad:10.0.8 "$HOME/.local/bin/pcb" layout --no-open --offline boards/lite_fc/lite_fc.zen ;;
    place) rm -f "$OUT/drc.json"; kicad python3 layout/place.py "$BOARD" ;;
    dump) kicad python3 layout/dump.py "$BOARD" "$OUT/board.json" ;;
    route) $PY layout/router.py "$OUT/board.json" "$OUT/routes.json" ;;
    apply) kicad python3 layout/place.py "$BOARD" && kicad python3 layout/route_apply.py "$BOARD" "$OUT/routes.json" ;;
    drc) kicad kicad-cli pcb drc --format json --severity-all --units mm -o "$OUT/drc.json" "$BOARD" >/dev/null &&
        $PY layout/drc_summary.py "$OUT/drc.json" ;;
    plot) $PY layout/plot.py "$OUT/board.json" "$OUT/view" $( [ -f "$OUT/drc.json" ] && echo "$OUT/drc.json" ) ;;
    fab) $PY layout/fab.py ;;
    *) echo "unknown step $step" >&2; exit 2 ;;
    esac
done
