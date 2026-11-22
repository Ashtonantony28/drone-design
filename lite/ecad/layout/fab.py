#!/usr/bin/env python3
"""Mill outputs for the Lite FC (host Python; kicad-cli runs in the KiCad 10 container).

    python3 layout/fab.py            (run.sh fab; run from lite/ecad/)

Writes fab/:
    gerbers/   lite_fc-B_Cu.gbr (main copper, the side the mill cuts first), lite_fc-F_Cu.gbr
               (top: VBAT/GND tracks only), lite_fc-Edge_Cuts.gbr, silk and fab layers for
               reference, plus the Gerber job file
    drill/     lite_fc-PTH.drl / lite_fc-NPTH.drl (Excellon, mm, absolute origin) and drill maps
    lite_fc-bottom-mill.svg   bottom copper + outline + drills, mirrored (as the mill sees it)
    lite_fc-top.svg           top copper + outline + drills, as seen from the top
    lite_fc-assembly-top.svg  top fab/silk layer with part outlines, for hand assembly
    lite_fc-top.png, lite_fc-bottom.png, lite_fc-bottom-mill.png  (layout/plot.py pictures)

KICAD_RUN overrides the container command (e.g. KICAD_RUN="" to use a local kicad-cli).
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = "boards/lite_fc/layout/lite_fc/lite_fc.kicad_pcb"
FAB = "fab"


def kicad(*args):
    run = os.environ.get("KICAD_RUN")
    if run is None:
        cmd = ["docker", "run", "--rm", "-u", "%d" % os.getuid(), "-e", "HOME=/tmp", "-v", "%s:%s" % (HERE, HERE),
               "-w", HERE, "kicad/kicad:10.0.8"]
    else:
        cmd = run.split()
    cmd += list(args)
    print("+ kicad-cli", " ".join(args[1:4]), "...")
    subprocess.run(cmd, check=True, cwd=HERE)


def main():
    os.chdir(HERE)
    for d in ("gerbers", "drill"):
        shutil.rmtree(os.path.join(FAB, d), ignore_errors=True)
        os.makedirs(os.path.join(FAB, d))
    kicad("kicad-cli", "pcb", "export", "gerbers", "--layers",
          "B.Cu,F.Cu,Edge.Cuts,F.Silkscreen,B.Silkscreen,F.Fab,B.Fab", "--no-protel-ext",
          "-o", FAB + "/gerbers/", BOARD)
    kicad("kicad-cli", "pcb", "export", "drill", "--format", "excellon", "--drill-origin", "absolute",
          "--excellon-units", "mm", "--excellon-separate-th", "--generate-map", "--map-format", "svg",
          "-o", FAB + "/drill/", BOARD)
    common = ["--mode-single", "--page-size-mode", "2", "--exclude-drawing-sheet", "--drill-shape-opt", "2"]
    kicad("kicad-cli", "pcb", "export", "svg", "--layers", "B.Cu,Edge.Cuts", "--mirror", *common,
          "-o", FAB + "/lite_fc-bottom-mill.svg", BOARD)
    kicad("kicad-cli", "pcb", "export", "svg", "--layers", "F.Cu,Edge.Cuts", *common,
          "-o", FAB + "/lite_fc-top.svg", BOARD)
    kicad("kicad-cli", "pcb", "export", "svg", "--layers", "F.Fab,F.Silkscreen,Edge.Cuts", *common,
          "-o", FAB + "/lite_fc-assembly-top.svg", BOARD)
    for src, dst in (("view_top", "lite_fc-top"), ("view_bottom", "lite_fc-bottom"), ("view_mill", "lite_fc-bottom-mill")):
        p = os.path.join("layout", "build", src + ".png")
        if os.path.exists(p):
            shutil.copy(p, os.path.join(FAB, dst + ".png"))
        else:
            print("missing %s: run layout/run.sh plot first" % p, file=sys.stderr)
    for root, _, files in os.walk(FAB):
        for f in sorted(files):
            print("  %s (%d bytes)" % (os.path.join(root, f), os.path.getsize(os.path.join(root, f))))


if __name__ == "__main__":
    main()
