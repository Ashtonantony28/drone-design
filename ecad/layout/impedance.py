#!/usr/bin/env python3
"""Differential impedance of an edge-coupled microstrip pair on the board's own stackup.

    python3 layout/impedance.py [board.kicad_pcb] [--gap 0.15] [--target 90] [--width W]

The stackup (copper and dielectric thickness, epsilon_r, solder mask) is read from the
(stackup ...) block of the .kicad_pcb, which `pcb layout` writes from the Board() config in
nf_fc.zen (JLC04161H-7628).  KiCad's own calculator has no command-line interface, so the
field problem is solved here: a 2-D finite-difference solution of Laplace's equation on the
cross-section (quasi-static TEM), once with the dielectrics and once in air.

    odd mode  (electric wall on the symmetry plane):  Z_odd  = 1 / (c * sqrt(C_odd * C_odd_air))
    even mode (magnetic wall):                        Z_even likewise
    Z_diff = 2 * Z_odd,   Z_common = Z_even / 2

Geometry: outer layer (L1 or L4 - the stackup is symmetric) over the adjacent plane (L2 / L3),
rectangular traces of the outer copper thickness, solder mask over the laminate and over the
traces.  The single-ended result is checked against the Hammerstad-Jensen closed form.

Pure Python + numpy/scipy.  Used by check_layout.py (test_usb_pair) and preroute.py.
"""
import math
import re
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np

C0 = 299792458.0
EPS0 = 8.8541878128e-12
MASK_ER = 3.8          # LPI solder mask (KiCad's default for a mask layer without epsilon_r)
MASK_MIN = 0.02        # mm: KiCad writes 0.01; JLC's mask is 0.02-0.03 mm over laminate - use at least this


def read_stackup(board_path):
    """Outer copper thickness, prepreg height and epsilon_r, mask thickness of the outer layer pair."""
    text = Path(board_path).read_text()
    block = text[text.index("(stackup"):]
    layers = re.findall(r'\(layer "([^"]+)"(.*?)\n\t\t\t\)', block, re.S)
    out = []
    for name, body in layers:
        d = {"name": name}
        for key in ("type", "material"):
            m = re.search(r'\(%s "([^"]*)"\)' % key, body)
            if m:
                d[key] = m.group(1)
        for key in ("thickness", "epsilon_r", "loss_tangent"):
            m = re.search(r"\(%s ([0-9.]+)" % key, body)
            if m:
                d[key] = float(m.group(1))
        out.append(d)
        if name == "B.SilkS":
            break
    names = [d["name"] for d in out]
    f = names.index("F.Cu")
    cu, di, ref = out[f], out[f + 1], out[f + 2]
    mask = out[names.index("F.Mask")]
    assert di["type"] in ("prepreg", "core") and ref["type"] == "copper"
    b = names.index("B.Cu")
    assert abs(out[b - 1]["thickness"] - di["thickness"]) < 1e-9 and abs(out[b]["thickness"] - cu["thickness"]) < 1e-9, \
        "stackup is not symmetric: L4 needs its own calculation"
    return {"t": cu["thickness"], "h": di["thickness"], "er": di["epsilon_r"], "mask": max(mask.get("thickness", 0.0), MASK_MIN),
            "mask_er": mask.get("epsilon_r", MASK_ER), "layers": out}


def copper_spans(stack):
    """z position (mm, centre) of each copper layer, for via lengths."""
    z, pos = 0.0, {}
    for d in stack["layers"]:
        t = d.get("thickness", 0.0)
        if d.get("type") == "copper":
            pos[d["name"]] = z + t / 2
        if d.get("type") in ("copper", "prepreg", "core"):
            z += t
    return pos


def _solve(w, s, h, t, er, mask, mask_er, odd, air, step):
    """Capacitance per metre of one trace of the pair (half cross-section, symmetry plane at x=0)."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import spsolve
    # everything snapped to the grid: i0..i1 trace nodes, j0..j1 its bottom and top, jm mask
    i0, i1 = int(round(s / 2 / step)), int(round((s / 2 + w) / step))
    j0 = int(round(h / step))
    j1 = j0 + int(round(t / step))
    km = int(round(mask / step))
    nx = i1 + int(round(8 * h / step))
    ny = j1 + int(round(8 * h / step))
    eps = np.ones((ny, nx))            # cell-centred permittivity, cell (j, i) spans nodes j..j+1, i..i+1
    if not air:
        eps[:j0, :] = er
        eps[j0:j0 + km, :] = mask_er                                   # mask on the laminate
        eps[j0:j1 + km, max(i0 - km, 0):i1 + km] = mask_er             # mask around the trace
    cond = np.zeros((ny + 1, nx + 1), bool)
    cond[j0:j1 + 1, i0:i1 + 1] = True
    V = np.where(cond, 1.0, 0.0)
    fixed = cond.copy()
    fixed[0, :] = True
    fixed[-1, :] = True
    fixed[:, -1] = True
    if odd:
        fixed[:, 0] = True
    idx = -np.ones((ny + 1, nx + 1), int)
    free = ~fixed
    idx[free] = np.arange(free.sum())
    # edge weights: a horizontal edge between nodes (j,i)-(j,i+1) borders cells (j-1,i) and (j,i)
    pe = np.pad(eps, 1, mode="edge")
    wx = 0.5 * (pe[0:ny + 1, 1:nx + 1] + pe[1:ny + 2, 1:nx + 1])      # (ny+1, nx)
    wy = 0.5 * (pe[1:ny + 1, 0:nx + 1] + pe[1:ny + 1, 1:nx + 2])      # (ny, nx+1)
    if not odd:
        wy[:, 0] *= 0.5   # edges on the symmetry line belong half to this half-plane
    rows, cols, vals = [], [], []
    rhs = np.zeros(free.sum())
    diag = np.zeros(free.sum())

    def couple(a_idx, b_idx, a_fixed_v, b_fixed_v, wgt):
        a, b, wv = a_idx.ravel(), b_idx.ravel(), wgt.ravel()
        av, bv = a_fixed_v.ravel(), b_fixed_v.ravel()
        both = (a >= 0) & (b >= 0)
        rows.extend([a[both], b[both]])
        cols.extend([b[both], a[both]])
        vals.extend([-wv[both], -wv[both]])
        np.add.at(diag, a[a >= 0], wv[a >= 0])
        np.add.at(diag, b[b >= 0], wv[b >= 0])
        m = (a >= 0) & (b < 0)
        np.add.at(rhs, a[m], wv[m] * bv[m])
        m = (b >= 0) & (a < 0)
        np.add.at(rhs, b[m], wv[m] * av[m])

    couple(idx[:, :-1], idx[:, 1:], V[:, :-1], V[:, 1:], wx)
    couple(idx[:-1, :], idx[1:, :], V[:-1, :], V[1:, :], wy)
    n = free.sum()
    rows.append(np.arange(n))
    cols.append(np.arange(n))
    vals.append(diag)
    A = coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(n, n)).tocsr()
    V[free] = spsolve(A, rhs)
    energy = (wx * (V[:, 1:] - V[:, :-1]) ** 2).sum() + (wy * (V[1:, :] - V[:-1, :]) ** 2).sum()
    return EPS0 * energy    # C = 2W / V^2 with V = 1


@lru_cache(maxsize=None)
def pair(w, s, h, t, er, mask=MASK_MIN, mask_er=MASK_ER, step=None):
    """(Z_diff, Z_odd, Z_even, eps_eff_odd) of an edge-coupled microstrip pair; lengths in mm."""
    step = step or 0.005
    z = {}
    for odd in (True, False):
        c = _solve(w, s, h, t, er, mask, mask_er, odd, False, step)
        ca = _solve(w, s, h, t, er, mask, mask_er, odd, True, step)
        z[odd] = (1.0 / (C0 * math.sqrt(c * ca)), c / ca)
    return 2 * z[True][0], z[True][0], z[False][0], z[True][1]


def hammerstad(w, h, er):
    """Single microstrip, zero thickness, no mask (Hammerstad 1975) - sanity check of the solver."""
    u = w / h
    if u <= 1:
        ee = (er + 1) / 2 + (er - 1) / 2 * ((1 + 12 / u) ** -0.5 + 0.04 * (1 - u) ** 2)
        return 60 / math.sqrt(ee) * math.log(8 / u + u / 4)
    ee = (er + 1) / 2 + (er - 1) / 2 * (1 + 12 / u) ** -0.5
    return 120 * math.pi / (math.sqrt(ee) * (u + 1.393 + 0.667 * math.log(u + 1.444)))


def width_for(target, s, stack, lo=0.08, hi=0.45):
    """Trace width (mm) that gives the target differential impedance at gap s."""
    f = lambda w: pair(round(w, 4), s, stack["h"], stack["t"], stack["er"], stack["mask"], stack["mask_er"])[0] - target
    for _ in range(12):
        mid = (lo + hi) / 2
        if f(mid) > 0:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def main(argv):
    board = next((a for a in argv if a.endswith(".kicad_pcb")), str(Path(__file__).resolve().parent.parent / "boards/nf_fc/layout/nf_fc/nf_fc.kicad_pcb"))
    opt = {argv[i][2:]: float(argv[i + 1]) for i in range(len(argv) - 1) if argv[i].startswith("--")}
    st = read_stackup(board)
    print("stackup from %s:" % Path(board).name)
    print("  outer copper %.4f mm, prepreg to the next plane %.4f mm (er %.2f), mask %.3f mm (er %.1f)" % (st["t"], st["h"], st["er"], st["mask"], st["mask_er"]))
    # solver check: a wide gap, no mask, thin copper -> the single microstrip formula
    zo = pair(0.3, 3.0, st["h"], 0.0, st["er"], 0.0, 1.0)[1]
    print("  solver check: single 0.30 mm microstrip, t->0, no mask: %.1f ohm (Hammerstad %.1f ohm)" % (zo, hammerstad(0.3, st["h"], st["er"])))
    target, gap = opt.get("target", 90.0), opt.get("gap", 0.15)
    if "width" in opt:
        w = opt["width"]
    else:
        w = width_for(target, gap, st)
        print("  %.0f ohm differential at gap %.3f mm -> width %.4f mm" % (target, gap, w))
    for ww, ss in ((w, gap), (round(w, 2), gap), (round(w, 2) - 0.02, gap), (round(w, 2) + 0.02, gap), (0.1867, 0.127)):
        zd, zodd, zev, ee = pair(round(ww, 4), ss, st["h"], st["t"], st["er"], st["mask"], st["mask_er"])
        print("  w %.4f  gap %.3f : Zdiff %.1f ohm  (Zodd %.1f, Zeven %.1f, eps_eff,odd %.2f)" % (ww, ss, zd, zodd, zev, ee))


if __name__ == "__main__":
    main(sys.argv[1:])
