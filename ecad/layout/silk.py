"""Silkscreen clean-up for nf_fc (KiCad Python; called by route.py finish, or on its own).

    python3 layout/silk.py <board.kicad_pcb>

The footprints bring their library silkscreen and the designators sit where place.py put
them; on a 36 mm board with parts on both sides that collides.  This pass makes the silk
layers pass KiCad's three silkscreen checks:

  1. footprint graphics that cross the board edge, lie on a pad's mask opening or touch the
     graphics of another footprint are taken off (of two footprints the smaller part loses
     its outline: R, C, FB before D and L before everything else);
  2. every designator that is shown (place.py SILK_REFS) and the board texts are moved to the
     nearest spot beside their part that is clear of pads, graphics, other texts and the
     edge; a designator with no such spot within reach goes to the fab layer.

Deterministic: footprints are handled in designator order.
"""
import sys

import pcbnew

MM = pcbnew.FromMM
BOARD_HALF = 18.0
ORIGIN = (100.0, 100.0)
EDGE_MARGIN = 0.35        # silk to board edge
PAD_MARGIN = 0.12         # silk to a pad (mask opening = pad; the check wants no overlap)
SILK_MARGIN = 0.16        # silk to silk of another item (rule: 0.1)
REACH = 3.0               # a designator moves at most this far from its part's outline
STEP = 0.2
RANK = (("R", "C", "FB"), ("D", "L"))


def rank(ref):
    name = ref.rstrip("0123456789")
    for i, group in enumerate(RANK):
        if name in group:
            return i
    return len(RANK)


def box(item):
    b = item.GetBoundingBox()
    return (pcbnew.ToMM(b.GetLeft()) - ORIGIN[0], pcbnew.ToMM(b.GetTop()) - ORIGIN[1],
            pcbnew.ToMM(b.GetRight()) - ORIGIN[0], pcbnew.ToMM(b.GetBottom()) - ORIGIN[1])


def gap(a, b):
    return max(a[0] - b[2], b[0] - a[2], a[1] - b[3], b[1] - a[3])


def inside_board(b):
    h = BOARD_HALF - EDGE_MARGIN
    return b[0] >= -h and b[1] >= -h and b[2] <= h and b[3] <= h


def tidy(board):
    # take every container before the first Remove(): the SWIG list accessors stop working after it
    drawings = list(board.GetDrawings())
    footprints = sorted(board.GetFootprints(), key=lambda f: f.GetReference())
    sides = {pcbnew.F_SilkS: pcbnew.F_Cu, pcbnew.B_SilkS: pcbnew.B_Cu}
    pads = {s: [] for s in sides}
    for fp in footprints:
        for p in fp.Pads():
            for silk, cu in sides.items():
                if p.IsOnLayer(cu) or p.GetDrillSize().x > 0:
                    pads[silk].append(box(p))
    # ---- 1. footprint graphics ---------------------------------------------------
    graphics = {s: [] for s in sides}     # [ref, box, item, footprint]
    for fp in footprints:
        for g in fp.GraphicalItems():
            if g.GetLayer() in sides and g.Type() == pcbnew.PCB_SHAPE_T:
                graphics[g.GetLayer()].append([fp.GetReference(), box(g), g, fp])
    condemned = []     # removed last: after a Remove() the SWIG wrappers return raw objects
    for silk in sides:
        doomed = set()
        items = graphics[silk]
        for i, (ref, b, g, fp) in enumerate(items):
            if not inside_board(b) or any(gap(b, p) < PAD_MARGIN for p in pads[silk]):
                doomed.add(i)
        for i, (ref, b, g, fp) in enumerate(items):
            for j in range(i + 1, len(items)):
                oref, ob = items[j][0], items[j][1]
                if oref == ref or i in doomed or j in doomed or gap(b, ob) >= SILK_MARGIN:
                    continue
                doomed.add(i if (rank(ref), ref) < (rank(oref), oref) else j)
        for i in sorted(doomed):
            condemned.append((items[i][3], items[i][2]))
        graphics[silk] = [it for i, it in enumerate(items) if i not in doomed]
    # ---- 2. texts -------------------------------------------------------------------
    placed = {s: [] for s in sides}
    moved = dropped = 0

    def free(silk, b, own_ref):
        if not inside_board(b):
            return False
        if any(gap(b, p) < PAD_MARGIN for p in pads[silk]):
            return False
        if any(gap(b, it[1]) < SILK_MARGIN for it in graphics[silk]):
            return False
        return not any(gap(b, o) < SILK_MARGIN for o in placed[silk])

    def settle(text, b, silk, anchor, own_ref, reach):
        """Move the text (bounding box b) to the free position nearest to its anchor box; False if there is none."""
        w, h = b[2] - b[0], b[3] - b[1]
        pos = text.GetPosition()
        off = (pcbnew.ToMM(pos.x) - ORIGIN[0] - (b[0] + b[2]) / 2, pcbnew.ToMM(pos.y) - ORIGIN[1] - (b[1] + b[3]) / 2)
        cx, cy = (anchor[0] + anchor[2]) / 2, (anchor[1] + anchor[3]) / 2
        n = int(round((reach + max(anchor[2] - anchor[0], anchor[3] - anchor[1]) / 2 + max(w, h)) / STEP))
        cands = []
        for ix in range(-n, n + 1):
            for iy in range(-n, n + 1):
                c = (cx + ix * STEP, cy + iy * STEP)
                tb = (c[0] - w / 2, c[1] - h / 2, c[0] + w / 2, c[1] + h / 2)
                d = gap(tb, anchor)
                if d <= reach:
                    cands.append((round(max(d, 0.0), 3), round(abs(c[0] - cx) + abs(c[1] - cy), 3), iy, ix, c, tb))
        for _, _, _, _, c, tb in sorted(cands):
            if free(silk, tb, own_ref):
                text.SetPosition(pcbnew.VECTOR2I(MM(ORIGIN[0] + c[0] + off[0]), MM(ORIGIN[1] + c[1] + off[1])))
                placed[silk].append(tb)
                return True
        return False

    for d in drawings:
        if d.GetLayer() in sides and d.Type() == pcbnew.PCB_SHAPE_T:
            graphics[d.GetLayer()].append([None, box(d), d, None])
    for d in sorted((d for d in drawings if d.GetLayer() in sides and d.Type() == pcbnew.PCB_TEXT_T), key=lambda d: d.GetText()):
        b = box(d)
        if not settle(d, b, d.GetLayer(), b, None, 6.0):
            placed[d.GetLayer()].append(b)
    for fp in footprints:
        ref = fp.Reference()
        silk = ref.GetLayer()
        if silk not in sides or not ref.IsVisible():
            continue
        body = [box(p) for p in fp.Pads()]
        anchor = (min(b[0] for b in body), min(b[1] for b in body), max(b[2] for b in body), max(b[3] for b in body))
        if settle(ref, box(ref), silk, anchor, fp.GetReference(), REACH):
            moved += 1
        else:
            ref.SetLayer(pcbnew.B_Fab if fp.IsFlipped() else pcbnew.F_Fab)
            ref.SetPosition(fp.GetPosition())
            dropped += 1
    for fp, g in condemned:
        fp.Remove(g)
    removed = len(condemned)
    print("silk: %d footprint graphics removed, %d designators placed, %d moved to the fab layer" % (removed, moved, dropped))


if __name__ == "__main__":
    brd = pcbnew.LoadBoard(sys.argv[1])
    tidy(brd)
    brd.Save(sys.argv[1])
