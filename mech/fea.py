"""Finite-element checks on the carbon bottom plate (the primary structure).

Mesh: gmsh tetrahedra from the build123d solid, quadratic (P2) displacement
elements in scikit-fem. Material: quasi-isotropic carbon laminate treated as
isotropic linear-elastic (spec materials.carbon_3k_quasi_iso); this is the
usual first-pass assumption for a 0/90/±45 plate loaded in bending.

Boundary condition for every case: the plate is clamped where the stack
screws pass through it (the four 30.5 mm holes). The battery + avionics are
~55 % of the mass and sit on those screws, so the centre is the stiff, heavy
body that the arms cantilever from.

Cases
  thrust : full-throttle thrust (full battery) on each motor pad, up.
  crash  : the whole vehicle lands on ONE foot at `crash_load_g`; the full
           impact force goes into that foot's ring and is reacted at the
           stack. Conservative: arm inertia relief is ignored.
  modal  : first natural frequencies with the motor+prop, duct and foot
           masses lumped onto the motor pad and duct ring (duct stiffness
           ignored, which lowers the frequencies, i.e. conservative).

`python -m mech.fea` writes out/fea.json and exits non-zero if a requirement
fails (stress safety factors, mode separation from the hover rotor frequency).
"""
from __future__ import annotations

import json
import math
import os
import sys
import tempfile

import numpy as np

from build123d import export_step

from .frame import bottom_plate, dims, foot, load_spec, motor_xy

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def mesh_plate(spec: dict, h: float = 1.6):
    import gmsh
    from skfem import MeshTet
    plate = bottom_plate(spec)
    with tempfile.TemporaryDirectory() as td:
        step = os.path.join(td, "plate.step")
        export_step(plate, step)
        gmsh.initialize()
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.importShapes(step)
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", h)
        gmsh.option.setNumber("Mesh.MeshSizeMin", h * 0.4)
        gmsh.model.mesh.generate(3)
        tags, coords, _ = gmsh.model.mesh.getNodes()
        etypes, _, enodes = gmsh.model.mesh.getElements(3)
        gmsh.finalize()
    pts = coords.reshape(-1, 3)
    idx = {t: i for i, t in enumerate(tags)}
    tets = np.array([idx[t] for t in enodes[list(etypes).index(4)]]).reshape(-1, 4)
    used = np.unique(tets)
    remap = -np.ones(len(pts), dtype=int)
    remap[used] = np.arange(len(used))
    p = pts[used].T * 1e-3  # metres
    t = remap[tets].T
    return MeshTet(p, t)


def setup(spec: dict, mesh):
    from skfem import Basis, ElementTetP2, ElementVector
    from skfem.models.elasticity import lame_parameters, linear_elasticity
    from skfem import asm, BilinearForm
    from skfem.helpers import dot
    mat = spec["materials"]["carbon_3k_quasi_iso"]
    basis = Basis(mesh, ElementVector(ElementTetP2()))
    lam, mu = lame_parameters(mat["E"], mat["nu"])
    K = asm(linear_elasticity(lam, mu), basis)

    @BilinearForm
    def mass(u, v, w):
        return mat["density"] * dot(u, v)

    M = asm(mass, basis)
    return basis, K, M, lam, mu


def node_sets(spec: dict, basis):
    """Return DOF index arrays for clamp / motor pads / duct rings / feet."""
    g, d = spec["geometry_mm"], dims(spec)
    X = basis.doflocs  # (3, ndofs) for vector basis: each dof location
    x, y, z = X * 1e3
    s = g["stack_spacing"] / 2
    clamp = np.zeros(X.shape[1], bool)
    for sx in (-1, 1):
        for sy in (-1, 1):
            clamp |= np.hypot(x - sx * s, y - sy * s) < g["stack_clamp_diameter"] / 2
    pads, rings, feet = [], [], []
    for (mx, my) in motor_xy(spec):
        r = np.hypot(x - mx, y - my)
        pads.append(r < 11.0)
        rings.append((r > d["ring_ri"] - 0.1) & (r < d["ring_ro"] + 0.1))
        fb = foot(spec, mx, my).bounding_box()
        fx, fy = (fb.min.X + fb.max.X) / 2, (fb.min.Y + fb.max.Y) / 2
        feet.append((np.hypot(x - fx, y - fy) < 3.0) & (z < 0.05))
    return clamp, pads, rings, feet


def von_mises(basis, u, lam, mu):
    from skfem import Basis, ElementTetP1
    from skfem.helpers import sym_grad
    # stress at quadrature points, report the max over the mesh
    w = basis.interpolate(u)
    eps = sym_grad(w)  # (3,3,nel,nqp)
    tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
    sig = 2 * mu * eps
    for i in range(3):
        sig[i, i] += lam * tr
    sxx, syy, szz = sig[0, 0], sig[1, 1], sig[2, 2]
    sxy, syz, szx = sig[0, 1], sig[1, 2], sig[2, 0]
    vm = np.sqrt(0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2)
                 + 3 * (sxy ** 2 + syz ** 2 + szx ** 2))
    return vm


def static_case(basis, K, clamp_dofs_mask, loads):
    """loads: list of (dof-location mask over vector dofs, total force vector N)."""
    from skfem import condense, solve
    f = np.zeros(K.shape[0])
    comp = np.arange(K.shape[0]) % 3  # ElementVector interleaves components per node? use basis.nodal_dofs
    for mask, F in loads:
        for c in range(3):
            if F[c] == 0:
                continue
            sel = np.where(mask & (comp_of(basis) == c))[0]
            f[sel] += F[c] / len(sel)
    D = np.where(clamp_dofs_mask)[0]
    u = solve(*condense(K, f, D=D))
    return u


_comp_cache = {}


def comp_of(basis):
    """Component index (0,1,2) of every global DOF of a vector basis."""
    key = id(basis)
    if key not in _comp_cache:
        comp = np.zeros(basis.N, int)
        for c, name in enumerate(("u^1", "u^2", "u^3")):
            comp[basis.get_dofs().all(name)] = c
        _comp_cache[key] = comp
    return _comp_cache[key]


def run(spec: dict | None = None, h: float = 1.6, n_modes: int = 6) -> dict:
    from scipy.sparse.linalg import eigsh
    from skfem import condense
    spec = spec or load_spec()
    req = spec["requirements"]
    mat = spec["materials"]["carbon_3k_quasi_iso"]
    mesh = mesh_plate(spec, h)
    basis, K, M, lam, mu = setup(spec, mesh)
    clamp, pads, rings, feet = node_sets(spec, basis)
    out = {"mesh": {"nodes": int(mesh.p.shape[1]), "tets": int(mesh.t.shape[1]), "dofs": int(basis.N)},
           "failures": []}

    # ---- thrust case
    prop_path = os.path.join(ROOT, "out", "propulsion.json")
    with open(prop_path) as f:
        prop = json.load(f)["nominal"]
    T = prop["max_full"]["thrust_per_motor_n"]
    u = static_case(basis, K, clamp, [(pads[i], (0, 0, T)) for i in range(4)])
    vm = von_mises(basis, u, lam, mu)
    uz = u[comp_of(basis) == 2]
    out["thrust"] = {"force_per_motor_n": T, "max_von_mises_mpa": float(vm.max() / 1e6),
                     "max_deflection_mm": float(np.abs(uz).max() * 1e3),
                     "safety_factor": float(mat["strength"] / vm.max())}
    if out["thrust"]["safety_factor"] < req["min_safety_factor_thrust"]:
        out["failures"].append(f"thrust SF {out['thrust']['safety_factor']:.2f} < {req['min_safety_factor_thrust']}")

    # ---- crash case (foot 0)
    with open(os.path.join(ROOT, "out", "mech_checks.json")) as f:
        mass_kg = json.load(f)["mass_g"] / 1000.0
    Fc = mass_kg * spec["environment"]["g"] * req["crash_load_g"]
    u = static_case(basis, K, clamp, [(feet[0], (0, 0, Fc))])
    vm = von_mises(basis, u, lam, mu)
    uz = u[comp_of(basis) == 2]
    out["crash"] = {"force_n": Fc, "max_von_mises_mpa": float(vm.max() / 1e6),
                    "max_deflection_mm": float(np.abs(uz).max() * 1e3),
                    "safety_factor": float(mat["strength"] / vm.max())}
    if out["crash"]["safety_factor"] < req["min_safety_factor_crash"]:
        out["failures"].append(f"crash SF {out['crash']['safety_factor']:.2f} < {req['min_safety_factor_crash']}")

    # ---- modal
    pr = spec["parts"]
    duct_mass = None
    from .frame import duct
    duct_mass = duct(spec, 0, 0).volume * 1e-9 * spec["materials"]["pa12_cf_print"]["density"]
    pad_mass = (pr["motor"]["mass_g"] + pr["prop"]["mass_g"]) / 1000.0
    ring_mass = duct_mass + 0.4e-3
    Ma = M.tolil(copy=True)
    nodal = basis.nodal_dofs  # (3, nverts) vertex dofs per component
    vert_xyz = mesh.p * 1e3
    for i, (mx, my) in enumerate(motor_xy(spec)):
        r = np.hypot(vert_xyz[0] - mx, vert_xyz[1] - my)
        d = dims(spec)
        for sel, m_add in ((r < 11.0, pad_mass), ((r > d["ring_ri"] - 0.1) & (r < d["ring_ro"] + 0.1), ring_mass)):
            vs = np.where(sel)[0]
            for c in range(3):
                for dof in nodal[c, vs]:
                    Ma[dof, dof] += m_add / len(vs)
    Ma = Ma.tocsr()
    D = np.where(clamp)[0]
    I = np.setdiff1d(np.arange(basis.N), D)
    Kc, Mc = K[I][:, I], Ma[I][:, I]
    vals, _ = eigsh(Kc, k=n_modes, M=Mc, sigma=0.0, which="LM")
    freqs = np.sqrt(np.abs(np.sort(vals))) / (2 * math.pi)
    out["modal"] = {"freqs_hz": freqs.tolist(), "duct_mass_g": duct_mass * 1e3,
                    "pad_mass_g": pad_mass * 1e3}
    with open(prop_path) as f:
        props = json.load(f)
    for corner in ("nominal", "pessimistic"):
        f_rot = props[corner]["hover_rotor_hz"]
        sep = abs(freqs[0] - f_rot) / f_rot
        out["modal"][f"separation_{corner}"] = sep
        out["modal"][f"hover_rotor_hz_{corner}"] = f_rot
        if sep < req["min_mode_separation"]:
            out["failures"].append(f"first mode {freqs[0]:.0f} Hz within {sep*100:.0f} % of hover rotor "
                                   f"{f_rot:.0f} Hz ({corner})")
    return out


def main():
    h = float(sys.argv[1]) if len(sys.argv) > 1 else 1.6
    r = run(h=h)
    print(f"mesh: {r['mesh']}")
    for k in ("thrust", "crash"):
        c = r[k]
        print(f"{k:6s}: max von Mises {c['max_von_mises_mpa']:.1f} MPa, deflection {c['max_deflection_mm']:.3f} mm, "
              f"SF {c['safety_factor']:.2f}")
    m = r["modal"]
    print("modes (Hz):", np.round(m["freqs_hz"], 0),
          f"| hover rotor {m['hover_rotor_hz_nominal']:.0f} Hz (sep {m['separation_nominal']*100:.0f} %), "
          f"pessimistic {m['hover_rotor_hz_pessimistic']:.0f} Hz (sep {m['separation_pessimistic']*100:.0f} %)")
    print("FAIL" if r["failures"] else "PASS", *r["failures"], sep="\n  ")
    with open(os.path.join(ROOT, "out", "fea.json"), "w") as f:
        json.dump(r, f, indent=1)
    sys.exit(1 if r["failures"] else 0)


if __name__ == "__main__":
    main()
