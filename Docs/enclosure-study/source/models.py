"""Construct and validate three concept assemblies from the shared mechanical references."""

import itertools
import json

import numpy as np
from common import (
    archive,
    bbox,
    both,
    box,
    casepose,
    clean,
    cut,
    cyl,
    cylz,
    elems,
    join,
    move,
    readstep,
    rr,
    transform,
    vol,
)
from encoder import Y, front_details, reference, stock_collar, thumbwheel
from exporter import export_assembly
from lcd_window import fitted_window, verify_window
from OCC.Core.BRep import BRep_Builder, BRep_Tool
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.GProp import GProp_GProps
from OCC.Core.STEPControl import STEPControl_AsIs
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import TopoDS_Compound, topods
from paths import BOARD, OUTPUT, POLYCASE
from paths import WORK as W

out = OUTPUT / "models"
out.mkdir(parents=True, exist_ok=True)
raw = list(elems(readstep(POLYCASE), TopAbs_SOLID))
front0 = casepose(raw[0])
back0 = casepose(raw[2])
# The bare KiCad export and the case use different axes. Preserve the board seating datum.
board = transform(readstep(BOARD), 0, -1, 0, -96.186, -1, 0, 0, 143.94, 0, 0, -1, -0.449)
lx = 0.4
ly = -36.29
ss = [cyl(3.175, (-33, 36.94, 1.05), (1, 0, 0), 66), cyl(3.45, (0, -72, 2.20), (0, 1, 0), 9)]
anten = box(-9, 65, -4, 18, 27, 18)
bport = cylz(5, 51, -23, 3.15, 24)
cover_blank = cut(
    front0,
    *ss,
    anten,
    *[cylz(x, y, 4, 1.65, 12) for x, y in [(-18.196, -47.69), (18.2415, -47.71)]],
)
front = cut(
    cover_blank,
    cylz(0, 36.94, 5, 3.7, 12),
    rr(lx - 7, ly - 17.4, 5, 14, 34.8, 10, 0.9),
    rr(lx - 9, ly - 19.3, 10.4, 18, 38.6, 5, 1.4),
)
stockfront, stocklens, lens_tape = fitted_window(cut(cover_blank, cylz(0, 36.94, 4, 5.15, 12)), lx, ly)
front = front_details(front)
collar = stock_collar()
back = cut(back0, *ss, anten, bport)
quoted_shells = [
    (back, "KT-70XXT0 machined base", "45494C"),
    (stockfront, "Rev B cover - inside seat and beveled LCD viewing opening", "686F72"),
]
shell_step = out / "signalsnagger-rev04-kt70-wide-window-rfq.step"
export_assembly(shell_step, "Rev B - KT-70XXT0 machined shells", quoted_shells)
_, quoted_cover = elems(readstep(shell_step), TopAbs_SOLID)
verify_window(quoted_cover)
export_assembly(
    out / "signalsnagger-lcd-window-installed-rev-b.step",
    "Rev B - interior lens fitted to front cover",
    [quoted_shells[1], (stocklens, "Clear polycarbonate lens", "A5DFE7"), (lens_tape, "4 mm adhesive on lens viewing face", "CC975B")],
)
loop_ref = transform(archive("PartShape54"), -1, 0, 0, 80, 0, 1, 0, 80.3, 0, 0, -1, 12.4)
# Root cheeks replace the separate assembly clamp and leave the cable entry open.
root = rr(-25.3, 70.2, -9.30, 50.6, 26.2, 18.0, 3.0)
loop_common = cut(join(loop_ref, root), anten)
sense_bore = cyl(1.35, (0, 65, 9.6), (0, 1, 0), 190)
loop_common = cut(loop_common, sense_bore)


# Roll exposed edges on the pod and its thin removable lid.
def rim(s, r):
    f = BRepFilletAPI_MakeFillet(s)
    for e in elems(s, TopAbs_EDGE):
        b = bbox(e)
        if b[5] - b[2] < 0.0001:
            f.Add(r, topods.Edge(e))
    return f.Shape()


# A single tongue-and-screw lid opens to the rear of the receiver.
pod = cut(rim(rr(-38, 73, -31.5, 76, 46, 20, 4.8), 0.85), rr(-35.5, 75.5, -34, 71, 41, 19.8, 3.0))
screw_pt = (32.6, 95.8)
pod = join(pod, cylz(*screw_pt, -31.5, 4.4, 17.8))
pod = cut(pod, cylz(*screw_pt, -32.1, 1.85, 7.3))
for yy in (82, 103):
    pod = cut(pod, box(-37.1, yy - 0.22, -30.05, 3.0, 8.44, 1.95))
lid = cut(rim(rr(-38, 73, -33.7, 76, 46, 2.2, 4.8), 0.7), cylz(*screw_pt, -35, 1.45, 7))
for yy in (82, 103):
    lid = join(lid, box(-34.9, yy, -31.6, 1.5, 8, 3.13), box(-36.8, yy, -29.78, 3.3, 8, 1.31))
for yy in (76.0, 115.1):
    lid = join(lid, rr(-27, yy, -31.65, 49, 1.05, 1.6, 0.3))
# Broad bonding pad follows the rear mold surface. Narrow side returns skirt the screw heads.
pad = rr(-13.5, 38, -20.5, 27, 47, 12.65, 5)
shoulder = rr(-25, 65.9, -13.3, 50, 16.5, 5.0, 3)
returns = [rr(x, 58, -11, 3.0, 18.2, 7.5, 1.25) for x in (-25.4, 22.4)]
# Remove inner material of the returns while retaining the outer rear envelope.
guard = box(-22.8, 45, -7.9, 45.6, 25, 13)
deck = rr(-29, 70.25, -14.0, 58, 29, 4.7, 4)
baseblank = join(pod, pad, shoulder, deck, *returns)
baseblank = cut(baseblank, guard, bport, cyl(3, (5, 50.7, -16.6), (0, 1, 0), 29))
# Shape the returns to the exterior of the case. Sweep the molded part's outer wall
# clearance inward using a cutter for the inner side of each return.
inner_guards = [box(x, 57, -7.9, 3.8, 13, 8.5) for x in (-25, 21.2)]
baseblank = cut(baseblank, *inner_guards, rr(-33.7, 78.2, -29.7, 58.9, 34.5, 15.1, 2.7))
saddle = cut(baseblank, back0, front0)
# Separate-loop edition keeps two positive locating keys and two screw lands at the root.
keys = [rr(x, 76.5, -11.60, 4.5, 10.5, 2.5, 1) for x in (-22.8, 18.3)]
sockets = [rr(x - 0.22, 76.28, -11.83, 4.94, 10.94, 2.75, 1.1) for x in (-22.8, 18.3)]
root_holes = [cylz(x, 75.4, -12, 1.7, 30) for x in (-15, 15)]
loop_det = cut(join(loop_common, *keys), *root_holes)
mounts = [cylz(x, 75.4, -19, 4.1, 8.3) for x in (-15, 15)]
pilots = [cylz(x, 75.4, -16.8, 2.05, 8.2) for x in (-15, 15)]
sback = cut(
    join(back, baseblank, *mounts), box(-29.5, 70.14, -9.30, 59, 28.2, 32), *sockets, *pilots
)
integral = join(back, baseblank, loop_common)
carrier = join(saddle, loop_common)
# Study references. The exact bare PCB is distinguished from proxy controls and pack.
pack = rr(-32.2, 80, -26.5, 56, 31, 11, 2.1)
rod = cyl(1.24, (0, 67, 9.6), (0, 1, 0), 220)
lcd = rr(lx - 6.05, ly - 16.5, 7.15, 12.1, 33.0, 0.6, 0.5)
knob = thumbwheel()
body, shaft = reference()
lens = rr(lx - 8.8, ly - 19.1, 10.5, 17.6, 38.2, 0.95, 1.25)
pipes = [cylz(x, y, 4.0, 1.5, 7.85) for x, y in [(-18.196, -47.69), (18.2415, -47.71)]]
boots = [cyl(2.75, (side * 22.1, 36.94, 1.05), (side, 0, 0), 3.2) for side in (-1, 1)]
screw = join(cylz(*screw_pt, -34.9, 2.65, 1.2), cylz(*screw_pt, -33.75, 1.23, 7.0))


def compound(shapes):
    b = BRep_Builder()
    s = TopoDS_Compound()
    b.MakeCompound(s)
    for sh in shapes:
        b.Add(s, sh)
    return s


refs = {
    "pcb": board,
    "pack": pack,
    "rod": rod,
    "lens": lens,
    "stocklens": stocklens,
    "lensTape": lens_tape,
    "body": body,
    "shaft": shaft,
    "lcd": lcd,
    "pipes": compound(pipes),
    "boots": compound(boots),
    "lidScrew": screw,
}
refs["caseScrews"] = compound([casepose(s) for s in [raw[1], *raw[3:]]])
refs["loopScrews"] = compound(
    [join(cylz(x, 75.4, -15.4, 1.48, 27.85), cylz(x, 75.4, 12.41, 2.75, 2.2)) for x in (-15, 15)]
)
colors = {
    "body": "8B9BA6",
    "shaft": "B6B1A4",
    "collar": "B46F32",
    "caseScrews": "65747B",
    "loopScrews": "67747C",
    "one": "397F86",
    "rear": "397F86",
    "smallrear": "397F86",
    "front": "BCD2D8",
    "lid": "E99A47",
    "saddle": "397F86",
    "loop": "496E9D",
    "stockrear": "8C9298",
    "stockfront": "CED0D3",
    "pcb": "39774F",
    "pack": "7F7696",
    "rod": "BF9950",
    "knob": "333E46",
    "lens": "69858A",
    "stocklens": "69858A",
    "lensTape": "CC975B",
    "lcd": "526B61",
    "pipes": "BCA97D",
    "boots": "525E65",
    "lidScrew": "697781",
}
pieces = {
    "one": integral,
    "front": front,
    "lid": lid,
    "stockrear": back,
    "stockfront": stockfront,
    "collar": collar,
    "knob": knob,
    "saddle": carrier,
    "smallrear": sback,
    "loop": loop_det,
}
labels = {
    "caseScrews": "Reference - KT-70 cover screw",
    "loopScrews": "Reference - loop retaining screw",
    "one": "01 Printed rear - integral loop and battery bay",
    "front": "02 Printed front cover",
    "lid": "Battery lid - two tongues and one screw",
    "stockrear": "01 Machined KT-70 base",
    "stockfront": "02 Machined KT-70 front cover - Rev B beveled LCD aperture",
    "saddle": "03 Bonded printed loop and battery saddle",
    "smallrear": "01 Printed rear - battery bay and keyed socket",
    "loop": "02 Detachable printed loop",
    "pcb": "Reference - rev 0.4 PCB body",
    "pack": "Reference - 450 mAh pack envelope",
    "rod": "Reference - sense rod",
    "knob": "Printed 44 mm thumbwheel - D socket",
    "body": "Reference - SW203 fixed body and smooth collar",
    "shaft": "Reference - SW203 moving shaft",
    "collar": "Printed splash collar - bonded to molded front",
    "lcd": "Reference - LCD viewing area",
    "lens": "Reference - window lens",
    "stocklens": "Rev B polycarbonate lens - inside cover",
    "lensTape": "4 mm adhesive frame - viewing face of lens",
    "pipes": "Reference - two light pipes",
    "boots": "Reference - switch actuators",
    "lidScrew": "Reference - battery screw head",
}
designs = {
    "three-piece": ["one", "front", "lid", "knob"],
    "polycase-hybrid": ["saddle", "stockrear", "stockfront", "lid", "collar", "knob"],
    "detachable-loop": ["smallrear", "loop", "front", "lid", "knob"],
}
common_refs = [k for k in refs if k not in {"lens", "stocklens", "lensTape", "loopScrews"}]
design_refs = {
    "three-piece": common_refs + ["lens"],
    "polycase-hybrid": common_refs + ["stocklens", "lensTape"],
    "detachable-loop": common_refs + ["lens", "loopScrews"],
}


def export(file, keys):
    export_assembly(
        out / (file + ".step"),
        "SignalSnagger rev 0.4 - " + file,
        [(pieces.get(k, refs.get(k)), labels[k], colors[k]) for k in keys],
    )


def mesh(s):
    BRepMesh_IncrementalMesh(s, 0.11, False, 0.3, True)
    arr = []
    for face in elems(s, TopAbs_FACE):
        face = topods.Face(face)
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation(face, loc)
        if not tri:
            continue
        tr = loc.Transformation()
        vs = np.array([tri.Node(i).Transformed(tr).Coord() for i in range(1, tri.NbNodes() + 1)])
        ids = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]) - 1
        if face.Orientation() == TopAbs_REVERSED:
            ids = ids[:, [0, 2, 1]]
        arr.append(vs[ids])
    return np.concatenate(arr).astype("float32")


for k, s in list(pieces.items()):
    solids = list(elems(s, TopAbs_SOLID))
    print("CHECK", k, len(solids), [round(vol(i), 2) for i in solids], bbox(s), flush=True)
    assert len(solids) == 1, (k, len(solids))
    s = clean(solids[0])
    assert BRepCheck_Analyzer(s).IsValid(), k
    pieces[k] = s
for file, keys in designs.items():
    for a, b in itertools.combinations(keys + ["pcb", "pack"], 2):
        sa = pieces.get(a, refs.get(a))
        sb = pieces.get(b, refs.get(b))
        overlap = vol(both(sa, sb))
        assert overlap < 0.03, ("INTERFERENCE", file, a, b, overlap)
    # A translation of 0.8 mm cannot close a larger rest gap anywhere along the stroke.
    # The shaft moves with the wheel, so its socket fit is checked separately.
    fixed_parts = [pieces[k] for k in keys if k != "knob"] + [
        refs[name] for name in design_refs[file] if name != "shaft"
    ]
    rest_gap = min(BRepExtrema_DistShapeShape(knob, fixed).Value() for fixed in fixed_parts)
    assert rest_gap > 0.8, (file, "wheel travel", rest_gap)
    print(f"{file}: nearest stationary surface at rest {rest_gap:.3f} mm")
    assert vol(both(knob, shaft)) < 0.015, (file, "wheel socket")
    for k in keys:
        assert vol(both(pieces[k], body)) < 0.015, (file, k, "encoder body")
    export(file, keys + design_refs[file])
from OCC.Core.STEPControl import STEPControl_Writer

knobfile = out / "thumbwheel-44mm.step"
local = move(knob, 0, -Y, -13.8)
writer = STEPControl_Writer()
assert int(writer.Transfer(local, STEPControl_AsIs)) == 1 and int(writer.Write(str(knobfile))) == 1


def precise_volume(s):
    props = GProp_GProps()
    brepgprop.VolumeProperties(s, props, 1e-6)
    return props.Mass()


check = readstep(knobfile)
assert (
    len(list(elems(check, TopAbs_SOLID))) == 1
    and BRepCheck_Analyzer(check).IsValid()
    and abs(precise_volume(check) - precise_volume(local)) < 0.001
)
# Use a physical cut through the assembly so the brief shows the true section and turned-down rim.
for key, s in {
    "front": front,
    "stockfront": stockfront,
    "knob": knob,
    "collar": collar,
    "body": body,
    "shaft": shaft,
    "pcb": board,
}.items():
    np.save(W / ("section-" + key + ".npy"), mesh(both(s, box(-26, Y, -9, 52, 0.30, 43))))
for key, s in {**pieces, **refs}.items():
    np.save(W / (key + ".npy"), mesh(s))
contact = move(both(back, move(saddle, 0, 0, 0.12)), 0, 0, -0.15)
np.save(W / "contact.npy", mesh(contact))
(W / "scene.json").write_text(
    json.dumps({"colors": colors, "labels": labels, "designs": designs, "refs": design_refs})
)

# Leave the antenna-end screw towers on the coupon so the existing rear establishes the
# board-to-front height. The transverse cut also provides a flat face for the print bed.
cut_y = 8.0
xmin, _, zmin, xmax, ymax, zmax = bbox(front)
coupon = both(
    front, box(xmin - 1, cut_y, zmin - 1, xmax - xmin + 2, ymax - cut_y + 1, zmax - zmin + 2)
)
coupon_on_cut = transform(coupon, 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0)
# Put the crown on the bed to leave the blind D-socket open from above.
wheel_on_crown = transform(knob, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0)
for name, shape in [
    ("rev04-recessed-encoder-section", coupon_on_cut),
    ("thumbwheel-44mm", wheel_on_crown),
]:
    assert len(list(elems(shape, TopAbs_SOLID))) == 1 and BRepCheck_Analyzer(shape).IsValid(), name
    xmin, ymin, zmin, *_ = bbox(shape)
    printable = move(shape, -xmin, -ymin, -zmin)
    BRepMesh_IncrementalMesh(printable, 0.02, False, 0.09, True)
    stl = StlAPI_Writer()
    stl.SetASCIIMode(False)
    path = out / (name + ".stl")
    if not stl.Write(printable, str(path)):
        raise OSError(f"Could not write {path}")
