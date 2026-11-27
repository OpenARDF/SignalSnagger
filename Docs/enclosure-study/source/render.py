import ctypes as ct
import json

import numpy as np
from paths import WORK as wdir
from PIL import Image

sc = json.loads((wdir / "scene.json").read_text())
meta = {}
meshes = {k: np.load(wdir / (k + ".npy")) for k in sc["labels"]}
meshes["contact"] = np.load(wdir / "contact.npy")
sc["colors"]["contact"] = "E49B48"
cdll = ct.CDLL(str(wdir / "raster.so"))


def unit(v):
    return np.array(v) / np.linalg.norm(v)


def render(
    name, keys, eye=(0.48, -0.45, 1), offsets=None, up=(0, 1, 0), crop=None, sizes=(1100, 1100)
):
    offsets = offsets or {}
    cam = unit(eye)
    xx = unit(np.cross(up, cam))
    yy = unit(np.cross(cam, xx))
    rot = np.stack([xx, -yy, cam], axis=1)
    world = np.concatenate([meshes[k] + np.array(offsets.get(k, (0, 0, 0))) for k in keys])
    objids = np.concatenate(
        [np.full(len(meshes[k]), i + 1, dtype=np.int16) for i, k in enumerate(keys)]
    )
    normal = np.cross(world[:, 1] - world[:, 0], world[:, 2] - world[:, 0])
    normal = normal / np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-9)
    V = world @ rot
    if crop:
        mn, mx = np.array(crop[:2]), np.array(crop[2:])
    else:
        mn = np.min(V[:, :, :2], axis=(0, 1))
        mx = np.max(V[:, :, :2], axis=(0, 1))
    sz = mx - mn
    pad = 12.0
    scale = min((sizes[0] - 2 * pad) / sz[0], (sizes[1] - 2 * pad) / sz[1])
    w, h = np.ceil(sz * scale + 2 * pad).astype("int")
    V[:, :, :2] = (V[:, :, :2] - mn) * scale + pad
    light = unit(cam * 0.78 - xx * 0.45 + yy * 0.7)
    fill = unit(cam * 0.1 + xx * 0.9 + yy * 0.4)
    f = 0.51 + 0.56 * np.maximum(0, normal @ light) + 0.11 * np.maximum(0, normal @ fill)
    colors = np.array([[int(sc["colors"][k][i : i + 2], 16) for i in (0, 2, 4)] for k in keys])
    C = np.clip((colors[objids - 1] * f[:, None]), 0, 255).astype(np.uint8)
    pix = np.full((h, w, 3), 255, np.uint8)
    buf = np.full((h, w), -1e8, np.float32)
    ids = np.zeros((h, w), np.int16)
    norm = np.zeros((h, w, 3), np.float32)
    args = [
        np.ascontiguousarray(a)
        for a in (V.astype(np.float32), C, pix, buf, ids, objids, norm, normal.astype(np.float32))
    ]
    cdll.draw(int(w), int(h), len(world), *[ct.c_void_p(a.ctypes.data) for a in args])
    # Slightly darken silhouettes and crease lines, keeping planar triangle edges invisible.
    edge = np.zeros((h, w), bool)
    for axis in (0, 1):
        jj = np.roll(ids, 1, axis=axis)
        nn = np.roll(norm, 1, axis=axis)
        z = np.roll(buf, 1, axis=axis)
        edge |= (ids != jj) | (
            (ids > 0) & (jj > 0) & ((np.sum(norm * nn, axis=2) < 0.80) | (abs(buf - z) > 2.8))
        )
    edge &= ids > 0
    pix[edge] = (pix[edge] * 0.69).astype("uint8")
    Image.fromarray(pix).save(wdir / (name + ".png"))
    meta[name] = {
        "w": int(w),
        "h": int(h),
        "scale": scale,
        "mn": mn.tolist(),
        "pad": pad,
        "rot": rot.tolist(),
    }


def closeview(name, keys, center, span, eye, offs=None):
    cam = unit(eye)
    xx = unit(np.cross((0, 1, 0), cam))
    yy = unit(np.cross(cam, xx))
    u, v = np.asarray(center) @ np.stack([xx, -yy], axis=1)
    render(
        name,
        keys,
        eye,
        offs,
        crop=(u - span[0] / 2, v - span[1] / 2, u + span[0] / 2, v + span[1] / 2),
        sizes=(1400, 1050),
    )


allrefs = [
    "pcb",
    "pack",
    "rod",
    "body",
    "shaft",
    "lens",
    "pipes",
    "boots",
    "lidScrew",
    "caseScrews",
]
for file, struct in sc["designs"].items():
    keys = struct + allrefs + (["loopScrews"] if file == "detachable-loop" else [])
    render(file + "-front", keys, (0.88, -0.48, 1.36), sizes=(1200, 1450))
    render(file + "-rear", keys, (-0.83, -0.49, -1.45), sizes=(1150, 1450))
for k in ["one", "front", "lid", "stockrear", "stockfront", "saddle", "smallrear", "loop"]:
    eye = {
        "lid": (0.54, 0.24, 1),
        "front": (0.42, -0.32, -1.25),
        "stockfront": (0.42, -0.32, -1.25),
        "saddle": (0.48, 0.40, -1.18),
        "stockrear": (0.55, -0.35, 1.3),
    }.get(k, (0.66, -0.40, 1.26))
    render("part-" + k, [k], eye, sizes=(780, 880))
# In each assembly the views use different part positions only to reveal the joints.
render(
    "explode-three-piece",
    ["one", "front", "lid", "pcb", "pack"],
    (0.93, -0.45, -0.8),
    {"front": (-14, -26, 104), "pcb": (-6, -12, 58), "pack": (0, 0, -39), "lid": (8, 0, -88)},
    sizes=(1650, 1370),
)
render(
    "explode-polycase-hybrid",
    ["saddle", "stockrear", "stockfront", "lid", "pcb", "pack"],
    (0.93, -0.40, -0.82),
    {
        "stockrear": (-4, -23, 46),
        "stockfront": (-18, -50, 149),
        "pcb": (-11, -39, 95),
        "pack": (0, 0, -34),
        "lid": (8, 0, -77),
    },
    sizes=(1650, 1370),
)
render(
    "explode-detachable-loop",
    ["smallrear", "loop", "front", "lid", "pcb", "pack"],
    (0.92, -0.43, -0.8),
    {
        "loop": (0, 48, 14),
        "front": (-14, -26, 102),
        "pcb": (-5, -11, 54),
        "pack": (0, 0, -34),
        "lid": (8, 0, -76),
    },
    sizes=(1650, 1370),
)
closeview("open-bay", ["one", "pack"], (0, 94, -23), (106, 72), (0.28, 0.30, -1.6))
closeview("empty-bay", ["one"], (0, 94, -23), (106, 72), (0.28, 0.30, -1.6))
closeview("tongues", ["lid"], (0, 96, -31), (93, 54), (0.5, 0.7, 1.45))
closeview(
    "saddle-bond",
    ["saddle", "stockrear"],
    (0, 67, -8),
    (120, 105),
    (0.76, -0.9, 0.72),
    {"stockrear": (-3, -20, 23)},
)
closeview(
    "key-joint",
    ["smallrear", "loop", "loopScrews"],
    (0, 79, -1),
    (100, 89),
    (0.75, -0.82, 0.75),
    {"loop": (0, 12, 22), "loopScrews": (0, 12, 40)},
)
closeview(
    "controls",
    ["front", "body", "shaft", "knob", "boots", "lens", "pipes"],
    (0, -0.5, 6),
    (86, 152),
    (0.42, -0.25, 1.7),
)
closeview("wire-port-small", ["smallrear"], (0, 57, -3), (85, 89), (0.55, -0.65, 1.45))
closeview("wire-port", ["one"], (0, 57, -3), (85, 89), (0.55, -0.65, 1.45))
closeview("contact-map", ["stockrear", "contact"], (0, 45, -8), (69, 76), (0.34, 0.10, -1.9))
for part in ["front", "stockfront", "knob", "collar", "body", "shaft", "pcb"]:
    k = "section-" + part
    meshes[k] = np.load(wdir / (k + ".npy"))
    sc["colors"][k] = sc["colors"][part]
for tech in ["printed", "hybrid"]:
    keys = ["front"] if tech == "printed" else ["stockfront", "collar"]
    render(
        "reach-" + tech,
        keys + ["body", "shaft", "knob", "boots"],
        eye=(0, 0, 1),
        crop=(-31, -62, 31, -12),
        sizes=(800, 660),
    )
    section = ["section-" + k for k in keys + ["knob", "body", "shaft", "pcb"]]
    render(
        "section-" + tech,
        section,
        eye=(0, -1, 0),
        up=(0, 0, 1),
        crop=(-26.8, -28.4, 26.8, 2.8),
        sizes=(1550, 920),
    )
    # Oblique view with the wheel removed shows the paths hidden in the assembled receiver.
    closeview(
        "seat-" + tech,
        keys + ["pcb", "body", "shaft"],
        (0, 36.94, 8),
        (59, 36),
        (0.22, -1.05, 1.6),
    )
(wdir / "projections.json").write_text(json.dumps(meta))
