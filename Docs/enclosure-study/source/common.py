"""Open CASCADE operations shared by the enclosure study."""

import zipfile

from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepFilletAPI import BRepFilletAPI_MakeFillet
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCC.Core.BRepTools import breptools
from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf
from OCC.Core.GProp import GProp_GProps
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Shape, topods
from paths import LOOP
from paths import WORK as W


def transform(s, *rows):
    tr = gp_Trsf()
    tr.SetValues(*rows)
    return BRepBuilderAPI_Transform(s, tr, True).Shape()


def move(s, x, y, z):
    return transform(s, 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z)


def elems(s, t):
    e = TopExp_Explorer(s, t)
    while e.More():
        yield e.Current()
        e.Next()


def bbox(s):
    bb = Bnd_Box()
    brepbndlib.AddOptimal(s, bb, False, False)
    return tuple(round(x, 5) for x in bb.Get())


def vol(s):
    pr = GProp_GProps()
    brepgprop.VolumeProperties(s, pr)
    return pr.Mass()


def readstep(p):
    r = STEPControl_Reader()
    assert int(r.ReadFile(str(p))) == 1
    r.TransferRoots()
    return r.OneShape()


def box(x, y, z, w, d, h):
    return BRepPrimAPI_MakeBox(gp_Pnt(x, y, z), w, d, h).Shape()


def cyl(r, p, d, h):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*p), gp_Dir(*d)), r, h).Shape()


def cylz(x, y, z, r, h):
    return cyl(r, (x, y, z), (0, 0, 1), h)


def boolean(c, s, t):
    b = c(s, t)
    b.SetFuzzyValue(0.00001)
    b.Build()
    assert b.IsDone()
    return b.Shape()


def join(s, *other):
    for t in other:
        s = boolean(BRepAlgoAPI_Fuse, s, t)
    return s


def cut(s, *other):
    for t in other:
        s = boolean(BRepAlgoAPI_Cut, s, t)
    return s


def both(s, t):
    return boolean(BRepAlgoAPI_Common, s, t)


def rr(x, y, z, w, d, h, r):
    s = box(x, y, z, w, d, h)
    f = BRepFilletAPI_MakeFillet(s)
    for e in elems(s, TopAbs_EDGE):
        b = bbox(e)
        if b[5] - b[2] > h - 0.0001:
            f.Add(r, topods.Edge(e))
    return f.Shape()


def clean(s):
    u = ShapeUpgrade_UnifySameDomain(s, True, True, False)
    u.Build()
    return u.Shape()


def archive(n):
    with zipfile.ZipFile(LOOP) as zz:
        p = W / (n + ".brp")
        p.write_bytes(zz.read(n + ".brp"))
        s = TopoDS_Shape()
        assert breptools.Read(s, str(p), BRep_Builder())
        return s


def casepose(s):
    return transform(s, 0, 0, -1, 0, -1, 0, 0, 0, 0, 1, 0, 0)
