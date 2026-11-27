"""Write AP214 assemblies with individually named, colored solids and check the reimport."""

import re

from common import bbox, elems, readstep, vol
from OCC.Core.BRep import BRep_Builder
from OCC.Core.Interface import Interface_Static
from OCC.Core.STEPControl import STEPControl_AsIs, STEPControl_Writer
from OCC.Core.TopAbs import TopAbs_SOLID
from OCC.Core.TopoDS import TopoDS_Compound


def export_assembly(path, title, items):
    assembly = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(assembly)
    flat = []
    for shape, label, color in items:
        solids = list(elems(shape, TopAbs_SOLID))
        for i, s in enumerate(solids):
            builder.Add(assembly, s)
            flat.append((s, label + (" " + str(i + 1) if len(solids) > 1 else ""), color))
    writer = STEPControl_Writer()
    Interface_Static.SetIVal("write.step.assembly", 1)
    assert (
        int(writer.Transfer(assembly, STEPControl_AsIs)) == 1 and int(writer.Write(str(path))) == 1
    )
    data = path.read_text()
    # Map each writer-assigned product to its solid before attaching names and display styles.
    rootnum = re.search(r"Open CASCADE STEP translator [0-9.]+ (\d+)", data)[1]
    records = {int(k): v for k, v in re.findall(r"#(\d+)\s*=\s*(.*?);", data, re.DOTALL)}

    def ids(st):
        return list(map(int, re.findall(r"#(\d+)", st)))

    extra = []

    def new(s):
        n = maxid + len(extra) + 1
        extra.append(f"#{n} = {s};")
        return n

    maxid = max(records)
    count = 0
    for rec in records.values():
        if not rec.startswith("SHAPE_DEFINITION_REPRESENTATION("):
            continue
        pd, rep = ids(rec)
        rr = records[rep]
        proddef = ids(records[pd])[-1]
        form = ids(records[proddef])[0]
        prod = records[ids(records[form])[-1]]
        pro = re.search(rf"translator [0-9.]+ {rootnum}(?:\.(\d+))?'", prod)
        if not pro or not pro[1]:
            continue
        index = int(pro[1]) - 1
        s, name, color = flat[index]
        solid = [
            i
            for i in ids(rr)[:-1]
            if records[i].startswith(("MANIFOLD_SOLID_BREP(", "BREP_WITH_VOIDS("))
        ]
        assert len(solid) == 1, (name, rr)
        count += 1
        rgb = ",".join(f"{int(color[i : i + 2], 16) / 255:.6f}" for i in (0, 2, 4))
        co = new(f"COLOUR_RGB('',{rgb})")
        fill = new(f"FILL_AREA_STYLE_COLOUR('',#{co})")
        fa = new(f"FILL_AREA_STYLE('',(#{fill}))")
        ss = new(f"SURFACE_STYLE_FILL_AREA(#{fa})")
        sd = new(f"SURFACE_SIDE_STYLE('',(#{ss}))")
        su = new(f"SURFACE_STYLE_USAGE(.BOTH.,#{sd})")
        ps = new(f"PRESENTATION_STYLE_ASSIGNMENT((#{su}))")
        si = new(f"STYLED_ITEM('',(#{ps}),#{solid[0]})")
        new(f"MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(#{si}),#{ids(rr)[-1]})")
    assert count == len(flat)

    def named(m):
        num = m.group(1)
        return flat[int(num) - 1][1] if num else title

    data = re.sub(rf"Open CASCADE STEP translator [0-9.]+ {rootnum}(?:\.(\d+))?", named, data)
    data = data.replace(
        "ENDSEC;\nEND-ISO-10303-21;", "\n".join(extra) + "\nENDSEC;\nEND-ISO-10303-21;"
    )
    path.write_text(data)
    result = list(elems(readstep(path), TopAbs_SOLID))
    assert len(result) == len(flat)
    for k, (rr, (orig, name, _)) in enumerate(zip(result, flat)):
        assert abs(vol(rr) - vol(orig)) < 0.1, (name, vol(rr), vol(orig))
        assert all(abs(a - b) < 0.01 for a, b in zip(bbox(rr), bbox(orig))), (name, k)
    print("STEP verified", path.name, count, flush=True)
