"""SSG-LCD-04 Rev B, in millimeters in the assembled KT-70 coordinate system."""

from common import bbox, both, box, clean, cut, elems, rr
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.Core.BRepTools import breptools
from OCC.Core.GProp import GProp_GProps
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopoDS import topods


def bottom_face(solid, z):
    for face in elems(solid, TopAbs_FACE):
        bounds = bbox(face)
        if abs(bounds[2] - z) < 1e-5 and abs(bounds[5] - z) < 1e-5:
            return topods.Face(face)
    raise ValueError("Missing horizontal LCD window profile")


def fitted_window(cover, x, y):
    inside, outside = 9.0424, 11.6332  # Molded faces at the window, measured from the case seam.
    seat = inside + 0.60
    tape_thickness, lens_thickness = 0.13, 0.95

    def profile(width, length, radius, z, height):
        return rr(x - width / 2, y - length / 2, z, width, length, height, radius)

    def bevel_wire(z, offset):
        # Offsetting both the sides and the corner arcs gives a 45° wall all around.
        solid = profile(14.00 + 2 * offset, 34.80 + 2 * offset, 0.90 + offset, z, 1)
        return breptools.OuterWire(bottom_face(solid, z))

    # Extend the cutters beyond the molded surfaces so the lip ends at the seat plane.
    bevel = BRepOffsetAPI_ThruSections(True, True)
    bevel.AddWire(bevel_wire(seat - 0.15, 0))
    bevel.AddWire(bevel_wire(seat, 0))
    bevel.AddWire(bevel_wire(outside + 1, outside + 1 - seat))
    bevel.Build()
    assert bevel.IsDone()
    pocket = profile(23.80, 44.60, 5.20, inside - 1, seat - inside + 1)
    machined_cover = clean(cut(cover, pocket, bevel.Shape()))

    tape_z = seat - tape_thickness
    lens = profile(23.40, 44.20, 5.00, tape_z - lens_thickness, lens_thickness)
    adhesive = cut(
        profile(23.00, 43.80, 4.80, tape_z, tape_thickness),
        profile(15.00, 35.80, 0.80, tape_z, tape_thickness),
    )
    return machined_cover, lens, adhesive


def verify_window(cover):
    """Compare horizontal sections of the delivered shell to the Rev B drawing."""
    x, y = 0.40, -36.29
    seat_z = 9.0424 + 0.60

    def area(shape):
        props = GProp_GProps()
        brepgprop.SurfaceProperties(shape, props)
        return props.Mass()

    # Sampling on both sides of the seat detects a missing or misplaced pocket.
    # The throat, mid-bevel and mouth sections check the taper around all four corners.
    for height in (-0.59, -0.01, 0.01, 1.00, 1.98):
        z = seat_z + height
        if height < 0:
            width, length, radius = 23.80, 44.60, 5.20
            feature = "inside lens recess"
        else:
            width, length, radius = 14.00 + 2 * height, 34.80 + 2 * height, 0.90 + height
            feature = "45-degree exterior bevel"
        expected = bottom_face(rr(x - width / 2, y - length / 2, z, width, length, 1, radius), z)
        # A narrow band outside the profile catches oversize cuts without sampling adjacent molded features.
        crop = bottom_face(box(x - width / 2 - 0.3, y - length / 2 - 0.3, z, width + 0.6, length + 0.6, 1), z)
        opening = cut(crop, cover)
        difference = area(opening) + area(expected) - 2 * area(both(opening, expected))
        if abs(difference) > 0.001:  # Numerical area tolerance in mm² for nominal CAD sections.
            raise ValueError(f"SSG-LCD-04 Rev B: {feature} differs at Z {z:.4f} mm")
