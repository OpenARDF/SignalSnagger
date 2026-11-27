"""Printed wheel, encoder clearance references, recessed front and bonded collar."""

import math

from common import both, box, clean, cut, cylz, join, rr
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeRevol
from OCC.Core.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Vec

Y = 36.94
board_z = -0.449
skin = 11.6332


def face(points):
    p = BRepBuilderAPI_MakePolygon()
    for pt in points:
        p.Add(gp_Pnt(*pt))
    p.Close()
    return BRepBuilderAPI_MakeFace(p.Wire()).Face()


def radial(profile):
    return BRepPrimAPI_MakeRevol(
        face([(r, Y, z) for r, z in profile]),
        gp_Ax1(gp_Pnt(0, Y, 0), gp_Dir(0, 0, 1)),
        2 * math.pi,
    ).Shape()


def drain_profile(sign, low, high, width):
    # A continuous floor falls toward the outside of the grip. The side port stays above the button.
    points = [
        (sign * x, Y - width / 2, z) for x, z in [(8, low), (27, low - 1.0), (27, high), (8, high)]
    ]
    return BRepPrimAPI_MakePrism(face(points), gp_Vec(0, width, 0)).Shape()


def front_details(front):
    bowl = radial([(5.15, 6.40), (12.0, 6.40), (13.0, 7.40), (13.0, 10.6), (5.15, 10.6)])
    front = join(front, bowl)
    for sign in (-1, 1):
        rib = both(drain_profile(sign, 6.4, 9.65, 5.6), box(-23, Y - 5, 3, 46, 10, 10))
        front = join(front, rib)
    front = cut(front, cylz(0, Y, 8.2, 11.3, 8), cylz(0, Y, 4, 5.15, 10))
    for sign in (-1, 1):
        front = cut(front, drain_profile(sign, 8.20, 24, 2.8))
    # Raised central rim shields the running clearance. Its bore steps in above the fixed nose.
    island = radial(
        [
            (5.15, 8.18),
            (7.15, 8.18),
            (7.15, 9.5),
            (6.65, 10.05),
            (6.65, 15.30),
            (6.45, 15.50),
            (4.0, 15.50),
            (4.0, 11.25),
            (5.15, 11.25),
        ]
    )
    return clean(join(front, island))


def stock_collar():
    return radial(
        [
            (4.0, skin),
            (10.65, skin),
            (10.65, 12.0),
            (10.15, 12.40),
            (7.1, 12.40),
            (6.65, 12.85),
            (6.65, 15.30),
            (6.45, 15.50),
            (4.0, 15.50),
        ]
    )


def d_shape(z, r, flat, h):
    return cut(cylz(0, Y, z, r, h), box(flat, Y - r - 1, z - 1, 2 * r, 2 * r + 2, h + 2))


def reference():
    body = rr(-6.2, Y - 6.7, board_z, 12.4, 13.4, 6.1, 0.45)
    body = join(body, cylz(0, Y, board_z + 6.08, 4.25, 5.02))
    shaft = join(cylz(0, Y, board_z + 11.1, 3, 1.90), d_shape(board_z + 13, 3, 1.5, 12))
    return clean(body), clean(shaft)


def thumbwheel():
    # The shallow outer rim is reachable with the hand wrapped around the case. The central crown clears the full-length shaft.
    cap = radial(
        [
            (0, 16.85),
            (5.05, 16.85),
            (5.05, 21.85),
            (7.5, 21.85),
            (7.5, 13.80),
            (8.65, 13.80),
            (8.65, 21.60),
            (19.85, 15.65),
            (20.50, 14.15),
            (21.55, 14.15),
            (22, 14.6),
            (22, 16.75),
            (21.4, 17.40),
            (9.0, 24.15),
            (5.35, 26.2),
            (4.7, 26.4),
            (0, 26.4),
        ]
    )
    # An internal stop sits at the nominal shaft end. Calibrate a retaining fit with a socket coupon.
    cap = cut(cap, d_shape(13.8, 3.04, 1.54, board_z + 25 - 13.8))
    for i in range(40):
        a = 2 * math.pi * i / 40
        cap = cut(cap, cylz(22.30 * math.cos(a), Y + 22.30 * math.sin(a), 13.9, 0.79, 4.6))
    return clean(cap)
