import json
from pathlib import Path

from paths import OUTPUT
from paths import WORK as w
from PIL import Image
from reportlab.lib.colors import HexColor
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas
from reportlab.platypus import Paragraph

out = OUTPUT / "pdf"
out.mkdir(parents=True, exist_ok=True)
# Use metric-compatible Helvetica when desktop Arial is unavailable.
font_dir = Path("/System/Library/Fonts/Supplemental")
for filename, alias, fallback in [
    ("Arial.ttf", "A", "Helvetica"),
    ("Arial Bold.ttf", "B", "Helvetica-Bold"),
]:
    if (font_dir / filename).is_file():
        pdfmetrics.registerFont(TTFont(alias, str(font_dir / filename)))
    else:
        pdfmetrics.registerFont(pdfmetrics.Font(alias, fallback, "WinAnsiEncoding"))
pdfmetrics.registerFontFamily("A", normal="A", bold="B")
ink = "#183238"
mut = "#476069"
teal = "#086C67"
line = "#CCDBD8"
pale = "#EFF5F3"
gold = "#B36928"
blue = "#48658A"
scene = json.loads((w / "scene.json").read_text())
proj = json.loads((w / "projections.json").read_text())
dims = (792, 612)
M = 35
maxX = 757


def col(s):
    return HexColor(s)


def txt(x, y, t, size=11, font="A", color=ink):
    C.setFont(font, size)
    C.setFillColor(col(color))
    C.drawString(x, y, t)


def mid(x, y, t, size=11, font="A", color=ink):
    C.setFont(font, size)
    C.setFillColor(col(color))
    C.drawCentredString(x, y, t)


def rule(x, y, xx, yy, k=line, lw=0.7):
    C.setStrokeColor(col(k))
    C.setLineWidth(lw)
    C.line(x, y, xx, yy)


def rectangle(x, y, ww, h, c, rad=0):
    C.setFillColor(col(c))
    C.roundRect(x, y, ww, h, rad, fill=1, stroke=0)


def textblock(x, y, ww, t, size=11.0, lead=15.3, color=ink):
    o = Paragraph(
        t,
        ParagraphStyle(
            "p", fontName="A", fontSize=size, leading=lead, textColor=col(color), spaceAfter=0
        ),
    )
    _, h = o.wrap(ww, 900)
    o.drawOn(C, x, y - h)
    return y - h


def image(n, x, y, ww, h):
    p = w / (n + ".png")
    iw, ih = Image.open(p).size
    sc = min(ww / iw, h / ih)
    ww2 = iw * sc
    h2 = ih * sc
    bx = x + (ww - ww2) / 2
    by = y + (h - h2) / 2
    C.drawImage(str(p), bx, by, ww2, h2, mask="auto")
    return bx, by, ww2, h2


def source(t, url):
    return '<link href="' + url + '" color="' + teal + '"><u>' + t + "</u></link>"


def header(name, section, title, subtitle):
    txt(M, 581, "SignalSnagger  /  PCB rev 0.4", 10.3, "B", teal)
    C.setFillColor(col(mut))
    C.setFont("A", 9.5)
    C.drawRightString(maxX, 581, section)
    rule(M, 569, maxX, 569)
    txt(M, 536, title, 26, "B")
    textblock(M, 522, 720, subtitle, 11.7, 16, mut)


def foot(n, study):
    rule(M, 31, maxX, 31)
    txt(M, 17, study + "    /    Mechanical concept, November 2026", 8.3, "A", mut)
    C.setFont("A", 8.3)
    C.drawRightString(maxX, 17, str(n) + " / 4")


def pill(x, y, num):
    C.setFillColor(col("#193F43"))
    C.setStrokeColor(col("#FFFFFF"))
    C.setLineWidth(1.3)
    C.circle(x, y, 8.5, stroke=1, fill=1)
    mid(x, y - 3.3, str(num), 9.7, "B", "#FFFFFF")


def mark(n, box, xyz, num, delta):
    pr = proj[n]
    bx, by, pw, ph = box
    rot = pr["rot"]
    px = sum(xyz[i] * rot[i][0] for i in range(3))
    py = sum(xyz[i] * rot[i][1] for i in range(3))
    x = bx + (pr["pad"] + (px - pr["mn"][0]) * pr["scale"]) * pw / pr["w"]
    y = by + ph - (pr["pad"] + (py - pr["mn"][1]) * pr["scale"]) * ph / pr["h"]
    ex = x + delta[0]
    ey = y + delta[1]
    rule(x, y, ex, ey, "#FFFFFF", 2.7)
    rule(x, y, ex, ey, "#254D52", 0.9)
    C.setFillColor(col("#254D52"))
    C.circle(x, y, 1.8, stroke=0, fill=1)
    pill(ex, ey, num)


def tag(n, photo, pt, x, y, label):
    pr = proj[n]
    bx, by, pw, ph = photo
    u = sum(pt[i] * pr["rot"][i][0] for i in range(3))
    v = sum(pt[i] * pr["rot"][i][1] for i in range(3))
    u = bx + (pr["pad"] + (u - pr["mn"][0]) * pr["scale"]) * pw / pr["w"]
    v = by + ph - (pr["pad"] + (v - pr["mn"][1]) * pr["scale"]) * ph / pr["h"]
    tw = pdfmetrics.stringWidth(label, "B", 8.4) + 10
    tx = x + tw / 2
    ty = y + 4
    rule(u, v, tx, ty, "#FFFFFF", 2.8)
    rule(u, v, tx, ty, "#31535B", 1)
    C.setFillColor(col("#FFFFFF"))
    C.setStrokeColor(col("#567176"))
    C.circle(u, v, 2.1, fill=1, stroke=1)
    rectangle(x, y - 3, tw, 13, "#EFF5F3", 3)
    txt(x + 5, y + 0.4, label, 8.4, "B", "#233E43")


def legendchip(x, y, color, label):
    rectangle(x, y - 2, 10, 10, "#" + color, 2)
    txt(x + 15, y, label, 9.1, "A", mut)


docs = {
    "three-piece": {
        "file": "signalsnagger-enclosure-feasibility",
        "name": "Three-piece printed receiver",
        "rows": [
            (
                "one",
                "Integrated rear",
                "The ring and grip share one continuous body. The battery pocket is on the far side of the shoulder.",
            ),
            (
                "front",
                "Removable front",
                "The window, encoder and light pipes register to this cover. The opposite face shows the PCB capture towers.",
            ),
            (
                "lid",
                "Battery lid",
                "The two tabs engage first. The single screw closes the opposite edge.",
            ),
        ],
        "pts": [((69, 178, 1), (-22, 9)), ((-15, -63, 112), (-17, 0)), ((21, 94, -120), (16, 10))],
        "detail": "Access to the board and battery",
        "intro": "Follow the wiring from each compartment to the PCB before closing the front.",
        "cards": [
            (
                "controls",
                "Rev 0.4 control face",
                "SW201 and SW202 sit on opposed sides at the encoder station. The front has the drained encoder recess, stepped LCD lens seat and two light-pipe bores. The headphone jack exits through the bottom seam.",
            ),
            (
                "wire-port",
                "Two wiring passages",
                "The top-edge notch accepts the antenna leads. The round hole in the rear floor enters a separate channel to the battery. Check connector passage and strain relief with the terminated harness.",
            ),
            (
                "tongues",
                "Single-screw lid",
                "Two broad tongues on the underside slide into pockets in the bay wall. Close the opposite edge onto its screw boss. The Ø2.9 mm lid hole clears an M2.5 screw. The boss sits beside the cell. Fit a captive retainer for production.",
            ),
        ],
        "next": "Print a two-corner section of the shell and the side-switch and jack sections first. Then slice the complete rear with the pocket and antenna grooves visible. Check support removal, neck stiffness and warping before printing a full unit.",
        "printer": "The rear measures <b>160 × 310.2 × 43.9 mm</b>. Rotated flat at 45°, it spans about <b>291.5 × 291.5 mm</b>. Allow roughly 302 × 302 mm of usable bed for a 5 mm brim.",
    },
    "polycase-hybrid": {
        "file": "signalsnagger-polycase-hybrid",
        "name": "Machined Polycase with printed saddle",
        "rows": [
            (
                "saddle",
                "Printed loop and saddle",
                "This complete part carries the battery pocket and bonds to the outside of the base.",
            ),
            (
                "stockrear",
                "Machined KT-70 base",
                "The molded grip carries the PCB. Factory #4-24 × 5/8 in screws enter from the back.",
            ),
            (
                "stockfront",
                "Machined KT-70 front",
                "The cover requires a 45° exterior LCD bevel and an inside lens recess. A 4 mm adhesive border bonds the lens to the seat.",
            ),
            (
                "lid",
                "Printed battery lid",
                "The lid comes off independently of the factory case screws.",
            ),
        ],
        "pts": [
            ((70, 179, 0), (-19, 8)),
            ((4, -66, 38), (-17, 3)),
            ((-18, -89, 161), (-14, -2)),
            ((21, 94, -108), (18, 8)),
        ],
        "detail": "Saddle bond and LCD window",
        "intro": "The saddle bonds to the rear half. Every custom KT-70 front cover requires the exterior LCD bevel and the inside lens recess shown below.",
        "cards": [
            (
                "contact-map",
                "Bond footprint on the base",
                "Orange marks the CAD contact patch on the back of the molded base. The central opening is the battery lead passage. The two upper factory screw recesses remain accessible.",
            ),
            (
                "saddle-bond",
                "Shoulder and side returns",
                "The saddle bears against the upper end and wraps the rear corners. Dry-fit both halves and check the seam. Machine the antenna passage through the upper end and the separate battery passage through the rear.",
            ),
            (
                "hybrid-window",
                "Required LCD bevel and recess",
                "Carry the 45° bevel through all four corners to the 34.80 × 14.00 mm throat. Machine the 44.60 × 23.80 mm recess 0.60 mm deep from inside. Bond the lens to its flat seat with the continuous 4 mm border. See machining drawing sheet 2.",
            ),
        ],
        "next": "Start with a short saddle sample and a spare molded base. For a solvent-weld trial, use ABS filament and an ABS-to-ABS cement. Test adhesion to the molded surface and apply a sideways load to the cured sample before bonding a complete loop.",
        "printer": "The saddle measures <b>160 × 202.3 × 43.9 mm</b>. A 5 mm brim gives a 170 × 212.3 mm footprint. It fits a usable 220 mm square bed, or a 250 × 210 mm bed with the long dimension along 250 mm.",
    },
    "detachable-loop": {
        "file": "signalsnagger-detachable-loop",
        "name": "Four-piece receiver with detachable loop",
        "rows": [
            (
                "smallrear",
                "Printed rear and socket",
                "Holds the PCB and battery. Two sockets in the shoulder receive the loop keys.",
            ),
            (
                "loop",
                "Detachable loop",
                "Two keys locate the loop. Two screws pull its foot down onto the shoulder.",
            ),
            (
                "front",
                "Removable front",
                "Four #4-24 × 5/8 in cover screws capture the PCB. Pull the wheel off before lifting the front.",
            ),
            (
                "lid",
                "Battery lid",
                "Two tabs and one M2.5 screw close the separate battery pocket.",
            ),
        ],
        "pts": [
            ((6, -47, -9), (-17, 2)),
            ((68, 214, 13), (-18, 9)),
            ((-14, -72, 113), (-15, -2)),
            ((20, 95, -107), (16, 10)),
        ],
        "detail": "How the loop joint comes apart",
        "intro": "The keys locate the loop and take sideways load. The two screws clamp the joint faces together.",
        "cards": [
            (
                "key-joint",
                "Keyed loop foot",
                "The blue foot lifts out of the two sockets in the teal shoulder. Two M3 screws pass through 3.4 mm clearance holes into blind insert pockets. Use the selected insert’s pilot diameter for fabrication.",
            ),
            (
                "wire-port",
                "Antenna and battery leads",
                "The wide upper notch opens the antenna wiring route into the PCB area. Provide a disconnect and slack for lifting the loop. The round rear-floor port leads separately to the battery bay.",
            ),
            (
                "tongues",
                "Independent battery lid",
                "Insert the two tongues under the bay wall, close the lid, then fit the single screw. A blind screw well keeps the fastener outside the modeled pack envelope.",
            ),
        ],
        "next": "Print the loop foot and receiver shoulder as short coupons. Check key clearance and pull-out after repeated assembly, then flex the joined coupons sideways. Test a complete loop for tuning, null depth and front/back sense against the existing receiver.",
        "printer": "The two large prints are the <b>160 × 170.1 × 24 mm loop</b> and the <b>76 × 188.9 × 31.5 mm rear</b>. Both fit a usable 220 mm square bed with a 5 mm brim, one part per print.",
    },
}

summary = {
    "three-piece": (
        "Print the rear shell, antenna frame and battery bay as one piece. A recessed front and a one-screw battery lid complete the enclosure. A fourth print is the thumbwheel.",
        "The loop has no root joint to loosen. Assembly needs few fasteners, and the encoder recess and drains print with the front. There is no custom machining charge.",
        "The long rear needs about 302 × 302 mm of usable bed with a brim. A failed print or broken loop costs a complete rear. The printed board seats and water paths need physical checks.",
    ),
    "polycase-hybrid": (
        "Machine the tested KT-70XXT0 and bond a printed antenna-and-battery saddle to its rear. Print the battery lid, a small splash collar for the flat front, and the thumbwheel.",
        "The molded case preserves the proven grip and board seats. The largest print fits a 220 mm square bed. The flat front needs a round encoder opening, and the battery lid takes one screw.",
        "Quote #154911 estimates <b>$1,024 for 100 machined shell sets</b> with the Rev B bevel and inside pocket ($9.24 per set plus $100 setup). Subject to drawing review. Lens, printing, hardware, shipping and tax are extra.",
    ),
    "detachable-loop": (
        "Print the receiver rear with a battery bay and keyed socket. Attach the separately printed loop with two M3 screws. A recessed front and battery lid bring the enclosure to four parts, plus the thumbwheel.",
        "Every part fits a 220 mm square bed. A damaged loop can be replaced independently, and the encoder recess and drains print with the front. The battery lid takes one screw.",
        "The loop joint adds hardware, a wiring disconnect and assembly time. Key fit, screw retention and wire strain need testing after repeated removal. More interfaces can loosen in field use.",
    ),
}
encoder_notes = {
    "printed": (
        "The printed front brings an annular floor down toward the top of the seated encoder body. The pocket floor is 3.4 mm below the outer face. A central raised lip surrounds the free shaft. Four chassis towers retain the PCB. Keep the encoder seated on its board anchors.",
        "The wheel skirt overlaps the raised lip. Two channels fall from the well to the exterior sidewalls, above the buttons. Keep them clear of print debris. Water can still reach the moving shaft if the well floods.",
        "Print a short front section with both drain exits. Grip it with either hand and gloves. Rotate without clicking or hitting a side button, then press and release. Splash the unpowered assembly at several grip angles with dry paper by SW203. Look for pooling and wet marks. After drying, check counts and switch return.",
    ),
    "hybrid": (
        "Keep the molded front flat. The study uses a Ø10.3 mm through-hole, a thin flanged collar bonded over it and the deep socket in the wheel. The case towers hold the board. Measure the seated nose, shaft exposure and bond thickness before ordering machining.",
        "The flange needs a continuous water-resistant bond to the ABS. The stationary lip sits inside the wheel skirt. Runoff sheds across the flat cover. The shaft opening remains a water path. If a threaded substitute needs a lower nut seat, quote a larger through-cut with a gasketed drop-in cup.",
        "Machine a spare KT-70 cover at the encoder station. Dry-fit the collar and wheel before bonding. Try both thumbs and gloves, then verify every press returns. Splash the unpowered cover at several angles. Place dry paper at the flange and shaft to locate leaks. After drying, verify counts and switch return.",
    ),
}


def encoder_page(design, d):
    tech = "hybrid" if design == "polycase-hybrid" else "printed"
    hybrid = tech == "hybrid"
    title = (
        "The thumbwheel over the flat Polycase front"
        if hybrid
        else "The thumbwheel and its recessed front"
    )
    header(
        d["name"],
        "Rotary encoder",
        title,
        "SW203 on the rev 0.4 board is the Bourns PEC12R-4225F-S0024. Its 6 mm D-shaft turns continuously and also operates a push switch.",
    )
    txt(35, 475, "Section through the shaft", 12.0, "B")
    txt(476, 475, "Front with the wheel removed", 12.0, "B")
    n = "section-" + tech
    photo = image(n, 38, 294, 401, 171)
    tag(n, photo, (-17, 36.94, 18.4), 38, 442, "Thumb rim")
    tag(n, photo, (-3.05, 36.94, 19), 42, 407, "D-socket")
    tag(n, photo, (0, 36.94, 4.0), 45, 304, "SW203")
    tag(n, photo, (6.5, 36.94, 14.9), 347, 424, "Fixed lip")
    tag(n, photo, (8.0, 36.94, 14.4), 345, 445, "Moving skirt")
    if hybrid:
        tag(n, photo, (9.4, 36.94, 12.0), 350, 338, "Bond flange")
    else:
        tag(n, photo, (11.4, 36.94, 8.0), 344, 338, "Drain floor")
    n = "seat-" + tech
    photo = image(n, 466, 294, 291, 171)
    if hybrid:
        tag(n, photo, (8.3, 37.4, 12.22), 650, 307, "Bonded collar")
    else:
        tag(n, photo, (-23.0, 36.94, 7.4), 476, 307, "Side outlet")
        tag(n, photo, (8.9, 37.9, 8.2), 670, 307, "Recess")
    rule(452, 300, 452, 463)
    first, wet, test = encoder_notes[tech]
    for x, title, body in [
        (35, "Mounting height", first),
        (281, "Moisture protection", wet),
        (
            527,
            "Thumb reach and travel",
            "The scalloped Ø44 mm rim comes within 3.4 mm of either side of the 50.8 mm grip. The crown clears the nominal 25 mm shaft and rises 14.8 mm above the panel. Tune the printed D-socket on a shaft sample for pull-off retention. Check for side load and accidental clicks. Allow 0.8 mm push travel plus build tolerance.",
        ),
    ]:
        rule(x, 285, x + 229, 285)
        txt(x, 263, title, 12.4, "B")
        assert textblock(x, 251, 226, body, 10.1, 13.8) > 135, (design, title)
    rectangle(M, 77, 722, 69, pale, 6)
    txt(M + 11, 130, "Bench check", 10.9, "B", teal)
    assert textblock(M + 11, 122, 698, test, 10.0, 13.4) > 78
    note = "Bourns rates the PEC12R IP40, with no water protection. For operation in rain, qualify a seal designed for continuous rotation and push travel, or a sealed encoder and panel interface."
    textblock(M, 72, 721, note, 8.7, 11.6, mut)
    textblock(
        M,
        46,
        723,
        source("Bourns PEC12R drawing and IP rating", encurl)
        + " &nbsp;·&nbsp; Rev 0.4 KiCad, SW203 &nbsp;·&nbsp; May 2024 prototype fit notes identify the threaded PEC12R-3222F-S0024 substitute.",
        8.2,
        10,
    )
    foot(4, d["name"])
    C.showPage()


encurl = "https://www.bourns.com/docs/product-datasheets/pec12r.pdf"

hurl = "https://www.polycase.com/kt-70"
furl = "https://github.com/OpenARDF/SignalSnagger/tree/f26c0c2887d23c9d8e13382aa85f70450e9b8317/FreeCAD/square-loop"
for design, d in docs.items():
    C = canvas.Canvas(str(out / (d["file"] + ".pdf")), pagesize=dims, pageCompression=1)
    C.setTitle(d["name"] + " | SignalSnagger rev 0.4")
    C.setAuthor("OpenARDF mechanical feasibility study")
    header(d["name"], "Feasibility brief", d["name"], "")
    approach, pros, cons = summary[design]
    txt(M, 510, "Executive summary", 14.0, "B", teal)
    assert textblock(M, 500, 720, approach, 11.3, 15.5) > 463
    for x, title, body in [(M, "Advantages", pros), (407, "Disadvantages", cons)]:
        txt(x, 447, title, 11.6, "B")
        assert textblock(x, 438, 347, body, 10.6, 14.5) > 372, (design, title)
    rectangle(M, 344, 722, 29, pale, 5)
    txt(M + 12, 354, "3D models", 10.3, "B", teal)
    txt(M + 82, 354, "Assembly   " + design + ".step", 10.1, "B")
    txt(418, 354, "Separate knob   thumbwheel-44mm.step", 10.1, "B")
    for i, (key, lbl) in enumerate([("front", "Front"), ("rear", "Rear")]):
        x = M + 219 * i
        image(design + "-" + key, x + 5, 78, 209, 253)
        mid(x + 108, 62, lbl, 10.2, "B")
    rule(477, 63, 477, 327)
    txt(496, 317, "Thumb reach at the grip", 11.0, "B", teal)
    rn = "reach-" + ("hybrid" if design == "polycase-hybrid" else "printed")
    reach = image(rn, 495, 181, 251, 127)
    tag(rn, reach, (20.7, 36.94, 16), 669, 197, "Wheel rim")
    textblock(
        495,
        173,
        254,
        "A 44 mm wheel on a 50.8 mm grip leaves <b>3.4 mm to each side</b>. This is the proposed reach for turning it with the thumb while holding the receiver.",
        9.4,
        12.5,
    )
    txt(495, 114, "Printer fit", 10.6, "B", teal)
    assert textblock(495, 106, 262, d["printer"], 9.0, 11.6) > 49
    textblock(
        M,
        48,
        722,
        "Rev 0.4 PCB mounting holes are Ø3.175 mm on 123.954 × 34.954 mm centers. The main grip is 139.8 × 50.8 × 20.8 mm.",
        8.4,
        10,
        mut,
    )
    foot(1, d["name"])
    C.showPage()
    header(
        d["name"],
        "Parts",
        "Exploded assembly",
        "The numbers identify the main enclosure parts. The PCB is green and the pack is purple. Page 4 shows the rotary control separately.",
    )
    n = "explode-" + design
    box2 = image(n, 45, 134, 375, 345)
    for j, (pt, delta) in enumerate(d["pts"], 1):
        mark(n, box2, pt, j, delta)
    legendchip(67, 111, scene["colors"]["pcb"], "Rev 0.4 PCB")
    legendchip(191, 111, scene["colors"]["pack"], "Pack envelope")
    rule(432, 104, 432, 473)
    rows = d["rows"]
    rH = 363 / len(rows)
    for j, (k, title, body) in enumerate(rows, 1):
        top = 475 - (j - 1) * rH
        image("part-" + k, 448, top - rH + 9, 76, rH - 15)
        pill(544, top - 17, j)
        txt(560, top - 21, title, 11.6, "B")
        textblock(533, top - 38, 220, body, 10.1, 14.2, mut)
        if j < len(rows):
            rule(448, top - rH, 752, top - rH)
    hardware = (
        "The prototype’s PEC12R-3222F-S0024 has an M9 × 0.75 bushing and a 22.5 mm shaft. Check whether the stock front leaves room for the washer and nut. If the threads fall short, use a flanged recessed cup. Verify nut access, gasket compression and wheel engagement on the shorter shaft."
        if design == "polycase-hybrid"
        else "The prototype’s PEC12R-3222F-S0024 has an M9 × 0.75 bushing and a 22.5 mm shaft. If this part is fitted to rev 0.4, put the nut-and-washer seat at the bushing shoulder. Check gasket compression, thread and tool access, and wheel engagement on the shorter shaft."
    )
    rectangle(M, 43, 722, 52, pale, 5)
    txt(M + 12, 78, "If the fitted encoder has a threaded bushing", 10.8, "B", teal)
    assert textblock(M + 12, 71, 700, hardware, 9.3, 12.3) > 44
    foot(2, d["name"])
    C.showPage()
    header(d["name"], "Mechanical details", d["detail"], d["intro"])
    xs = [35, 281, 527]
    for (img, title, body), x in zip(d["cards"], xs):
        if design == "detachable-loop" and img == "wire-port":
            img = "wire-port-small"
        if img == "hybrid-window":
            for offset, view, label in [(0, img, "Outside · lens fitted"), (116, "hybrid-window-pocket", "Inside · empty pocket")]:
                image(view, x + offset, 348, 110, 129)
                mid(x + offset + 55, 332, label, 8.4, "B")
        else:
            photo = image(img, x + 1, 329, 226, 145)
        if img in ("wire-port", "wire-port-small"):
            tag(img, photo, (5, 51, -6.6), x + 133, 340, "Battery port")
            tag(img, photo, (2, 68, -3.9), x + 2, 457, "Antenna slot")
        if img == "tongues":
            tag(img, photo, (-35.5, 87, -29.4), x + 2, 335, "Tabs")
            tag(img, photo, (32.6, 95.8, -31.6), x + 163, 454, "Screw hole")
        if img == "contact-map":
            tag(img, photo, (-6, 48, -9.1), x + 4, 340, "Bond area")
            tag(img, photo, (17.46, 61.98, -8.0), x + 131, 457, "Screw access")
        if img == "controls":
            tag(img, photo, (0, 36.94, 26.4), x + 5, 452, "Encoder")
            tag(img, photo, (0.4, -36.29, 11.5), x + 153, 340, "LCD lens")
        if img == "key-joint":
            tag(img, photo, (-20.5, 81, -9.4), x + 1, 337, "Key socket")
            tag(img, photo, (15, 87.4, 52.5), x + 157, 454, "M3 screw")
        rule(x, 318, x + 229, 318)
        txt(x, 296, title, 12.6, "B")
        bottom = textblock(x, 284, 227, body, 10.5, 15.1)
        assert bottom > 175, (title, bottom)
    rectangle(M, 91, 722, 70, pale, 7)
    txt(M + 13, 141, "First physical check", 12.3, "B", teal)
    textblock(M + 13, 131, 690, d["next"], 10.3, 14.4)
    if design == "polycase-hybrid":
        links = [
            ("KT-70 model and hardware", hurl),
            ("Polycase machining", "https://www.polycase.com/faq/cnc-machining"),
            (
                "ABS cement compatibility",
                "https://assemblyadhesives.com/products/abs/weld-on-2354/",
            ),
        ]
    else:
        links = [
            ("KT-70 geometry", hurl),
            ("Antenna CAD and assembly notes", furl),
            ("LCD outline", "https://www.displayvisions.us/fileadmin/eng/pdf/doma/dogs104e.pdf"),
        ]
    textblock(
        M,
        85,
        719,
        "The rev 0.4 STEP board body is 1.515 mm thick. KiCad specifies 1.605 mm finished. Verify the capture height and all openings with the populated board, inserting the jack end first. Compare antenna null, sense and compass bearing with the tested receiver.",
        8.9,
        12,
        mut,
    )
    textblock(M, 53, 720, " &nbsp;·&nbsp; ".join(source(t, u) for t, u in links), 8.5, 11)
    foot(3, d["name"])
    C.showPage()
    encoder_page(design, d)
    C.save()
