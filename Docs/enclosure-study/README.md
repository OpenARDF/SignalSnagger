# SignalSnagger Rev 0.4 enclosure study

This study uses the tested Polycase KT-70XXT0 grip and the Rev 0.4 PCB as its mechanical references. All three concepts give the rechargeable pack its own service lid, keep the battery and antenna leads on separate routes, and place a wide thumbwheel over the encoder. The models are dimensioned concepts. Physical fit, rain exposure, structural performance and RF behavior still need first articles.

## Choose a brief

| Architecture | Main advantage | Main cost or limitation | Files |
| --- | --- | --- | --- |
| Integrated rear, antenna and battery bay | Three structural prints and a continuous loop root. The front incorporates the encoder recess. | The rear needs about 302 × 302 mm of usable bed with a 5 mm brim. Loop damage means replacing the rear. | [Brief](../../output/pdf/signalsnagger-enclosure-feasibility.pdf) · [Assembly STEP](../../output/models/three-piece.step) |
| Molded KT-70 with bonded saddle | Keeps the tested molded grip and board capture. The saddle fits a 220 mm square bed. | Requires a machined case and two qualified bonds, one for the saddle and one for the encoder splash collar. | [Brief](../../output/pdf/signalsnagger-polycase-hybrid.pdf) · [Assembly STEP](../../output/models/polycase-hybrid.step) |
| Printed receiver with detachable loop | Every print fits a 220 mm square bed and the loop can be replaced on its own. | The keyed joint needs two M3 screws, a wire disconnect and repeated-use testing. | [Brief](../../output/pdf/signalsnagger-detachable-loop.pdf) · [Assembly STEP](../../output/models/detachable-loop.step) |

Each brief opens with an executive summary and names its CAD files. Page 2 shows the main enclosure parts, page 3 the architecture-specific joints and wiring, and page 4 the rotary control in section. The custom knob is an additional print in every concept. The hybrid uses four prints in addition to the two molded halves. They are the saddle, battery lid, splash collar and thumbwheel.

Open the viewer locally to rotate, explode or isolate the models. GitHub displays the source of the HTML file. The standalone wheel uses a local origin so it can be imported into CAD independently. [Offline 3D viewer](../../output/models/enclosure-viewer.html) · [44 mm thumbwheel STEP](../../output/models/thumbwheel-44mm.step)

## Print the encoder fit sample

| Print | Orientation supplied in the STL | What it reproduces |
| --- | --- | --- |
| [Recessed encoder section](../../output/models/rev04-recessed-encoder-section.stl) | Stand on the flat transverse cut. The antenna end is at the top of the print. | A 61.9 mm length of the printed front with the full 50.8 mm grip width, two antenna-end screw towers, side-button openings, encoder well and both drains. |
| [44 mm thumbwheel](../../output/models/thumbwheel-44mm.stl) | Flat center of the crown down, blind D-socket up. | The study wheel for the 6 mm D-shaft of the unthreaded PEC12R-4225F-S0024. |

Import in **millimeters at 100%** and keep the supplied orientations. Both files sliced as single manifold parts in PrusaSlicer 2.9.7 with an MK3S-size bed, a 0.4 mm nozzle, PETG, 0.15 mm layers, three perimeters, a 5 mm brim and build-plate-only supports. Remove the brim and supports. Check the mating seam, pilot holes, wheel skirt and socket for loose plastic.

1. With the receiver off, remove its knob and front. Leave the Rev 0.4 board seated in the rear shell. Set the printed section over SW203 and the side buttons with the cut end toward the headphone jack. It should seat on the original seam before the screws are tightened. Start the two original antenna-end cover screws from the rear by hand and snug them gently.
2. Align the shaft flat and slide the wheel on by hand. Stop if the shaft binds or the PCB flexes. A tight D-socket can change the result of the push test. If the fitted encoder has a threaded bushing, the substitute needs a different seat and has a shorter shaft.
3. Turn the wheel through a full revolution, press and release it several times, then repeat while gripping the rear shell in either hand. Check for scraping, a missed return, or an accidental side-button press. The nominal rim sits 3.4 mm inside each side of the grip. CAD gives 0.85 mm minimum wheel-to-section clearance at rest and 0.55 mm with the wheel depressed 0.8 mm. Measure travel on the actual switch and compare it with the wheel removed.

## Mechanical references and datums

- **Case.** The flush, textured KT-70XXT0 is 139.80 × 50.80 × 20.78 mm. Its four assembly towers capture the board near the screw axes. The tested XX variant omits the optional extra MB mounting bosses. The STEP supplies the outer radii, seam, shoulders and cover screws used in the studies. [Case STEP](../../FreeCAD/KT-70XXT0%20ASS%27Y.STEP) · [Polycase product and drawings](https://www.polycase.com/kt-70)
- **PCB.** The routed Rev 0.4 board is approximately 133.35 × 44.35 mm with four Ø3.175 mm capture holes on 123.954 × 34.954 mm centers. The KiCad design stack is 1.605 mm, while its bare STEP body is 1.515 mm. Use a measured populated board to set capture height and preload. The included bare-board export preserves the geometry used in this study, including the side-switch notches. It was made from the recovered Rev 0.4 layout before that layout was committed. [Board STEP used in the assemblies](inputs/rev04-board.step)
- **Antenna.** The reference frame has a lower groove for pickup coax, a wider groove above it for flat ribbon and a 220 mm brass sense element. Install the coax first. The original separate L-block and temporary assembly shim explain the cable entry and build order. Preserve those functions when reshaping the root. The archived print projects and toolpaths describe the original frame. [Editable reference assembly](../../FreeCAD/square-loop/square-loop.FCStd) · [Illustrated assembly notes](../../FreeCAD/square-loop/notes.pdf) · [Reference slicer project](../../FreeCAD/square-loop/square-loop-loop-main-top.3mf)
- **Source identity.** The source PCB SHA-256 is `917fed24e2890b2d0eb447ca1b56ba0804786704a0d6086ef326e53f60fbb96a`. The preserved STEP SHA-256 is `dff5ec020355aa7fbeaea87a6303bc9779f64c39281f0f1f16eca2b935833d05`. The case and loop references were taken from repository commit `f26c0c2887d23c9d8e13382aa85f70450e9b8317`.

In the enclosure coordinate system, X runs across the grip, +Y points toward the antenna and +Z points out through the front. For the Rev 0.4 KiCad board coordinates below, enclosure X = board Y − 96.186 and enclosure Y = 143.94 − board X, in millimeters. The populated encoder face of the bare board is at Z = −0.449. Footprint origins locate components. Measure the actual contact, shaft, optic or barrel before making a machining drawing.

| Interface | KiCad X, Y in mm | Mechanical consequence |
| --- | --- | --- |
| SW201 and SW202 | 107.000, 78.312 and 107.000, 114.070 | Opposed side buttons at the encoder station. Fit the actuator to the short switch stroke and check return with the case tightened. |
| SW203 | 107.000, 96.186 | Thumbwheel on the grip centerline, enclosure Y = 36.94 mm. |
| D203 and D204 | 191.630, 77.990 and 191.650, 114.4275 | Two separate light pipes. Size the final bores to the selected optics and check daylight visibility. |
| LCD | See module drawing | The DOGS104 viewing area is 33.0 × 12.1 mm and is offset on the module. Provide a stepped seat for a nominal 1 mm clear lens. Confirm the socketed height. |
| J301 | 210.500, 96.186 | Provide barrel and plug-shoulder clearance at the bottom seam. Install the PCB jack end first. |
| Antenna J102 and J104 | 81.150, 85.695 and 81.150, 104.075 | Route the coax and ribbon through the top-edge opening. Allow for terminated connectors, bend radius and strain relief. |
| Battery BT201 | 122.415, 82.475 | Route the pack leads through the separate rear-floor channel. Preserve access to the main and balance connectors. |

Component drawings provide the functional dimensions for actuator travel, the display window and the jack barrel. [TL1014 switch](https://www.e-switch.com/wp-content/uploads/2022/06/TL1014.pdf) · [LCD outline](https://www.displayvisions.us/fileadmin/eng/pdf/doma/dogs104e.pdf) · [Switchcraft headphone jack](https://www.switchcraft.com/3-5mm-smt-jack-narrow-body-2b-double-open-circuit-/35rasmt2bhsntrx/)

## Encoder decisions and water paths

The Rev 0.4 schematic and layout call for **Bourns PEC12R-4225F-S0024** at SW203. Its 25 mm D-shaft has a nominal 6 mm diameter and a 4.5 mm dimension across the flat. This version has a smooth fixed nose and mounts on the PCB. The four case towers establish board height. Pull off the wheel to service the front. [Bourns order code and dimensional drawings, pp. 1 and 4](https://www.bourns.com/docs/product-datasheets/pec12r.pdf)

Project fit notes from May 2024 identify a **PEC12R-3222F-S0024** substitute with an M9 × 0.75 bushing and a 22.5 mm shaft. The prototype's encoder nut therefore describes a different component on the same mechanical problem. The prototype trial found little usable thread at the molded face with the encoder seated on the board. June fitting notes also called for a low, secure, textured knob with a D-shaped socket. Check the marking and shaft dimensions on the actual receiver before freezing the front. Keep the PCB and encoder seated to preserve jack, display and connector alignment.

| Feature | Current concept | Follow-on measurement |
| --- | --- | --- |
| Wheel | Scalloped 44 mm rim, 3.4 mm inside each edge of the nominal 50.8 mm grip. A closed crown accommodates the full-length shaft. Its center projects 14.8 mm above the flat cover, while the rim is lower. | Try left and right hands, wet fingers and gloves. Check single-thumb turns, intentional pushes, release, unintended side-switch presses and snagging. Compare a second rim diameter if the grip test calls for it. |
| Socket and travel | The blind D-socket uses nominal Ø6.08 mm and a flat 1.54 mm from center. Its modeled shaft engagement is about 7.7 mm. The minimum nominal wheel-to-fixed-part gap is 0.85 mm at rest in every assembly. A 0.8 mm axial translation, the upper series travel allowance, clears the fixed parts throughout the stroke. | Print a small D-socket sample in the intended material and orientation. Measure grip, pull-off force, repeated removal and clearance at maximum press. Recheck engagement if the installed shaft is shorter. |
| Printed front | Its annular floor is 3.4 mm below the outer face. The recess brings the local mounting surface toward the upper encoder body. A fixed lip projects inside the moving wheel skirt. Two open channels fall to the outside above the side buttons. | Measure the seated body and smooth nose. Check that tightening the cover leaves the anchors unloaded. For a threaded unit, size the well for the shoulder gasket, washer, nut and tool. Keep the outlets clear after printing. |
| Molded front | A proposed Ø10.3 mm through-hole and a bonded flanged splash collar preserve the flat molded wall. The knob socket reaches down around the shaft. Water can run out across the face. | Trial on a spare front. For a threaded unit, measure actual thread engagement. If the nut sits too high, a larger CNC through-cut and a separately gasketed recessed cup can supply a lower mounting floor. Test the cup/flange joint and removal sequence. |
| Moisture | The covered lip sheds splash. A sealed stationary flange addresses its joint to the case. Water can still travel beside a shaft that rotates and moves axially. Bourns specifies IP40, which provides no water-ingress rating. | Operate and splash an unpowered specimen in several orientations. Place dry witness paper near the shaft and flange separately. Check the PCB side and any pooled water. After drying, check counts and switch return. For a specified rain requirement, qualify a continuous-rotation/push seal or a suitably sealed encoder and its panel joint. |

## Printing, fastening and service

The integrated rear is 160 × 310.2 × 43.9 mm. Its flat diagonal footprint is about 291.5 mm square. The hybrid saddle is 160 × 202.3 × 43.9 mm, or 170 × 212.3 mm with a 5 mm brim. It fits a 250 × 210 mm bed with the long dimension along 250 mm. The detachable loop is 160 × 170.1 × 24 mm and its rear is 76 × 188.9 × 31.5 mm. Slice each part at the actual usable bed limits. Toolpaths, supports, warping and print time remain open for the new parts. The separate sense rod is installed after printing.

- **Material and orientation.** The archived PETG loop project used a 0.4 mm nozzle and 0.15 mm layers. Plain PETG is a starting point for printed grip trials. Test root flex, layer direction, accessible support removal and the wheel socket. An ABS saddle suits an ABS-to-ABS solvent-cement trial. Long ABS prints can warp, particularly on open printers. A PETG saddle needs its own demonstrated adhesive or mechanical joint. [Archived loop print files](../../FreeCAD/square-loop) · [ABS cement product information](https://assemblyadhesives.com/products/abs/weld-on-2354/)
- **Hardware.** Retain the four factory #4-24 × 5/8 in cover screws and their stepped seats. The battery lid has two tongues and one M2.5 screw through a Ø2.9 mm clearance hole. Its blind boss lies outside the pack envelope. Select insert, pilot and screw length together and check the screw when fully seated. The detachable root uses two M3 screws through Ø3.4 mm holes and two positive locating keys. Check insert retention under sideways loads and after repeated removal.
- **Battery.** The modeled shoulder bay opens to the rear. The reference pack envelope is 56 × 31 × 11 mm for the 450 mAh 2S pack. A previously considered 300 mAh envelope is smaller in plan and 12 mm thick. Measure the selected wrapped pack, its cable exits, main plug and balance plug. Allow the pack maker’s clearance, smooth support and an extraction route. Check that a battery change leaves the electronics and antenna undisturbed.
- **Joints.** In the hybrid, keep bonding land and side returns on the fixed rear. Leave the front seam and all screw heads accessible. Dry-fit and cure a coupon on a spare textured molded shell, then apply sideways and peel loads. In the detachable design, provide a disconnect and service slack before withdrawing the keyed loop. In the one-piece design, test the neck and pocket roof before committing to a full-length print.

## Validation record and next physical trial

| Established by this study | Still needs a specimen |
| --- | --- |
| All printed/machined primary parts are single valid CAD solids. The assemblies reopen with 20, 22 and 23 solids respectively, including reference hardware. Solid counts, positions and volumes are checked on reimport. | Measure a populated Rev 0.4 and the tested case. Build a two-corner tower coupon and a section containing the actual encoder, LCD, side buttons or jack as appropriate. Check closing force, tool access and support removal. |
| Nominal positions clear the modeled board, pack and structural parts. The wheel clears the modeled fixed hardware through 0.8 mm push travel. The PCB model is a bare body and controls are simplified envelopes. | Check full rotation, retention, rocking, repeated push, moisture paths and actual control heights. Take measurements with the cover secured. Record material, printer, orientation and dimensional changes. |
| CAD gives bed footprints and separate battery and antenna feed-throughs. Both the assembled and exploded views derive from those models. | Slice full-size parts. Route terminated wiring, cycle the battery lid, load the loop joint, and try a representative drop with a dummy pack. Compare mass and balance in the hand. |
| The molded receiver is the control for a radio comparison. The archived frame describes conductor routing and the sense element. | With final pack, wiring and screws, compare tuning over 3.5–3.7 MHz, null depth and direction, front/back sense, and compass bearing through a full turn. Earlier trials reported compass deflection with steel-cased cells. |

Record the as-built encoder part, knob dimensions and socket fit, actual pack dimensions, photos of both shell halves and each leak location, the slicer profile, joint or bond failure mode, and radio/compass observations alongside the next prototype. These measurements determine whether to keep the printed well, order the machined front, or revise the wheel.

## Rebuild or adapt the study

The checked-in PDFs, STEP files and HTML viewer open independently. To regenerate them, run `just enclosure-study` from the repository root. The recipe builds in `build/enclosure-study/` and places finished files in `build/enclosure-study/output/` for comparison against the checked-in `output/`. STEP export includes generation timestamps.

The builder uses a C compiler, Python with pythonocc-core 7.8, NumPy and Pillow, and Python with ReportLab and Pillow. The CAD and PDF tools can share an interpreter. Set `CAD_PYTHON` or `PDF_PYTHON` to use separate interpreters, and `CC` to select the C compiler. The study was rebuilt on macOS with pythonocc-core 7.8.1.1. A C compiler and the CAD tools are needed only to rebuild the artifacts.

Start with [the assembly geometry](source/models.py) for the shells, openings, leads and fasteners, [the encoder geometry](source/encoder.py) for the removable wheel and its mating features, and [the brief source](source/briefs.py) for the illustrated report text. The shared [build entry point](source/rebuild.py) also regenerates the browser viewer from the same meshes.
