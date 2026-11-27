import base64
import io
import json

import numpy as np
from paths import OUTPUT, SOURCE
from paths import WORK as w
from PIL import Image

sc = json.loads((w / "scene.json").read_text())


def b64(s):
    return base64.b64encode(s).decode()


data = {
    "designs": sc["designs"],
    "refs": [
        k
        for k in sc["labels"]
        if k not in {part for group in sc["designs"].values() for part in group}
    ],
    "mesh": {},
    "poster": {},
}
for k in sc["labels"]:
    a = np.load(w / (k + ".npy")).reshape(-1, 3)
    v, ix = np.unique(np.rint(a * 100).astype("<i2"), axis=0, return_inverse=True)
    data["mesh"][k] = {
        "v": b64(v.tobytes()),
        "i": b64(ix.astype("<u4").tobytes()),
        "b": [float(t) for t in [*a.min(0), *a.max(0)]],
        "c": sc["colors"][k],
    }
for d in sc["designs"]:
    im = Image.open(w / (d + "-front.png")).convert("RGB")
    im.thumbnail((400, 650))
    f = io.BytesIO()
    im.save(f, "JPEG", quality=85)
    data["poster"][d] = "data:image/jpeg;base64," + b64(f.getvalue())
out = OUTPUT / "models/enclosure-viewer.html"
out.write_text(
    (SOURCE / "viewer.template")
    .read_text()
    .replace("DATA_PLACEHOLDER", json.dumps(data, separators=(",", ":")))
)

# A static preview accompanies the offline viewer for clients that cannot embed WebGL.
from PIL import ImageDraw, ImageFont

preview = Image.new("RGB", (1050, 395), "white")
draw = ImageDraw.Draw(preview)
font = ImageFont.load_default(size=18)
for index, (name, caption) in enumerate(
    [
        ("three-piece-front", "Front"),
        ("three-piece-rear", "Rear"),
        ("explode-three-piece", "Exploded"),
    ]
):
    with Image.open(w / (name + ".png")) as panel:
        panel.thumbnail((330, 345), Image.Resampling.LANCZOS)
        center = index * 350 + 175
        preview.paste(panel, (center - panel.width // 2, (350 - panel.height) // 2 + 4))
        draw.text((center, 365), caption, font=font, anchor="mt", fill="#183238")
preview.save(OUTPUT / "models" / "three-piece-preview.png")
