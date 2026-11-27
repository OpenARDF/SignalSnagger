"""Build study artifacts into the ignored build directory, using local tools."""

import os
import subprocess
import sys

from paths import OUTPUT, SOURCE, WORK


def main():
    WORK.mkdir(parents=True, exist_ok=True)
    cad = os.environ.get("CAD_PYTHON", sys.executable)
    pdf = os.environ.get("PDF_PYTHON", sys.executable)
    compiler = os.environ.get("CC", "cc")
    subprocess.run(
        [
            compiler,
            "-O2",
            "-fPIC",
            "-shared",
            str(SOURCE / "raster.c"),
            "-o",
            str(WORK / "raster.so"),
        ],
        check=True,
    )
    for executable, script in [
        (cad, "models.py"),
        (cad, "render.py"),
        (cad, "viewer.py"),
        (pdf, "briefs.py"),
    ]:
        subprocess.run([executable, str(SOURCE / script)], check=True)
    print(f"Generated study files in {OUTPUT}")


if __name__ == "__main__":
    main()
