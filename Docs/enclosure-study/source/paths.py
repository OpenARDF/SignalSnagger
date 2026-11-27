"""Paths shared by the CAD model, figures, offline viewer and PDF briefs."""

from pathlib import Path

SOURCE = Path(__file__).resolve().parent
STUDY = SOURCE.parent
REPOSITORY = STUDY.parents[1]
WORK = REPOSITORY / "build" / "enclosure-study"
OUTPUT = WORK / "output"
POLYCASE = REPOSITORY / "FreeCAD" / "KT-70XXT0 ASS'Y.STEP"
LOOP = REPOSITORY / "FreeCAD" / "square-loop" / "square-loop.FCStd"
BOARD = STUDY / "inputs" / "rev04-board.step"
