"""rs_driver's decoding must not change unnoticed (docs/VENDOR_DIFF.md).

Goldens in golden/decode/v1.5.18/ were made with rs_driver 1.5.18 (cd35885, the submodule of the
released v1.5.18 image) before the update: for every model 1.5.18 decodes, two packet seeds and two
distance settings, one line per point cloud (sequence, stamp, size, NaN count, a hash of every
point's x, y, z, intensity, ring and timestamp). The submodule in this checkout must give the same
lines, except where docs/VENDOR_DIFF.md lists a difference: those runs must match the golden in
golden/decode/<version>/ instead, and must differ from 1.5.18 (so the list stays true).

RS_DRIVER_OLD=<path of an rs_driver 1.5.18 tree> also rebuilds the goldens' source and checks it
still gives them (CI sets it). REGEN=1 rewrites the current version's goldens (never 1.5.18's).
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "decode"))
import decode  # noqa: E402

HERE = Path(__file__).resolve().parent
GOLDEN = HERE / "golden" / "decode"
EXPECTED = json.loads((GOLDEN / "expected_differences.json").read_text())

pytestmark = pytest.mark.skipif(decode.compiler() is None, reason="needs g++")


@pytest.fixture(scope="module")
def current():
    return decode.build(decode.SUBMODULE, decode.tempdir())


def runs(report: str) -> dict[str, str]:
    out: dict[str, list[str]] = {}
    key = None
    for line in report.splitlines():
        if line.startswith("# "):
            key = line[2:]
            out[key] = []
        else:
            out[key].append(line)
    return {k: "\n".join(v) for k, v in out.items()}


@pytest.mark.parametrize("model", decode.OLD_MODELS)
def test_decoding_unchanged_since_1_5_18(current, model):
    report = decode.model_report(current, model)
    expected = runs((GOLDEN / "v1.5.18" / f"{model}.txt").read_text())
    got = runs(report)
    assert set(got) == set(expected)
    listed = {d["run"]: d for d in EXPECTED["differences"] if d["run"].startswith(model + " ")}
    for run, text in got.items():
        if run in listed:
            assert text != expected[run], f"{run}: listed as a difference but decodes as 1.5.18 did"
            golden = GOLDEN / listed[run]["version"] / (run.replace(" ", "_") + ".txt")
            if os.environ.get("REGEN"):
                golden.parent.mkdir(exist_ok=True)
                golden.write_text(f"# {run}\n{text}\n")
            assert golden.read_text() == f"# {run}\n{text}\n", f"{run}: not its listed golden"
        else:
            assert text == expected[run], f"{run}: decodes differently from rs_driver 1.5.18"


def test_every_listed_difference_names_a_run():
    names = {f"{m} seed={s} min={lo} max={hi}" for m in decode.OLD_MODELS for s in decode.SEEDS
             for lo, hi in decode.DISTANCES}
    for d in EXPECTED["differences"]:
        assert d["run"] in names and d["why"], d


@pytest.mark.skipif(not os.environ.get("RS_DRIVER_OLD"), reason="RS_DRIVER_OLD not set")
@pytest.mark.parametrize("model", decode.OLD_MODELS)
def test_goldens_are_rs_driver_1_5_18(model, tmp_path_factory):
    exe = getattr(test_goldens_are_rs_driver_1_5_18, "exe", None)
    if exe is None:
        exe = decode.build(Path(os.environ["RS_DRIVER_OLD"]), tmp_path_factory.mktemp("old"))
        test_goldens_are_rs_driver_1_5_18.exe = exe
    first = decode.run(exe, "synth", model, "7", "10", "0.2", "200").splitlines()[0]
    assert first.startswith("rs_driver 1.5.18 "), first
    assert decode.model_report(exe, model) == (GOLDEN / "v1.5.18" / f"{model}.txt").read_text()


NEW_MODELS = ("RSFAIRY", "RSEMX")  # added in rs_driver 1.5.19; RSAIRYLITE_ETH (1.5.20) has no generator


@pytest.mark.parametrize("model", NEW_MODELS)
def test_models_new_in_1_5_19_decode(current, model):
    """No 1.5.18 to compare with: the golden pins today's output, and it has point clouds."""
    report = decode.model_report(current, model)
    golden = GOLDEN / "v1.5.20" / f"new-{model}.txt"
    if os.environ.get("REGEN"):
        golden.write_text(report)
    assert report == golden.read_text()
    assert any(line.startswith("cloud ") and " points=0 " not in line for line in report.splitlines())
