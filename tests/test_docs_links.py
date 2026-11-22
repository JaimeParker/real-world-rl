"""Keep runtime documentation hints aligned with the published RST sources."""
from __future__ import annotations

import ast
from pathlib import Path
import re
import subprocess

import pytest

from real_world_rl.common.docs_links import DOCS_BASE_URL, docs_url

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    ("lang", "anchor", "suffix"),
    [
        ("en", None, "en/reference/devices.html"),
        ("zh-cn", None, "reference/devices.html"),
        ("en", "en-devices-zed-cameras", "en/reference/devices.html#en-devices-zed-cameras"),
        ("zh-cn", "devices-zed-cameras", "reference/devices.html#devices-zed-cameras"),
        ("en", "verbatim-label", "en/reference/devices.html#verbatim-label"),
    ],
)
def test_docs_url_format(lang, anchor, suffix):
    assert DOCS_BASE_URL == "https://real-world-rl.readthedocs.io/zh-cn/latest/"
    assert docs_url("reference/devices", anchor, lang=lang) == DOCS_BASE_URL + suffix


def test_docs_url_defaults_to_english():
    assert docs_url("tutorials/station") == DOCS_BASE_URL + "en/tutorials/station.html"


def test_runtime_docs_url_targets_exist():
    matches = subprocess.run(
        ["grep", "-rlE", "--include=*.py", r"\bdocs_url\s*\(", "real_world_rl", "native"],
        cwd=ROOT, capture_output=True, text=True, check=True,
    )
    calls = 0
    for filename in matches.stdout.splitlines():
        tree = ast.parse((ROOT / filename).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            name = node.func.id if isinstance(node.func, ast.Name) else getattr(node.func, "attr", None)
            if name != "docs_url":
                continue
            calls += 1
            args = [ast.literal_eval(arg) for arg in node.args]
            kwargs = {arg.arg: ast.literal_eval(arg.value) for arg in node.keywords}
            page = args[0] if args else kwargs["page"]
            anchor = args[1] if len(args) > 1 else kwargs.get("anchor")
            source = ROOT / "docs/en" / f"{page}.rst"
            context = f"{filename}:{node.lineno}"
            assert source.is_file(), f"{context}: missing {source}"
            labels = re.findall(r"^\.\. _(.+):\s*$", source.read_text(encoding="utf-8"), re.MULTILINE)
            if anchor is not None:
                assert anchor in labels, f"{context}: missing label {anchor} in {source}"
    assert calls, "No runtime documentation links were checked"
