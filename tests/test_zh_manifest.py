"""Chinese translation source tracking, independent of learning dependencies."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/check_zh_manifest.py"
SPEC = importlib.util.spec_from_file_location("check_zh_manifest", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)


class ManifestTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.manifest = self.root / checker.MANIFEST
        self.manifest.parent.mkdir(parents=True)
        (self.root / "source.md").write_text("# Source\n", encoding="utf-8")
        (self.root / "translation.md").write_text("# 译文\n", encoding="utf-8")
        self.document = {
            "translation": "translation.md", "source_sha256": "", "source": "source.md",
        }
        self.write_manifest()

    def write_manifest(self) -> None:
        self.manifest.write_text(json.dumps({"documents": [self.document]}, indent=2) + "\n",
                                 encoding="utf-8")

    def run_script(self, *flags: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run([sys.executable, str(SCRIPT), str(self.root), *flags],
                              capture_output=True, text=True, check=False)

    def test_stale_is_warning_by_default_and_strict_failure(self) -> None:
        original = self.manifest.read_bytes()
        result = self.run_script()
        self.assertEqual(result.returncode, 0)
        self.assertIn("stale: source.md", result.stdout)
        self.assertEqual(self.run_script("--strict").returncode, 1)
        self.assertEqual(self.manifest.read_bytes(), original)

    def test_update_preserves_format_order_and_metadata(self) -> None:
        # CRLF, tabs, compact objects, escaped Unicode, no trailing newline, and
        # identically named metadata keys must all survive unchanged.
        original = ('{\r\n\t"source_sha256":"metadata", "language":"\\u4e2d",\r\n'
                    '\t"documents": [{"translation":"translation.md", '
                    '"source_sha256" : "", "source":"source.md"}]}')
        self.manifest.write_bytes(original.encode())
        result = self.run_script("--update", "--strict")
        self.assertEqual(result.returncode, 0, result.stderr)
        digest = hashlib.sha256((self.root / "source.md").read_bytes()).hexdigest()
        expected = original.replace('"source_sha256" : ""', f'"source_sha256" : "{digest}"')
        self.assertEqual(self.manifest.read_bytes(), expected.encode())
        self.assertEqual(checker.check_manifest(self.root), [])
        self.assertEqual(self.run_script("--update").returncode, 0)
        self.assertEqual(self.manifest.read_bytes(), expected.encode())
        (self.root / "source.md").write_text("changed", encoding="utf-8")
        self.assertEqual(self.run_script("--strict").returncode, 1)

    def test_missing_source_and_translation_are_reported(self) -> None:
        (self.root / "source.md").unlink()
        (self.root / "translation.md").unlink()
        result = self.run_script("--update", "--strict")
        self.assertEqual(result.returncode, 1)
        self.assertIn("missing source: source.md", result.stdout)
        self.assertIn("missing translation: translation.md", result.stdout)
        self.assertEqual(json.loads(self.manifest.read_text())["documents"][0]["source_sha256"], "")
        self.assertEqual(self.run_script().returncode, 0)

    def test_updates_multiple_documents_without_reordering(self) -> None:
        second = {"source": "other.md", "translation": "translation.md", "source_sha256": "old"}
        (self.root / "other.md").write_bytes(b"other")
        self.manifest.write_text(json.dumps({"documents": [self.document, second]}, indent=4),
                                 encoding="utf-8")
        self.assertEqual(self.run_script("--update", "--strict").returncode, 0)
        self.assertEqual(checker.check_manifest(self.root), [])
        documents = json.loads(self.manifest.read_text())["documents"]
        self.assertEqual([item["source"] for item in documents], ["source.md", "other.md"])

    def test_repository_manifest_paths_exist(self) -> None:
        manifest = json.loads((ROOT / checker.MANIFEST).read_text(encoding="utf-8"))
        for document in manifest["documents"]:
            for key in ("source", "translation"):
                with self.subTest(path=document[key]):
                    if key == "source" and checker.submodule_not_checked_out(ROOT, document[key]):
                        self.skipTest(f"{document[key]}: submodule not checked out")
                    self.assertTrue((ROOT / document[key]).is_file(), document[key])


if __name__ == "__main__":
    unittest.main()
