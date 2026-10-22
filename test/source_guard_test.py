# SPDX-License-Identifier: GPL-3.0-only
"""Check source boundaries against real Git/filesystem changes, without builds."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
import source_guard


class SourceGuardTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="blofeld-source-guard-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "repo with spaces "
        self.root.mkdir()
        self.environment = dict(os.environ, GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL="/dev/null",
                                GIT_AUTHOR_NAME="Source guard test", GIT_AUTHOR_EMAIL="test@example.invalid",
                                GIT_COMMITTER_NAME="Source guard test",
                                GIT_COMMITTER_EMAIL="test@example.invalid")
        self.git("-c", "init.templateDir=", "init", "-q", "--initial-branch=main")
        (self.root / ".gitignore").write_text("build/\ndoc/local/\n")
        (self.root / "main.cpp").write_bytes(b"abc")
        (self.root / "main.cpp").chmod(0o644)
        self.git("add", ".gitignore", "main.cpp")
        self.git("-c", "core.hooksPath=/dev/null", "commit", "-qm", "Own source fixture")
        self.snapshot = source_guard.capture(self.root)

    def git(self, *arguments):
        return subprocess.check_output(["git", "-C", str(self.root), *arguments], env=self.environment)

    def changed(self):
        with self.assertRaisesRegex(ValueError, "Source boundary changed:") as raised:
            source_guard.check(self.snapshot, self.root)
        return json.loads(str(raised.exception).split(": ", 1)[1])

    def test_literal_identity_and_unchanged_boundary(self):
        self.assertEqual(set(self.snapshot["sources"]), {".gitignore", "main.cpp"})
        self.assertEqual(self.snapshot["sources"]["main.cpp"], {
            "sha256": "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
            "mode": 0o644, "link": None})
        source_guard.check(self.snapshot, self.root)

    def test_tracked_content_change(self):
        (self.root / "main.cpp").write_text("different own content")
        self.assertEqual(self.changed(), {"added": [], "removed": [], "modified": ["main.cpp"]})

    def test_untracked_addition(self):
        (self.root / "new source.cpp").write_text("own new source")
        self.assertEqual(self.changed(), {"added": ["new source.cpp"], "removed": [], "modified": []})

    def test_tracked_deletion(self):
        (self.root / "main.cpp").unlink()
        self.assertEqual(self.changed(), {"added": [], "removed": ["main.cpp"], "modified": []})

    def test_intentional_unstaged_deletion_candidate(self):
        (self.root / "main.cpp").unlink()
        snapshot = source_guard.capture(self.root)
        self.assertEqual(set(snapshot["sources"]), {".gitignore"})
        source_guard.check(snapshot, self.root)

    def test_rename_reports_both_boundaries(self):
        (self.root / "main.cpp").rename(self.root / "renamed.cpp")
        self.assertEqual(self.changed(), {
            "added": ["renamed.cpp"], "removed": ["main.cpp"], "modified": []})

    def test_mode_change(self):
        (self.root / "main.cpp").chmod(0o755)
        self.assertEqual(self.changed()["modified"], ["main.cpp"])

    def test_ignored_build_and_reference_inputs(self):
        for name in ("build/object.o", "doc/local/reference.bin"):
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"own ignored fixture")
        source_guard.check(self.snapshot, self.root)

    def test_symlink_target_content(self):
        target = Path(self.temporary.name) / "external-source"
        target.write_text("own external source")
        (self.root / "linked.cpp").symlink_to(target)
        self.snapshot = source_guard.capture(self.root)
        target.write_text("changed own external source")
        self.assertEqual(self.changed()["modified"], ["linked.cpp"])

    def test_symlink_destination_even_with_equal_content(self):
        first, second = (Path(self.temporary.name) / name for name in ("one", "two"))
        first.write_bytes(b"same own source")
        second.write_bytes(b"same own source")
        link = self.root / "linked.cpp"
        link.symlink_to(first)
        self.snapshot = source_guard.capture(self.root)
        link.unlink()
        link.symlink_to(second)
        self.assertEqual(self.changed()["modified"], ["linked.cpp"])

    def test_broken_symlink_is_not_omitted(self):
        (self.root / "broken.cpp").symlink_to(self.root / "absent")
        with self.assertRaises(OSError):
            source_guard.capture(self.root)

    def test_fifo_rejected_before_open(self):
        fifo = self.root / "main.cpp"
        fifo.unlink()
        os.mkfifo(fifo)
        original = source_guard.os.open

        def opened(path, *arguments):
            self.assertNotEqual(Path(path), fifo, "A nonregular source must never be opened")
            return original(path, *arguments)

        with patch.object(source_guard.os, "open", opened):
            with self.assertRaisesRegex(ValueError, "not a regular file"):
                source_guard.capture(self.root)

    def test_revision_change_even_with_equal_source_tree(self):
        previous = self.snapshot["revision"]
        self.git("-c", "core.hooksPath=/dev/null", "commit", "-q", "--allow-empty", "-m", "New revision")
        changes = self.changed()
        self.assertEqual(changes["revision"], [previous, self.git("rev-parse", "HEAD").decode().strip()])
        self.assertEqual(changes["added"] + changes["removed"] + changes["modified"], [])

    def test_invalid_schema(self):
        for value in ([], {}, {**self.snapshot, "version": 2}, {**self.snapshot, "version": True},
                      {**self.snapshot, "unknown": 1}):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "schema"):
                    source_guard.check(value, self.root)

    def test_cli_capture_and_check(self):
        destination = self.root / "doc/local/snapshot.json"
        for operation in ("capture", "check"):
            result = subprocess.run([sys.executable, str(TOOLS / "source_guard.py"), operation,
                                     str(destination), "--root", str(self.root)],
                                    env=self.environment, capture_output=True, text=True,
                                    timeout=5, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stderr, "")
            self.assertIn("Verified source boundary: 2 files", result.stdout)

    def test_nested_directory_is_not_a_worktree_root(self):
        nested = self.root / "nested"
        nested.mkdir()
        with self.assertRaisesRegex(ValueError, "worktree root"):
            source_guard.capture(nested)

    def test_different_worktree_rejects(self):
        foreign = {**self.snapshot, "root": str(Path(self.temporary.name) / "foreign")}
        with self.assertRaisesRegex(ValueError, "different worktree"):
            source_guard.check(foreign, self.root)

    def test_new_file_during_capture(self):
        original = source_guard.read_source

        def read(path):
            (self.root / "arrived.cpp").write_text("new source during capture")
            return original(path)

        with patch.object(source_guard, "read_source", read):
            with self.assertRaisesRegex(ValueError, "changed during capture"):
                source_guard.capture(self.root)

    def test_ignored_snapshot_publication_and_permissions(self):
        destination = self.root / "doc/local/snapshot.json"
        source_guard.save(self.snapshot, destination)
        self.assertEqual(destination.stat().st_mode & 0o777, 0o600)
        source_guard.check(json.loads(destination.read_text()), self.root)

    def test_publication_preserves_symlink_target(self):
        target = Path(self.temporary.name) / "untouched"
        target.write_text("retain this file")
        destination = self.root / "doc/local/snapshot.json"
        destination.parent.mkdir(parents=True)
        destination.symlink_to(target)
        source_guard.save(self.snapshot, destination)
        self.assertFalse(destination.is_symlink())
        self.assertEqual(target.read_text(), "retain this file")
        source_guard.check(json.loads(destination.read_text()), self.root)

    def test_refuses_source_snapshot_self_inclusion(self):
        destination = self.root / "would-be-source.json"
        with self.assertRaisesRegex(ValueError, "Git-ignored"):
            source_guard.save(self.snapshot, destination)
        self.assertFalse(destination.exists())

    def test_cli_added_file_is_not_success(self):
        destination = self.root / "doc/local/snapshot.json"
        source_guard.save(self.snapshot, destination)
        (self.root / "new.cpp").write_text("own new source")
        result = subprocess.run([sys.executable, str(TOOLS / "source_guard.py"), "check",
                                 str(destination), "--root", str(self.root)], env=self.environment,
                                capture_output=True, text=True, timeout=5, check=False)
        self.assertEqual(result.returncode, 1)
        self.assertEqual(result.stdout, "")
        self.assertIn('"added": ["new.cpp"]', result.stderr)


if __name__ == "__main__":
    unittest.main()
