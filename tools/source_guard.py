#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Capture/check a developer source boundary without running builds or fuzzers."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile


def git(root, *arguments):
    return subprocess.check_output(["git", "-C", str(root), *arguments])


def source_paths(root):
    return sorted(set(os.fsdecode(path) for path in git(
        root, "ls-files", "--cached", "--others", "--exclude-standard", "-z").split(b"\0") if path))


def signature(information):
    return (information.st_dev, information.st_ino, information.st_mode, information.st_size,
            information.st_mtime_ns, information.st_ctime_ns)


def read_source(path):
    before = path.lstat()
    link = os.readlink(path) if stat.S_ISLNK(before.st_mode) else None
    target = path.resolve(strict=True)
    expected = target.stat()
    if not stat.S_ISREG(expected.st_mode):
        raise ValueError(f"Source is not a regular file: {path}")
    descriptor = os.open(target, os.O_RDONLY | os.O_CLOEXEC | os.O_NONBLOCK | os.O_NOFOLLOW)
    with os.fdopen(descriptor, "rb") as stream:
        opened = os.fstat(stream.fileno())
        if signature(expected) != signature(opened):
            raise ValueError(f"Source changed before hashing: {path}")
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1048576), b""):
            digest.update(block)
        if signature(opened) != signature(os.fstat(stream.fileno())):
            raise ValueError(f"Source changed while hashing: {path}")
    if (signature(before) != signature(path.lstat()) or target != path.resolve(strict=True) or
            signature(opened) != signature(target.stat())):
        raise ValueError(f"Source changed while hashing: {path}")
    return {"sha256": digest.hexdigest(), "mode": stat.S_IMODE(before.st_mode), "link": link}


def capture(root):
    root = Path(root).resolve(strict=True)
    if Path(os.fsdecode(git(root, "rev-parse", "--show-toplevel")).removesuffix("\n")).resolve() != root:
        raise ValueError("Source root must be the Git worktree root")
    revision = git(root, "rev-parse", "HEAD").decode("ascii").strip()
    paths = source_paths(root)
    sources = {}
    for name in paths:
        path = root / name
        # A tracked deletion may be intentional in an uncommitted candidate.
        # Broken symlinks remain errors rather than disappearing from coverage.
        if not path.exists() and not path.is_symlink():
            continue
        sources[name] = read_source(path)
    if paths != source_paths(root) or revision != git(root, "rev-parse", "HEAD").decode().strip():
        raise ValueError("Source membership or revision changed during capture")
    return {"version": 1, "root": str(root), "revision": revision, "sources": sources}


def check(snapshot, root):
    if (not isinstance(snapshot, dict) or set(snapshot) != {"version", "root", "revision", "sources"}
            or type(snapshot["version"]) is not int or snapshot["version"] != 1
            or not isinstance(snapshot["sources"], dict)):
        raise ValueError("Invalid source snapshot schema")
    current = capture(root)
    if current["root"] != snapshot["root"]:
        raise ValueError("Source snapshot belongs to a different worktree")
    previous, actual = snapshot["sources"], current["sources"]
    changes = {"added": sorted(actual.keys() - previous.keys()),
               "removed": sorted(previous.keys() - actual.keys()),
               "modified": sorted(name for name in previous.keys() & actual.keys()
                                  if previous[name] != actual[name])}
    if current["revision"] != snapshot["revision"]:
        changes["revision"] = [snapshot["revision"], current["revision"]]
    if any(changes.values()):
        raise ValueError("Source boundary changed: " + json.dumps(changes, ensure_ascii=True))


def save(snapshot, destination):
    destination = Path(destination).absolute()
    root = Path(snapshot["root"])
    resolved = destination.resolve()
    if resolved.is_relative_to(root) and subprocess.run(
            ["git", "-C", str(root), "check-ignore", "-q", "--", str(resolved)],
            check=False).returncode != 0:
        raise ValueError("Snapshot output must be outside the worktree or Git-ignored")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=destination.parent,
                                         prefix=".source-guard-", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(json.dumps(snapshot, indent=2, sort_keys=True) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("capture", "check"))
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    arguments = parser.parse_args()
    try:
        if arguments.operation == "capture":
            snapshot = capture(arguments.root)
            save(snapshot, arguments.snapshot)
        else:
            snapshot = json.loads(arguments.snapshot.read_text())
            check(snapshot, arguments.root)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        parser.exit(1, f"Source verification failed: {error}\n")
    print(f"Verified source boundary: {len(snapshot['sources'])} files at {snapshot['revision']}")


if __name__ == "__main__":
    main()
