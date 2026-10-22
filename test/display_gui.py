# SPDX-License-Identifier: GPL-3.0-only
"""Run the native GTK viewer under Xvfb with real directory lifecycle events."""

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gui.frame import FILE_BYTES, HEADER, Frame, read_frame


def require(condition, message):
    if not condition:
        raise ValueError(message)


def replace(directory, data):
    temporary = directory / ".frame.tmp"
    temporary.write_bytes(data)
    temporary.replace(directory / "display.pbm")


def parser(directory):
    data = bytearray(1024)
    data[0], data[16], data[-1] = 0x81, 0x40, 0x01
    expected = Frame(bytes(data))
    replace(directory, HEADER + data)
    require(read_frame(directory / "display.pbm") == expected, "Literal complete PBM payload")
    require(expected.on(0, 0) and expected.on(7, 0) and expected.on(1, 1) and expected.on(127, 63),
            "Independent MSB-first, row-stride and final-pixel coordinates")
    require(not expected.on(1, 0) and not expected.on(0, 1) and not expected.on(126, 63),
            "Neighbor pixels remain off")
    for invalid in (b"", HEADER + data[:-1], HEADER + data + b"\0",
                    b"P1\n128 64\n" + data, b"P4\n127 64\n" + data):
        replace(directory, invalid)
        try:
            read_frame(directory / "display.pbm")
        except ValueError:
            continue
        raise ValueError("A fixed malformed display must be rejected")
    (directory / "display.pbm").unlink()
    os.mkfifo(directory / "display.pbm")
    try:
        read_frame(directory / "display.pbm")
    except ValueError:
        pass
    else:
        raise ValueError("A FIFO is not an LCD frame")
    (directory / "display.pbm").unlink()
    (directory / "display.pbm").symlink_to(directory / "missing.pbm")
    try:
        read_frame(directory / "display.pbm")
    except OSError:
        pass
    else:
        raise ValueError("A frame symlink must not be followed")
    require(FILE_BYTES == 1034, "Bounded 1-bit image including exact publication header")
    return expected


def gtk_test(directory, expected):
    from gui.display import DisplayWindow
    from gi.repository import Gio, GLib, Gtk

    context = GLib.MainContext.default()

    def until(condition):
        deadline = time.monotonic() + 5
        while not condition():
            require(time.monotonic() < deadline, "Bounded GTK notification/render update")
            while context.pending():
                context.iteration(False)
            time.sleep(0.005)

    application = Gtk.Application(application_id="org.blofeld_emu.DisplayTest",
                                  flags=Gio.ApplicationFlags.NON_UNIQUE)
    application.register(None)
    runtime = directory / "new-session/runtime"
    view = DisplayWindow(application, runtime)
    view.window.present()
    try:
        until(lambda: view.draws > 0)
        require(view.frame is None and "Waiting" in view.status.get_text(),
                "GUI starts before runtime/emulator exists")
        target = runtime / "blofeld-emu"
        target.mkdir(parents=True)
        replace(target, HEADER + expected.bits)
        until(lambda: view.frame == expected and view.draws > 1)
        dark = Frame(b"\xff" * 1024)
        light = Frame(bytes(1024))
        for frame in (dark, light, dark):
            replace(target, HEADER + frame.bits)
        until(lambda: view.frame == dark)
        replace(target, b"broken")
        until(lambda: view.frame is None and "unavailable" in view.status.get_text())
        replace(target, HEADER + expected.bits)
        until(lambda: view.frame == expected)
        (target / "display.pbm").unlink()
        until(lambda: view.frame is None and "Waiting" in view.status.get_text())
        old = runtime / "old-display"
        target.rename(old)
        target.mkdir()
        replace(target, HEADER + light.bits)
        until(lambda: view.frame == light)
        replace(old, HEADER + dark.bits)
        require(read_frame(view.watch.path) == light, "Recreated directory owns current frames")
        shutil.rmtree(runtime)
        until(lambda: view.frame is None)
        target.mkdir(parents=True)
        replace(target, HEADER + expected.bits)
        until(lambda: view.frame == expected)
        require(view.watch.monitors and len(view.watch.monitors) == 2,
                "Current directory and its parent are monitored after recovery")
        # Save a real GTK widget rendering only when explicitly requested by the
        # developer test environment; this is not a product screenshot stream.
        preview = os.environ.get("BLOFELD_GUI_TEST_PREVIEW")
        if preview:
            until(lambda: view.draws > 2)
            paintable = Gtk.WidgetPaintable.new(view.window)
            snapshot = Gtk.Snapshot.new()
            paintable.snapshot(snapshot, view.window.get_width(), view.window.get_height())
            node = snapshot.to_node()
            require(node is not None, "GTK window snapshot contains rendered content")
            texture = view.window.get_renderer().render_texture(node, None)
            require(texture.save_to_png(preview), "Save actual GTK preview")
    finally:
        view.watch.close()
        view.window.destroy()
        require(not view.watch.monitors and not view.watch.pending, "GUI shutdown releases all watchers")
    print("GTK display: initial absence, atomic frames, redraw, invalid input, deletion and directory/session recreation passed")


def main():
    if "--gtk" not in sys.argv:
        if not shutil.which("xvfb-run") or not importlib.util.find_spec("gi"):
            print("SKIP GTK display: install GTK4/PyGObject and Xvfb")
            return 77
        probe = subprocess.run([sys.executable, "-c", "import gi; gi.require_version('Gtk','4.0'); import cairo"],
                               capture_output=True, check=False)
        if probe.returncode:
            print("SKIP GTK display: GTK4/PyGObject/Cairo unavailable")
            return 77
        return subprocess.run(["xvfb-run", "-a", sys.executable, str(Path(__file__)), "--gtk"],
                              check=False, timeout=60).returncode
    with tempfile.TemporaryDirectory(prefix="blofeld-display-gui-") as temporary:
        directory = Path(temporary)
        expected = parser(directory)
        gtk_test(directory, expected)
    return 0


if __name__ == "__main__":
    sys.exit(main())
