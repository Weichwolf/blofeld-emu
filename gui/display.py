# SPDX-License-Identifier: GPL-3.0-only
"""Small native GTK4 viewer for the firmware-driven Blofeld LCD."""

import argparse
import os
from pathlib import Path
import signal
import sys

import cairo
import gi

gi.require_version("Gtk", "4.0")
from gi.repository import Gio, GLib, Gtk, Pango

from .frame import HEIGHT, WIDTH, runtime_directory
from .watch import DisplayWatch


def paint(context, width, height, frame):
    context.set_source_rgb(0.12, 0.12, 0.12)
    context.paint()
    if frame is None:
        return
    scale = max(1, min(width // WIDTH, height // HEIGHT))
    context.save()
    context.translate((width - WIDTH * scale) // 2, (height - HEIGHT * scale) // 2)
    context.scale(scale, scale)
    context.set_antialias(cairo.ANTIALIAS_NONE)
    context.set_source_rgb(0.94, 0.95, 0.96)
    context.rectangle(0, 0, WIDTH, HEIGHT)
    context.fill()
    context.set_source_rgb(0.04, 0.05, 0.06)
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if frame.on(x, y):
                context.rectangle(x, y, 1, 1)
    context.fill()
    context.restore()


class DisplayWindow:
    def __init__(self, application, runtime):
        self.frame = None
        self.draws = 0
        self.window = Gtk.ApplicationWindow(application=application, title="Blofeld Display")
        self.window.set_default_size(552, 324)
        self.window.set_size_request(296, 196)
        header = Gtk.HeaderBar()
        self.window.set_titlebar(header)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        for side in ("top", "bottom", "start", "end"):
            getattr(box, f"set_margin_{side}")(12)
        self.area = Gtk.DrawingArea(hexpand=True, vexpand=True)
        self.area.set_content_width(WIDTH * 4)
        self.area.set_content_height(HEIGHT * 4)
        self.area.set_draw_func(self._draw)
        self.status = Gtk.Label(label="Waiting for Blofeld display…", xalign=0)
        self.status.set_ellipsize(Pango.EllipsizeMode.END)
        self.status.set_tooltip_text(str(Path(runtime) / "blofeld-emu/display.pbm"))
        box.append(self.area)
        box.append(self.status)
        self.window.set_child(box)
        self.watch = DisplayWatch(runtime, self._changed)
        self.window.connect("close-request", self._close)

    def _changed(self, frame, message):
        if frame != self.frame:
            self.frame = frame
            self.area.queue_draw()
        self.status.set_text(message)

    def _draw(self, _area, context, width, height):
        paint(context, width, height, self.frame)
        self.draws += 1

    def _close(self, _window):
        self.watch.close()
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-dir", type=Path, help="XDG runtime root (default: current user)")
    arguments = parser.parse_args()
    if os.getuid() == 0:
        parser.exit(1, "Run the display viewer as a regular user.\n")
    try:
        runtime = arguments.runtime_dir or runtime_directory()
        if not runtime.is_absolute():
            raise ValueError("The runtime directory must be absolute")
    except ValueError as error:
        parser.exit(1, f"{error}\n")
    application = Gtk.Application(application_id="org.blofeld_emu.Display",
                                  flags=Gio.ApplicationFlags.NON_UNIQUE)
    windows = []

    def activate(app):
        if not windows:
            windows.append(DisplayWindow(app, runtime))
        windows[0].window.present()

    application.connect("activate", activate)
    def stop():
        application.quit()
        return GLib.SOURCE_CONTINUE

    sources = [GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signum, stop)
               for signum in (signal.SIGINT, signal.SIGTERM)]
    try:
        return application.run([])
    finally:
        for window in windows:
            window.watch.close()
            window.window.destroy()
        for source in sources:
            GLib.source_remove(source)


if __name__ == "__main__":
    sys.exit(main())
