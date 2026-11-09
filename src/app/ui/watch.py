# SPDX-License-Identifier: GPL-3.0-only
"""Watch directory replacements and coalesce presentation updates on the GTK loop."""

from pathlib import Path

from gi.repository import Gio, GLib

from .frame import publisher_present, read_frame


class DisplayWatch:
    def __init__(self, runtime, changed):
        self.directory = Path(runtime).absolute() / "blofeld-emu"
        self.path = self.directory / "display.pbm"
        self.presence = self.directory / "display.live"
        self.changed = changed
        self.monitors = {}
        self.pending = 0
        self.closed = False
        self.refresh()
        self.poll = GLib.timeout_add(1000, self._poll)

    def _poll(self):
        self.schedule()
        return GLib.SOURCE_CONTINUE

    def _watch(self):
        nearest = self.directory
        while not nearest.is_dir() and nearest != nearest.parent:
            nearest = nearest.parent
        paths = {nearest, nearest.parent}
        desired = {}
        for path in paths:
            information = path.stat()
            desired[path] = (information.st_dev, information.st_ino)
        for path, (identity, monitor) in list(self.monitors.items()):
            if desired.get(path) != identity:
                monitor.cancel()
                del self.monitors[path]
        for path, identity in desired.items():
            if path not in self.monitors:
                monitor = Gio.File.new_for_path(str(path)).monitor_directory(
                    Gio.FileMonitorFlags.WATCH_MOVES, None)
                monitor.set_rate_limit(16)
                monitor.connect("changed", self._event)
                self.monitors[path] = identity, monitor

    def _event(self, _monitor, file, other, _event):
        for entry in (file, other):
            if entry is None or entry.get_path() is None:
                continue
            path = Path(entry.get_path())
            if path in (self.path, self.presence) or self.directory.is_relative_to(path):
                self.schedule()
                return

    def schedule(self):
        if not self.closed and not self.pending:
            self.pending = GLib.timeout_add(16, self.refresh)

    def refresh(self):
        self.pending = 0
        if self.closed:
            return GLib.SOURCE_REMOVE
        running = False
        try:
            self._watch()
            running = publisher_present(self.presence)
            if running:
                frame = read_frame(self.path)
                message = ""
            else:
                frame, message = None, "Emulator off"
        except FileNotFoundError:
            frame = None
            message = ("Waiting for original LCD…" if running else
                       "Emulator off")
        except (OSError, ValueError, GLib.Error) as error:
            frame, message = None, f"Display unavailable: {error}"
        self.changed(frame, message)
        return GLib.SOURCE_REMOVE

    def close(self):
        self.closed = True
        if self.poll:
            GLib.source_remove(self.poll)
            self.poll = 0
        if self.pending:
            GLib.source_remove(self.pending)
            self.pending = 0
        for _, monitor in self.monitors.values():
            monitor.cancel()
        self.monitors.clear()
