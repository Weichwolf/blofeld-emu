# SPDX-License-Identifier: GPL-3.0-only
"""Isolated PipeWire Pulse service and policy-only WirePlumber; no device discovery."""

import contextlib
import os
from pathlib import Path
import shutil
import subprocess
import sysconfig

from host_audio import require, terminate, until


def reference_tools():
    prefix = os.environ.get("BLOFELD_PIPEWIRE_TOOL_PREFIX")
    if prefix:
        root = Path(prefix).resolve(strict=True)
        programs = [root / "bin" / name for name in ("pipewire", "wireplumber")]
        require(all(os.access(program, os.X_OK) for program in programs),
                "The configured private PipeWire prefix must contain both reference tools")
        return programs, root
    programs = [shutil.which(name) for name in ("pipewire", "wireplumber")]
    return programs, None


def environment(directory, prefix):
    runtime = directory / "runtime"
    runtime.mkdir(mode=0o700)
    for name in ("config", "state", "data", "cache"):
        (directory / name).mkdir(mode=0o700)
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(("PIPEWIRE_", "WIREPLUMBER_", "PW_", "SPA_", "PULSE_"))}
    env.update(XDG_RUNTIME_DIR=str(runtime), XDG_CONFIG_HOME=str(directory / "config"),
               XDG_STATE_HOME=str(directory / "state"), XDG_DATA_HOME=str(directory / "data"),
               XDG_CACHE_HOME=str(directory / "cache"), XDG_CONFIG_DIRS=str(directory / "config"),
               LC_ALL="C", PIPEWIRE_RUNTIME_DIR=str(runtime),
               PIPEWIRE_REMOTE="blofeld-reference", PIPEWIRE_CONFIG_PREFIX="",
               PIPEWIRE_CONFIG_DIR=str(directory / "config"),
               DBUS_SESSION_BUS_ADDRESS="unix:path=" + str(runtime / "no-session-bus"),
               DBUS_SYSTEM_BUS_ADDRESS="unix:path=" + str(runtime / "no-system-bus"),
               PULSE_SERVER="unix:" + str(runtime / "native"), PULSE_RUNTIME_PATH=str(runtime))
    if prefix:
        libraries = prefix / "lib" / sysconfig.get_config_var("MULTIARCH")
        env.update(LD_LIBRARY_PATH=str(libraries),
                   PIPEWIRE_MODULE_DIR=str(libraries / "pipewire-0.3"),
                   WIREPLUMBER_MODULE_DIR=str(libraries / "wireplumber-0.5"),
                   WIREPLUMBER_CONFIG_DIR=str(prefix / "share/wireplumber"),
                   WIREPLUMBER_DATA_DIR=str(prefix / "share/wireplumber"))
    return env


def configuration(directory, minimum_request):
    require(minimum_request in (0, 1024), "Unknown directed PipeWire request-floor case")
    config = directory / "config" / "reference.conf"
    floor = f"pulse.min.req = {minimum_request}/48000" if minimum_request else ""
    config.write_text("""context.properties = {
 core.daemon = true
 core.name = blofeld-reference
 support.dbus = false
 mem.allow-mlock = false
 default.clock.rate = 48000
}
context.spa-libs = {
 audio.convert.* = audioconvert/libspa-audioconvert
 audio.adapt = audioconvert/libspa-audioconvert
 support.* = support/libspa-support
}
context.modules = [
 { name = libpipewire-module-protocol-native }
 { name = libpipewire-module-spa-node-factory }
 { name = libpipewire-module-metadata }
 { name = libpipewire-module-client-node }
 { name = libpipewire-module-adapter }
 { name = libpipewire-module-link-factory }
 { name = libpipewire-module-access }
 { name = libpipewire-module-protocol-pulse }
]
pulse.properties = {
 server.address = [ "unix:""" + str(directory / "runtime/native") + """" ]
 """ + floor + """
}
stream.properties = { adapter.auto-port-config = { mode = dsp } }
context.objects = [
 { factory = spa-node-factory args = {
   factory.name = support.node.driver
   node.name = Dummy-Driver
   node.group = pipewire.dummy
   priority.driver = 20000
 } }
]
""")
    # Clients need the native transport, not the daemon's Pulse listener/config.
    (directory / "config/client.conf").write_text("""context.properties = {
 support.dbus = false
 mem.allow-mlock = false
}
context.spa-libs = {
 audio.convert.* = audioconvert/libspa-audioconvert
 support.* = support/libspa-support
}
context.modules = [
 { name = libpipewire-module-protocol-native }
 { name = libpipewire-module-client-node }
 { name = libpipewire-module-adapter }
 { name = libpipewire-module-metadata }
]
""")
    return config


@contextlib.contextmanager
def service(directory, alternate=False, minimum_request=0):
    directory = directory.resolve()
    programs, prefix = reference_tools()
    require(all(programs), "PipeWire reference tools are unavailable")
    env = environment(directory, prefix)
    config = configuration(directory, minimum_request)
    endpoint = directory / "runtime/native"
    server = "unix:" + str(endpoint)
    children = []
    with contextlib.ExitStack() as stack:
        def launch(name, arguments):
            log = stack.enter_context((directory / (name + ".log")).open("w"))
            child = subprocess.Popen(arguments, env=env, stdout=log, stderr=subprocess.STDOUT)
            children.append(child)
            return child
        try:
            daemon = launch("pipewire", [str(programs[0]), "-c", config.name])
            until(endpoint.exists, daemon, "the private PipeWire Pulse socket")
            policy = launch("wireplumber", [str(programs[1]), "-p", "policy"])

            def pactl(*args):
                require(policy.poll() is None, "Private WirePlumber policy exited")
                return subprocess.check_output(["pactl", "--server", server, *args],
                                               env=env, text=True, timeout=4)
            pactl("load-module", "module-null-sink", "sink_name=fixture", "format=s32le",
                  "rate=48000", "channels=2", "channel_map=front-left,front-right")
            until(lambda: "Default Sink: fixture" in pactl("info"), daemon,
                  "the private default null sink")
            pactl("set-default-sink", "fixture")
            if alternate:
                pactl("load-module", "module-null-sink", "sink_name=alternate", "format=s16le",
                      "rate=44100", "channels=2", "channel_map=front-right,front-left")
            info = pactl("info")
            require("on PipeWire" in info, "The reference service must actually be PipeWire")
            (directory / "server-info.txt").write_text(info)
            (directory / "sinks.json").write_text(pactl("--format=json", "list", "sinks"))
            yield daemon, server, env
        finally:
            for child in reversed(children):
                terminate(child)
