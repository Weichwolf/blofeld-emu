# SPDX-License-Identifier: GPL-3.0-only
"""Compare strict native DSP observations with pinned external silicon captures."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


REGISTERS = {
    "pc", "sr", "omr", "la", "lc", "sp", "ssh", "ssl", "ep", "sz", "sc", "vba",
    "x0", "x1", "y0", "y1", "a0", "a1", "a2", "b0", "b1", "b2",
    *(f"{prefix}{index}" for prefix in "rnm" for index in range(8)),
}
STACK_FIELDS = {f"s{half}{slot:02x}" for half in "hl" for slot in range(1, 16)}
REGISTER_CAPTURE = "registers"
FULL_CAPTURE = "registers-memory-stack"
MEMORY_CAPTURE = "registers-memory"
STACK_CAPTURE = "registers-stack"
SCOPES = {REGISTER_CAPTURE: (False, False), MEMORY_CAPTURE: (True, False),
          STACK_CAPTURE: (False, True), FULL_CAPTURE: (True, True)}


def sparse_key(key):
    return re.fullmatch(r"(?:[xyp]m[0-9a-f]{4}|s[hl]0[1-9a-f])", key) is not None


def parse_fill(fields):
    if len(fields) != 7 or fields[0] != "fill" or any(
            re.fullmatch(r"[0-9a-fA-F]{1,6}", value) is None for value in fields[1:]):
        raise ValueError("Malformed DSP corpus fill")
    values = tuple(int(value, 16) for value in fields[1:])
    if not values[0] & 1 or not values[2] & 1 or max(values[4:]) > 0xf1:
        raise ValueError("Unsupported DSP corpus fill")
    return values


def sparse_default(key, fill):
    if not sparse_key(key):
        return None
    if fill is None or key.startswith("pm"):
        return 0
    address = int(key[2:], 16)
    if key.startswith(("xm", "ym")):
        index = 0 if key.startswith("xm") else 2
        return (fill[index] * address + fill[index + 1]) & 0xffffff
    index = 4 if key.startswith("sh") else 5
    return fill[index] + address - 1


def parse_dump(path, *, fill=None):
    cases = {}
    current = None
    fill_seen = False
    for line in path.read_text().splitlines():
        fields = line.split()
        if fields[:1] == ["fill"]:
            if cases or fill_seen or parse_fill(fields) != fill:
                raise ValueError(f"Unexpected, mismatched or duplicate fill in {path}")
            fill_seen = True
            continue
        if fields == ["end"]:
            if current is None:
                raise ValueError(f"Unmatched end in {path}")
            if not REGISTERS - {"ssh", "ssl"} <= current.keys():
                raise ValueError(f"Missing architectural register in {path}")
            if current["sp"] != 0 and (current.get("ssh") is None or current.get("ssl") is None):
                raise ValueError(f"Missing nonempty stack top in {path}")
            current = None
            continue
        if len(fields) != 2:
            raise ValueError(f"Malformed dump line in {path}: {line}")
        key, value = fields
        if key == "case":
            if current is not None or value in cases:
                raise ValueError(f"Duplicate or unclosed case in {path}: {value}")
            cases[value] = current = {}
            continue
        if current is None or key in current or not (key in REGISTERS or sparse_key(key)):
            raise ValueError(f"Unsupported or duplicate dump field in {path}: {key}")
        if value == "-" and key in {"ssh", "ssl"}:
            current[key] = None
        else:
            parsed = int(value, 16)
            if not 0 <= parsed <= 0xffffff:
                raise ValueError(f"Out-of-range dump field in {path}: {key}")
            current[key] = parsed
    if fill is not None and not fill_seen:
        raise ValueError(f"Missing DSP corpus fill in {path}")
    if current is not None or not cases:
        raise ValueError(f"Truncated or empty dump: {path}")
    return cases


def observed_key(key, observations):
    memory, stack = SCOPES[observations]
    return (key in REGISTERS or (stack and key in STACK_FIELDS) or
            (memory and key.startswith(("xm", "ym", "pm")) and sparse_key(key)))


def validate_capture(expected, observations):
    if observations not in SCOPES:
        raise ValueError(f"Unsupported silicon capture observations: {observations}")
    memory, stack = SCOPES[observations]
    for fields in expected.values():
        if any(not observed_key(key, observations) for key in fields):
            raise ValueError("Capture contains undeclared memory/stack observations")
    for enabled, prefix in ((memory, "xm"), (stack, "sh")):
        if enabled and not any(key.startswith(prefix) for case in expected.values() for key in case):
            raise ValueError("Pinned silicon capture lacks promised memory/stack observations")


def compare(actual, expected, *, observations, fill=None):
    if observations not in SCOPES:
        raise ValueError(f"Unsupported silicon capture observations: {observations}")
    if list(actual) != list(expected):
        raise AssertionError("Native DSP did not reach every hardware corpus case in order")
    comparisons = 0
    for name, wanted in expected.items():
        observed = actual[name]
        if any(not observed_key(key, observations) for key in wanted):
            raise ValueError("Capture contains undeclared memory/stack observations")
        keys = {key for key in wanted.keys() | observed.keys() if observed_key(key, observations)}
        if SCOPES[observations][1] and (missing := STACK_FIELDS - observed.keys()):
            raise AssertionError(f"{name}: incomplete native stack observation: {sorted(missing)}")
        for key in sorted(keys):
            default = sparse_default(key, fill)
            want = wanted.get(key, default)
            got = observed.get(key, default)
            if got != want:
                raise AssertionError(f"{name} {key}: native={got!r}, silicon={want!r}")
            if want is not None:
                comparisons += 1
    return comparisons


def main():
    binary, reference, manifest_path, output_path = sys.argv[1:]
    reference = Path(reference)
    if not reference.is_dir():
        print("SKIP: pinned external MCPX silicon corpus is absent; see doc/dsp-validation.txt")
        return 77
    manifest = json.loads(Path(manifest_path).read_text())
    output = Path(output_path)
    output.mkdir(parents=True, exist_ok=True)
    for bank in manifest["banks"]:
        directory = reference / bank["name"]
        for name, identity in bank["files"].items():
            data = (directory / name).read_bytes()
            if len(data) != identity["bytes"] or hashlib.sha256(data).hexdigest() != identity["sha256"]:
                raise ValueError(f"External silicon corpus identity mismatch: {directory / name}")
        fill = tuple(bank["fill"]) if "fill" in bank else None
        first = (directory / "corpus.meta").read_text().splitlines()[0].split()
        metadata_fill = parse_fill(first) if first[:1] == ["fill"] else None
        if fill != metadata_fill:
            raise ValueError("Manifest and metadata fill disagree")
        target = output / f"{bank['name']}.native.txt"
        subprocess.run([binary, str(directory), str(target)], check=True)
        expected = parse_dump(directory / "results_hw.txt", fill=fill)
        actual = parse_dump(target, fill=fill)
        if len(expected) != bank["cases"]:
            raise ValueError("Pinned silicon case count mismatch")
        validate_capture(expected, bank["observations"])
        comparisons = compare(actual, expected, observations=bank["observations"], fill=fill)
        print(f"{bank['name']}: {len(expected)} cases, {comparisons} matching observed values "
              f"({bank['observations']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
