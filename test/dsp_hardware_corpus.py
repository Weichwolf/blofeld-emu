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


def sparse_key(key):
    return re.fullmatch(r"(?:[xyp]m[0-9a-f]{4}|s[hl]0[1-9a-f])", key) is not None


def parse_dump(path):
    cases = {}
    current = None
    for line in path.read_text().splitlines():
        fields = line.split()
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
    if current is not None or not cases:
        raise ValueError(f"Truncated or empty dump: {path}")
    return cases


def validate_capture(expected, observations):
    if observations == REGISTER_CAPTURE:
        if any(key not in REGISTERS for case in expected.values() for key in case):
            raise ValueError("Register-only capture contains undeclared memory/stack observations")
    elif observations == FULL_CAPTURE:
        for prefix in ("xm", "sh"):
            if not any(key.startswith(prefix) for case in expected.values() for key in case):
                raise ValueError("Pinned silicon capture lacks promised memory/stack observations")
    else:
        raise ValueError(f"Unsupported silicon capture observations: {observations}")


def compare(actual, expected, *, observations):
    if observations not in (REGISTER_CAPTURE, FULL_CAPTURE):
        raise ValueError(f"Unsupported silicon capture observations: {observations}")
    if list(actual) != list(expected):
        raise AssertionError("Native DSP did not reach every hardware corpus case in order")
    comparisons = 0
    for name, wanted in expected.items():
        observed = actual[name]
        keys = wanted.keys() | observed.keys()
        if observations == REGISTER_CAPTURE:
            if wanted.keys() - REGISTERS:
                raise ValueError("Register-only capture contains undeclared memory/stack observations")
            keys &= REGISTERS
        elif missing := STACK_FIELDS - observed.keys():
            raise AssertionError(f"{name}: incomplete native stack observation: {sorted(missing)}")
        for key in sorted(keys):
            default = 0 if sparse_key(key) else None
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
        target = output / f"{bank['name']}.native.txt"
        subprocess.run([binary, str(directory), str(target)], check=True)
        expected = parse_dump(directory / "results_hw.txt")
        actual = parse_dump(target)
        if len(expected) != bank["cases"]:
            raise ValueError("Pinned silicon case count mismatch")
        validate_capture(expected, bank["observations"])
        comparisons = compare(actual, expected, observations=bank["observations"])
        print(f"{bank['name']}: {len(expected)} cases, {comparisons} matching observed values "
              f"({bank['observations']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
