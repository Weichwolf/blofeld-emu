# SPDX-License-Identifier: GPL-3.0-only
"""Generate six fixed own branch windows; assemble, execute and download nothing."""
import argparse
import json
from pathlib import Path


def reference_program():
    source = ["; SPDX-License-Identifier: GPL-3.0-only", "        org p:$100"]
    instructions = []
    cases = []

    def emit(text):
        source.append("        " + text)
        instructions.append(text)
        return len(instructions) - 1

    def quiet():
        for _ in range(8):
            emit("nop")

    emit("move #>$305,omr")
    emit("move #>$c00300,sr")
    quiet()
    forms = [("jset", 3, "skip"), ("jclr", 3, "skip"),
             ("jset", 2, "skip"), ("jclr", 2, "skip"),
             ("jset", 2, "sequential"), ("jclr", 2, "sequential")]
    for index, (operation, bit, style) in enumerate(forms):
        emit("move #>0,x0")
        quiet()
        before = len(instructions) - 1
        label = f"case{index}_target"
        branch = emit(f"{operation} #{bit},omr,{label}")
        marker = emit("move #>$654321,x0") if style == "skip" else None
        source.append(label + ":")
        target = len(instructions)
        quiet()
        taken = bool((0x305 >> bit) & 1) == (operation == "jset")
        executes_marker = marker is not None and not taken
        cases.append({"name": f"{operation}_omr_bit{bit}_{style}", "op": operation,
                      "bit": bit, "taken": taken, "branch_index": branch,
                      "target_index": target, "marker_index": marker,
                      "before_anchor": before, "after_anchor": target + 5,
                      "expected_x0": 0x654321 if executes_marker else 0,
                      "expected_anchor_clocks": 4 + int(not taken) +
                                                (2 if executes_marker else 0) + 6})
    source.append("        end")
    return "\n".join(source) + "\n", {"instructions": instructions, "cases": cases,
        "scope": "Own initial-map OMR register-branch windows; FM TableA-1 baseline4 and "
                 "A.2.6.3 extra false-conditional successor Decode clock. Not silicon phases."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, nargs="?",
                        default=Path(__file__).resolve().parents[1] /
                                "doc/local/dsp-branch-reference-generated")
    args = parser.parse_args()
    source, metadata = reference_program()
    args.directory.mkdir(parents=True, exist_ok=True)
    (args.directory / "branches.asm").write_text(source)
    (args.directory / "cases.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print("Prepared", len(metadata["cases"]), "fixed windows and",
          len(metadata["instructions"]), "own static instructions")


if __name__ == "__main__":
    main()
