# SPDX-License-Identifier: GPL-3.0-only
"""Prepare fixed own DSP pipeline sequences for an optional offline reference tool."""
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

    def pad(count=8):
        for _ in range(count):
            emit("nop")

    def case(kind, producer, body, gap, expected, register):
        emit("move #>$c00300,sr")
        emit("move #>$100,r0")
        emit("move #>$ffffff,m0")
        emit("move #>$ffffff,x1" if kind == "agu_m0" else "move #>$100,x1")
        emit("move #>$400000,x0")
        emit("move #>$400000,y0")
        emit("move #>$13579b,b1")
        pad()
        producer_index = len(instructions)
        emit(producer)
        pad(gap)
        body_index = len(instructions)
        emit(body)
        pad()
        cases.append(dict(kind=kind, gap=gap, producer=producer, body=body,
                          producer_index=producer_index, body_index=body_index,
                          window_start=producer_index - 6, window_end=body_index + 5,
                          expected_value=expected, expected_register=register))

    emit("move #>$305,omr")
    emit("move #>$c00300,sr")
    pad()
    for register in ("r0", "n0", "m0", "r1"):
        for gap in range(4):
            case("agu_" + register, "move x1," + register, "move x:(r0),y1", gap, 0x13579b, "y1")
    for gap in (0, 1, 2, 8):
        case("arithmetic", "mpy x0,y0,a", "move a1,x1", gap, 0x200000, "x1")
        case("arithmetic_other", "mpy x0,y0,a", "move b1,x1", gap, 0x13579b, "x1")
        case("transfer", "move x0,a1", "move a1,x1", gap, 0x400000, "x1")
        case("status_move", "mpy x0,y0,a", "move sr,x1", gap, 0xc00310, "x1")
        case("status_bit", "mpy x0,y0,a", "btst #2,sr", gap, 0xc00310, "sr")
    case("status_move_control", "nop", "move sr,x1", 0, 0xc00300, "x1")
    case("status_bit_control", "nop", "btst #2,sr", 0, 0xc00300, "sr")
    pad()
    source.append("        end")
    return "\n".join(source) + "\n", dict(cases=cases, instructions=instructions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, nargs="?",
                        default=Path(__file__).resolve().parents[1] / "doc/local/dsp-pipeline-reference",
                        help="Local output directory; this tool assembles or executes nothing")
    args = parser.parse_args()
    source, metadata = reference_program()
    args.directory.mkdir(parents=True, exist_ok=True)
    (args.directory / "interlocks.asm").write_text(source)
    (args.directory / "cases.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print("Prepared", len(metadata["cases"]), "fixed cases and",
          len(metadata["instructions"]), "own instructions")


if __name__ == "__main__":
    main()
