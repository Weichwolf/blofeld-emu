# SPDX-License-Identifier: GPL-3.0-only
"""Prepare own whole/partial accumulator status-pipeline reference sequences."""
import argparse
import json
from pathlib import Path


def reference_program():
    instructions = ["move #>$305,omr"] + ["nop"] * 8
    cases = []
    producers = [(part, "move " + part + ",x0", False)
                 for part in ("a0", "a1", "a2", "a", "b1", "b")]
    producers += [(reg + "_limit", "move " + reg + ",x0", True) for reg in ("a", "b")]
    for kind, producer, limited in producers:
        whole = kind in ("a", "b") or limited
        status = 0xc00340 if limited else 0xc00380 if whole else 0xc00300
        for consumer in ("move", "bit"):
            for gap in range(4):
                start = len(instructions)
                instructions.append("move #>$c00300,sr")
                for reg in ("a0", "a1", "a2", "b0", "b1", "b2"):
                    value = 0x400000 if not limited and reg in ("a1", "b1") else 0
                    if limited and reg == kind[0] + "2":
                        value = 1
                    instructions.append(f"move #>${value:x},{reg}")
                instructions.extend(["nop"] * 8)
                producer_index = len(instructions)
                instructions.extend([producer] + ["nop"] * gap)
                body_index = len(instructions)
                bit = 6 if limited else 7
                instructions.append("move sr,x1" if consumer == "move" else f"btst #{bit},sr")
                instructions.extend(["nop"] * 8)
                cases.append(dict(kind=kind, consumer=consumer, gap=gap, start_index=start,
                                  producer_index=producer_index, body_index=body_index,
                                  window_start=producer_index - 6, window_end=body_index + 5,
                                  expected_register="x1" if consumer == "move" else "sr",
                                  expected_value=status if consumer == "move" else status | ((status >> bit) & 1),
                                  expected_bus_value=0x7fffff if limited else
                                  0x400000 if kind in ("a1", "a", "b1", "b") else 0,
                                  expected_stall_cycles=max(2-gap, 0) if whole else 0))
    source = ["; SPDX-License-Identifier: GPL-3.0-only", "        org p:$100"]
    source.extend("        " + instruction for instruction in instructions)
    source.append("        end")
    return "\n".join(source) + "\n", dict(instructions=instructions, cases=cases)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    source, metadata = reference_program()
    args.directory.mkdir(parents=True, exist_ok=True)
    (args.directory / "interlocks.asm").write_text(source)
    (args.directory / "cases.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print("Prepared", len(metadata["cases"]), "own status-pipeline cases")


if __name__ == "__main__":
    main()
