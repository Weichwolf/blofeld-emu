# SPDX-License-Identifier: GPL-3.0-only
"""Check the original boot prefix under nominal MOVE-to-AGU history."""

import json
from pathlib import Path
import sys

import original_dsp_boot
import original_dsp_clock


def main():
    inputs = [Path(name) for name in sys.argv[1:4]]
    frozen = {str(path): original_dsp_clock.identity(path) for path in inputs if path.is_file()}
    code = original_dsp_boot.main(extra=("agu",))
    if code:
        return code
    trace = Path(sys.argv[4])
    # ROM address preloads precede multi-clock polling/DO; stage R4 preload
    # precedes MOVEP/REP. The repeated L store updates its pointer, not a MOVE
    # register destination. Thus FM A.2.4 adds zero clocks throughout this prefix.
    # Any positive stall would contradict the independent complete nominal ledger.
    result = original_dsp_clock.verify(trace)
    assert all(original_dsp_clock.identity(Path(name)) == expected for name, expected in frozen.items())
    result.update(scope="Original prefix with nominal MOVE-to-AGU history; no physical pipeline claim",
                  inputs=frozen, expected_additional_agu_units=0)
    Path(str(trace) + ".verification.json").write_text(json.dumps(result, indent=2) + "\n")
    print("Original DSP boot prefix retains its independent ledger with explicit AGU history.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
