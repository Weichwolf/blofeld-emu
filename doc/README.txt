Project documentation and references

architecture.md                 Current target architecture
../OBJECTIVE.md                  Complete instrument acceptance
../ACCEPTANCE.tsv                Separate native-platform qualification results
firmware-fusion-candidates.json  Hash-pinned semantic translation candidates
runtime-references.json          Binary translation, native runtime and causality references
hardware/sources.json            Manufacturer hardware specifications
hardware/board-photos.json       Primary board photographs
function-references.json         Manufacturer function documentation
downloads.json                  Official firmware/manual/tool input identities
midi-references.json             MIDI/SMF and host transport references
library/sources.json            Official sample-library identities
produktbilder/quellen.json       Manufacturer product-image references
reference-tools.json            Manufacturer development-tool references
dsp-corpus.json                 External silicon-capture source identities

Code is the current implementation; architecture.md is the intended target.
Keep necessary source/size/hash provenance. Git contains history; no diaries,
obsolete attempts or duplicate status archives. Manufacturer inputs/private
analysis stay untracked in build/local/; generated artifacts stay in build/.

Coordinate only in private https://github.com/Weichwolf/blofeld-src/issues:
23 whole-instrument performance/platform ownership; 28 DSP AOT/entry closure;
25 arithmetic/filter/LFO; 26 MIDI/SysEx; 27 MCU IRQ/display/USB ordering;
29 documentation cleanup. Existing specialist topics remain authoritative:
19 voice/audio; 9 SCI; 20 USB; 4 samples/NAND; 5 peer turnaround; 21 Keyboard.
