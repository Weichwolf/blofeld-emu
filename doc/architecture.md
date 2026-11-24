# Architecture

Blofeld-emu runs the original Blofeld Desktop 1.25 firmware with SL in a
standalone instrument. The MCU handles controls, MIDI and device state. The DSP
runs the synth and produces the original stereo PCM.

```text
 original firmware + code catalogues ──prepare──▶ native MCU/DSP modules
                                                        │
 profile + persistent images ──────────────────────────▶│
                                                        ▼
                         MIDI ◀──────────────────────▶ app
                                                        │
                                                   virtual instrument
                                               ┌─────────┴─────────┐
                                               │ MCU ⇄ channels ⇄ DSP │
                                               └─────────┬─────────┘
                                                 LCD ────┤
                                                 PCM ────┴──▶ audio or WAV
```

## Components

| Area | Role |
| --- | --- |
| [`src/app`](../src/app) | Profile loading, native bindings, live and offline sessions, LCD viewer |
| [`src/vi`](../src/vi) | Blofeld memory, storage, display and device contracts |
| [`src/host`](../src/host) | Native linking, files, scheduling and audio/MIDI transport |
| cpu12 and dsp56300 | Independent CPU translators compiled through LLVM 19 |

`--prepare` translates and validates the firmware modules before playback.
Startup loads the prepared objects, creates fresh MCU and DSP state, and
executes the original reset and boot. The cores run on separate owners and
communicate through timestamped device channels. The host's audio demand
paces live execution. Offline playback uses the MIDI file's timeline and
writes the produced PCM to WAV.

Linux uses JACK for audio and MIDI. macOS uses Core Audio and Core MIDI.
The host callbacks exchange bounded audio and MIDI buffers. The LCD viewer
runs independently. Device settings and sample storage live in the selected
profile's persistent images.

## DSP memory

The product uses host page protections for direct P/X/Y accesses. The checked
mode also validates word initialization. The ordinary RAM differences cover
low P `000000..000fff`, X `000000..002fff`, Y `000000..001fff`, and the
OMR-dependent high X/Y windows and P aliases. Read-only reserved Reverb X
`010000..015fff` in MS=1/MSW=11 has identical bounds in both modes.

| Checked contents | 4 KiB direct page | 16 KiB direct page |
| --- | --- | --- |
| P main boot ROM `ff0000..ff00bf` | `ff0000..ff03ff` | `ff0000..ff0fff` |
| P EEPROM ROM `ffff00..ffff50` | `fffc00..ffffff` | `fff000..ffffff` |
| Y reserved word `016000` | `016000..0163ff` | `016000..016fff` |
| Y reserved word `ffffff` | `fffc00..ffffff` | `fff000..ffffff` |

[Project objective](../OBJECTIVE.md) ·
[Qualification](../ACCEPTANCE.tsv) · [Reference index](README.txt)
