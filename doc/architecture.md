# Architecture

The emulated MCU starts the DSP during the original Blofeld Desktop 1.25 boot.
The MCU handles controls, MIDI, the LCD and device state. The DSP runs the synth
and produces stereo PCM.

## Preparation

```text
Waldorf downloads + NXP bootstrap listing + project catalogues
                           │ blofeld-profile
                           ▼
                  Local Desktop profile
                    │              └── persistent storage ──────┐
     firmware, ROM, catalogues                                   │
                    │ --prepare (LLVM 19)                        ▼
                    └──▶ Native MCU/DSP modules ────────▶ Instrument VM

Live MIDI or MIDI file ──▶ MCU ◀── timed device I/O ──▶ DSP ──▶ Live audio or WAV
                            │
                            └──▶ LCD viewer (live)
```

The CMake target `blofeld-prepare` builds the emulator, assembles the local
profile with `blofeld-profile`, and runs `--prepare`. The last step compiles and
validates the firmware and device bindings for the host, including the DSP code
variants selected by the original loader. The cache stores the native modules.

## Components

| Area | Role |
| --- | --- |
| [`src/app`](../src/app) | Profile loading, native bindings, live and offline sessions, LCD viewer |
| [`src/vi`](../src/vi) | Blofeld memory, storage, display and device contracts |
| [`src/host`](../src/host) | Native linking, files, scheduling and audio/MIDI transport |
| cpu12 and dsp56300 | Independent CPU translators compiled through LLVM 19 |

Startup loads the prepared modules and executes the original reset and boot.
Each core runs on a separate host thread. The two cores communicate through
timestamped device channels. The host's audio demand paces live execution.
Offline rendering follows the MIDI file's timeline and writes the DSP's PCM
to WAV.

Linux uses JACK for audio and MIDI. macOS uses Core Audio and Core MIDI.
The host callbacks exchange bounded audio and MIDI buffers. The LCD viewer
runs independently. Device settings and sample storage live in the selected
profile's persistent images.

On Linux x86-64, two workers run selected original DSP voice code on private
register and RAM snapshots. The DSP owner applies completed work in guest order
after checking the original inputs, the memory mapping and pending device events.

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
