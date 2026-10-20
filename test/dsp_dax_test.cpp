// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_dax.hpp"
#include "dax_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace blofeld::test::dax_test {
namespace {
void literal_waves() {
  for (unsigned selection = 0; selection < 4; ++selection) {
    DspDax serial;
    serial.observe(inputs(false, selection));
    serial.write(control, selection << 3U);
    constexpr unsigned all_b_flags = 0xe000;
    serial.write(non_audio, all_b_flags);
    serial.write(audio_b, 0); // The address alias never selects the A/B channel.
    serial.write(audio_b, sample_mask);
    serial.set_active(true);
    require(collect(serial, {.count = subframe_units, .selection = selection}) == zero_z,
            "Literal zero-word Z waveform is independent of the encoder/decoder round trip");
    require(collect(serial, {.count = subframe_units, .selection = selection}) == ones_y,
            "Literal all-ones audio/VUC/parity waveform follows on channel B");
    serial.write(non_audio, 0);
    serial.write(audio_a, 1);
    serial.write(audio_a, 0);
    require(collect(serial, {.count = subframe_units, .selection = selection}) == lsb_x,
            "A literal bit-zero impulse pins LSB-first audio order and parity position");
    static_cast<void>(collect(serial, {.count = subframe_units, .selection = selection}));
    serial.set_active(false);
    serial.write(audio_a, 0x800000);
    serial.write(audio_b, 0);
    serial.set_active(true);
    require(collect(serial, {.count = subframe_units, .selection = selection}) == msb_z,
            "A literal high-bit impulse and reset block start pin the other audio boundary");
    if (selection != 0) {
      require(rejects([&] { static_cast<void>(serial.read(status)); }).contains("ED55"),
              "Unknown slave XAUR reliability cannot become a successful hardware status read");
      require(serial.read(status, empty | last_block) == empty,
              "Known XADE/XBLK remain inspectable with the erratum bit excluded");
    }
  }
}

void dividers() {
  constexpr std::array periods{8U, 2U, 3U, 4U};
  for (unsigned selection = 0; selection < periods.size(); ++selection) {
    DspDax serial;
    prime(serial, selection);
    for (unsigned cycle = 1; cycle <= periods.at(selection) * 3; ++cycle) {
      serial.observe(inputs(true, selection));
      serial.observe(inputs(true, selection));
      serial.observe(inputs(false, selection));
      require(serial.half_bits() == cycle / periods.at(selection),
              "Repeated observations do not replace selected input edges or divider ratios");
    }
  }
}

void underrun_and_non_audio() {
  DspDax serial;
  prime(serial);
  static_cast<void>(collect(serial, frame_units));
  require(serial.read(status) == empty, "The first real frame upload requests new data");
  serial.write(audio_a, first_sample);
  serial.write(audio_a, other_sample);
  constexpr unsigned initial_flags = 0x8400;
  constexpr unsigned replacement_flags = 0x5400;
  serial.write(non_audio, initial_flags);
  const auto a = collect(serial, subframe_units);
  serial.write(non_audio, replacement_flags);
  const auto b = collect(serial, subframe_units);
  require(decode(a, 'X') == Decoded{.audio = first_sample, .flags = 1} &&
              decode(b, 'Y') == Decoded{.audio = other_sample, .flags = 4},
          "Channel B non-audio data is the frame-start snapshot, not a later XNADR write");
  const auto repeated = collect(serial, frame_units);
  require(decode(std::string_view(repeated).substr(0, subframe_units), 'X') ==
                  Decoded{.audio = first_sample, .flags = 5} &&
              decode(std::string_view(repeated).substr(subframe_units), 'Y') ==
                  Decoded{.audio = other_sample, .flags = 2},
          "A real empty frame retains both previous audio words and uploads current non-audio");
  require(serial.read(status) == (empty | underrun), "Master XAUR records the actual underrun");
  serial.write(audio_b, other_sample);
  require(serial.read(status) == (empty | underrun), "One write cannot clear an armed pair error");
  serial.write(audio_b, first_sample);
  require(serial.read(status) == 0, "Read then complete pair clears actual empty/underrun flags");
  const auto recovered = collect(serial, frame_units);
  require(decode(std::string_view(recovered).substr(0, subframe_units), 'X').audio ==
                  other_sample &&
              decode(std::string_view(recovered).substr(subframe_units), 'Y').audio == first_sample,
          "Both aliases retain the next alternating pair after underrun recovery");
}

void blocks_and_restart() {
  DspDax serial;
  prime(serial);
  constexpr unsigned frames = 193;
  constexpr unsigned block_frames = 192;
  constexpr unsigned last_frame = 191;
  for (unsigned frame = 0; frame < frames; ++frame) {
    const auto wire = collect(serial, frame_units);
    const char preamble = frame == 0 || frame == block_frames ? 'Z' : 'X';
    require(decode(std::string_view(wire).substr(0, subframe_units), preamble).audio == 0 &&
                decode(std::string_view(wire).substr(subframe_units), 'Y').audio == 0,
            "The full192-frame block and first reused frame have independently fixed preambles");
    require((serial.read(status) & last_block) == (frame >= last_frame ? last_block : 0U),
            "XBLK starts on frame191 and stays latched without a pair write");
  }
  serial.set_active(false);
  serial.write(audio_a, 0);
  serial.write(audio_a, 0);
  serial.set_active(true);
  static_cast<void>(collect(serial, subframe_units));
  serial.write(control, restart);
  static_cast<void>(collect(serial, subframe_units));
  require(serial.read(control) == restart, "XSB does not truncate the current B subframe");
  const auto restarted = collect(serial, subframe_units);
  require(restarted.substr(0, 8) == "00010111" && serial.read(control) == 0,
          "The next frame consumes XSB and begins a fresh Z block");
}

void unknowns_and_reset() {
  DspDax serial;
  require(serial.read(control) == 0 && serial.read(status) == 0,
          "XCTR/XSTR cold reset does not invent a data-ready flag");
  for (const auto reg : {non_audio, audio_a, audio_b}) {
    static_cast<void>(rejects([&] { static_cast<void>(serial.read(reg)); }));
  }
  serial.observe(inputs(false));
  serial.set_active(true);
  for (unsigned i = 0; i < 7; ++i) {
    serial.observe(inputs(true));
    serial.observe(inputs(false));
  }
  require(rejects([&] { serial.observe(inputs(true)); }).contains("XNADR"),
          "The first upload cannot synthesize uninitialized non-audio data");
  serial.write(non_audio, 0);
  require(rejects([&] { serial.observe(inputs(true)); }).contains("retained"),
          "An unprimed cold underrun cannot synthesize zero audio");
  serial.write(audio_a, first_sample);
  require(rejects([&] { serial.observe(inputs(true)); }).contains("partial"),
          "Underspecified partial-frame underrun fails visibly without advancing");
  serial.write(audio_a, other_sample);
  serial.observe(inputs(true));
  require(serial.half_bits() == 1 && !serial.output().level,
          "After explicit priming the rejected same edge emits exactly the first Z bit");
  serial.set_active(false);
  serial.write(non_audio, 0x400);
  serial.reset();
  serial.write(audio_a, 0);
  serial.write(audio_a, 0);
  serial.set_active(true);
  const auto kept = collect(serial, frame_units);
  require(decode(std::string_view(kept).substr(0, subframe_units), 'Z') ==
                  Decoded{.audio = 0, .flags = 1} &&
              decode(std::string_view(kept).substr(subframe_units), 'Y') ==
                  Decoded{.audio = 0, .flags = 0},
          "XNADR survives all resets while both old audio buffers are discarded");

  DspDax invalid;
  for (const unsigned value : {1U, 7U, 0x40U}) {
    static_cast<void>(rejects([&] { invalid.write(control, value); }));
  }
  static_cast<void>(rejects([&] { invalid.write(non_audio, 1); }));
  static_cast<void>(rejects([&] { invalid.write(status, 0); }));
  invalid.set_active(true);
  static_cast<void>(rejects([&] { static_cast<void>(invalid.output()); }));
  static_cast<void>(rejects([&] { invalid.write(control, 8); }));
  static_cast<void>(rejects([&] { invalid.observe({.core_clock = true}); }));
}

void pending_input() {
  DspDax serial;
  serial.observe(inputs(false));
  serial.write(non_audio, 0);
  serial.write(audio_a, 1);
  serial.write(audio_b, 2);
  serial.write(audio_a, 3);
  require(rejects([&] { serial.write(audio_b, 4); }).contains("full"),
          "Full XADR rejects an overwrite without consuming the alternating channel");
  serial.set_active(true);
  require(decode(collect(serial, subframe_units), 'Z').audio == 1,
          "The first A buffer upload makes room for the pending future A");
  serial.write(audio_b, 4);
  require(decode(collect(serial, subframe_units), 'Y').audio == 2,
          "The current B sample is retained while the next B waits in XADR");
  const auto next = collect(serial, frame_units);
  require(decode(std::string_view(next).substr(0, subframe_units), 'X').audio == 3 &&
              decode(std::string_view(next).substr(subframe_units), 'Y').audio == 4,
          "The real pending-register path delivers both next-frame samples in order");
}
}
}

int main() {
  using namespace blofeld::test::dax_test;
  try {
    literal_waves();
    dividers();
    underrun_and_non_audio();
    blocks_and_restart();
    unknowns_and_reset();
    pending_input();
    native_checks();
    dma_checks();
    std::cout << "DAX literal biphase frames, native ports, buffers and DMA lines passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
