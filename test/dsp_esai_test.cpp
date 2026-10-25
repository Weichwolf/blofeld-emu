// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_esai.hpp"
#include "esai_fixture.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <exception>
#include <iostream>
#include <string>

namespace blofeld::test::esai {
namespace {
constexpr unsigned mask_half = 0xffff;

struct Layout {
  unsigned code;
  unsigned slot;
  unsigned word;
};

// Independent table8-5 rows, ordered by word/slot length rather than encoding.
constexpr std::array layouts{
    Layout{.code = 0, .slot = 8, .word = 8},    Layout{.code = 4, .slot = 12, .word = 8},
    Layout{.code = 8, .slot = 16, .word = 8},   Layout{.code = 12, .slot = 20, .word = 8},
    Layout{.code = 16, .slot = 24, .word = 8},  Layout{.code = 24, .slot = 32, .word = 8},
    Layout{.code = 1, .slot = 12, .word = 12},  Layout{.code = 5, .slot = 16, .word = 12},
    Layout{.code = 9, .slot = 20, .word = 12},  Layout{.code = 13, .slot = 24, .word = 12},
    Layout{.code = 21, .slot = 32, .word = 12}, Layout{.code = 2, .slot = 16, .word = 16},
    Layout{.code = 6, .slot = 20, .word = 16},  Layout{.code = 10, .slot = 24, .word = 16},
    Layout{.code = 18, .slot = 32, .word = 16}, Layout{.code = 3, .slot = 20, .word = 20},
    Layout{.code = 7, .slot = 24, .word = 20},  Layout{.code = 15, .slot = 32, .word = 20},
    Layout{.code = 30, .slot = 24, .word = 24}, Layout{.code = 31, .slot = 32, .word = 24}};

std::string expected(const Layout layout, const unsigned options) {
  const auto all = std::bitset<word_bits>(input).to_string();
  auto payload = all.substr((options & 8U) != 0 ? 8 : 0, layout.word);
  if ((options & 1U) != 0) { std::ranges::reverse(payload); }
  const bool right = (options & 2U) != 0;
  char padding = right ? payload.front() : payload.back();
  if ((options & 4U) != 0) { padding = '0'; }
  const std::string pad(layout.slot - layout.word, padding);
  return right ? pad + payload : payload + pad;
}

void format_case(const Layout layout, const unsigned options) {
  const bool polarity = (options & 16U) != 0;
  const bool inverted = (options & 32U) != 0;
  DspEsai serial(DspEsaiInstance::primary);
  constexpr unsigned clock_polarity = 0x40000;
  constexpr unsigned frame_polarity = 0x80000;
  serial.write(clock, (polarity ? clock_polarity : 0U) | (inverted ? frame_polarity : 0U));
  serial.write(common, (options & 8U) != 0 ? 0x100U : 0U);
  serial.write(0, input);
  constexpr unsigned lsb = 0x40;
  constexpr unsigned right = 0x80;
  constexpr unsigned zero = 0x20000;
  constexpr unsigned bit_sync = 0x8000;
  constexpr unsigned shift = 10;
  serial.write(control,
               1U | bit_sync | (layout.code << shift) | ((options & 1U) != 0 ? lsb : 0U) |
                   ((options & 2U) != 0 ? right : 0U) | ((options & 4U) != 0 ? zero : 0U));
  baseline(serial, polarity, inverted);
  const auto data = slot(serial, layout.slot, true, polarity, inverted);
  require(data[0] == expected(layout, options), "Independent binary-string serial format");
  require(data[1] == std::string(layout.slot, 'z'), "Disabled channel stays high impedance");
  require(serial.read(status) == empty_even_frame, "First slot raises TFS/TDE/TEDE only");
}

void format_matrix() {
  constexpr unsigned option_count = 64;
  for (const auto layout : layouts) {
    for (unsigned options = 0; options < option_count; ++options) {
      if ((options & 8U) != 0 && layout.word > 16) { continue; }
      format_case(layout, options);
    }
  }
}

void unknown_and_strict_configuration() {
  DspEsai serial(DspEsaiInstance::secondary);
  require(serial.read(status) == 0 && serial.read(mask_a) == mask_half,
          "Reset status is clear and all documented time slots are enabled");
  require(serial.read(mask_b) == mask_half && serial.read(control) == 0,
          "Reset controls and both independent mask halves");
  require(rejects([&] { static_cast<void>(serial.read(0)); }).contains("Y:0xffff80"),
          "Write-only TX read fails with the actual secondary address");
  constexpr std::array invalid_controls{1U, 0x40001U, 0x400001U, 0x20ac01U, 0xac01U};
  for (const auto value : invalid_controls) {
    static_cast<void>(rejects([&] { serial.write(control, value); }));
    require(serial.read(control) == 0, "Rejected configuration leaves controls unchanged");
  }
  serial.write(control, normal24 | 1U);
  serial.set_active(true);
  static_cast<void>(rejects([&] { static_cast<void>(serial.read(status)); }));
  static_cast<void>(rejects([&] { static_cast<void>(serial.outputs()); }));
  static_cast<void>(rejects([&] { serial.observe({}); }));
  static_cast<void>(rejects([&] { serial.observe({.clock = true, .frame_sync = true}); }));
  baseline(serial);
  const auto unknown = slot(serial);
  require(unknown[0] == std::string(word_bits, '?'), "Unwritten data stays driven but unknown");
  require((serial.read(status) & underrun_flag) != 0, "Cold unknown transfer also underruns");
  serial.set_active(false);
  require(serial.outputs().driven == 0 && serial.read(status) == 0,
          "Individual reset clears status/output without fabricating samples");
  serial.write(0, other);
  baseline(serial);
  require(slot(serial)[0] == "000100100011010001010110", "Explicit reprime survives activation");
  serial.reset();
  require(serial.read(control) == 0 && serial.read(mask_b) == mask_half,
          "Hardware/software reset restores controls and mask values");
}

void underrun_and_all_channel_clear() {
  DspEsai serial(DspEsaiInstance::primary);
  constexpr unsigned two_lanes = 0x21;
  serial.write(0, input);
  serial.write(5, other);
  serial.write(control, normal24 | two_lanes);
  baseline(serial);
  const auto first = slot(serial);
  const auto repeated = slot(serial);
  require(first == repeated, "Underrun retransmits actual previous TX contents");
  serial.write(0, input);
  serial.write(5, other);
  require((serial.read(status) & underrun_flag) != 0,
          "Writing every TX without a preceding status read does not clear TUE");
  serial.write(0, other);
  require((serial.read(status) & underrun_flag) != 0,
          "Status read plus only one enabled channel cannot clear TUE");
  serial.write(5, input);
  require(serial.read(status) == 0x2000, "All enabled writes clear TDE and armed TUE");
  const auto next = slot(serial);
  require(next[0] == "000100100011010001010110" && next[5] == "101001011100001110011111",
          "Subsequent transfer uses each newly written data register");
  require(serial.frames_started() == 3 && serial.slots_started() == 3,
          "Counters describe only externally observed frames/slots");
}

void network_masks_and_skip() {
  DspEsai serial(DspEsaiInstance::primary);
  constexpr unsigned three_slots = 0x400;
  constexpr unsigned network = 0x100;
  serial.write(clock, three_slots);
  serial.write(mask_a, 5);
  serial.write(0, input);
  serial.write(control, normal24 | network | 1U);
  baseline(serial);
  const auto first = slot(serial);
  serial.write(mask_a, 0);
  serial.write(0, other);
  require(slot(serial, word_bits, false)[0] == std::string(word_bits, 'z'),
          "Masked slot does not consume pending transmit data");
  require(slot(serial, word_bits, false)[0] == "000100100011010001010110",
          "In-flight frame retains its original slot mask");
  require(first[0] == "101001011100001110011111", "Initial network slot uses primed data");
  serial.write(0, input);
  require(slot(serial)[0] == std::string(word_bits, 'z'), "New mask takes effect at next frame");
  static_cast<void>(slot(serial, word_bits, false));
  static_cast<void>(slot(serial, word_bits, false));
  require(serial.read(status) == 0, "Masked slots do not fabricate an empty/underrun request");

  DspEsai skipped(DspEsaiInstance::primary);
  skipped.write(0, input);
  skipped.write(control, normal24 | 1U);
  baseline(skipped);
  static_cast<void>(slot(skipped));
  skipped.write(6, 0);
  require(slot(skipped)[0] == std::string(word_bits, 'z'), "TSR emits a complete high-Z slot");
  require(skipped.read(status) == empty_even_frame, "Intentional TSR gap does not underrun");
  skipped.write(0, other);
  require(slot(skipped)[0] == "000100100011010001010110", "TX resumes after the TSR slot");
}

void deferred_disable_and_sync_failures() {
  DspEsai serial(DspEsaiInstance::primary);
  serial.write(0, input);
  serial.write(control, normal24 | 1U);
  baseline(serial);
  std::string data(1, sample(bit(serial, true), 0));
  serial.write(control, normal24);
  for (unsigned i = 1; i < word_bits; ++i) { data.push_back(sample(bit(serial, false), 0)); }
  require(data == "101001011100001110011111", "TEx clear completes the current serial word");
  static_cast<void>(bit(serial, false));
  require(serial.outputs().driven == 0, "Disabled channel releases at the following output edge");
  serial.write(0, other);
  serial.write(control, normal24 | 1U);
  baseline(serial);
  require(slot(serial)[0] == "000100100011010001010110",
          "Fully disabled transmitter restarts only from a new observed frame");

  DspEsai early(DspEsaiInstance::primary);
  early.write(0, input);
  early.write(control, normal24 | 1U);
  baseline(early);
  static_cast<void>(bit(early, true));
  const auto old = early.outputs();
  require(rejects([&] { static_cast<void>(bit(early, true)); }).contains("multiple bits"),
          "Word-length sync is not silently interpreted as bit sync");
  require(early.outputs() == old, "Rejected input observation preserves current serial output");
  static_cast<void>(bit(early, false));
  require(rejects([&] { static_cast<void>(bit(early, true)); }).contains("Early"),
          "A new frame cannot silently truncate an active word");
}

void all_lanes_and_upper_slot_mask() {
  DspEsai all(DspEsaiInstance::primary);
  constexpr std::array words{0x123456U, 0xabcdefU, 0x800001U, 0x7ffffeU, 0x0f0f0fU, 0xf0f0f0U};
  for (unsigned lane = 0; lane < words.size(); ++lane) { all.write(lane, words.at(lane)); }
  all.write(control, normal24 | lane_mask);
  baseline(all);
  const auto data = slot(all);
  for (unsigned lane = 0; lane < words.size(); ++lane) {
    require(data.at(lane) == std::bitset<word_bits>(words.at(lane)).to_string(),
            "All six independent transmit data registers reach their own serial lanes");
    all.write(lane, words.at(lane));
    if (lane != words.size() - 1) {
      require(all.read(status) == empty_even_frame, "TDE waits for all six CPU writes");
    }
  }
  require(all.read(status) == 0x2000, "Sixth write clears all-channel empty request");

  DspEsai upper(DspEsaiInstance::secondary);
  constexpr unsigned last_slot = 17;
  constexpr unsigned frame_rate = last_slot << 9U;
  constexpr unsigned network = 0x100;
  upper.write(clock, frame_rate);
  upper.write(mask_a, 0);
  upper.write(mask_b, 2);
  upper.write(0, other);
  upper.write(control, normal24 | network | 1U);
  baseline(upper);
  for (unsigned i = 0; i < last_slot; ++i) {
    require(slot(upper, word_bits, i == 0)[0] == std::string(word_bits, 'z'),
            "Lower mask and upper bit0 keep slots0 through16 disabled");
  }
  require(slot(upper, word_bits, false)[0] == "000100100011010001010110",
          "Upper mask bit1 enables exactly slot17 without consuming earlier data");
  constexpr unsigned odd_empty = 0x28000;
  require(upper.read(status) == odd_empty, "Odd slot17 raises TDE/TODE, without TFS/TUE");
  static_cast<void>(rejects([&] { upper.write(6, 0); }));
}
}
}

int main() {
  try {
    using namespace blofeld::test::esai;
    format_matrix();
    unknown_and_strict_configuration();
    underrun_and_all_channel_clear();
    network_masks_and_skip();
    deferred_disable_and_sync_failures();
    all_lanes_and_upper_slot_mask();
    native_checks();
    std::cout
        << "ESAI external-clock transmit formats, flags, slots, native access/restore passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
