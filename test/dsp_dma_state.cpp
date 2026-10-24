// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_dma.hpp"
#include "dma_fixture.hpp"

#include <array>

namespace blofeld::test::dma_test {
namespace {
struct StatusMaskCase {
  unsigned control;
  unsigned enabled;
};

void masked_status_case(const StatusMaskCase input) {
  // FM figure10-6/table10-10: DTD is per-channel; DACT/DCH describe global activity.
  constexpr unsigned activity = 0xf00;
  constexpr unsigned reserved = 0xfff0c0;
  constexpr unsigned full_status = 0xffffff;
  const auto [control, enabled] = input;
  Harness test;
  for (unsigned channel = 0; channel < 6; ++channel) {
    if ((enabled & (1U << channel)) != 0) { configure(test.dma, control, 0, channel); }
  }
  for (unsigned selected = 1; selected <= all_done; ++selected) {
    if ((selected & enabled) == 0) {
      require(test.dma.read(status, selected | reserved) == selected,
              "Untimed channels do not obscure independent reset DTD or reserved-zero bits");
    } else {
      require(rejects([&] {
                static_cast<void>(test.dma.read(status, selected));
              }).contains("cycle timing"),
              "Selecting an enabled channel still requires actual elapsed reference cycles");
    }
  }
  require(test.dma.read(status, reserved) == 0 && test.dma.read(status, 0) == 0,
          "Unrequested and reserved fields cannot require an unrelated DMA clock sample");
  for (const auto mask : {activity, 0x100U, 0x200U, 0x400U, 0x800U, full_status}) {
    if (enabled == 0) {
      require(test.dma.read(status, mask) == (all_done & mask),
              "Disabled DMA has known reset status fields");
    } else {
      require(
          rejects([&] { static_cast<void>(test.dma.read(status, mask)); }).contains("cycle timing"),
          "Any unobserved enabled channel prevents fabricating global activity fields");
    }
  }
  for (unsigned cycle = 0; cycle < 3; ++cycle) {
    require(!test.tick(0, true) && test.reads.empty() && test.writes.empty(),
            "Status reads consume no DMA time, request or external bus access");
    require(test.dma.read(status, all_done) == (cycle == 2 ? all_done & ~enabled : all_done),
            "Only three explicit reference cycles clear exactly the enabled DTD bits");
  }
}

void masked_status_without_time() {
  for (const auto control : {word_copy, software_copy}) {
    for (unsigned enabled = 0; enabled <= all_done; ++enabled) {
      masked_status_case({.control = control, .enabled = enabled});
    }
  }
}

void priorities() {
  Harness equal;
  static_cast<void>(equal.tick());
  for (unsigned channel = 0; channel < 6; ++channel) {
    configure(equal.dma, word_copy, 0, channel);
  }
  constexpr std::array order{0U, 1U, 2U, 3U, 4U, 5U};
  for (const auto expected : order) {
    static_cast<void>(equal.tick(1));
    const auto word = equal.tick(1);
    require(word && word->channel == expected,
            "Equal-priority channels service each captured request in literal round-robin order");
  }
  require(equal.dma.read(status) == all_done, "All six actual one-word channels finish");

  Harness interrupted;
  static_cast<void>(interrupted.tick());
  constexpr unsigned high = word_copy | 0x60000 | 0x800; // Priority3,IRQ B.
  configure(interrupted.dma, word_copy, 0, 3);
  configure(interrupted.dma, high);
  static_cast<void>(interrupted.tick(1));
  const auto low_word = interrupted.tick(3);
  require(low_word && low_word->channel == 3,
          "A new higher-priority edge cannot interrupt the low-priority destination phase");
  static_cast<void>(interrupted.tick(2));
  const auto high_word = interrupted.tick(2);
  require(high_word && high_word->channel == 0,
          "The higher-priority word starts at the next complete word boundary");
}

void captured_disable_and_recovery() {
  Harness test;
  static_cast<void>(test.tick());
  configure(test.dma, word_copy, 1);
  test.fail_read = true;
  require(rejects([&] { static_cast<void>(test.tick(1)); }).contains("source"),
          "Source faults retain cycle/edge/pending state");
  require(test.reads.empty() && test.writes.empty() && test.cycle == 1,
          "Rejected source read consumes no external operation or reference cycle");
  test.fail_read = false;
  static_cast<void>(test.tick(1));
  const auto checkpoint = test.dma;
  constexpr unsigned changed_after_read = 0x123456;
  test.data = changed_after_read;
  for (unsigned replay = 0; replay < 2; ++replay) {
    test.dma = checkpoint;
    test.cycle = 2;
    test.writes.clear();
    test.dma.write(control0, word_copy & ~enable);
    require(test.dma.read(status, 1) == 1,
            "The documented enable pipeline can still expose the old done bit before cycle3");
    static_cast<void>(test.tick(0, true));
    require(test.writes.empty() && test.reads.size() == 1,
            "A denied destination grant retains the already-read data without rereading");
    test.fail_write = true;
    require(rejects([&] { static_cast<void>(test.tick()); }).contains("destination"),
            "Destination fault retains the held word and current pointer/counter");
    require(test.dma.read(counter0) == 1 && test.writes.empty(),
            "Failed destination has not completed or advanced the actual counter");
    test.fail_write = false;
    const auto word = test.tick();
    require(word && word->value == ring_start && test.writes[0].value == ring_start &&
                test.dma.read(counter0) == 0 && test.dma.read(status) == all_done,
            "Both restores write the captured old value, finish exactly once and honor DE clear");
  }

  Harness pending;
  static_cast<void>(pending.tick());
  configure(pending.dma, word_copy);
  static_cast<void>(pending.tick(1, true));
  pending.dma.write(control0, word_copy & ~enable);
  static_cast<void>(pending.tick());
  const auto word = pending.tick();
  require(word && pending.dma.read(status) == all_done,
          "DE clear preserves a word request captured before the first bus grant");
}

void strict_inputs_and_configuration() {
  Harness test;
  static_cast<void>(test.tick());
  configure(test.dma, word_copy);
  for (const auto reg : {source0, destination0, counter0}) {
    static_cast<void>(rejects([&] { test.dma.write(reg, 0); }));
  }
  static_cast<void>(rejects([&] { test.dma.write(control0, word_copy | 0x20000); }));
  constexpr std::array unsupported{
      0xc90244U, 0x890244U, 0x800244U, 0x88ba44U, 0x880264U, 0x880247U, 0x88024cU, 0xa8b634U};
  for (const auto control : unsupported) {
    DspDma invalid;
    configure(invalid, word_copy & ~enable);
    static_cast<void>(rejects([&] { invalid.write(control0, control); }));
    require(invalid.read(control0) == (word_copy & ~enable),
            "Unsupported enable preserves prior register state");
  }
  require(rejects([&] {
            static_cast<void>(test.dma.advance({.cycle = 2, .timing = timing, .grant = grant}, {}));
          }).contains("cycle"),
          "Skipped reference cycles do not stand in for missing bus phases");
  static_cast<void>(rejects([&] {
    static_cast<void>(test.dma.advance({.cycle = 1, .timing = timing, .grant = grant}, {}));
  }));
  static_cast<void>(test.tick());
  static_cast<void>(test.tick(1, true));
  static_cast<void>(test.tick(0, true));
  require(rejects([&] { static_cast<void>(test.tick(1, true)); }).contains("repeated"),
          "A second regular trigger cannot silently overwrite a still-pending word");

  DspDma unknown;
  configure(unknown, word_copy);
  require(rejects([&] { static_cast<void>(unknown.read(status)); }).contains("timing"),
          "A configured channel does not fabricate elapsed cycles for a status read");
  static_cast<void>(rejects([&] {
    static_cast<void>(unknown.advance(
        {.cycle = 0, .known_requests = 1, .requests = 1, .timing = timing, .grant = grant}, {}));
  }));
}

void late_peripheral() {
  Harness test;
  constexpr unsigned esai_secondary_request = 22;
  constexpr unsigned peripheral = word_copy | (esai_secondary_request << 11U);
  constexpr unsigned asserted = 1U << 22U;
  configure(test.dma, peripheral);
  static_cast<void>(test.tick(asserted));
  const auto word = test.tick(asserted);
  require(word && word->value == ring_start && test.dma.read(status) == all_done,
          "APR23 section5.6: a held peripheral request survives initialization before DMA");
}
}

void state_checks() {
  masked_status_without_time();
  priorities();
  captured_disable_and_recovery();
  strict_inputs_and_configuration();
  late_peripheral();
}
}
