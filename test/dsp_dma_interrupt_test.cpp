// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_dma.hpp"
#include "blofeld/dsp_interrupts.hpp"
#include "dma_fixture.hpp"
#include "dsp_dma_interrupt_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace blofeld::test::dma_irq {
namespace {
using namespace dma_test;

void original_priority_and_fields() {
  DspInterrupts controller;
  constexpr unsigned dma1_vector = 0x1a;
  controller.write(iprc, original_priority);
  require(controller.read(iprc) == original_priority && controller.priority(dma1_vector) == 0 &&
              controller.priority(0x10) == 1,
          "Original IPRC4006 enables DMA1 IPL0 and IRQA IPL1");
  for (unsigned channel = 0; channel < channels; ++channel) {
    const auto vector = first_vector + (2 * channel);
    const std::array<std::uint32_t, 1> pending{vector};
    for (unsigned encoded = 0; encoded < 4; ++encoded) {
      const auto value = encoded << (priority_shift + (2 * channel));
      controller.write(iprc, value);
      require(controller.read(iprc) == value, "Each IPRC field retains exactly its own bits");
      for (unsigned mask = 0; mask < 4; ++mask) {
        const auto expected =
            encoded > mask
                ? std::optional{DspInterruptRequest{.vector = vector, .level = encoded - 1}}
                : std::nullopt;
        require(controller.select(pending, mask) == expected,
                "All six DMA fields obey disabled/IPL0/IPL1/IPL2 and CPU mask rules");
      }
    }
  }
  controller.reset();
  require(controller.read(iprc) == 0, "Peripheral reset disables all DMA priority fields");
}

std::optional<DspInterruptRequest> pair_request(const std::array<std::uint32_t, 2> &pending,
                                                const unsigned a,
                                                const unsigned b,
                                                const unsigned mask) {
  if (a > mask && (a >= b || b <= mask)) {
    return DspInterruptRequest{.vector = pending[1], .level = a - 1};
  }
  if (b > mask) { return DspInterruptRequest{.vector = pending[0], .level = b - 1}; }
  return std::nullopt;
}

void priority_pairs() {
  DspInterrupts controller;
  for (unsigned first = 0; first < channels; ++first) {
    for (unsigned second = first + 1; second < channels; ++second) {
      const std::array<std::uint32_t, 2> pending{first_vector + (2 * second),
                                                 first_vector + (2 * first)};
      for (unsigned a = 0; a < 4; ++a) {
        for (unsigned b = 0; b < 4; ++b) {
          controller.write(
              iprc, (a << (priority_shift + (2 * first))) | (b << (priority_shift + (2 * second))));
          for (unsigned mask = 0; mask < 4; ++mask) {
            const auto expected = pair_request(pending, a, b, mask);
            require(controller.select(pending, mask) == expected,
                    "DMA arbitration selects greatest eligible IPL, then earlier channel");
          }
        }
      }
    }
  }
  constexpr std::array<std::uint32_t, 9> ranked{
      0x10, 0x18, 0x1a, 0x1c, 0x1e, 0x20, 0x22, 0x44, 0x54};
  constexpr unsigned core_ipl2 = 0xfff003;
  constexpr unsigned peripheral_ipl2 = 0x30c;
  controller.write(iprc, core_ipl2);
  controller.write(iprp, peripheral_ipl2);
  for (unsigned first = 0; first < ranked.size(); ++first) {
    for (unsigned second = first + 1; second < ranked.size(); ++second) {
      const std::array pending{ranked.at(second), ranked.at(first)};
      require(controller.select(pending, 0) ==
                  DspInterruptRequest{.vector = ranked.at(first), .level = 2},
              "NXP table4-5: IRQA precedes DMA0..5, then SHI and TIM, regardless of arrival");
    }
  }
}

struct Completion {
  unsigned mode = 0;
  unsigned channel = 0;
  bool enabled = false;
};

void block_completion(const Completion input) {
  const auto mode = input.mode;
  const auto channel = input.channel;
  const auto enabled = input.enabled;
  Harness test;
  static_cast<void>(test.tick());
  const auto control = mode | (enabled ? interrupt_enable : 0);
  configure(test.dma, control, 0, channel);
  require(test.dma.interrupt_sources() == 0,
          "Setting DIE over reset DTD=1 cannot invent a completion event");
  require(!test.tick(1) && test.reads.size() == 1 && test.dma.interrupt_sources() == 0,
          "An actual source read is not an end-of-block interrupt");
  require(!test.tick(1, true) && test.dma.interrupt_sources() == 0,
          "A destination wait cannot complete the transfer or assert an IRQ");
  test.fail_write = true;
  static_cast<void>(rejects([&] { static_cast<void>(test.tick(1)); }));
  require(test.writes.empty() && test.dma.interrupt_sources() == 0,
          "A failed destination cannot fabricate completion or interrupt state");
  test.fail_write = false;
  const auto word = test.tick(1);
  const auto expected = enabled ? 1U << channel : 0;
  require(word && word->channel == channel && word->value == ring_start &&
              test.writes.size() == 1 && test.dma.read(counter0 - (4 * channel)) == 0 &&
              test.dma.interrupt_sources() == expected,
          "Only the last real destination store and counter reload assert a DMA IRQ");
  if (mode == repeat_copy) {
    static_cast<void>(rejects([&] { static_cast<void>(test.dma.read(status)); }));
  } else {
    require(test.dma.read(status) == all_done,
            "One-shot DTD and all unrelated channels retain independent done values");
  }
  if (enabled) {
    test.dma.acknowledge_interrupt(channel);
  } else {
    static_cast<void>(rejects([&] { test.dma.acknowledge_interrupt(channel); }));
  }
  require(test.dma.interrupt_sources() == 0, "Service consumes exactly the captured latch");
}

void block_completions() {
  constexpr std::array modes{word_copy, line_copy, software_copy, repeat_copy};
  for (const auto mode : modes) {
    for (unsigned channel = 0; channel < channels; ++channel) {
      for (const bool enabled : {false, true}) {
        block_completion({.mode = mode, .channel = channel, .enabled = enabled});
      }
    }
  }
}

void enable_and_latch_lifetime() {
  Harness test;
  static_cast<void>(test.tick());
  constexpr unsigned control = word_copy | interrupt_enable;
  configure(test.dma, control);
  static_cast<void>(test.tick(1));
  test.dma.write(control0, word_copy);
  require(test.tick(1).has_value() && test.dma.interrupt_sources() == 0,
          "DIE can be cleared while active and disables the subsequent completion interrupt");
  test.dma.write(control0, control & ~enable);
  require(test.dma.interrupt_sources() == 0, "Enabling DIE cannot replay an uncaptured old DTD");
  static_cast<void>(test.tick());
  test.dma.write(control0, control);
  static_cast<void>(test.tick(1));
  require(test.tick(1).has_value() && test.dma.interrupt_sources() == 1,
          "A new real block completion captures its own pending event");
  test.dma.write(control0, word_copy & ~enable);
  require(test.dma.interrupt_sources() == 0, "Cleared DIE gates delivery of a latched request");
  static_cast<void>(rejects([&] { test.dma.acknowledge_interrupt(0); }));
  test.dma.write(control0, control & ~enable);
  require(test.dma.interrupt_sources() == 1, "Only service clears a captured request, not DIE");
  static_cast<void>(rejects([&] { test.dma.acknowledge_interrupt(channels); }));
  static_cast<void>(rejects([&] { test.dma.acknowledge_interrupt(UINT32_MAX); }));
  require(test.dma.interrupt_sources() == 1, "Invalid acknowledgements cannot consume a latch");
  const auto saved = test.dma;
  test.dma.acknowledge_interrupt(0);
  require(test.dma.interrupt_sources() == 0 && test.dma.read(status) == all_done,
          "IRQ service clears its latch without clearing DTD or inventing another completion");
  test.dma = saved;
  require(test.dma.interrupt_sources() == 1, "A controller value copy retains the real latch");
  test.dma.reset();
  require(test.dma.interrupt_sources() == 0 && test.dma.read(status) == all_done,
          "Reset clears requests independently of the reset DTD bits");
}

void explicit_stop() {
  constexpr std::array modes{word_copy, line_copy, software_copy, repeat_copy};
  for (const auto mode : modes) {
    for (unsigned phase = 0; phase < 3; ++phase) {
      Harness test;
      static_cast<void>(test.tick());
      const auto control = mode | interrupt_enable;
      constexpr unsigned remaining = 2;
      configure(test.dma, control, remaining);
      if (phase == 1) { static_cast<void>(test.tick(1, true)); }
      if (phase == 2) { static_cast<void>(test.tick(1)); }
      test.dma.write(control0, control & ~enable);
      const bool held = phase == 2 || (phase == 1 && (mode == word_copy || mode == repeat_copy));
      require(test.dma.interrupt_sources() == (held ? 0U : 1U),
              "DE clear waits for an already captured word before requesting service");
      if (held) {
        if (phase != 2) { static_cast<void>(test.tick()); }
        require(test.tick().has_value() && test.dma.interrupt_sources() == 1 &&
                    test.writes.size() == 1 && test.dma.read(counter0) == remaining - 1,
                "Stopped transfer signals only after its actual held destination store");
      } else {
        require(test.reads.empty() && test.writes.empty(),
                "Stopping without a captured word cannot fabricate external I/O");
      }
      test.dma.acknowledge_interrupt(0);
      require(test.dma.interrupt_sources() == 0, "The explicit-stop event is serviced once");
    }
  }
}

void repeating_coalesces() {
  Harness test;
  static_cast<void>(test.tick());
  configure(test.dma, repeat_copy | interrupt_enable);
  for (unsigned block = 0; block < 2; ++block) {
    static_cast<void>(test.tick(1));
    require(test.tick(1).has_value() && test.dma.interrupt_sources() == 1,
            "Repeating blocks request service even when DTD lifetime remains unqualified");
    static_cast<void>(test.tick());
  }
  require(test.writes.size() == 2, "Two end-of-block events follow two genuine stores");
  test.dma.acknowledge_interrupt(0);
  require(test.dma.interrupt_sources() == 0,
          "A hardware request latch does not invent an IRQ queue");
}

void enable_during_word() {
  Harness test;
  static_cast<void>(test.tick());
  configure(test.dma, word_copy);
  static_cast<void>(test.tick(1));
  test.dma.write(control0, word_copy | interrupt_enable);
  require(test.dma.interrupt_sources() == 0,
          "Live DIE enable does not turn an already-read source into a completion");
  require(test.tick(1).has_value() && test.dma.interrupt_sources() == 1,
          "Live DIE enable takes effect at the genuine destination completion");
}

void multiword_completion() {
  constexpr std::array modes{word_copy, line_copy, software_copy, repeat_copy};
  for (const auto mode : modes) {
    Harness test;
    static_cast<void>(test.tick());
    configure(test.dma, mode | interrupt_enable, 1);
    require(!test.tick(1) && test.tick(1).has_value() && test.writes.size() == 1 &&
                test.dma.read(counter0) == 0 && test.dma.interrupt_sources() == 0,
            "The first actual word decrements a two-word block without asserting completion");
    if (mode == word_copy || mode == repeat_copy) { static_cast<void>(test.tick()); }
    require(!test.tick(1) && test.dma.interrupt_sources() == 0,
            "The final source read still has not completed the block or reloaded its counter");
    const auto last = test.tick(1);
    const bool linear = mode == software_copy;
    require(last && test.writes.size() == 2 && last->value == ring_start + (linear ? 1U : 0U) &&
                test.dma.read(counter0) == 1 && test.dma.interrupt_sources() == 1,
            "The last genuine destination store reloads the literal count before requesting IRQ");
  }
}
}
}

int main() {
  try {
    using namespace blofeld::test::dma_irq;
    original_priority_and_fields();
    priority_pairs();
    block_completions();
    enable_and_latch_lifetime();
    explicit_stop();
    repeating_coalesces();
    enable_during_word();
    multiword_completion();
    blofeld::test::dma_irq::native_checks();
    std::cout
        << "DMA IRQ priorities, real completion/stop/latch events and native service passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
