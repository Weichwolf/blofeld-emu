// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_dma.hpp"
#include "dma_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace blofeld::test::dma_test {
namespace {
void reset_and_pipeline() {
  Harness test;
  require(test.dma.read(status) == all_done, "DSTR resets to all six done bits");
  for (unsigned channel = 0; channel < 6; ++channel) {
    require(test.dma.read(control0 - (channel * 4)) == 0, "Each control register resets to zero");
    static_cast<void>(rejects([&] { static_cast<void>(test.dma.read(source0 - (channel * 4))); }));
  }
  const auto unknown_offset = rejects([&] { static_cast<void>(test.dma.read(offset0)); });
  require(unknown_offset.contains("register read at X:0xfffff3"),
          "Unknown register reads identify the actual operation and target");
  const auto read_only = rejects([&] { test.dma.write(status, 0); });
  require(read_only.contains("register write at X:0xfffff4"),
          "Read-only register writes identify the actual operation and target");
  static_cast<void>(test.tick());
  configure(test.dma, word_copy);
  require(test.dma.read(status) == all_done, "DE does not erase the documented DTD pipeline delay");
  static_cast<void>(test.tick());
  static_cast<void>(test.tick());
  require(test.dma.read(status) == all_done, "DTD remains set for two additional reference cycles");
  static_cast<void>(test.tick());
  require(test.dma.read(status) == zero_not_done,
          "The third additional cycle clears only this DTD bit");
  require(!test.tick(1) && test.reads.size() == 1 && test.writes.empty(),
          "A rising regular request starts only the source read");
  require(test.dma.read(status) == zero_active,
          "Active channel0 is visible between read and write");
  const auto done = test.tick(1);
  require(done && done->value == ring_start && test.writes.size() == 1 &&
              test.dma.read(status) == all_done && test.dma.read(control0) == (word_copy & ~enable),
          "A separate destination cycle completes the literal one-word block and clears DE");
  test.dma.reset();
  require(test.dma.read(status) == all_done && !test.dma.active(),
          "Reset clears pending/phase state");
  static_cast<void>(rejects([&] { static_cast<void>(test.dma.read(counter0)); }));
}

void triple_tables() {
  // DSP56300FM table10-4, S=100,T0=10,T1=100; layout encodings from figure10-4.
  constexpr std::array addresses{
      100U, 101U, 102U, 112U, 113U, 114U, 214U, 215U, 216U, 226U, 227U, 228U};
  constexpr std::array counters{
      std::array{
          0x1042U, 0x1041U, 0x1040U, 0x1002U, 0x1001U, 0x1000U, 0x42U, 0x41U, 0x40U, 2U, 1U, 0U},
      std::array{0x40042U,
                 0x40041U,
                 0x40040U,
                 0x40002U,
                 0x40001U,
                 0x40000U,
                 0x42U,
                 0x41U,
                 0x40U,
                 2U,
                 1U,
                 0U},
      std::array{0x41002U,
                 0x41001U,
                 0x41000U,
                 0x40002U,
                 0x40001U,
                 0x40000U,
                 0x1002U,
                 0x1001U,
                 0x1000U,
                 2U,
                 1U,
                 0U}};
  constexpr unsigned source_begin = 100;
  constexpr unsigned first_offset = 10;
  constexpr unsigned second_offset = 100;
  constexpr unsigned after = 328;
  constexpr unsigned source_three = 0x980604;
  constexpr unsigned destination_three = 0x980640;
  for (unsigned selection = 0; selection < counters.size(); ++selection) {
    for (const bool destination_side : {false, true}) {
      Harness test;
      for (const auto reg : {offset0, offset2}) { test.dma.write(reg, first_offset); }
      for (const auto reg : {offset1, offset3}) { test.dma.write(reg, second_offset); }
      const auto control =
          (destination_side ? destination_three : source_three) | (selection << 4U);
      configure(test.dma, control, counters.at(selection)[0]);
      const auto changing = destination_side ? destination0 : source0;
      test.dma.write(control0, control & ~enable);
      test.dma.write(changing, source_begin);
      test.dma.write(control0, control);
      for (unsigned i = 0; i < addresses.size(); ++i) {
        require(test.dma.read(counter0) == counters.at(selection).at(i),
                "Each pre-transfer counter matches an independently transcribed manual table");
        require(!test.tick(), "The source phase does not complete a word");
        require(test.dma.read(changing) == addresses.at(i), "Read phase never advances pointers");
        const auto word = test.tick();
        require(word && (destination_side ? word->destination.address : word->source.address) ==
                            addresses.at(i),
                "Every actual DMA endpoint matches the literal table, on either side");
      }
      require(test.dma.read(changing) == after &&
                  test.dma.read(counter0) == counters.at(selection)[0] &&
                  test.dma.read(status) == all_done,
              "The last transfer applies T1 and reloads all three actual counter fields");
    }
  }
}

void linear_and_dual() {
  Harness linear;
  configure(linear.dma, software_copy & ~enable, 2);
  constexpr unsigned almost_wrap = 0xfffffe;
  linear.dma.write(source0, almost_wrap);
  linear.dma.write(destination0, almost_wrap);
  linear.dma.write(control0, software_copy);
  constexpr std::array expected{0xfffffeU, 0xffffffU, 0U};
  for (const auto address : expected) {
    static_cast<void>(linear.tick());
    const auto word = linear.tick();
    require(word && word->source.address == address && word->destination.address == address,
            "APR23 linear source/destination +1 wraps in24-bit address arithmetic");
  }
  require(linear.dma.read(source0) == 1 && linear.dma.read(counter0) == 2,
          "Single counter reloads without resetting a linear pointer");

  Harness dual;
  dual.dma.write(offset0, 10);
  constexpr unsigned dual_control = 0x980204;
  constexpr unsigned dual_count = 0x1002;
  configure(dual.dma, dual_control & ~enable, dual_count);
  constexpr unsigned dual_start = 100;
  dual.dma.write(source0, dual_start);
  dual.dma.write(control0, dual_control);
  constexpr std::array table{100U, 101U, 102U, 112U, 113U, 114U};
  for (const auto address : table) {
    static_cast<void>(dual.tick());
    const auto word = dual.tick();
    require(word && word->source.address == address && word->destination.address == destination,
            "Table10-3 dual-counter source and fixed destination have literal addresses");
  }
  constexpr unsigned dual_after = 124;
  require(dual.dma.read(source0) == dual_after && dual.dma.read(counter0) == dual_count,
          "The dual counter applies the final offset instead of a spurious increment");
}

void ring_and_unknown_done() {
  Harness test;
  static_cast<void>(test.tick());
  test.dma.write(offset0, 1);
  test.dma.write(offset1, ring_back);
  configure(test.dma, repeat_ring, ring_count);
  constexpr unsigned request_bit = 1U << 22U;
  constexpr unsigned words = 64;
  for (unsigned i = 0; i < words + 1; ++i) {
    static_cast<void>(test.tick(request_bit));
    const auto word = test.tick();
    require(word && word->source.address == ring_start + (i % words) &&
                word->value == ring_start + (i % words),
            "Sixty-five actual source reads establish the64-word circle and first reused word");
    if (i + 1 == words) {
      require(test.dma.read(source0) == ring_start && test.dma.read(counter0) == ring_count,
              "DCO000fc0 is64word modeC, not a4033word flat count");
      require(rejects([&] { static_cast<void>(test.dma.read(status)); }).contains("DTD"),
              "Repeating DTD lifetime is visibly unqualified instead of guessed");
      require(test.dma.read(status, 0x100) == 0, "Qualified inactive DACT remains inspectable");
    }
  }
  test.dma.write(control0, repeat_ring & ~enable);
  require(test.dma.read(status) == all_done && !test.dma.active(),
          "Explicit disable without a captured word has a known done state");
}
}
}

int main() {
  using namespace blofeld::test::dma_test;
  try {
    reset_and_pipeline();
    triple_tables();
    linear_and_dual();
    ring_and_unknown_done();
    state_checks();
    native_checks();
    std::cout
        << "DMA independent counters, word phases, native registers and serial refill passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
