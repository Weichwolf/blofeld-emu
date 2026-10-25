// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp_norm_fixture.hpp"
#include "keyboard_initial_fixture.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_forward_fixture.hpp"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <tuple>

namespace blofeld::test::keyboard_initial {
namespace {
std::string failure(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected withdrawn initial Keyboard pad observation to fail");
}
} // namespace

void Fixture::replay() {
  const auto saved = pair.checkpoint();
  const LcdFixture saved_display(pair.mcu.bus(), *display);
  const std::array counts{received, irq_queries, irq_unknown, trailer_bytes};
  const auto before_mcu = update_forward::registers(pair.mcu.cpu().state());
  const auto before_dsp = pair.dsp.cpu().debug_state();
  using End = decltype(std::tuple(update_forward::registers(pair.mcu.cpu().state()),
                                  pair.dsp.cpu().debug_state(),
                                  pair.mcu.run_clock().elapsed_ticks(),
                                  pair.dsp.run_clock().elapsed_ticks(),
                                  pair.link.pad_queries,
                                  pair.link.msteps,
                                  pair.link.dsteps,
                                  irq_queries));
  const auto restore = [&] {
    pair.restore_owners(saved);
    received = counts[0];
    irq_queries = counts[1];
    irq_unknown = counts[2];
    trailer_bytes = counts[3];
    display = std::make_unique<LcdFixture>(pair.mcu.bus(), saved_display);
    attach_irq();
    pair.reconnect(saved);
    require(update_forward::registers(pair.mcu.cpu().state()) == before_mcu &&
                pair.dsp.cpu().debug_state() == before_dsp,
            "Trusted joint restoration preserves the actual helper-return boundary");
  };
  restore();
  pair.link.fault = gp::PadFault::mcu;
  const auto error = failure([&] { finish(); });
  require(error.contains("Unknown MCU GPIO input at 0x0260") && error.contains("pc=0xbad3") &&
              trailer_bytes == 2 && received == source_words * 4 && pair.mcu.timing_faulted() &&
              !pair.mcu.run_clock().faulted() && !pair.dsp.timing_faulted(),
          "Withdrawn pad knowledge fails the first physical trailer read after two cached bytes");
  const auto failed_mcu = update_forward::registers(pair.mcu.cpu().state());
  const auto failed_dsp = pair.dsp.cpu().debug_state();
  const auto ticks = pair.mcu.run_clock().elapsed_ticks();
  static_cast<void>(failure([&] { pair.mcu.cpu().step(); }));
  static_cast<void>(failure([&] { static_cast<void>(pair.mcu.checkpoint()); }));
  require(update_forward::registers(pair.mcu.cpu().state()) == failed_mcu &&
              pair.dsp.cpu().debug_state() == failed_dsp &&
              pair.mcu.run_clock().elapsed_ticks() == ticks,
          "Failed native retry/capture preserves both owners and the consumed cached prefix");
  pair.trace << "PAD_FAILURE trailer=2 samples=" << received << " unknown=1\n";
  std::optional<End> expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    pair.trace << "REPLAY index=" << replay << '\n';
    restore();
    finish();
    const End actual(update_forward::registers(pair.mcu.cpu().state()),
                     pair.dsp.cpu().debug_state(),
                     pair.mcu.run_clock().elapsed_ticks(),
                     pair.dsp.run_clock().elapsed_ticks(),
                     pair.link.pad_queries,
                     pair.link.msteps,
                     pair.link.dsteps,
                     irq_queries);
    if (expected) { require(actual == expected.value(), "Joint trailer suffix state differs"); }
    expected = actual;
    require(display->counts().writes == lcd_writes && display->counts().reads == lcd_reads,
            "Restored trailer leaves the actual completed original LCD state untouched");
  }
  pair.trace << "REPLAY_END restores=2 identical=1\n";
}
} // namespace blofeld::test::keyboard_initial
