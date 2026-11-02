// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu_dsp_runtime_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
namespace native = test::mcu_dsp;
using native::require;
constexpr unsigned direction = 0x16;
constexpr unsigned word_value = 0xff16;
// HC12 LDD immediate / STD extended: the low byte is the declared shared DDRB.
constexpr std::array<std::uint8_t, 6> program{0xcc, 0xff, direction, 0x7c, 0, hc12::ddra};
enum class Mode : std::uint8_t {
  normal,
  unspecified,
  outside,
  wrong_byte,
  wrong_delivery,
  unshared
};

void bind(native::Fixture &fixture,
          const Mode mode,
          unsigned &notifications,
          const std::uint16_t selected = hc12::ddrb) {
  fixture.mcu.set_cycle_observer([&fixture, mode, selected](const Hc12BusCycle cycle) {
    const bool shared = cycle.kind == 'W' && cycle.address && *cycle.address == hc12::ddra;
    std::optional<std::uint16_t> byte;
    if (shared && mode != Mode::unspecified) {
      if (mode == Mode::outside) {
        byte = hc12::ddrb + 1;
      } else if (mode == Mode::wrong_byte) {
        byte = hc12::ddra;
      } else {
        byte = selected;
      }
    }
    if (mode == Mode::unshared) { byte = hc12::ddrb; }
    fixture.runtime->begin_cycle(cycle, shared, fixture.provide_inputs, fixture.observer(), byte);
  });
  fixture.mcu.bus().set_observer([&fixture, mode, selected, &notifications](BusEvent event) {
    if (!event.write || event.address != selected) { return; }
    if (mode == Mode::wrong_delivery) { event.address = hc12::ddra; }
    fixture.runtime->observe_write(
        event,
        [&] {
          const bool head = selected == hc12::ddra;
          require(event.address == selected &&
                      event.value == (head ? native::byte_mask : direction) &&
                      fixture.mcu.bus().read(hc12::ddra) == native::byte_mask &&
                      fixture.mcu.bus().read(hc12::ddrb) == (head ? 0U : direction),
                  "The declared native byte is delivered at its actual sub-write boundary");
          ++notifications;
        },
        fixture.provide_inputs,
        fixture.observer());
  });
}

native::Options options(const McuWriteOrder order) {
  auto contract = native::runtime_inputs();
  contract.writes = order;
  return {.mcu_program = program, .dsp_program = {}, .runtime = contract, .release = true};
}

std::array<unsigned, 6> registers(const CpuState state) {
  return {state.pc, state.d, state.x, state.y, state.sp, state.ccr};
}

void success(const McuWriteOrder order, const std::uint16_t selected) {
  native::Fixture f(options(order));
  unsigned notifications = 0;
  bind(f, Mode::normal, notifications, selected);
  require(f.step().instruction_executed && f.mcu.cpu().state().d == word_value,
          "The real LDD establishes the independent word operand");
  const auto prefix = f.runtime->checkpoint();
  require(f.step().instruction_executed && notifications == 1 &&
              f.mcu.cpu().state().d == word_value && !f.runtime->faulted(),
          "One actual STD publishes exactly its declared shared byte");
  const auto mcu = registers(f.mcu.cpu().state());
  const auto dsp = f.dsp.cpu().registers();
  const auto progress = f.dsp.cpu().progress();
  const auto ticks = f.runtime->end_ticks();
  const auto cycles = f.runtime->completed_cycles();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.restore(prefix);
    notifications = 0;
    bind(f, Mode::normal, notifications, selected);
    require(f.step().instruction_executed && notifications == 1 &&
                registers(f.mcu.cpu().state()) == mcu && f.dsp.cpu().registers() == dsp &&
                f.dsp.cpu().progress() == progress && f.runtime->end_ticks() == ticks &&
                f.runtime->completed_cycles() == cycles,
            "Fresh MCU/DSP owners retain either declared byte's ordering and full state");
  }
}

void guards() {
  for (const auto mode :
       {Mode::unspecified, Mode::outside, Mode::wrong_byte, Mode::wrong_delivery, Mode::unshared}) {
    native::Fixture f(options(McuWriteOrder::before_mcu));
    unsigned notifications = 0;
    bind(f, mode, notifications);
    const auto error = native::rejects([&] { f.run(2); });
    const bool declaration =
        mode == Mode::unspecified || mode == Mode::outside || mode == Mode::unshared;
    require(
        error.contains(declaration ? "declared word shared write" : "must match") &&
            f.runtime->faulted() && notifications == 0,
        "Missing/outside/foreign declarations and wrong delivered bytes cannot publish success");
  }
}
}

int main() {
  try {
    for (const auto selected : {hc12::ddra, hc12::ddrb}) {
      success(McuWriteOrder::before_mcu, selected);
      success(McuWriteOrder::after_mcu, selected);
    }
    guards();
    std::cout << "Native shared word: two orders, both bytes, eight fresh-owner suffixes, five "
                 "strict guards passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
