// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_dsp_runtime_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::mcu_dsp;
constexpr unsigned shared_address = 1, shared_value = 0x42;
constexpr std::uint64_t initial_ticks = 450, recovered = 540, write_ticks = 720;
// Independently encoded LDAB #42;STAB direct PORTB: one then two IPBus periods.
constexpr std::array<std::uint8_t, 4> program{0xc6, shared_value, 0x5b, shared_address};

enum class Delivery : std::uint8_t { normal, dropped, wrong_address };

Options options(const McuWriteOrder order) {
  auto contract = runtime_inputs();
  contract.writes = order;
  return {.mcu_program = program,
          .dsp_program = {},
          .shared_address = shared_address,
          .runtime = contract,
          .release = true};
}

void tie_case(const McuWriteOrder order, const Delivery delivery = Delivery::normal) {
  Fixture f(options(order));
  f.mcu.bus().write(hc12::portb, 0);
  f.mcu.bus().write(hc12::ddrb, byte_mask);
  f.attach();
  auto expected = f.dsp.cpu().registers();
  expected.pc = boot + 1;
  std::vector<std::string> events;
  unsigned input_calls = 0;
  f.provide_inputs = [&] {
    ++input_calls;
    return cycle_inputs();
  };
  f.on_cycle = [&](const DspRuntimeCycle &cycle) {
    require(cycle.completion_ticks == recovered + ((cycle.cycle + 1) * dsp_period) &&
                cycle.completion_ticks - cycle.start_ticks == dsp_period,
            "Native periods do not match the independently expected45tick bypass clock");
    events.push_back("DSP:" + std::to_string(cycle.completion_ticks));
    require(rejects([&] { f.runtime->end_instruction(); }).contains("healthy inactive"),
            "Native completion callback reentered an active coordinator");
  };
  f.on_write = [&](const BusEvent &event) {
    const auto pads = f.mcu.bus().observe_pads(McuPort::b, byte_mask);
    require(event.value == shared_value && pads.known == byte_mask && pads.levels == shared_value,
            "Signal observation preceded the actual native MCU GPIO write");
    const auto expected_instructions = order == McuWriteOrder::before_mcu ? 1U : 0U;
    require(f.dsp.cpu().progress().instructions == expected_instructions,
            "Actual DSP retirement crossed the independently ordered shared-write boundary");
    events.push_back("MCU:" + std::to_string(f.runtime->end_ticks()));
  };
  if (delivery == Delivery::dropped) { f.deliver_write = false; }
  if (delivery == Delivery::wrong_address) {
    f.mcu.bus().set_observer([&](BusEvent event) {
      if (!event.write || event.address != shared_address) { return; }
      ++event.address;
      f.runtime->observe_write(event, [] {}, f.provide_inputs, f.observer());
    });
  }
  require(f.step().instruction_executed && f.runtime->end_ticks() == recovered &&
              f.dsp.clock().hardware_reset_ticks() == recovered && input_calls == 0,
          "Five native reset periods plus LDAB did not complete90 reset-recovery ticks");
  if (order == McuWriteOrder::unqualified || delivery != Delivery::normal) {
    const auto error = rejects([&] { static_cast<void>(f.step()); });
    const std::string_view delivery_error =
        delivery == Delivery::dropped ? "lost a shared write" : "must match";
    const auto reason =
        order == McuWriteOrder::unqualified ? "Unqualified simultaneous" : delivery_error;
    require(error.contains(reason) && f.runtime->faulted() && input_calls == 3 &&
                f.runtime->completed_cycles() == 3 && f.dsp.cpu().progress().instructions == 0,
            "Rejected shared-write order/delivery lost its actual three-cycle native prefix");
    rejects([&] { f.runtime->end_instruction(); });
    return;
  }
  require(f.step().instruction_executed && !f.runtime->faulted() && input_calls == 4 &&
              f.runtime->completed_cycles() == 4 && f.runtime->ties() == 1 &&
              f.runtime->end_ticks() == write_ticks && f.mcu.run_clock().bus_cycles() == 8 &&
              f.dsp.cpu().registers() == expected &&
              f.dsp.cpu().progress() ==
                  dsp56300::Progress{.instructions = 1, .estimated_cycles = 1},
          "Actual LDAB/STAB and one DSP NOP lost literal time or complete native register state");
  std::vector<std::string> tied;
  for (const auto &event : events) {
    if (event.ends_with(std::to_string(write_ticks))) { tied.push_back(event); }
  }
  const std::vector<std::string> dsp_first{"DSP:720", "MCU:720"};
  const std::vector<std::string> mcu_first{"MCU:720", "DSP:720"};
  require(tied == (order == McuWriteOrder::before_mcu ? dsp_first : mcu_first),
          "Shared native write and DSP completion did not follow their declared tie order");
}

void input_case(const bool missing) {
  auto settings = options(McuWriteOrder::before_mcu);
  if (!missing) { settings.runtime.maximum_cycles_per_period = 1; }
  Fixture f(settings);
  f.attach();
  if (missing) { f.provide_inputs = {}; }
  require(f.step().instruction_executed && f.runtime->end_ticks() == recovered &&
              initial_ticks + recovery_ticks == recovered,
          "A missing running-cycle provider changed genuine held/recovery time");
  require(rejects([&] { static_cast<void>(f.step()); }).contains("cycle inputs or exceeded") &&
              f.runtime->faulted() && f.runtime->completed_cycles() == (missing ? 0U : 1U),
          "Missing DMA observations or cycle budget fabricated a native DSP period");
}
}

int main() {
  try {
    tie_case(McuWriteOrder::before_mcu);
    tie_case(McuWriteOrder::after_mcu);
    tie_case(McuWriteOrder::unqualified);
    tie_case(McuWriteOrder::after_mcu, Delivery::dropped);
    tie_case(McuWriteOrder::after_mcu, Delivery::wrong_address);
    input_case(true);
    input_case(false);
    std::cout << "Seven native MCU/dsp56300 write-order and input-failure cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
