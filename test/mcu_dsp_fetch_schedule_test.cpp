// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_dsp_runtime_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
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
    require(
        rejects([&] { static_cast<void>(f.runtime->checkpoint()); }).contains("healthy inactive"),
        "Active native completion callback cannot capture a joint snapshot");
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

struct Frame {
  std::uint64_t ticks;
  dsp56300::Registers registers;
  dsp56300::Progress progress;
  std::optional<dsp56300::InstructionEffect> pending;
  bool operator==(const Frame &) const = default;
};

dsp56300::Registers expected_nops(const unsigned count) {
  constexpr unsigned status = 0xc00300;
  constexpr unsigned mode = 0x305;
  dsp56300::Registers expected{};
  expected.pc = boot + count;
  expected.sr = status;
  expected.omr = mode;
  expected.m.fill(dsp56300::word_mask);
  return expected;
}

void delay_slot(const unsigned consumed) {
  // Authored NOP; LDX #ffff; DBNE X,self: one+two setup periods, three delay slots.
  constexpr std::array<std::uint8_t, 7> mcu_program{0xa7, 0xce, 0xff, 0xff, 0x04, 0x35, 0xfd};
  constexpr unsigned store_x0_y_absolute = 0x4c7000;
  constexpr unsigned reserved = 0x2000;
  constexpr unsigned loaded_index = 0xffff;
  constexpr unsigned expected_ccr = 0xd8;
  constexpr unsigned setup_ticks = 80;
  // A leading native NOP allows the long store's extension F2 before its
  // actual write. Additional NOPs shift that write through the MCU slots.
  constexpr std::array<unsigned, 3> completed_prefix{4, 6, 8};
  constexpr std::array<unsigned, 3> failure_ticks{765, 855, 945};
  constexpr unsigned first_offer = 810;
  constexpr unsigned mcu_period = 90;
  const auto padding = consumed + 1;
  std::array<std::uint32_t, 5> dsp_program{};
  dsp_program.at(padding) = store_x0_y_absolute;
  dsp_program.at(padding + 1) = reserved;
  Fixture f({.mcu_program = mcu_program, .dsp_program = dsp_program, .release = true});
  std::vector<Frame> trace;
  f.on_cycle = [&](const DspRuntimeCycle &cycle) {
    const auto count = cycle.cycle + 1;
    const auto pc_updates = count < 3 ? 0 : static_cast<unsigned>((count - 1) / 2);
    const auto retired = count < 2 ? 0 : (count - 2) / 2;
    require(cycle.completion_ticks == recovered + (count * dsp_period) &&
                cycle.completion_ticks - cycle.start_ticks == dsp_period &&
                f.dsp.cpu().registers() == expected_nops(std::min(pc_updates, padding)) &&
                f.dsp.cpu().progress() ==
                    dsp56300::Progress{.instructions = retired, .estimated_cycles = retired},
            "Every accepted F1/F2/PCU/retirement period retains independent complete NOP state");
    const auto pending = f.dsp.cpu().pending_instruction();
    trace.push_back({.ticks = cycle.completion_ticks,
                     .registers = f.dsp.cpu().registers(),
                     .progress = f.dsp.cpu().progress(),
                     .pending = pending ? std::optional{pending->effect} : std::nullopt});
  };
  f.attach();
  f.run(2);
  require(f.mcu.cpu().state().x == loaded_index && f.mcu.run_clock().bus_cycles() == 8,
          "Genuine reset vector,NOP and two-period LDX precede actual DBNE execution");
  const auto saved = f.runtime->checkpoint();
  const auto chip = f.dsp.checkpoint();
  std::string first_error;
  std::vector<Frame> suffix;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.restore(saved);
    trace.clear();
    const auto error = rejects([&] { static_cast<void>(f.step()); });
    const auto pending = f.dsp.cpu().pending_instruction();
    const auto completed = completed_prefix.at(consumed);
    require(f.runtime->faulted() && f.mcu.timing_faulted() && f.dsp.clock().faulted() &&
                f.mcu.run_clock().bus_cycles() == 8 + consumed &&
                f.mcu.run_clock().elapsed_ticks() == setup_ticks + (consumed * 10) &&
                f.mcu.run_clock().oscillator_cycles() == 16 + (consumed * 2) &&
                f.mcu.cpu().state().x == loaded_index - 1 &&
                f.mcu.cpu().state().ccr == expected_ccr &&
                f.runtime->completed_cycles() == completed &&
                f.dsp.clock().elapsed_ticks() == recovered + (completed * dsp_period) &&
                f.dsp.clock().hardware_reset_ticks() == recovered &&
                f.dsp.clock().timer_clk2_edges() == completed / 2 &&
                f.runtime->end_ticks() == first_offer + (consumed * mcu_period) &&
                f.dsp.cpu().registers() == expected_nops(padding) &&
                f.dsp.cpu().progress() ==
                    dsp56300::Progress{.instructions = padding, .estimated_cycles = padding},
            "Only completed MCU/DSP periods advance; actual DBNE decrement survives the fault");
    require(error.contains("Unmapped DSP memory write at Y:0x002000") &&
                error.contains("HC12 cycle execution pc=0x8004 ccr=0xd8") &&
                error.contains("attempted-ticks=" + std::to_string(failure_ticks.at(consumed))) &&
                pending && pending->effect.kind == dsp56300::EffectKind::write &&
                pending->effect.space == dsp56300::Space::y &&
                pending->effect.address == reserved && pending->effect.pc == boot + padding &&
                pending->effect.value == 0,
            "Actual native write retains operation,value,PC,address and its uncharged deadline");
    rejects([&] { static_cast<void>(f.runtime->checkpoint()); });
    f.dsp.restore(chip);
    rejects([&] { f.runtime->end_instruction(); });
    if (replay == 0) {
      first_error = error;
      suffix = trace;
    }
    require(error == first_error && trace == suffix,
            "Joint recovery repeats every native effect and the exact interrupted MCU slot");
  }
  f.restore(saved);
  require(!f.runtime->faulted() && f.mcu.cpu().state().x == loaded_index,
          "A full joint snapshot restores both owners after unilateral chip recovery refused");
}

void contract_guards() {
  Fixture f;
  OwnDspMachine slow({.extal_hz = dsp_hz,
                      .timebase_hz = dsp_hz,
                      .switch_timing = DspClockSwitch::reference_cycle_boundary});
  const auto before = slow.cpu().registers();
  require(rejects([&] {
            const McuDspRuntime bad(runtime_inputs(), f.mcu, slow);
          }).contains("exact positive common timebase") &&
              slow.cpu().registers() == before && slow.clock().elapsed_ticks() == 0,
          "Incompatible common timebase refuses before native DSP reset or execution");
  for (const bool budget : {false, true}) {
    OwnDspMachine target({.extal_hz = dsp_hz,
                          .timebase_hz = common_hz,
                          .switch_timing = DspClockSwitch::reference_cycle_boundary});
    auto settings = runtime_inputs();
    if (budget) {
      settings.maximum_cycles_per_period = 0;
    } else {
      settings.timing = OwnDspAutomaticTiming::unqualified;
    }
    const auto initial = target.cpu().registers();
    const auto error = rejects([&] { const McuDspRuntime bad(settings, f.mcu, target); });
    require(error.contains(budget ? "coherent event ownership" : "qualified timing") &&
                target.cpu().registers() == initial && target.clock().elapsed_ticks() == 0,
            "Zero budget or unqualified effect timing refuses before changing native DSP state");
  }
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
    for (unsigned slot = 0; slot < 3; ++slot) { delay_slot(slot); }
    contract_guards();
    std::cout
        << "Native MCU/dsp56300 schedule:13 cases,6 restored delay-slot fault suffixes passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
