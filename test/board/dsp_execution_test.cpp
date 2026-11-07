// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_execution.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned extal = 12000000;
constexpr std::uint64_t timebase = 540000000, period = 45, recovery_ticks = 90;
constexpr std::uint64_t initial_ticks = recovery_ticks * 2;
constexpr unsigned boot_address = 0xff0000;
constexpr auto jump = dsp56300::isa::opcode<dsp56300::isa::Opcode::jmp_absolute_short>().front();
constexpr auto nop = dsp56300::isa::nop.front();
unsigned checks = 0;

void require(const bool condition, const std::string_view message) {
  ++checks;
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

struct Device {
  DspMachine machine{{.extal_hz = extal,
                      .timebase_hz = timebase,
                      .switch_timing = DspClockSwitch::functional_region_boundary}};
  DspExecution timeline{
      machine,
      0,
      DspResetInputs{.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};

  explicit Device(const std::span<const dsp56300::Word> program) {
    constexpr std::array bootstrap{jump};
    machine.bus().install_rom({.start = boot_address, .words = bootstrap});
    for (unsigned index = 0; index < program.size(); ++index) {
      machine.bus().write({.address = index, .value = program[index]});
    }
    require(timeline.advance_reset_to(recovery_ticks) == recovery_ticks,
            "Reset assertion advances actual held EXTAL time");
    timeline.release_hardware_reset();
    require(timeline.advance_reset_to(initial_ticks) == recovery_ticks &&
                !machine.clock().hardware_reset_active(),
            "Only completed reset recovery releases native execution");
  }

  std::optional<DspExecutionResult> advance(const std::uint64_t until,
                                            const DspPeripheralObserver observer = {}) {
    return timeline.advance_native_region(
        until,
        DspBoundary::through,
        {.timing = DspDmaTiming::reference_sampled_cycles, .grant = DspDmaGrant::available},
        observer);
  }
};

void complete_instruction_time() {
  constexpr std::array program{nop, jump};
  Device device(program);
  const auto first = device.advance(initial_ticks + period);
  require(first && first->execution && first->execution->retired == 1 &&
              first->execution->fault.kind == dsp56300::FaultKind::none &&
              first->execution->progress.estimated_cycles == 3 &&
              device.machine.cpu().registers().pc == 0,
          "The genuine bootstrap jump executes atomically at its first native boundary");
  for (unsigned cycles = 2; cycles <= 3; ++cycles) {
    const auto partial = device.advance(initial_ticks + (cycles * period));
    require(partial && !partial->execution &&
                device.machine.cpu().state().progress.instructions == 1 &&
                device.machine.clock().elapsed_ticks() == initial_ticks + (cycles * period),
            "An external event inside the jump advances time without redispatching guest code");
  }
  const auto next = device.advance(initial_ticks + (4 * period));
  require(next && next->execution && next->execution->retired == 1 &&
              next->execution->progress ==
                  dsp56300::Progress{.instructions = 2, .estimated_cycles = 4} &&
              device.machine.cpu().registers().pc == 1,
          "The next instruction starts after the full preceding instruction time");
  constexpr std::uint64_t final_cycles = 10000;
  const auto until = initial_ticks + (final_cycles * period);
  while (device.machine.clock().elapsed_ticks() < until) {
    require(device.advance(until).has_value(), "Every native/device span makes real progress");
  }
  require(device.machine.cpu().state().progress.estimated_cycles == final_cycles &&
              device.machine.clock().elapsed_cycle_units() == final_cycles,
          "Large native grants retain matching instruction and peripheral progress");
}

void memory_switch_boundary(const bool split) {
  constexpr unsigned prefix = 32;
  constexpr unsigned memory_switch = 0x80;
  constexpr unsigned omr_register = 58;
  constexpr auto move_omr =
      dsp56300::isa::opcode<dsp56300::isa::Opcode::movec_effective>().front() | 0xb400 |
      (omr_register & 0x1f);
  constexpr unsigned initial_omr = 0x305;
  constexpr unsigned high_bank = 0xc000;
  constexpr unsigned original_end = 0x12000;
  constexpr unsigned switched_end = 0x10000;
  constexpr std::uint64_t command_cycles = 3 + prefix + 2;
  std::array<dsp56300::Word, prefix + 6> program{};
  program.fill(nop);
  program[prefix] = move_omr;
  program[prefix + 1] = initial_omr | memory_switch;
  program.back() = jump;
  Device device(program);
  constexpr unsigned aliased_program = 0x1000;
  constexpr unsigned datum = 0x123456;
  device.machine.bus().write({.space = DspSpace::x, .address = switched_end, .value = datum});
  const auto horizon_cycles = split ? command_cycles - 1 : command_cycles + 10;
  const auto horizon = initial_ticks + (horizon_cycles * period);
  while (device.machine.cpu().state().progress.estimated_cycles < command_cycles) {
    const auto step = device.advance(horizon);
    require(step && (!step->execution || step->execution->fault.kind == dsp56300::FaultKind::none),
            "The authored NOP/MOVEC prefix executes without a core or device fault");
  }
  const auto end = [&] {
    return high_bank + device.machine.bus().data_windows(DspSpace::x)[2].words.size();
  };
  require(device.machine.cpu().registers().omr == (initial_omr | memory_switch) &&
              device.machine.cpu().state().progress.estimated_cycles == command_cycles &&
              device.machine.clock().elapsed_cycle_units() == command_cycles - (split ? 1U : 0U) &&
              end() == (split ? original_end : switched_end) &&
              !device.machine.bus().inspect_known_program(aliased_program),
          "The alias remains absent until the real post-command switch delay completes");
  if (split) {
    require(device.machine.bus().read({.space = DspSpace::x, .address = switched_end}) == datum,
            "An event inside MOVEC still observes the original data bank");
    const auto paid = device.advance(initial_ticks + (command_cycles * period));
    require(paid && !paid->execution && end() == switched_end &&
                !device.machine.bus().inspect_known_program(aliased_program),
            "Completing MOVEC admits the switch without executing the next instruction");
  }
  for (unsigned after = 1; after <= 3; ++after) {
    const auto step = device.advance(initial_ticks + ((command_cycles + after) * period));
    const auto mapped = device.machine.bus().inspect_known_program(aliased_program);
    require(step && (after == 3 ? mapped == datum : !mapped),
            "The original RAM word becomes program memory after three subsequent cycles");
  }
}

struct Observed {
  Device &device;
  std::vector<std::array<std::uint64_t, 3>> calls;

  static void before(void *context, const unsigned pc) {
    auto &self = *static_cast<Observed *>(context);
    self.calls.push_back({0, pc, self.device.machine.clock().elapsed_ticks()});
  }

  static void after(void *context, const unsigned pc) {
    auto &self = *static_cast<Observed *>(context);
    self.calls.push_back({1, pc, self.device.machine.clock().elapsed_ticks()});
  }
};

void real_peripheral_boundary() {
  constexpr std::array program{nop, dsp56300::isa::reset.front(), nop, jump};
  Device device(program);
  Observed observed{.device = device, .calls = {}};
  const DspPeripheralObserver observer{
      .context = &observed, .preparing = Observed::before, .completed = Observed::after};
  constexpr std::uint64_t horizon = initial_ticks + (1000 * period);
  std::optional<DspExecutionResult> effect;
  while (observed.calls.empty()) { effect = device.advance(horizon, observer); }
  constexpr std::uint64_t reset_completion_cycles = 11;
  constexpr std::uint64_t prefix_ticks = initial_ticks + (4 * period);
  const std::vector<std::array<std::uint64_t, 3>> expected{{0, 1, prefix_ticks},
                                                           {1, 1, prefix_ticks}};
  require(
      observed.calls == expected && effect && effect->execution && effect->execution->yielded &&
          device.machine.cpu().state().progress ==
              dsp56300::Progress{.instructions = 3, .estimated_cycles = reset_completion_cycles} &&
          device.machine.clock().elapsed_ticks() ==
              initial_ticks + (reset_completion_cycles * period),
      "Real peripheral RESET observes its completed prefix once and yields after retirement");
}

void wait_retains_cpu_progress() {
  constexpr std::array program{dsp56300::isa::wait.front(), nop};
  Device device(program);
  constexpr std::uint64_t wait_cycles = 13;
  constexpr std::uint64_t later_cycles = 113;
  const auto entered = initial_ticks + (wait_cycles * period);
  while (device.machine.clock().elapsed_ticks() < entered) {
    require(device.advance(entered).has_value(), "WAIT entry reaches its actual clock boundary");
  }
  const auto previous = device.machine.cpu().state();
  require(previous.execution == dsp56300::ExecutionState::waiting &&
              previous.progress ==
                  dsp56300::Progress{.instructions = 2, .estimated_cycles = wait_cycles},
          "Bootstrap and native WAIT retire before sleep");
  require(device.advance(initial_ticks + (later_cycles * period)).has_value() &&
              device.machine.cpu().state().progress == previous.progress &&
              device.machine.clock().elapsed_cycle_units() == later_cycles,
          "Sleeping advances device time without adding CPU instructions or nominal CPU cycles");
}
}

int main() {
  try {
    complete_instruction_time();
    memory_switch_boundary(false);
    memory_switch_boundary(true);
    real_peripheral_boundary();
    wait_retains_cpu_progress();
    std::cout << checks << " functional DSP execution checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
