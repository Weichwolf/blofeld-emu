// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
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
#include <stdexcept>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr Word irqa_vector = 0x10, priority_register = 0xffffff;
constexpr Word foreground = 0x300, interrupted = foreground + 3, boot = 0xff0000;
constexpr Word register_mask = 0x1f, immediate_mode = 0xb400, sr_register = 57, vba_register = 48;
constexpr auto control_move = isa::opcode<isa::Opcode::movec_effective>().front() | immediate_mode;
constexpr auto move_sr = control_move | (sr_register & register_mask);
constexpr auto move_vba = control_move | (vba_register & register_mask);
constexpr auto jump = isa::opcode<isa::Opcode::jmp_absolute_short>().front();
constexpr auto nop = isa::nop.front();
constexpr unsigned extal = 12000000;
constexpr std::uint64_t timebase = 540000000, reset_ticks = 90;
unsigned checks = 0;

void require(const bool condition, const char *message) {
  ++checks;
  if (!condition) { throw std::runtime_error(message); }
}

struct Device {
  DspMachine machine{{.extal_hz = extal,
                      .timebase_hz = timebase,
                      .switch_timing = DspClockSwitch::reference_cycle_boundary}};

  explicit Device(const Word base, const bool missing_second) {
    constexpr Word masked = 0x300;
    const std::array bootstrap{move_sr, masked, move_vba, base, jump | foreground};
    machine.bus().install_rom({.start = boot, .words = bootstrap});
    const std::array program{move_sr, Word{0}, nop, nop};
    for (unsigned word = 0; word < program.size(); ++word) {
      machine.bus().write({.address = foreground + word, .value = program[word]});
    }
    machine.bus().write({.address = base + irqa_vector, .value = nop});
    if (!missing_second) { machine.bus().write({.address = base + irqa_vector + 1, .value = nop}); }
    machine.begin_hardware_reset(
        {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
    require(machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
            "The real test device owns reset assertion time");
    machine.release_hardware_reset();
    require(machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
            "The real test device completes reset recovery");
    const auto started = machine.run(3);
    require(started.retired == 3 && started.fault.kind == FaultKind::none &&
                machine.cpu().registers().pc == foreground && machine.cpu().registers().vba == base,
            "Authored native reset code establishes real masked status, VBA and foreground");
    machine.bus().observe_irqa(true);
    machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
    machine.bus().observe_irqa(false);
    require(
        machine.bus().interrupt_request(0) ==
                DspInterruptRequest{.vector = irqa_vector, .level = 1} &&
            !machine.admit_interrupt({.address = foreground, .opcode = move_sr, .extension = 0}),
        "A latched edge remains masked by the original SR rather than being acknowledged");
    const auto unmasked = machine.run(2);
    require(unmasked.retired == 2 && unmasked.fault.kind == FaultKind::none &&
                machine.cpu().registers().pc == interrupted &&
                machine.admit_interrupt({.address = interrupted, .opcode = nop}) &&
                machine.interrupt_pending(),
            "Real MOVEC and foreground NOP release masking and admit the actual request");
  }
};

void check_vector(const Word base, const bool missing_second) {
  Device device(base, missing_second);
  auto &machine = device.machine;
  const auto before = machine.cpu().state();
  const auto first_word = machine.io().inspect_program(base + irqa_vector);
  const auto second_word = machine.io().inspect_program(base + irqa_vector + 1);
  require(first_word == nop && second_word.has_value() != missing_second &&
              machine.interrupt_pending() && machine.bus().interrupt_request(0),
          "Inspection of either vector word cannot acknowledge or execute the source");
  const auto first = machine.run_unclocked();
  if (missing_second) {
    const auto &failure = machine.io().failure();
    require(first.retired == 0 && first.fault.kind == FaultKind::memory_read &&
                first.fault.space == Space::program &&
                first.fault.address == base + irqa_vector + 1 && failure && failure->cause &&
                failure->address == base + irqa_vector + 1 && machine.interrupt_pending() &&
                machine.bus().interrupt_request(0) && first.progress == before.progress,
            "An unknown second word fails before first-slot retirement or source acknowledgement");
    return;
  }
  require(first.retired == 1 && first.fault.kind == FaultKind::none &&
              !machine.interrupt_pending() && !machine.bus().interrupt_request(0),
          "The native pair prefetch acknowledges only after both real vector reads succeed");
  const auto second = machine.run_unclocked();
  require(second.retired == 1 && second.fault.kind == FaultKind::none &&
              machine.cpu().registers() == before.registers &&
              second.progress == Progress{.instructions = before.progress.instructions + 2,
                                          .estimated_cycles = before.progress.estimated_cycles + 2},
          "The latched second NOP preserves the complete foreground register state");
  const auto resumed = machine.run_unclocked();
  auto expected = before.registers;
  expected.pc = interrupted + 1;
  require(resumed.retired == 1 && resumed.fault.kind == FaultKind::none &&
              machine.cpu().registers() == expected && !machine.interrupt_pending(),
          "Native execution resumes the original foreground without an extra vector read");
  const auto refused = machine.run_unclocked();
  require(refused.retired == 0 && refused.fault.kind == FaultKind::memory_read &&
              refused.fault.space == Space::program && refused.fault.address == interrupted + 1 &&
              !machine.interrupt_pending(),
          "Missing foreground code fails without fabricated retirement or acknowledgement");
}
}

int main() {
  try {
    for (const Word base : {Word{0}, Word{0x100}}) {
      check_vector(base, false);
      check_vector(base, true);
    }
    std::cout << checks << " native interrupt ownership checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
