// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "cpu12/memory.hpp"
#include "isa.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_memory.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
namespace isa = cpu12::isa;
using cpu12::Byte;
using isa::Opcode;
constexpr std::uint16_t entry = 0x8000, loader = 0x8100, first = 0x1400, stack = 0x2000;
constexpr unsigned program_size = 64, result_address = 0x1300, byte_mask = 0xff;
constexpr unsigned protection = 0x7f0d, security = 0x7f0f, unsecured = 0xfe;
constexpr unsigned oscillator = 12000000, expected_d = 0x1934;
constexpr std::uint64_t run_ticks = 100000;
constexpr Byte masks = 0xd0, tail_value = 0xa5, old_value = 1, new_value = 7;
constexpr Byte initial_value = 0x12, outcome_value = 0x19, initial_low = 0x34;
constexpr Byte relocated_ram = 0x29;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

constexpr std::array<Byte, 2> address(const unsigned value) {
  return {static_cast<Byte>(value >> 8U), static_cast<Byte>(value & byte_mask)};
}

McuFlashImage image() {
  std::array<Byte, McuFlashImage::byte_count> bytes;
  bytes.fill(isa::nop.front());
  bytes.front() = isa::wai.front();
  constexpr auto replacement = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                            std::array<Byte, 1>{new_value},
                                            isa::opcode<Opcode::staa_extended>(),
                                            address(first + 1),
                                            isa::opcode<Opcode::ldaa_immediate>(),
                                            std::array<Byte, 1>{tail_value},
                                            isa::opcode<Opcode::staa_extended>(),
                                            address(first + program_size - 1),
                                            isa::opcode<Opcode::jmp_extended>(),
                                            address(first));
  std::ranges::copy(replacement, bytes.begin() + loader - entry);
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::internal_flash_reads,
          .ram_program = McuProgramWindow{.first = first, .size = program_size}};
}

constexpr auto finite_program() {
  return isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                      std::array<Byte, 1>{initial_value},
                      isa::opcode<Opcode::adda_immediate>(),
                      std::array<Byte, 1>{new_value},
                      isa::opcode<Opcode::staa_extended>(),
                      address(result_address),
                      isa::wai);
}

struct Fixture {
  explicit Fixture(const std::span<const Byte> program,
                   const bool complete = true,
                   const McuRunContract &inputs = contract())
      : machine(image(), {}, inputs) {
    machine.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    for (unsigned offset = 0; offset < program_size - (complete ? 0U : 1U); ++offset) {
      machine.bus().write(static_cast<std::uint16_t>(first + offset),
                          offset < program.size() ? program[offset] : isa::nop.front());
    }
    start(first);
  }

  void start(const std::uint16_t pc) {
    machine.cpu().set_state({.pc = pc, .d = initial_low, .sp = stack, .ccr = masks});
  }

  CpuStep run() { return machine.run_until(machine.run_clock().elapsed_ticks() + run_ticks); }

  [[nodiscard]] bool retained() { return !program_bytes().empty(); }

  [[nodiscard]] std::span<const Byte> program_bytes() {
    for (const auto &window : machine.bus().memory_windows()) {
      if (window.first == first && window.bytes.size() == program_size && !window.writable &&
          window.known.empty()) {
        return window.bytes;
      }
    }
    return {};
  }

  [[nodiscard]] Byte read(const std::size_t location) const {
    const auto byte = machine.bus().inspect_memory(static_cast<std::uint16_t>(location));
    if (!byte.value) { throw std::runtime_error("Expected an initialized actual RAM byte"); }
    return *byte.value;
  }

  McuMachine machine;
};

void complete_execution_and_neighbors() {
  constexpr auto program = finite_program();
  Fixture fixture(program);
  require(!fixture.retained(), "Loading RAM does not prematurely publish a program image");
  const auto result = fixture.run();
  const auto cpu = fixture.machine.cpu().state();
  require(fixture.retained() && result.retired == 4 && fixture.machine.cpu().waiting() &&
              cpu.pc == first + program.size() && cpu.d == expected_d && cpu.sp == stack - 9 &&
              cpu.ccr == masks && fixture.read(result_address) == outcome_value,
          "Native RAM code preserves literal arithmetic, WAI, flags and writable stack/results");
  // Reset + LDAA #imm + ADDA #imm + STAA extended + WAI (OSSSSsf).
  constexpr unsigned expected_cycles = 5 + 1 + 1 + 3 + 7;
  require(fixture.machine.cpu().cycles() == expected_cycles &&
              fixture.machine.run_clock().bus_cycles() == expected_cycles,
          "RAM qualification preserves reset and instruction cycles");
  for (std::size_t index = 0; index < program.size(); ++index) {
    require(fixture.read(first + index) == program[index], "Execution never patches code bytes");
  }
  auto inputs = contract();
  inputs.ram_program.reset();
  Fixture ordinary(program, true, inputs);
  const auto original = ordinary.run();
  const auto baseline = ordinary.machine.cpu().state();
  require(!ordinary.retained() && original.retired == result.retired && baseline.pc == cpu.pc &&
              baseline.d == cpu.d && baseline.x == cpu.x && baseline.y == cpu.y &&
              baseline.sp == cpu.sp && baseline.ccr == cpu.ccr &&
              ordinary.machine.run_clock().elapsed_ticks() ==
                  fixture.machine.run_clock().elapsed_ticks(),
          "Qualified and ordinary RAM execution agree without changing device time");
}

void reload_through_native_stores() {
  constexpr auto program = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{old_value},
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(result_address),
                                        isa::opcode<Opcode::jmp_extended>(),
                                        address(first));
  Fixture fixture(program);
  static_cast<void>(fixture.run());
  require(fixture.retained() && fixture.read(result_address) == old_value,
          "Warm the actual old program before replacement");
  const auto old_image = fixture.program_bytes();
  const auto revision = fixture.machine.bus().code_revision();
  fixture.start(loader);
  const auto prefix = fixture.run();
  require(prefix.retired == 2 && fixture.machine.cpu().state().pc == loader + 5 &&
              fixture.read(first + 1) == new_value && !fixture.retained() &&
              fixture.machine.bus().code_revision() == revision + 1,
          "The first native program write retires its image and yields at the precise opcode");
  static_cast<void>(fixture.run());
  require(fixture.retained() && fixture.read(result_address) == new_value &&
              fixture.read(first + program_size - 1) == tail_value &&
              fixture.machine.bus().code_revision() == revision + 2,
          "Remaining loader writes use ordinary RAM; execution publishes one new generation");
  static_cast<void>(fixture.run());
  require(fixture.read(result_address) == new_value &&
              fixture.machine.bus().code_revision() == revision + 2,
          "Warm re-entry cannot execute stale native code or republish an unchanged image");
  require(old_image[1] == old_value && fixture.program_bytes().data() != old_image.data(),
          "Replacing RAM never mutates the older immutable program binding");
  fixture.machine.bus().write(first + 1, old_value);
  fixture.machine.bus().write(first + program_size - 1, isa::nop.front());
  fixture.start(first);
  static_cast<void>(fixture.run());
  require(fixture.read(result_address) == old_value &&
              fixture.program_bytes().data() == old_image.data(),
          "Reloading identical bytes reuses the original binding and its retained native code");
}

void write_inside_running_image() {
  constexpr auto program = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{new_value},
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(first + 6),
                                        isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{old_value},
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(result_address),
                                        isa::wai);
  Fixture fixture(program);
  const auto prefix = fixture.run();
  require(prefix.retired == 2 && fixture.machine.cpu().state().pc == first + 5 &&
              fixture.read(first + 6) == new_value,
          "Even a native write inside its own image exits before the changed instruction");
  const auto result = fixture.run();
  require(result.retired == 3 && fixture.machine.cpu().waiting() &&
              fixture.read(result_address) == new_value,
          "The next image executes the actual newly written immediate, never the old decode");
}

void unknown_tail_and_mapping_changes() {
  Fixture partial(finite_program(), false);
  const auto revision = partial.machine.bus().code_revision();
  static_cast<void>(partial.run());
  require(!partial.retained() && partial.read(result_address) == outcome_value &&
              partial.machine.bus().code_revision() == revision &&
              !partial.machine.bus().inspect_memory(first + program_size - 1).value,
          "An unknown tail remains unknown and prevents whole-image qualification");
  Fixture complete(finite_program());
  static_cast<void>(complete.run());
  const auto copy = complete.machine.bus().clone();
  copy->write(first + 1, new_value);
  require(complete.read(first + 1) == initial_value && complete.retained(),
          "Replacing an independently owned copy cannot invalidate the original image");
  const auto reset = complete.machine.bus().clone();
  reset->reset();
  require(std::ranges::any_of(reset->memory_windows(),
                              [](const auto &window) {
                                return window.writable && first >= window.first &&
                                       first - window.first < window.bytes.size();
                              }),
          "Reset restores ordinary writable RAM at the former program address");
  complete.machine.bus().write(hc12::initrm, relocated_ram);
  complete.machine.bus().retire_instruction();
  require(!complete.retained(), "A committed decoder change retires the old logical image");
}

void invalid_windows_and_unknown_execution() {
  constexpr std::array invalid{McuProgramWindow{}, McuProgramWindow{.first = 65535, .size = 2}};
  for (const auto window : invalid) {
    auto inputs = contract();
    inputs.ram_program = window;
    bool refused = false;
    try {
      const McuMachine machine(image(), {}, inputs);
    } catch (const Fault &error) {
      refused = std::string_view(error.what()).contains("Invalid MCU RAM program window");
    }
    require(refused, "Empty and overflowing program windows fail explicitly");
  }
  Fixture partial(finite_program(), false);
  partial.start(first + program_size - 1);
  bool refused = false;
  try {
    static_cast<void>(partial.run());
  } catch (const Fault &error) {
    refused = std::string_view(error.what()).contains("Uninitialized or unmapped MCU instruction");
  }
  require(refused,
          "Entering an unknown program byte faults instead of qualifying or fabricating it");
}

void real_reset_rebinds_program() {
  Fixture fixture(finite_program());
  static_cast<void>(fixture.run());
  require(fixture.retained(), "Establish a native RAM program before real system reset");
  fixture.machine.begin_system_reset(McuResetSource::external, 3);
  require(!fixture.retained(), "Asserted reset retires the old execution binding");
  constexpr unsigned reset_edges = 128 + 3 + 64;
  require(fixture.machine.advance_system_reset(reset_edges, McuResetPad::high) == reset_edges &&
              !fixture.machine.system_reset_active() && !fixture.machine.timing_faulted(),
          "The real reset sequence reconstructs native owners without losing the RAM contract");
  fixture.start(first);
  static_cast<void>(fixture.run());
  require(fixture.retained() && fixture.read(result_address) == outcome_value,
          "Retained RAM bytes execute through a fresh native owner after system reset");
}
}

int main() {
  try {
    complete_execution_and_neighbors();
    reload_through_native_stores();
    write_inside_running_image();
    unknown_tail_and_mapping_changes();
    invalid_windows_and_unknown_execution();
    real_reset_rebinds_program();
    std::cout << "RAM program generations preserve native execution, writes, time and resets\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
