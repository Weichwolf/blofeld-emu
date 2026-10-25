// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu_checkpoint_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <optional>
#include <utility>

namespace blofeld::test {
namespace {
constexpr unsigned wai = 0x3e;
constexpr unsigned frame_size = 9;
constexpr unsigned masked = 0xd0;
constexpr unsigned unmasked = 0xc0;

void test_native_wait_and_branching() {
  constexpr std::array<std::uint8_t, 1> code{wai};
  McuMachine machine(checkpoint_image(code), BLOFELD_DEFAULT_SLA);
  drive_interrupts(machine.bus());
  constexpr CpuState initial{
      .pc = entry, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = stack, .ccr = unmasked};
  machine.cpu().set_state(initial);
  machine.cpu().step();
  const auto checkpoint = machine.checkpoint();
  constexpr std::array<std::uint8_t, frame_size> golden{
      0xc0, 0x34, 0x12, 0x56, 0x78, 0x9a, 0xbc, 0x80, 1};
  unsigned old_observer_calls = 0;
  machine.bus().set_observer([&old_observer_calls](const BusEvent &) { ++old_observer_calls; });
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    machine.restore(checkpoint);
    require(machine.cpu().waiting() && machine.cpu().state().sp == stack - frame_size,
            "Restore preserves native WAI and its already stacked frame");
    for (unsigned index = 0; index < golden.size(); ++index) {
      require(machine.bus().read(static_cast<std::uint16_t>(stack - frame_size + index)) ==
                  golden[index],
              "Checkpoint retains the independent native frame byte order");
    }
    require(!machine.cpu().step().instruction_executed && machine.cpu().waiting(),
            "Restored WAI cannot execute an opcode while idle");
    machine.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 1});
    const auto revision = machine.bus().revision();
    const auto wake = machine.cpu().step();
    require(wake.stack_reused && wake.interrupt_vector == irq_vector &&
                machine.cpu().state().pc == handler && machine.bus().revision() == revision,
            "Restored WAI wakes through the real vector without writing a second frame");
    machine.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    machine.cpu().step();
    const auto after = machine.cpu().state();
    require(after.pc == entry + 1 && after.d == initial.d && after.x == initial.x &&
                after.y == initial.y && after.sp == stack && after.ccr == unmasked,
            "Native RTI restores every independently expected architectural register");
    machine.bus().write(ram, static_cast<std::uint8_t>(attempt));
  }
  require(old_observer_calls == 0, "Restore does not retain external callbacks");
  machine.restore(checkpoint);
  reject([&machine] { static_cast<void>(machine.bus().read(ram)); },
         "Branch mutations must not initialize frozen checkpoint RAM");
  McuMachine branch(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  branch.restore(checkpoint);
  require(branch.cpu().waiting(), "A checkpoint can seed another compatible machine");
}

void test_unmask_delay() {
  for (const bool tap : {false, true}) {
    constexpr std::array<std::uint8_t, 3> cli{0x10, 0xef, wai};
    constexpr std::array<std::uint8_t, 3> transfer{0xb7, 2, wai};
    McuMachine machine(checkpoint_image(tap ? transfer : cli), BLOFELD_DEFAULT_SLA);
    drive_interrupts(machine.bus());
    constexpr unsigned flags_in_a = 0x4034;
    machine.cpu().set_state({.pc = entry, .d = flags_in_a, .sp = stack, .ccr = masked});
    machine.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 1});
    machine.cpu().step();
    const auto checkpoint = machine.checkpoint();
    machine.cpu().step();
    machine.cpu().step();
    machine.restore(checkpoint);
    require(!machine.cpu().pending_interrupt() && machine.cpu().step().instruction_executed &&
                machine.cpu().waiting() && machine.cpu().state().pc == entry + 3,
            "Restore retains CLI/TAP one-instruction deferral, permitting the following WAI");
    require(machine.cpu().step().stack_reused,
            "Pending IRQ uses the frame of the instruction protected by the restored delay");
  }
}

void test_code_ram_and_pending_mapping() {
  McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  constexpr std::array<std::uint8_t, 4> code{0x86, 1, 0x20, 0xfc};
  for (unsigned index = 0; index < code.size(); ++index) {
    machine.bus().write(static_cast<std::uint16_t>(ram + index), code[index]);
  }
  machine.cpu().set_state({.pc = ram, .sp = stack, .ccr = masked});
  machine.cpu().step();
  machine.cpu().step();
  constexpr unsigned initrg = 0x11;
  constexpr unsigned relocated = 0x30;
  constexpr unsigned relocated_initrg = 0x3011;
  machine.bus().write(initrg, relocated);
  const auto checkpoint = machine.checkpoint();
  machine.bus().retire_instruction();
  machine.bus().write(ram + 1, 2);
  machine.cpu().step();
  require(machine.cpu().state().d == 0x0200, "The mutated branch executes its changed immediate");
  machine.restore(checkpoint);
  require(machine.bus().read(initrg) == 0, "Snapshot retains the not-yet-retired old register map");
  machine.cpu().step();
  require(machine.cpu().state().d == 0x0100 && machine.bus().read(relocated_initrg) == relocated,
          "Restored decoder executes original RAM, then commits the deferred register map");
  machine.bus().write(relocated_initrg, 0);
  machine.bus().retire_instruction();
  require(machine.bus().read(relocated_initrg) == relocated,
          "Snapshot preserves write-once mapping history");
}

void test_fault_and_active_guards() {
  // LDAA $1200: RAM is initially unknown. Checkpoints cannot hide failed operands.
  constexpr std::array<std::uint8_t, 5> code{0xb6, 0x12, 0, 0x5a, 2};
  McuMachine machine(checkpoint_image(code), BLOFELD_DEFAULT_SLA);
  const auto checkpoint = machine.checkpoint();
  reject([&machine] { machine.cpu().step(); }, "Unknown RAM must fail visibly");
  reject([&machine] { static_cast<void>(machine.checkpoint()); },
         "A failed CPU step is not a completed checkpoint boundary");
  machine.restore(checkpoint);
  machine.bus().write(ram, 3);
  unsigned attempts = 0;
  machine.bus().set_observer([&machine, &checkpoint, &attempts](const BusEvent &) {
    reject([&machine] { static_cast<void>(machine.checkpoint()); },
           "A callback cannot capture a partially executed CPU step");
    reject([&machine, &checkpoint] { machine.restore(checkpoint); },
           "A callback cannot replace the currently executing CPU and bus");
    ++attempts;
  });
  machine.cpu().step();
  machine.cpu().step();
  constexpr unsigned expected_data = 0x0300;
  require(attempts != 0 && machine.cpu().state().d == expected_data,
          "Rejected reentrant snapshot operations do not interrupt a valid operand");
  machine.bus().set_observer({});
  static_cast<void>(machine.checkpoint());
  // Synthetic stack in documented reserved I/O exercises observer reentrancy
  // during interrupt writes. This deliberately is not an RTI frame test.
  constexpr unsigned observation_stack = 0xf0;
  machine.cpu().set_state({.pc = entry, .sp = observation_stack, .ccr = unmasked});
  drive_interrupts(machine.bus());
  machine.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 1});
  machine.bus().set_observer([&machine, &checkpoint, &attempts](const BusEvent &) {
    reject([&machine] { static_cast<void>(machine.checkpoint()); },
           "Interrupt stacking is also inside the checkpoint exclusion boundary");
    reject([&machine, &checkpoint] { machine.restore(checkpoint); },
           "Interrupt entry cannot destroy its active CPU through restore");
    ++attempts;
  });
  require(machine.cpu().step().interrupt_vector == irq_vector && attempts > 1,
          "Whole native interrupt entry is guarded as well as native P-code execution");
  machine.bus().set_observer({});
  machine.restore(checkpoint);
  machine.bus().set_observer([&machine, &checkpoint](const BusEvent &) {
    reject([&machine] { static_cast<void>(machine.checkpoint()); },
           "Direct host bus observations cannot capture a partially retired bus operation");
    reject([&machine, &checkpoint] { machine.restore(checkpoint); },
           "Direct host bus observations cannot destroy the bus dispatching their callback");
    reject([&machine] { machine.bus().set_observer({}); },
           "An active observer cannot destroy its own function object");
  });
  machine.bus().write(2, 0);
}

void test_failed_postincrement_retry() {
  // Own LDAA 1,X+ (A6 30), CPU12 indexed post-increment encoding.
  constexpr std::array<std::uint8_t, 2> code{0xa6, 0x30};
  constexpr unsigned following_byte = 0xa5;
  constexpr unsigned supplied_byte = 0x3c;
  constexpr unsigned result_d = 0x3c34;
  constexpr unsigned result_ccr = 0xc1;
  constexpr CpuState initial{
      .pc = entry, .d = 0x1234, .x = ram, .y = 0x9abc, .sp = stack, .ccr = 0xc5};
  McuMachine machine(checkpoint_image(code), BLOFELD_DEFAULT_SLA);
  drive_interrupts(machine.bus());
  machine.bus().write(ram + 1, following_byte);
  machine.cpu().set_state(initial);
  const auto checkpoint = machine.checkpoint();
  reject([&machine] { machine.cpu().step(); }, "Unknown indexed operand must fail visibly");
  const auto partial = machine.cpu().state();
  require(partial.pc == entry && partial.x == ram + 1 && partial.d == initial.d &&
              partial.y == initial.y && partial.sp == initial.sp && partial.ccr == initial.ccr,
          "The unretired native translation exposes only its actual post-increment prefix");
  reject([&machine] { static_cast<void>(machine.checkpoint()); },
         "Partial execution cannot become a completed checkpoint");
  reject([&machine] { machine.cpu().step(); },
         "A failed post-increment load cannot resume at the next known byte");
  const auto stopped = machine.cpu().state();
  require(stopped.pc == partial.pc && stopped.x == partial.x && stopped.d == partial.d &&
              stopped.y == partial.y && stopped.sp == partial.sp && stopped.ccr == partial.ccr,
          "Refused retry cannot change the failed instruction prefix");
  machine.restore(checkpoint);
  require(machine.cpu().state().x == ram && machine.cpu().state().pc == entry,
          "Coordinated restore recovers the actual pre-read CPU and unknown operand");
  machine.bus().write(ram, supplied_byte);
  machine.cpu().step();
  const auto recovered = machine.cpu().state();
  require(
      recovered.pc == entry + code.size() && recovered.x == ram + 1 && recovered.d == result_d &&
          recovered.ccr == result_ccr && recovered.y == initial.y && recovered.sp == initial.sp &&
          machine.bus().read(ram + 1) == following_byte,
      "Recovery executes the original address once instead of silently skipping its unknown byte");
}

void test_lifetime_and_empty_handle() {
  std::optional<McuCheckpoint> checkpoint;
  {
    McuMachine temporary(checkpoint_image(), BLOFELD_DEFAULT_SLA);
    temporary.bus().write(ram, 7);
    checkpoint.emplace(temporary.checkpoint());
  }
  McuMachine machine(checkpoint_image(), BLOFELD_DEFAULT_SLA);
  auto moved = std::move(*checkpoint);
  machine.bus().write(ram, 3);
  reject([&machine, &checkpoint] { machine.restore(*checkpoint); },
         "Moved-from snapshot handles must fail atomically");
  require(machine.bus().read(ram) == 3, "Failed restore preserves the original bus");
  machine.restore(moved);
  require(machine.bus().read(ram) == 7 && machine.cpu().state().pc == entry,
          "Snapshot owns memory after the source machine is destroyed");
}

void test_backend_failure_atomicity(const std::filesystem::path &temporary) {
  const auto path = temporary / "checkpoint-backend.sla";
  std::filesystem::copy_file(BLOFELD_DEFAULT_SLA, path);
  McuMachine machine(checkpoint_image(), path);
  const auto checkpoint = machine.checkpoint();
  machine.bus().write(ram, 7);
  auto *const bus_before = &machine.bus();
  auto *const cpu_before = &machine.cpu();
  {
    std::ofstream altered(path, std::ios::app | std::ios::binary);
    altered.exceptions(std::ios::failbit | std::ios::badbit);
    altered.put('x');
  }
  reject([&machine, &checkpoint] { machine.restore(checkpoint); },
         "Changed instruction backend must reject restore before replacement");
  reject([&machine] { static_cast<void>(machine.checkpoint()); },
         "A changed backing SLA cannot be relabeled as the initial native backend");
  require(&machine.bus() == bus_before && &machine.cpu() == cpu_before &&
              machine.bus().read(ram) == 7,
          "A backend validation failure preserves the live objects and their mutations");
  std::filesystem::remove(path);
  reject([&machine, &checkpoint] { machine.restore(checkpoint); },
         "Missing backend must fail before replacing live state");
  require(&machine.bus() == bus_before && machine.bus().read(ram) == 7,
          "Missing backend failure is atomic");
  std::filesystem::copy_file(BLOFELD_DEFAULT_SLA, path);
  machine.restore(checkpoint);
  reject([&machine] { static_cast<void>(machine.bus().read(ram)); },
         "Restoring identical backend bytes at another path recovers the original unknown RAM");
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 2) { throw blofeld::Fault("Usage: mcu-checkpoint-test temporary-directory"); }
    blofeld::test::test_native_wait_and_branching();
    blofeld::test::test_unmask_delay();
    blofeld::test::test_code_ram_and_pending_mapping();
    blofeld::test::test_fault_and_active_guards();
    blofeld::test::test_failed_postincrement_retry();
    blofeld::test::test_lifetime_and_empty_handle();
    blofeld::test::test_backend_failure_atomicity(arguments[1]);
    blofeld::test::test_checkpoint_peripherals();
    blofeld::test::test_checkpoint_flash();
    std::cout << "MCU in-process checkpoints, native replay, isolation and failure guards passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
