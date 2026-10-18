// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_flash.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
constexpr unsigned array_bytes = 32768;
constexpr unsigned array_base = 0x8000;
constexpr unsigned target = 0x8234;
constexpr unsigned sector_begin = 0x8200;
constexpr unsigned sector_end = 0x8400;
constexpr unsigned divider = 0x100;
constexpr unsigned control = 0x103;
constexpr unsigned protection = 0x104;
constexpr unsigned status = 0x105;
constexpr unsigned command = 0x106;
constexpr unsigned idle = 0xc0;
constexpr unsigned executing = 0x80;
constexpr unsigned access_error = 0x10;
constexpr unsigned protect_error = 0x20;
constexpr unsigned pattern = 0xa5;
constexpr unsigned erased = 0xff;
constexpr unsigned program = 0x20;
constexpr unsigned erase = 0x40;
constexpr unsigned mass = 0x41;
constexpr unsigned verify = 5;
constexpr unsigned clear_errors = 0x30;
constexpr unsigned word_data = 0x1234;
constexpr unsigned word_mask = 0xffff;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned bus_hz = 24000000;
constexpr unsigned reference_divider = 0x48;
constexpr unsigned irq_vector = 0xffb8;
constexpr unsigned irq_offset = irq_vector - array_base;
constexpr unsigned handler = 0x9000;
constexpr unsigned stack = 0x1ff0;
constexpr unsigned masked = 0xd0;
constexpr unsigned x_mask = 0x40;
constexpr unsigned unsecured_options = 0xfe;
constexpr unsigned data_high = word_data >> 8U;
constexpr unsigned data_low = word_data & erased;

void require(const bool value, const char *message) {
  if (!value) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const char *message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

std::array<std::uint8_t, array_bytes> image() {
  std::array<std::uint8_t, array_bytes> bytes{};
  bytes.fill(pattern);
  constexpr unsigned protection_field = 0x7f0d;
  constexpr unsigned security_field = 0x7f0f;
  bytes[protection_field] = erased;
  bytes[security_field] = unsecured_options;
  return bytes;
}

void prepare(blofeld::McuBus &bus) {
  bus.write(divider, reference_divider);
  bus.write(status, clear_errors);
  bus.observe_flash_clock({.oscillator_hz = oscillator_hz, .bus_hz = bus_hz});
}

struct Operation {
  std::uint16_t address = target;
  std::uint8_t command = program;
  std::uint16_t data = word_data;
};

void launch(blofeld::McuBus &bus, const Operation operation) {
  bus.write_memory({.address = operation.address, .size = 2, .value = operation.data});
  bus.write(command, operation.command);
  bus.write(status, executing);
}

void finish(blofeld::McuBus &bus) {
  bus.advance_flash(blofeld::FlashMilestone::transfer_buffer);
  bus.advance_flash(blofeld::FlashMilestone::complete_command);
}

void erase_target(blofeld::McuBus &bus) {
  launch(bus, {.command = erase});
  finish(bus);
}

void verify_idle_fault_context(blofeld::McuBus &bus) {
  bus.begin_instruction({.pc = word_data, .ccr = masked});
  constexpr unsigned idle_rmw = 0xf0;
  try {
    bus.write(status, idle_rmw);
  } catch (const blofeld::Fault &fault) {
    const std::string message = fault.what();
    require(message.find("MCU write at 0x0105") != std::string::npos &&
                message.find("pc=0x1234") != std::string::npos &&
                message.find("ccr=0xd0") != std::string::npos,
            "Unqualified register writes preserve operation, address, PC and CCR in diagnostics");
    return;
  }
  throw blofeld::Fault("Idle CBEIF RMW without a staged command requires qualification");
}

void test_sequence_errors() {
  blofeld::McuBus bus(image());
  require(bus.read(status) == idle && bus.read(command) == 0 && bus.read(control) == 0,
          "Independent FTS32K reset status has both empty/complete flags and no command");
  verify_idle_fault_context(bus);
  require(bus.read(status) == idle, "Rejected idle launch cannot change the command state");
  bus.write_memory({.address = target, .size = 2, .value = word_data});
  require(bus.read(status) == (idle | access_error), "An unwritten divider rejects array staging");
  prepare(bus);
  for (const unsigned width : {1U, 3U}) {
    bus.write_memory({.address = target, .size = width});
    require(bus.read(status) == (idle | access_error), "Only aligned word array writes are legal");
    bus.write(status, access_error);
  }
  bus.write_memory({.address = target + 1, .size = 2});
  require(bus.read(status) == (idle | access_error), "Misaligned word staging sets ACCERR");
  bus.write(status, access_error);
  bus.write_memory({.address = target, .size = 2});
  bus.write(control, 0);
  require(bus.read(status) == (idle | access_error),
          "An intervening configuration write aborts staging");
  bus.write(status, access_error);
  bus.write_memory({.address = target, .size = 2});
  bus.write(status, 0);
  require(bus.read(status) == (idle | access_error),
          "Writing zero CBEIF before launch aborts the sequence");
  bus.write(status, access_error);
  bus.write_memory({.address = target, .size = 2});
  bus.write(command, 0);
  require(bus.read(status) == (idle | access_error), "Invalid command encoding sets ACCERR");
  bus.write(status, access_error);
  bus.write_memory({.address = target, .size = 2});
  bus.write(command, erase);
  bus.write(command, erase);
  require(bus.read(status) == (idle | access_error), "A second FCMD write aborts the sequence");
  bus.write(status, clear_errors);
  constexpr unsigned protect_loader = 0xf8;
  bus.write(protection, protect_loader);
  bus.write_memory({.address = array_base, .size = 2});
  require(bus.read(status) == (idle | protect_error),
          "Protected array addresses set PVIOL before launch");
  bus.write(status, protect_error);
  bus.write_memory({.address = target, .size = 2});
  bus.write(command, mass);
  require(bus.read(status) == (idle | protect_error), "Any protection prevents mass erase");
  require(bus.read(target) == pattern && bus.read(array_base) == pattern,
          "Illegal sequences and protection violations never modify the array");
}

void test_sector_program_and_aliases() {
  blofeld::McuBus bus(image());
  prepare(bus);
  launch(bus, {.command = erase});
  require(bus.read(status) == 0, "Launch clears CBEIF/CCIF before the explicit buffer transfer");
  reject([&bus] { static_cast<void>(bus.read(target)); },
         "Busy flash reads cannot invent valid data");
  reject([&bus] { bus.check_execute({.address = array_base, .length = 1}); },
         "An algorithm invalidates instruction fetches throughout the single flash block");
  bus.advance_flash(blofeld::FlashMilestone::transfer_buffer);
  require(bus.read(status) == executing, "Transfer frees the buffer but leaves CCIF clear");
  bus.advance_flash(blofeld::FlashMilestone::complete_command);
  require(bus.read(status) == idle, "Only explicit completion restores CCIF");
  for (unsigned address = sector_begin; address < sector_end; ++address) {
    require(bus.read(static_cast<std::uint16_t>(address)) == erased,
            "Sector erase affects all 512 bytes");
  }
  require(bus.read(sector_begin - 1) == pattern && bus.read(sector_end) == pattern,
          "Sector alignment preserves both neighbors");
  launch(bus, {});
  finish(bus);
  constexpr unsigned alias_target = 0x4234;
  require(bus.read(target) == data_high && bus.read(target + 1) == data_low &&
              bus.read(alias_target) == data_high && bus.read(alias_target + 1) == data_low,
          "Word programming writes big-endian bytes into one physical array and its alias");
  launch(bus, {});
  reject([&bus] { bus.advance_flash(blofeld::FlashMilestone::transfer_buffer); },
         "Cumulative programming is not modeled as an ordinary bitwise AND");
  bus.reset();
  require(bus.read(target) == data_high && bus.read(target + 1) == data_low,
          "Reset drops a command not yet transferred without damaging prior committed data");
  prepare(bus);
  erase_target(bus);
  launch(bus, {.data = word_mask});
  finish(bus);
  launch(bus, {});
  reject([&bus] { bus.advance_flash(blofeld::FlashMilestone::transfer_buffer); },
         "Even an all-ones program consumes the word's known erase/program eligibility");
}

void test_pending_reset() {
  for (const bool complete_first : {false, true}) {
    blofeld::McuBus bus(image());
    prepare(bus);
    erase_target(bus);
    launch(bus, {});
    bus.advance_flash(blofeld::FlashMilestone::transfer_buffer);
    launch(bus, {.address = target + 2});
    if (complete_first) { bus.advance_flash(blofeld::FlashMilestone::complete_command); }
    bus.power_cycle();
    if (complete_first) {
      require(bus.read_memory({.address = target, .size = 2}) == word_data,
              "Power loss retains a committed first command when its successor has not started");
    } else {
      reject([&bus] { static_cast<void>(bus.read(target)); },
             "An active first program becomes unknown");
    }
    require(bus.read(target + 2) == erased && bus.read(target + 3) == erased,
            "Discarding the pending successor does not damage its untouched word");
  }
}

void test_operating_clock_bounds() {
  struct ClockCase {
    unsigned hz;
    bool valid;
  };

  constexpr std::array<ClockCase, 4> cases{{{.hz = 10799999, .valid = false},
                                            {.hz = 10800000, .valid = true},
                                            {.hz = 14400000, .valid = true},
                                            {.hz = 14400001, .valid = false}}};
  for (const auto input : cases) {
    blofeld::McuBus bus(image());
    bus.write(divider, reference_divider);
    bus.observe_flash_clock({.oscillator_hz = input.hz, .bus_hz = bus_hz});
    launch(bus, {.command = erase});
    if (input.valid) {
      finish(bus);
      require(bus.read(target) == erased,
              "Both 150 kHz and 200 kHz clock limits admit the reference transfer");
    } else {
      reject([&bus] { bus.advance_flash(blofeld::FlashMilestone::transfer_buffer); },
             "A one-Hz violation of either rational NVM clock limit cannot perform an erase");
      bus.reset();
      require(bus.read(target) == pattern, "An unstarted command preserves the original array");
    }
  }
}

void test_pipeline_verify_and_mass_erase() {
  blofeld::McuBus bus(image());
  prepare(bus);
  launch(bus, {.command = verify});
  finish(bus);
  require(bus.read(status) == idle, "Nonblank erase verification leaves BLANK clear");
  erase_target(bus);
  launch(bus, {});
  bus.advance_flash(blofeld::FlashMilestone::transfer_buffer);
  constexpr unsigned next_data = 0x5678;
  launch(bus, {.address = target + 2, .data = next_data});
  require(bus.read(status) == 0, "The second valid command occupies the launched buffer");
  bus.advance_flash(blofeld::FlashMilestone::complete_command);
  require(bus.read(status) == 0,
          "Completing an active command does not complete its pending successor");
  finish(bus);
  require(bus.read_memory({.address = target, .size = 2}) == word_data &&
              bus.read_memory({.address = target + 2, .size = 2}) == next_data,
          "The executing and buffered word/data remain independent");
  launch(bus, {.command = mass});
  finish(bus);
  for (unsigned address = array_base; address <= word_mask; ++address) {
    require(bus.read(static_cast<std::uint16_t>(address)) == erased,
            "Mass erase covers the physical block");
  }
  launch(bus, {.command = verify});
  finish(bus);
  require(bus.read(status) == (idle | 4U), "A completed full-block erase check sets BLANK");
  constexpr unsigned security_register = 0x101;
  require(bus.read(security_register) == unsecured_options,
          "An erased security field is not latched before reset");
  bus.reset();
  require(bus.read(security_register) == erased,
          "Reset samples the new nonvolatile security field");
}

struct ResetWindow {
  unsigned opcode = 0;
  unsigned phase = 0;
  bool power_loss = false;
};

void reset_device(blofeld::McuBus &bus, const bool power_loss) {
  if (power_loss) {
    bus.power_cycle();
  } else {
    bus.reset();
  }
}

void verify_reset_target(blofeld::McuBus &bus, const ResetWindow window) {
  const bool interrupted_write = window.phase == 3 && window.opcode != verify;
  if (!interrupted_write) {
    const unsigned expected = window.phase == 4 && window.opcode == program ? data_high : erased;
    require(bus.read(target) == expected,
            "Pre-transfer reset preserves bytes; completion commits the result");
    return;
  }
  reject([&bus] { static_cast<void>(bus.read(target)); },
         "An interrupted target has unknown contents");
  reject([&bus] { bus.check_execute({.address = target, .length = 1}); },
         "Unknown NVM cannot become executable after reset");
  if (window.opcode == program) {
    require(bus.read(target - 1) == erased && bus.read(target + 2) == erased,
            "An aborted word program invalidates only that word");
  }
}

void test_reset_window(const ResetWindow window) {
  blofeld::McuBus bus(image());
  prepare(bus);
  erase_target(bus);
  constexpr unsigned ram_byte = 0x1200;
  bus.write(ram_byte, pattern);
  bus.write_memory({.address = target, .size = 2, .value = word_data});
  if (window.phase >= 1) { bus.write(command, static_cast<std::uint8_t>(window.opcode)); }
  if (window.phase >= 2) { bus.write(status, executing); }
  if (window.phase >= 3) { bus.advance_flash(blofeld::FlashMilestone::transfer_buffer); }
  if (window.phase >= 4) { bus.advance_flash(blofeld::FlashMilestone::complete_command); }
  if (window.phase == 3 && window.opcode == mass) {
    reject([&bus, window] { reset_device(bus, window.power_loss); },
           "Interrupted mass erase leaves reset option fields unknown");
    reject([&bus] { static_cast<void>(bus.read(array_base)); },
           "Mass erase can invalidate the loader");
  } else {
    reset_device(bus, window.power_loss);
    require(bus.read(array_base) == (window.phase == 4 && window.opcode == mass ? erased : pattern),
            "Non-mass operations preserve the loader outside their target range");
  }
  verify_reset_target(bus, window);
  if (window.power_loss) {
    reject([&bus] { static_cast<void>(bus.read(ram_byte)); },
           "Power loss makes RAM contents unknown");
  } else {
    require(bus.read(ram_byte) == pattern,
            "Reset retains RAM while preserving interrupted-NVM unknownness");
  }
}

void test_reset_windows() {
  for (const auto opcode : {program, erase, mass, verify}) {
    for (unsigned phase = 0; phase < 5; ++phase) {
      for (const bool power_loss : {false, true}) {
        test_reset_window({.opcode = opcode, .phase = phase, .power_loss = power_loss});
      }
    }
  }
}

void test_clock_and_unknown_vector() {
  blofeld::McuBus bus(image());
  bus.write(divider, reference_divider);
  launch(bus, {.command = erase});
  reject([&bus] { bus.advance_flash(blofeld::FlashMilestone::transfer_buffer); },
         "No operating clock is invented from a divider setting");
  bus.reset();
  prepare(bus);
  constexpr unsigned vector_sector = 0xff00;
  launch(bus, {.address = vector_sector, .command = erase});
  finish(bus);
  constexpr unsigned reset_vector = 0xfffe;
  launch(bus, {.address = reset_vector});
  bus.advance_flash(blofeld::FlashMilestone::transfer_buffer);
  bus.reset();
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  reject([&cpu] { cpu.reset(); },
         "An interrupted reset-vector program cannot silently choose a boot address");
}

void test_native_interrupt_and_cache() {
  auto bytes = image();
  constexpr unsigned nop = 0xa7;
  constexpr unsigned clr_extended = 0x79;
  constexpr unsigned rti = 0x0b;
  constexpr unsigned load_a = 0x86;
  bytes[0] = nop;
  bytes[irq_offset] = handler >> 8U;
  bytes[irq_offset + 1] = 0;
  bytes[handler - array_base] = clr_extended;
  bytes[handler - array_base + 1] = 1;
  bytes[handler - array_base + 2] = 3;
  bytes[handler - array_base + 3] = rti;
  bytes[sector_begin - array_base] = load_a;
  bytes[sector_begin - array_base + 1] = 1;
  blofeld::McuBus bus(bytes);
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state({.pc = array_base, .sp = stack, .ccr = x_mask});
  bus.write(control, executing);
  require(cpu.step().interrupt_vector == irq_vector,
          "Flash CBEIE/CBEIF requests the documented FFB8 vector");
  cpu.step();
  cpu.step();
  require(cpu.state().pc == array_base && cpu.state().sp == stack && !cpu.pending_interrupt(),
          "Native handler clears FCNFG and returns through its real HC12 interrupt frame");
  cpu.set_state({.pc = sector_begin, .sp = stack, .ccr = masked});
  cpu.step();
  require(cpu.state().d == 0x0100, "Original synthetic immediate is cached before NVM mutation");
  prepare(bus);
  erase_target(bus);
  constexpr unsigned new_instruction = 0x8602;
  launch(bus, {.address = sector_begin, .data = new_instruction});
  finish(bus);
  cpu.set_state({.pc = sector_begin, .sp = stack, .ccr = masked});
  cpu.step();
  require(cpu.state().d == 0x0200,
          "Completed programming invalidates a previously cached flash operand");
}
}

int main() {
  try {
    test_sequence_errors();
    test_sector_program_and_aliases();
    test_pipeline_verify_and_mass_erase();
    test_pending_reset();
    test_operating_clock_bounds();
    test_reset_windows();
    test_clock_and_unknown_vector();
    test_native_interrupt_and_cache();
    std::cout
        << "FTS32K reference command/array/queue/reset/IRQ/cache checks passed; explicit "
        << "milestones are not calibrated timing, analog pulse phases or complete recovery.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
