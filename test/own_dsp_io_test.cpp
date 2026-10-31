// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp56300/core.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void check_gpio(blofeld::OwnDspIo &io, blofeld::DspBus &bus) {
  using namespace blofeld;
  using namespace dsp56300;
  constexpr Word data_register = 0xffffbd;
  constexpr Word direction_register = 0xffffbe;
  constexpr Word invalid_bit = 24;
  require(io.write(Space::x, direction_register, 0, 1) && io.write(Space::x, data_register, 0, 1),
          "Could not initialize actual GPIO output");
  bool previous = false;
  require(io.bit(Space::x, data_register, 0, 0, BitAction::clear, previous) && previous,
          "GPIO bit clear did not retain the previous output bit");
  const auto output = bus.gpio(DspPort::c).outputs();
  require(output.mask == 1 && (output.known & 1U) != 0 && (output.levels & 1U) == 0,
          "GPIO bit clear did not change the physical output model");
  require(!io.bit(Space::x, data_register, 0, invalid_bit, BitAction::test, previous),
          "Invalid bit index was accepted");
  const auto failure = io.failure();
  if (!failure) { throw std::runtime_error("Invalid bit index did not latch a fault"); }
  require(failure->operation == OwnDspIo::Operation::bit && failure->address == data_register &&
              failure->value == invalid_bit && failure->cause,
          "Invalid bit index lost its context or board exception");
  io.clear_failure();
}

void run() {
  using namespace blofeld;
  using namespace dsp56300;
  DspBus bus;
  OwnDspIo io(bus);
  constexpr Word stored_word = 0x123456;
  constexpr Word write_pc = 0x42;
  constexpr Word read_pc = 0x43;
  constexpr Word bit_test_pc = 0x44;
  constexpr Word bit_write_pc = 0x45;
  constexpr Word latched_write_pc = 0x46;
  constexpr Word unmapped_read_pc = 0x47;
  require(io.write(Space::x, 0x10, write_pc, stored_word), "Known X RAM write failed");
  Word value = 0;
  require(io.read(Space::x, 0x10, read_pc, value) && value == stored_word,
          "Known X RAM read returned the wrong data");
  bool previous = false;
  require(io.bit(Space::x, 0x10, bit_test_pc, 1, BitAction::test, previous) && previous,
          "A real memory bit test did not read the stored value");
  require(!io.bit(Space::x, 0x10, bit_write_pc, 2, BitAction::set, previous),
          "Unimplemented atomic memory RMW was accepted");
  const auto bit_failure = io.failure();
  if (!bit_failure) { throw std::runtime_error("Unsupported bit access did not latch a fault"); }
  require(bit_failure->operation == OwnDspIo::Operation::bit && bit_failure->address == 0x10 &&
              bit_failure->pc == bit_write_pc && bit_failure->value == 2 &&
              bit_failure->action == BitAction::set && !bit_failure->cause,
          "Unsupported bit access lost its context");
  require(!io.write(Space::x, 0x10, latched_write_pc, 0), "Faulted adapter accepted a later write");
  io.clear_failure();
  require(bus.read({.space = DspSpace::x, .address = 0x10}) == stored_word,
          "Rejected operations changed actual RAM");
  require(!io.read(Space::y, 0x2000, unmapped_read_pc, value), "Unmapped Y memory read succeeded");
  const auto read_failure = io.failure();
  if (!read_failure) { throw std::runtime_error("Unmapped read did not latch a fault"); }
  require(read_failure->operation == OwnDspIo::Operation::read && read_failure->space == Space::y &&
              read_failure->address == 0x2000 && read_failure->pc == unmapped_read_pc &&
              read_failure->cause,
          "Unmapped read lost the board exception or execution context");
  try {
    std::rethrow_exception(read_failure->cause);
  } catch (const blofeld::Fault &) { io.clear_failure(); }
  require(!io.failure(), "The board exception was not retained as a Fault");
  require(io.write(Space::program, 0, 0, 0), "Could not install own NOP");
  Core cpu(io, {.backend = Backend::reference});
  auto step = cpu.begin_fetch();
  for (unsigned effects = 0; effects < 16 && step.pending && !step.retired; ++effects) {
    step = cpu.advance_instruction(step.pending->ticket);
  }
  require(step.retired && step.fault.kind == FaultKind::none && cpu.registers().pc == 1,
          "Own DSP core did not execute through the real board bus");
  check_gpio(io, bus);
  require(io.reset_peripherals(1), "Board peripheral reset failed");
  require(bus.gpio(DspPort::c).outputs().mask == 0, "Peripheral reset retained GPIO outputs");
}
}

int main() {
  try {
    run();
    std::cout << "Independent DSP core and strict board bus adapter checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
