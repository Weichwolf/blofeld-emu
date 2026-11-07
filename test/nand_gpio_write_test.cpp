// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_core_fixture.hpp"
#include "dsp_rom_program.hpp"
#include "nand_gpio_fixture.hpp"
#include "nand_gpio_pins.hpp"
#include "nand_native_program.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"

#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld::test {
namespace {
using namespace nand_native;
constexpr unsigned failure_exit = 0xff4000;
constexpr unsigned status_command = 0x70;
constexpr unsigned program_command = 0x80;
constexpr unsigned program_confirm = 0x10;
constexpr unsigned erase_command = 0x60;
constexpr unsigned erase_confirm = 0xd0;
constexpr unsigned instruction_limit = 10000;
constexpr unsigned neighbor_cookie = 0x42;

void outputs(core::Program &program) {
  movep(program, nand_other_f, port_f_control, 'y');
  movep(program, output_f, port_f_direction, 'y');
}

void inputs(core::Program &program) {
  movep(program, idle_c, port_c_data, 'x');
  movep(program, input_f, port_f_direction, 'y');
  movep(program, all_inputs_f, port_f_control, 'y');
}

struct Wait {
  unsigned poll = 0;
  unsigned result = 0;
};

Wait status_wait(core::Program &program) {
  latch(program, {.value = status_command, .mode = nand_cle});
  inputs(program);
  program.bit({.operand = {.space = DspSpace::x, .address = port_c_data},
               .index = 8,
               .action = core::BitAction::clear});
  const auto poll = program.pc();
  // FM BRCLR Y:pp: prefix$0cc000,pp=$35,S=Y,bit6,relative displacement0.
  constexpr unsigned status_poll = 0x0cf546;
  program.emit(status_poll, 0);
  program.bit({.operand = {.space = DspSpace::y, .address = port_f_data},
               .index = 0,
               .action = core::BitAction::test});
  const auto result = program.pc();
  // FM13-80, tables12-13/12-18: absolute EA110000 and carry-set CCCC1000.
  constexpr unsigned absolute_carry_jump = 0x0af0a8;
  program.emit(absolute_carry_jump, failure_exit);
  program.bit({.operand = {.space = DspSpace::x, .address = port_c_data},
               .index = 8,
               .action = core::BitAction::set});
  outputs(program);
  return {.poll = poll, .result = result};
}

unsigned payload(const unsigned index) {
  constexpr unsigned multiplier = 29;
  return ((index * multiplier) + (index >> 8U)) & byte_mask;
}

struct Layout {
  Wait erase;
  Wait program;
  unsigned read_poll = 0;
};

Layout own_write_program(core::Program &program) {
  setup_gpio(program);
  program.immediate(DspRomRegister::r0, 0);
  program.immediate(DspRomRegister::m0, word_mask);
  latch(program, {.value = erase_command, .mode = nand_cle});
  for (unsigned index = 0; index < 3; ++index) { latch(program, {.value = 0, .mode = nand_ale}); }
  latch(program, {.value = erase_confirm, .mode = nand_cle});
  const auto erased = status_wait(program);
  latch(program, {.value = 0, .mode = nand_cle});
  latch(program, {.value = program_command, .mode = nand_cle});
  for (unsigned index = 0; index < 4; ++index) { latch(program, {.value = 0, .mode = nand_ale}); }
  for (unsigned index = 0; index < Nand::page_bytes; ++index) {
    latch(program, {.value = payload(index), .mode = 0});
  }
  latch(program, {.value = program_confirm, .mode = nand_cle});
  const auto programmed = status_wait(program);
  latch(program, {.value = 0, .mode = nand_cle});
  for (unsigned index = 0; index < 4; ++index) { latch(program, {.value = 0, .mode = nand_ale}); }
  inputs(program);
  const auto read_poll = program.pc();
  poll_ready(program);
  for (unsigned index = 0; index < Nand::page_bytes; ++index) { read_byte(program); }
  return {.erase = erased, .program = programmed, .read_poll = read_poll};
}

struct Execution {
  std::string reason;
  unsigned checks = 0;
};

Execution execute(DspMachine &cpu,
                  NandLink &link,
                  const Layout &layout,
                  const std::vector<NandOutcome> &outcomes,
                  const unsigned end,
                  std::ostream &trace) {
  Execution result;
  for (unsigned count = 0;
       cpu.cpu().registers().pc != end && cpu.cpu().registers().pc != failure_exit;
       ++count) {
    require(count < instruction_limit,
            "Own native NAND write/readback exceeded its instruction bound");
    const auto before = cpu.cpu().registers();
    trace << "pc=" << hex(before.pc, 6) << " sr=" << hex(before.sr, 6)
          << " r0=" << hex(before.r[0], 6) << '\n';
    if (before.pc == layout.erase.result || before.pc == layout.program.result) {
      const auto index = before.pc == layout.erase.result ? 0U : 1U;
      require(index < outcomes.size() &&
                  ((before.sr & carry) != 0) == (outcomes[index] == NandOutcome::fail),
              "Native BTST must consume the actual observed program/erase failure bit");
      ++result.checks;
    }
    try {
      dma_test::step_native(cpu);
      link.after_step({.pc = before.pc, .omr = before.omr});
    } catch (const Fault &fault) {
      result.reason = fault.what();
      trace << "fault=" << result.reason << '\n';
      break;
    }
  }
  return result;
}

void test_write(const std::vector<NandOutcome> &outcomes, std::ostream &trace) {
  trace << "outcomes=";
  for (const auto outcome : outcomes) { trace << (outcome == NandOutcome::pass ? 'p' : 'f'); }
  trace << '\n';
  core::Program program;
  const auto layout = own_write_program(program);
  DspMachine cpu(dma_test::native_clocks);
  auto &bus = cpu.bus();
  program.install(bus);
  nand_native::reset(cpu);
  Nand nand;
  nand.observe_power_ready();
  nand.observe_wp(true); // Explicit held-high lab observation, no board route is inferred.
  nand.observe_block(0, {.valid = true, .program_protected = false, .erase_protected = false});
  nand.observe_cell({.page = Nand::block_pages}, {.value = neighbor_cookie, .known = byte_mask});
  NandLink link(bus,
                nand,
                trace,
                {.poll_pcs = {layout.erase.poll, layout.program.poll, layout.read_poll},
                 .selected = true,
                 .outcomes = outcomes});
  const auto execution = execute(cpu, link, layout, outcomes, program.pc(), trace);
  trace << "final-pc=" << hex(cpu.cpu().registers().pc, 6) << " writes=" << link.writes()
        << " reads=" << link.reads() << " completed-writes=" << link.completed_writes() << '\n';
  const auto &reason = execution.reason;
  const auto checks = execution.checks;
  const bool missing =
      outcomes.empty() || (outcomes.size() == 1 && outcomes[0] == NandOutcome::pass);
  if (missing) {
    require(reason.contains("Unobserved NAND program/erase outcome") &&
                reason.contains("dsp-pc=") && link.completed_writes() == outcomes.size() &&
                nand.pending() == (outcomes.empty() ? NandPending::erase : NandPending::program) &&
                nand.cell({}).known == 0 &&
                link.writes() == (outcomes.empty() ? 0U : Nand::page_bytes),
            "Native status polling must not invent a successful program/erase result: " + reason);
    return;
  }
  require(reason.empty(), "Unexpected native NAND write/readback error: " + reason);
  const bool erase_failed = outcomes[0] == NandOutcome::fail;
  const bool failed = erase_failed || outcomes.back() == NandOutcome::fail;
  require(cpu.cpu().registers().pc == (failed ? failure_exit : program.pc()) &&
              checks == (erase_failed ? 1U : 2U) &&
              link.completed_writes() == (erase_failed ? 1U : 2U),
          "Native JCS must take the failure path directly from the NAND status bit");
  require(nand.cell({.page = Nand::block_pages}) ==
              NandByte{.value = neighbor_cookie, .known = byte_mask},
          "Native erase/program must preserve the neighboring block");
  if (failed) {
    require(cpu.cpu().registers().r[0] == 0 && nand.cell({}).known == 0,
            "Failed native command must leave its target unknown and skip all readback stores");
    return;
  }
  require(cpu.cpu().registers().r[0] == Nand::page_bytes && link.writes() == Nand::page_bytes &&
              link.reads() == Nand::page_bytes + 2 && link.commands() == 8 &&
              link.addresses() == 11 && link.polls() == 9,
          "Native erase/program/readback must emit every data byte and poll all three pending "
          "operations");
  for (unsigned index = 0; index < Nand::page_bytes; ++index) {
    require(
        nand.cell({.column = index}) ==
                NandByte{.value = static_cast<std::uint8_t>(payload(index)), .known = byte_mask} &&
            bus.read({.space = DspSpace::x, .address = index}) == payload(index),
        "Native GPIO program and MOVEP/store readback must preserve every main/spare byte");
  }
}
}
}

int main(const int count, char **arguments) {
  try {
    std::ostringstream transient;
    std::ofstream file;
    std::ostream *trace = &transient;
    if (count == 2) {
      file.open(arguments[1]);
      blofeld::test::require(file.is_open(), "Cannot open native NAND write trace");
      file.exceptions(std::ios::failbit | std::ios::badbit);
      trace = &file;
    } else if (count != 1) {
      throw blofeld::Fault("Usage: nand-gpio-write-test [private.trace]");
    }
    using blofeld::NandOutcome;
    blofeld::test::test_write({NandOutcome::pass, NandOutcome::pass}, *trace);
    blofeld::test::test_write({NandOutcome::pass, NandOutcome::fail}, *trace);
    blofeld::test::test_write({NandOutcome::fail}, *trace);
    blofeld::test::test_write({NandOutcome::pass}, *trace);
    blofeld::test::test_write({}, *trace);
    std::cout << "Native NAND erase/program/status/readback and failure branches passed; missing "
                 "outcomes reject.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
