// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "esai_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_operation_clock.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_timing.hpp"
#include "storage/nand_write_medium.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
constexpr unsigned hz = 1000000000, short_pulse = 15, reset_ns = 5000;
constexpr unsigned read_ns = 15000, program_ns = 500000, erase_ns = 3000000;
constexpr std::uint8_t full = 0xff, reset = 0xff, read_command = 0;
constexpr std::uint8_t program = 0x80, confirm_program = 0x10, erase = 0x60, confirm_erase = 0xd0;
constexpr std::uint8_t inhibited_bit = 2, inhibited_value = 0xa7;
constexpr std::array<std::uint8_t, 2> pattern{0xa5, 0x3c};
constexpr auto functional = NandTiming::functional_protocol;

Nand device(const bool known, const bool history) {
  NandImage image;
  NandArray::Page page;
  page.cells.fill({.value = full, .known = full});
  if (history) { page.programs = NandProgramCount{}; }
  if (known) {
    for (unsigned row = 0; row < Nand::block_pages; ++row) { image.pages.emplace(row, page); }
  }
  image.blocks.emplace(
      0, NandBlock{.valid = true, .program_protected = false, .erase_protected = false});
  Nand result(image, NandReadContinuation::k9f1208_j);
  result.observe_wp(true);
  return result;
}

NandCellProfile cells(const bool inhibit) {
  NandCellProfile result{.contract = NandCellContract::reference_pulse_inhibit, .cells = {}};
  if (inhibit) { result.cells.emplace(0, NandCellInhibit{.program = inhibited_bit}); }
  return result;
}

struct Wire {
  explicit Wire(const bool inhibit = false, const bool known = true, const bool history = true)
      : medium(device(known, history), hz, cells(inhibit)) {
    medium.observe_power_ready();
    observe();
  }

  void observe() {
    if (!pins.re_n) { throw Fault("Authored RE observation must be known"); }
    const bool re_n = pins.re_n.value();
    static_cast<void>(medium.advance(ticks));
    const auto input = write.observe(ticks, pins);
    if (input) {
      switch (input->kind) {
        case NandWriteKind::command: medium.command(input->value); break;
        case NandWriteKind::address: medium.address(input->value); break;
        case NandWriteKind::data: medium.write(input->value); break;
        default: throw Fault("Unknown authored input latch kind");
      }
    }
    const bool admitted = setup.observe(ticks,
                                        {.ce_n = pins.ce_n,
                                         .re_n = pins.re_n,
                                         .we_n = pins.we_n,
                                         .ale = pins.ale,
                                         .cle = pins.cle,
                                         .ready = medium.device().ready()},
                                        NandReadKind::array);
    output.observe(ticks, {.ce_n = pins.ce_n, .re_n = pins.re_n}, [&] {
      require(admitted, "A protocol source requires a genuine admitted falling RE edge");
      ++reads;
      return NandByte{.value = medium.read(), .known = full};
    });
    if (!previous_re && re_n) { medium.observe_read_end(NandReadEnd::re_high); }
    previous_re = re_n;
  }

  void latch(const NandWriteKind kind, const std::uint8_t value) {
    ++ticks;
    pins.cle = kind == NandWriteKind::command;
    pins.ale = kind == NandWriteKind::address;
    pins.data = {.value = value, .known = full};
    observe();
    ++ticks;
    pins.we_n = false;
    observe();
    ticks += short_pulse;
    pins.we_n = true;
    observe();
  }

  void command(const std::uint8_t value) { latch(NandWriteKind::command, value); }

  void address(const unsigned count) {
    for (unsigned index = 0; index < count; ++index) { latch(NandWriteKind::address, 0); }
  }

  void complete(const NandPending operation, const unsigned duration) {
    const auto event = medium.next_event();
    if (!event) { throw Fault("Expected a real pending NAND operation"); }
    require(event->operation == operation && event->ticks == ticks + duration &&
                medium.device().ready() == false,
            "Real commands schedule the independent operation duration and keep Ready low");
    ticks = event->ticks - 1;
    observe();
    require(medium.device().ready() == false && medium.device().pending() == operation,
            "Functional pad timing cannot complete an operation one tick early");
    if (operation == NandPending::program || operation == NandPending::erase) {
      require(medium.device().cell({.page = 0, .column = 0}).known == 0,
              "Pending write work cannot manufacture completed cell knowledge before the deadline");
    }
    ++ticks;
    observe();
    require(medium.device().ready() == true && medium.device().pending() == NandPending::none,
            "Only the genuine scheduled operation releases Busy");
  }

  std::uint8_t read() {
    ++ticks;
    pins.cle = false;
    pins.ale = false;
    observe();
    ++ticks;
    pins.re_n = false;
    observe();
    const auto data = output.output(ticks);
    require(data.driven == true && data.data.known == full && !output.next_transition(),
            "Logical output is the actual consumed NAND byte without a calibrated pad delay");
    const auto before = reads;
    require(output.output(ticks) == data && reads == before,
            "Read-only output queries cannot consume another NAND byte");
    ++ticks;
    pins.re_n = true;
    observe();
    require(output.output(ticks).driven == false,
            "Logical release stops driving at the observed rising RE edge");
    return data.data.value;
  }

  NandWriteMedium medium;
  NandWriteTiming write{hz, functional};
  NandReadSetup setup{hz, functional};
  NandReadTiming output{hz, functional};
  NandWritePins pins{.ce_n = false,
                     .re_n = true,
                     .we_n = true,
                     .cle = false,
                     .ale = false,
                     .data = {.value = full, .known = full}};
  std::uint64_t ticks = 0;
  unsigned reads = 0;
  bool previous_re = true;
};

void writes_and_restore(const bool inhibit) {
  Wire wire(inhibit);
  wire.command(reset);
  wire.complete(NandPending::reset, reset_ns);
  wire.command(read_command);
  wire.address(4);
  wire.complete(NandPending::read, read_ns);
  require(wire.read() == full, "The initial known erased byte comes from the actual array");
  wire.command(program);
  wire.address(4);
  for (const auto byte : pattern) { wire.latch(NandWriteKind::data, byte); }
  wire.command(confirm_program);
  require(
      wire.medium.device().cell({.page = 0, .column = 0}).known == 0 &&
          wire.medium.device().ready() == false,
      "A pending physical program withdraws cell knowledge instead of publishing a result early");
  const auto saved = wire;
  for (unsigned replay = 0; replay < 2; ++replay) {
    wire = saved;
    wire.complete(NandPending::program, program_ns);
    const auto expected = inhibit ? inhibited_value : pattern[0];
    require(wire.medium.device().cell({.page = 0, .column = 0}) ==
                    NandByte{.value = expected, .known = full} &&
                wire.medium.device().cell({.page = 0, .column = 1}) ==
                    NandByte{.value = pattern[1], .known = full} &&
                (wire.medium.device().status().value & 1U) == static_cast<unsigned>(inhibit),
            "Real cell physics computes programmed bits and failure status, including inhibit");
    wire.command(read_command);
    wire.address(4);
    wire.complete(NandPending::read, read_ns);
    require(wire.read() == expected && wire.read() == pattern[1],
            "Actual read edges consume independently expected programmed values in order");
    wire.command(erase);
    wire.address(3);
    if (inhibit) {
      require(
          rejects([&] { wire.command(confirm_erase); }).contains("unknown/invalid block condition"),
          "Failed program verification cannot requalify a block for a later erase");
      continue;
    }
    wire.command(confirm_erase);
    wire.complete(NandPending::erase, erase_ns);
    for (unsigned row = 0; row < Nand::block_pages; ++row) {
      for (unsigned column = 0; column < Nand::page_bytes; ++column) {
        require(wire.medium.device().cell({.page = row, .column = column}) ==
                    NandByte{.value = full, .known = full},
                "Genuine erase completion restores every independently expected cell bit");
      }
    }
  }
}

void strict_control() {
  constexpr unsigned first_fall = 100;
  NandWritePins pins{.ce_n = false,
                     .re_n = true,
                     .we_n = true,
                     .cle = true,
                     .ale = false,
                     .data = {.value = reset, .known = full}};
  NandWriteTiming strict(hz);
  NandWriteTiming logical(hz, functional);
  for (auto *model : {&strict, &logical}) {
    require(!model->observe(0, pins), "An initial high strobe cannot invent a latch");
    pins.we_n = false;
    require(!model->observe(first_fall, pins), "Falling WE does not perform a NAND command");
    pins.we_n = true;
  }
  const auto saved = strict;
  require(rejects([&] {
            static_cast<void>(strict.observe(first_fall + short_pulse, pins));
          }).contains("tWP") &&
              strict == saved,
          "The default candidate retains its exact short-pulse refusal and previous state");
  require(logical.observe(first_fall + short_pulse, pins) ==
              NandWriteLatch{.kind = NandWriteKind::command, .value = reset},
          "Only the explicitly selected logical policy admits the same known FF command edge");
}

void refusals() {
  constexpr auto invalid = NandTiming::unqualified;
  require(!rejects([&] { const NandWriteTiming model(hz, invalid); }).empty() &&
              !rejects([&] { const NandReadSetup model(hz, invalid); }).empty() &&
              !rejects([&] { const NandReadTiming model(hz, invalid); }).empty(),
          "Unknown timing policies fail at construction");
  Wire busy;
  busy.command(reset);
  require(rejects([&] { static_cast<void>(busy.read()); }).contains("Ready high") &&
              busy.reads == 0,
          "Logical timing cannot supply array data while the actual reset is Busy");
  Wire unknown(false, false);
  unknown.command(read_command);
  unknown.address(4);
  unknown.complete(NandPending::read, read_ns);
  require(!rejects([&] { static_cast<void>(unknown.read()); }).empty(),
          "Unknown array cells cannot become successful logical read values");
  Wire history(false, true, false);
  history.command(read_command);
  history.address(4);
  history.complete(NandPending::read, read_ns);
  history.command(program);
  history.address(4);
  history.latch(NandWriteKind::data, pattern[0]);
  require(rejects([&] { history.command(confirm_program); }).contains("unknown program history"),
          "Known FF cells alone cannot fabricate erase history or a successful program");
  Wire unsupported;
  constexpr std::uint8_t invalid_command = 0xa5;
  require(!rejects([&] { unsupported.command(invalid_command); }).empty(),
          "An unsupported real command cannot become a successful no-op");
  NandReadTiming output(hz, functional);
  output.observe(0, {.ce_n = false, .re_n = true});
  const auto saved = output;
  require(!rejects([&] { output.observe(1, {.ce_n = std::nullopt, .re_n = false}); }).empty() &&
              output == saved,
          "Unknown selection still fails without changing logical output state");
}
}

int main() {
  try {
    strict_control();
    writes_and_restore(false);
    writes_and_restore(true);
    refusals();
    std::cout << "Functional NAND edges retain real read/reset/program/erase/Busy/cell failures, "
                 "pending copies and strict candidate refusals\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
