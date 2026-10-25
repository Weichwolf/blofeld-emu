// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "nand_gpio_fixture.hpp"
#include "nand_program_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_store.hpp"

#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>

namespace blofeld::test {
namespace {
constexpr unsigned helper_base = 0x2e00;
constexpr unsigned helper_alias = 0x10000 + helper_base - 0x1000;
constexpr unsigned switched_omr = 0x600389;
constexpr unsigned buffer = 0xc000;
constexpr unsigned row = 37;
constexpr unsigned block = row / Nand::block_pages;
constexpr unsigned neighbor_page = (block + 1) * Nand::block_pages;
constexpr unsigned neighbor_cookie = 0x42;
constexpr unsigned old_cookie = 0x91;
constexpr unsigned source_words = Nand::page_bytes / 3;
constexpr unsigned runtime_offset = 33411;
constexpr unsigned instruction_limit = 20000;
constexpr unsigned pin_mask = 0x7ff;
constexpr unsigned status_mask = 0xc1;
constexpr unsigned failure_bit = 1;
constexpr unsigned absolute_jsr = 0x0bf080;
constexpr unsigned erase_command = 0x60;
constexpr unsigned erase_confirm = 0xd0;

struct Helper {
  unsigned index;
  unsigned words;
  unsigned erase;
  unsigned program;
  unsigned poll;
  unsigned read;
  unsigned data;
};

constexpr std::array<Helper, 2> helpers{{{.index = 3,
                                          .words = 388,
                                          .erase = 0x2f67,
                                          .program = 0x2f1b,
                                          .poll = 0x2f38,
                                          .read = 0x2ecb,
                                          .data = 0x2ed9},
                                         {.index = 5,
                                          .words = 400,
                                          .erase = 0x2f73,
                                          .program = 0x2f27,
                                          .poll = 0x2f44,
                                          .read = 0x2ed7,
                                          .data = 0x2ee5}}};

using Source = std::array<unsigned, source_words>;

Source source_words_from(const Firmware &firmware, const bool original) {
  Source result{};
  if (!original) {
    for (unsigned index = 0; index < source_words; ++index) {
      constexpr unsigned multiplier = 29;
      constexpr unsigned salt = 0x123;
      result.at(index) = ((index * multiplier) + salt) & pin_mask;
    }
    return result;
  }
  const auto bytes = std::span(firmware.update_blocks.at(4)).subspan(runtime_offset);
  require(bytes.size() >= Nand::page_bytes && !firmware.dsp.empty(),
          "Original runtime prefix must supply one full source page");
  for (unsigned index = 0; index < source_words; ++index) {
    result.at(index) = dsp_boot_word(
        std::span<const std::uint8_t, 3>(bytes.subspan(static_cast<std::size_t>(index) * 3, 3)));
  }
  const auto &first = firmware.dsp.front();
  require(result.at(0) == first.space && result.at(1) == first.words.size() &&
              result.at(2) == first.address,
          "Updater source prefix must begin with the independently parsed original DSP record");
  return result;
}

void install_inputs(DspBus &bus,
                    const Firmware &firmware,
                    const Helper helper,
                    const Source &source) {
  const auto bytes = std::span(firmware.update_blocks.at(helper.index));
  require(bytes.size() == static_cast<std::size_t>(helper.words) * 3,
          "Original helper must retain its pinned word count");
  // Initial fixture data in documented X RAM. Native OMR/NOP instructions later
  // expose the same physical words as P:2e00; no P-memory alias is invented.
  for (unsigned index = 0; index < helper.words; ++index) {
    bus.write({.space = DspSpace::x,
               .address = helper_alias + index,
               .value = dsp_boot_word(std::span<const std::uint8_t, 3>(
                   bytes.subspan(static_cast<std::size_t>(index) * 3, 3)))});
  }
  for (unsigned index = 0; index < source_words; ++index) {
    bus.write({.space = DspSpace::x, .address = buffer + index, .value = source.at(index)});
  }
}

unsigned own_caller(DspProgram &program, const unsigned entry) {
  setup_gpio(program);
  program.immediate("m0", word_mask);
  program.immediate("r0", buffer);
  program.immediate("a", row);
  program.immediate("omr", switched_omr);
  for (unsigned index = 0; index < 3; ++index) { program.emit("nop"); }
  const auto after_nops = program.pc();
  program.word(absolute_jsr);
  program.word(entry);
  return after_nops;
}

void prepare_nand(Nand &nand, const bool program) {
  nand.observe_power_ready();
  nand.observe_wp(true); // Explicit held-high laboratory observation only.
  nand.observe_block(block, {.valid = true, .program_protected = false, .erase_protected = false});
  nand.observe_cell({.page = neighbor_page}, {.value = neighbor_cookie, .known = byte_mask});
  if (program) {
    // Explicit completed erase is the isolated program routine's precondition.
    nand.command(erase_command);
    nand.address(row);
    nand.address(0);
    nand.address(0);
    nand.command(erase_confirm);
    nand.observe_completion(NandOutcome::pass);
    return;
  }
  std::array<std::uint8_t, Nand::page_bytes> old{};
  old.fill(old_cookie);
  for (unsigned page = block * Nand::block_pages; page < (block + 1) * Nand::block_pages; ++page) {
    nand.observe_bytes(page, 0, old);
  }
}

enum class Stop : std::uint8_t { none, loaded, active };

struct Result {
  std::string reason;
  unsigned pc = 0;
  unsigned source_end = 0;
};

Result execute(DspBus &bus,
               DspCore &cpu,
               NandLink &link,
               const unsigned after_nops,
               const Stop stop,
               std::ostream &trace) {
  for (unsigned count = 0; count < instruction_limit; ++count) {
    const auto before = cpu.state();
    trace << "pc=" << hex(before.pc, 6) << " sr=" << hex(before.sr, 6)
          << " omr=" << hex(before.omr, 6) << " r0=" << hex(before.r[0], 6)
          << " lc=" << hex(before.lc, 6) << '\n';
    if (before.pc == after_nops) {
      // Three executed own NOPs provide explicit switch-cycle observations.
      // No general instruction timing or calibrated NAND delay is inferred.
      bus.advance_instruction_cycles(3);
      trace << "observed-switch-cycles=3\n";
    }
    try {
      cpu.step();
      link.after_step(before);
      if ((stop == Stop::loaded && link.writes() == Nand::page_bytes) ||
          (stop == Stop::active && link.commands() == 2)) {
        trace << "cut-boundary=" << static_cast<unsigned>(stop) << '\n';
        return {.reason = {}, .pc = before.pc, .source_end = cpu.state().r[0]};
      }
    } catch (const Fault &fault) {
      trace << "expected-frontier=" << fault.what() << '\n';
      return {.reason = fault.what(), .pc = before.pc, .source_end = cpu.state().r[0]};
    }
  }
  throw Fault("Original NAND helper exceeded its native instruction bound");
}

struct Input {
  Helper helper;
  bool program = false;
  bool original_source = false;
  NandOutcome outcome = NandOutcome::pass;
  Stop stop = Stop::none;
  bool power = false;
};

void check_completed(const Nand &nand,
                     const NandLink &link,
                     const Result &result,
                     const Input input,
                     const Source &source) {
  require(result.reason.contains("read turnaround") && result.reason.contains("dsp-pc=") &&
              result.pc == input.helper.read && link.completed_writes() == 1 &&
              link.commands() == 3 && link.addresses() == (input.program ? 4U : 3U) &&
              link.writes() == (input.program ? Nand::page_bytes : 0U) && link.reads() == 0 &&
              link.polls() == 3 && nand.ready() == true && nand.status_mode() &&
              result.source_end == buffer + (input.program ? source_words : 0U),
          "Original routine must complete its exact write cycles before its status-turnaround "
          "frontier: " +
              result.reason);
  const auto status = nand.status();
  require(status.known == status_mask &&
              ((status.value & failure_bit) != 0) == (input.outcome == NandOutcome::fail),
          "Observed original write outcome must remain available without inventing unused bits");
  for (unsigned page = block * Nand::block_pages; page < (block + 1) * Nand::block_pages; ++page) {
    for (unsigned column = 0; column < Nand::page_bytes; ++column) {
      NandByte expected{.value = byte_mask, .known = byte_mask};
      if (input.program && page == row) {
        const unsigned shift = (2 - (column % 3)) * 8;
        expected.value = static_cast<std::uint8_t>((source.at(column / 3) >> shift) & byte_mask);
      }
      if (input.outcome == NandOutcome::fail && (!input.program || page == row)) { expected = {}; }
      require(nand.cell({.page = page, .column = column}) == expected,
              "Original native command must produce the independently expected whole-block "
              "contents/knowledge and preserve other pages");
    }
  }
}

struct Prefix {
  unsigned bytes = 0;
  unsigned raw = 0;
};

Prefix expected_prefix(const Source &source) {
  Prefix result;
  for (const auto word : source) {
    for (const unsigned shift : {16U, 8U, 0U}) {
      unsigned raw = word >> shift;
      constexpr unsigned sign = 0x800000;
      if ((word & sign) != 0) { raw |= word_mask ^ (word_mask >> shift); }
      if (raw > pin_mask) {
        result.raw = raw;
        return result;
      }
      ++result.bytes;
    }
  }
  throw Fault("Original source must expose its reserved Port F write before confirmation");
}

void check_original_source(const Nand &nand,
                           const NandLink &link,
                           const Result &result,
                           const Helper helper,
                           const Source &source) {
  const auto expected = expected_prefix(source);
  require(result.reason.contains("reserved-bit write at Y:0xfffff5") &&
              result.reason.contains("value=" + hex(expected.raw, 6)) && result.pc == helper.data &&
              link.writes() == expected.bytes && link.commands() == 1 && link.addresses() == 4 &&
              link.polls() == 0 && link.completed_writes() == 0 && link.reads() == 0 &&
              nand.pending() == NandPending::none && nand.column() == expected.bytes &&
              result.source_end == buffer + (expected.bytes / 3) + 1,
          "Original payload must retain its precise unqualified Port F frontier before any "
          "program confirmation: " +
              result.reason);
  for (unsigned column = 0; column < Nand::page_bytes; ++column) {
    require(nand.cell({.page = row, .column = column}) ==
                NandByte{.value = byte_mask, .known = byte_mask},
            "Unconfirmed original staging must leave every target cell unchanged");
  }
}

void check_interrupted(Nand &nand, const NandLink &link, const Result &result, const Input input) {
  const bool active = input.stop == Stop::active;
  auto pending = NandPending::none;
  if (active) { pending = input.program ? NandPending::program : NandPending::erase; }
  require(result.reason.empty() && nand.pending() == pending && link.completed_writes() == 0 &&
              link.polls() == 0 && link.reads() == 0 && link.commands() == (active ? 2U : 1U) &&
              link.writes() == (input.program ? Nand::page_bytes : 0U) &&
              result.source_end == buffer + (input.program ? source_words : 0U),
          "Native cut must occur on the actual last data/confirmation WE edge, before outcome");
  if (input.power) {
    nand.power_loss();
    require(!nand.ready().has_value(), "Native-command power cut withdraws readiness");
    nand.observe_power_ready();
    constexpr unsigned only_ready = 0x40;
    require(nand.status().known == only_ready,
            "Power-up observation must not restore WP or the interrupted result");
  } else {
    constexpr unsigned reset = 0xff;
    nand.command(reset);
    require(nand.ready() == false, "Native-command reset has its own pending completion");
    nand.observe_completion();
    constexpr unsigned reset_pass = 0xc0;
    require(nand.status() == NandByte{.value = reset_pass, .known = byte_mask},
            "Specified reset status does not establish success for the interrupted native write");
  }
  for (unsigned page = block * Nand::block_pages; page < (block + 1) * Nand::block_pages; ++page) {
    for (unsigned column = 0; column < Nand::page_bytes; ++column) {
      NandByte expected{.value = static_cast<std::uint8_t>(input.program ? byte_mask : old_cookie),
                        .known = byte_mask};
      if (active && (!input.program || page == row)) { expected = {}; }
      require(nand.cell({.page = page, .column = column}) == expected,
              "Native cut preserves unconfirmed cells and neighbors, but withdraws active targets");
    }
  }
  if (!active) {
    // Component retry probe after the original routine's unconfirmed full buffer.
    // This does not establish retry behavior of the complete original updater.
    nand.observe_wp(true);
    nand.command(0);
    constexpr unsigned program_command = 0x80;
    constexpr unsigned program_confirm = 0x10;
    nand.command(program_command);
    for (const auto byte : {0U, row, 0U, 0U}) { nand.address(static_cast<std::uint8_t>(byte)); }
    nand.write(0);
    nand.command(program_confirm);
    require(nand.pending() == NandPending::program,
            "Discarded native staging must not consume its main-array program allowance");
  }
}

void run(const Firmware &firmware, const Input input, std::ostream &trace) {
  trace << "helper=" << input.helper.index << " program=" << input.program
        << " original-source=" << input.original_source
        << " outcome=" << (input.outcome == NandOutcome::pass ? "pass" : "fail")
        << " stop=" << static_cast<unsigned>(input.stop) << " power=" << input.power << '\n';
  const auto source = source_words_from(firmware, input.original_source);
  DspProgram caller;
  const auto after_nops =
      own_caller(caller, input.program ? input.helper.program : input.helper.erase);
  DspBus bus;
  install_inputs(bus, firmware, input.helper, source);
  caller.install(bus);
  Nand nand;
  prepare_nand(nand, input.program);
  NandLink link(bus,
                nand,
                trace,
                {.poll_pcs = {input.helper.poll}, .selected = true, .outcomes = {input.outcome}});
  DspCore cpu(bus);
  const auto result = execute(bus, cpu, link, after_nops, input.stop, trace);
  if (input.stop != Stop::none) {
    check_interrupted(nand, link, result, input);
  } else if (input.original_source) {
    check_original_source(nand, link, result, input.helper, source);
  } else {
    check_completed(nand, link, result, input, source);
  }
  require(nand.cell({.page = neighbor_page}) ==
              NandByte{.value = neighbor_cookie, .known = byte_mask},
          "Original helper experiment must preserve the neighboring block");
  trace << "checked writes=" << link.writes() << " completed-writes=" << link.completed_writes()
        << " source-end=" << hex(result.source_end, 6) << '\n';
}

unsigned worker_number(const std::string_view text) {
  unsigned result = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  require(parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size(),
          "Invalid persistent-worker number");
  return result;
}

int persistent_worker(const Firmware &firmware, char **arguments) {
  const auto program = worker_number(arguments[3]);
  const auto fail = worker_number(arguments[4]);
  const auto requested_transaction = worker_number(arguments[5]);
  const auto requested_phase = worker_number(arguments[6]);
  require(program <= 1 && fail <= 1 && requested_transaction <= 2 &&
              requested_phase <= static_cast<unsigned>(NandFilePhase::directory_synced),
          "Invalid persistent-worker job or phase");
  const auto helper = helpers.back();
  const auto outcome = fail != 0 ? NandOutcome::fail : NandOutcome::pass;
  const auto source = source_words_from(firmware, false);
  DspProgram caller;
  const auto after_nops = own_caller(caller, program != 0 ? helper.program : helper.erase);
  DspBus bus;
  install_inputs(bus, firmware, helper, source);
  caller.install(bus);
  NandStore store(arguments[2]);
  store.observe_power_ready();
  store.observe_wp(true);
  std::ostringstream trace;
  NandLink link(
      bus, store, trace, {.poll_pcs = {helper.poll}, .selected = true, .outcomes = {outcome}});
  DspCore cpu(bus);
  unsigned transactions = 0;
  store.set_observer([&](const NandFilePhase phase) {
    if (phase == NandFilePhase::created) { ++transactions; }
    if (transactions - 1 != requested_transaction ||
        static_cast<unsigned>(phase) != requested_phase) {
      return;
    }
    const auto state = cpu.state();
    std::cout << "NAND_FILE " << requested_transaction << ' ' << requested_phase
              << " pc=" << hex(state.pc, 6) << " source=" << hex(state.r[0], 6)
              << " pending=" << static_cast<unsigned>(store.pending())
              << " ready=" << (store.ready() == true) << '\n'
              << std::flush;
    char resume = 0;
    require(static_cast<bool>(std::cin.get(resume)), "Persistent worker was not resumed");
  });
  const auto result = execute(bus, cpu, link, after_nops, Stop::none, trace);
  check_completed(store.device(),
                  link,
                  result,
                  {.helper = helper, .program = program != 0, .outcome = outcome},
                  source);
  require(transactions == 2,
          "Native helper must publish its start and observed outcome exactly once");
  std::cout << "Persistent native helper checked through its expected turnaround frontier.\n";
  return 0;
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3 && count != 7) {
      throw blofeld::Fault("Usage: original-nand-write-test firmware.zip trace | firmware.zip "
                           "state.bfnand program fail transaction phase");
    }
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    if (count == 7) { return blofeld::test::persistent_worker(firmware, arguments); }
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    for (const auto helper : blofeld::test::helpers) {
      for (const bool program : {false, true}) {
        for (const auto outcome : {blofeld::NandOutcome::pass, blofeld::NandOutcome::fail}) {
          blofeld::test::run(
              firmware, {.helper = helper, .program = program, .outcome = outcome}, trace);
        }
      }
      blofeld::test::run(
          firmware, {.helper = helper, .program = true, .original_source = true}, trace);
      for (const bool power : {false, true}) {
        blofeld::test::run(firmware,
                           {.helper = helper, .stop = blofeld::test::Stop::active, .power = power},
                           trace);
        for (const auto stop : {blofeld::test::Stop::loaded, blofeld::test::Stop::active}) {
          blofeld::test::run(
              firmware, {.helper = helper, .program = true, .stop = stop, .power = power}, trace);
        }
      }
    }
    std::cout << "Both original NAND helpers emitted erase/program cycles; observed outcomes and "
                 "unchanged payload frontiers checked. Full original update remains unqualified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
