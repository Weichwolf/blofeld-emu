// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/nand.hpp"
#include "dsp_norm_fixture.hpp"
#include "nand_program_fixture.hpp"
#include "nand_read_timing_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld::test::nand_records {
namespace {
using nand_read_reference::install_stage;
using nand_read_reference::Link;
constexpr unsigned buffer = 0xc000;
constexpr unsigned page_words = 176;
constexpr unsigned packing_entry = 0xbd;
constexpr unsigned record_entry = 0x7e;
constexpr unsigned continuation = record_entry + 1;
constexpr unsigned getter_entry = 0x9c;
constexpr unsigned runtime_store = 0x8e;
constexpr unsigned store_templates = 0x90;
constexpr unsigned next_write_edge = 0xa8;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 1080000000;
constexpr unsigned maximum_steps = 20000;
constexpr unsigned expected_records = 15;
constexpr unsigned expected_complete_records = 14;
constexpr std::array<unsigned, 3> expected_counts{29, 98, 4};
constexpr unsigned marker = 0x13579b;
constexpr unsigned tail_guard = buffer + (2 * page_words) - 1;
constexpr unsigned p_guard = 0x11d;
using Page = std::array<std::uint8_t, Nand::page_bytes>;

struct Word {
  DspSpace space;
  unsigned address;
  unsigned value;
  unsigned record;
};

struct Record {
  unsigned space;
  unsigned address;
  unsigned count;
  unsigned available;
};

struct Oracle {
  std::vector<Word> writes;
  std::vector<Record> records;
};

struct Ledger {
  unsigned stores = 0;
  unsigned patches = 0;
  std::array<unsigned, 3> counts{};
};

struct Joint {
  DspCheckpoint machine;
  Nand nand;
  Link::State link;
  Ledger ledger;
  DspDebugState debug;
  std::string prefix;
};

DspRunContract contract() {
  return {.clock = {.extal_hz = extal_hz,
                    .timebase_hz = timebase_hz,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_estimates};
}

Page read_page(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  Page page{};
  input.read(reinterpret_cast<char *>(page.data()), static_cast<std::streamsize>(page.size()));
  require(input.gcount() == static_cast<std::streamsize>(page.size()) &&
              input.get() == std::char_traits<char>::eof(),
          "Private prepared page must have exactly528 bytes");
  return page;
}

Oracle oracle(const Page &page) {
  std::array<unsigned, page_words> data{};
  constexpr unsigned high_shift = 16;
  for (std::size_t index = 0; index < data.size(); ++index) {
    const auto byte = index * 3;
    data[index] = (static_cast<unsigned>(page[byte]) << high_shift) |
                  (static_cast<unsigned>(page[byte + 1]) << 8U) | page[byte + 2];
  }
  Oracle result;
  unsigned cursor = 0;
  unsigned complete = 0;
  std::array<unsigned, 3> counts{};
  constexpr std::array spaces{DspSpace::p, DspSpace::x, DspSpace::y};
  while (cursor + 3 <= data.size()) {
    const auto mode = data[cursor];
    const auto count = data[cursor + 1];
    const auto address = data[cursor + 2];
    require(mode < spaces.size() && count != 0,
            "Original page record header must be bounded P/X/Y");
    cursor += 3;
    const auto available = std::min(count, page_words - cursor);
    const auto record = static_cast<unsigned>(result.records.size());
    result.records.push_back(
        {.space = mode, .address = address, .count = count, .available = available});
    for (unsigned index = 0; index < available; ++index) {
      result.writes.push_back({.space = spaces[mode],
                               .address = address + index,
                               .value = data[cursor + index],
                               .record = record});
    }
    counts[mode] += available;
    cursor += available;
    if (available != count) { break; }
    ++complete;
  }
  require(result.records.size() == expected_records && complete == expected_complete_records &&
              counts == expected_counts && cursor == page_words,
          "Independent original page census must match the pinned aggregate reference");
  return result;
}

void setup(DspBus &bus) {
  DspProgram program;
  setup_gpio(program);
  movep(program, input_f, port_f_direction, 'y');
  movep(program, all_inputs_f, port_f_control, 'y');
  program.immediate("omr", memory_mode);
  program.immediate("x0", marker);
  program.immediate("r0", 0);
  program.emit("move x0,x:(r0)");
  program.immediate("r0", 5);
  program.emit("move x0,x:(r0)");
  program.immediate("r0", 4);
  program.emit("move x0,y:(r0)");
  program.immediate("r0", tail_guard);
  program.emit("move x0,x:(r0)");
  program.immediate("r0", buffer);
  program.immediate("x0", 0);
  program.emit("move x0,x:(r0)");
  program.immediate("r2", buffer);
  program.immediate("r5", 1);
  program.immediate("lc", 0);
  program.immediate("la", 0);
  program.emit("jsr " + number(packing_entry));
  program.emit("jmp " + number(continuation));
  program.install(bus);
  // FM13-37 short BSR: fixed14-bit prefix, displacement bits9..6/4..0.
  constexpr unsigned short_bsr_mask = 0xfffc20;
  constexpr unsigned short_bsr = 0x050800;
  const auto instruction = bus.inspect_program(record_entry);
  const auto displacement = ((instruction >> 1U) & 0x1e0U) | (instruction & 0x1fU);
  require((instruction & short_bsr_mask) == short_bsr &&
              record_entry + displacement == getter_entry,
          "Original one-word BSR independently establishes the native post-read continuation");
}

void unknown_program_guard(DspBus &bus) {
  std::string error;
  try {
    static_cast<void>(bus.read({.address = p_guard}));
  } catch (const Fault &failure) { error = failure.what(); }
  require(error.contains("Unknown DSP memory read at P:" + hex(p_guard, 6)),
          "Untouched program RAM must remain unknown instead of acquiring a guessed word");
}

void prepare(Nand &nand, const Page &page) {
  nand.observe_bytes(0, 0, page);
  nand.observe_power_ready();
  nand.command(0);
  for (unsigned index = 0; index < 4; ++index) { nand.address(0); }
  nand.observe_completion();
}

void step(
    DspMachine &machine, Link &link, const Oracle &expected, Ledger &ledger, std::ostream &trace) {
  link.step({.observe = [&](const DspNativeAccess &access) {
    if (access.kind != DspNativeAccessKind::write || access.stage != DspNativeAccessStage::after) {
      return;
    }
    if (access.target.space == DspSpace::p && access.target.address == runtime_store) {
      require(ledger.patches < expected.records.size(),
              "Original loader cannot invent a new record header");
      const auto &record = expected.records[ledger.patches];
      require(access.value == machine.bus().inspect_program(store_templates + record.space),
              "Executed original MOVEM patches the actual next store from its selected template");
      ++ledger.patches;
      trace << "patch record=" << ledger.patches - 1 << " mode=" << record.space << '\n';
    } else if (access.pc == runtime_store) {
      require(ledger.stores < expected.writes.size(),
              "Original loader must not store beyond its supplied page");
      const auto &word = expected.writes[ledger.stores];
      require(access.target.space == word.space && access.target.address == word.address &&
                  access.value == word.value && ledger.patches == word.record + 1,
              "Each native self-modified P/X/Y store matches independent original record bytes");
      ++ledger.stores;
      ++ledger.counts.at(static_cast<std::size_t>(word.space));
      trace << "store record=" << word.record << " space=" << static_cast<unsigned>(word.space)
            << " address=" << hex(word.address, 6) << " value=" << hex(word.value, 6) << '\n';
    }
  }});
  const auto state = machine.cpu().state();
  trace << "state pc=" << hex(state.pc, 6) << " sp=" << state.sp << " lc=" << state.lc
        << " r0=" << hex(state.r[0], 6) << " r4=" << hex(state.r[4], 6)
        << " r7=" << hex(state.r[7], 6) << " patches=" << ledger.patches
        << " stores=" << ledger.stores << '\n';
}

std::string until_fault(
    DspMachine &machine, Link &link, const Oracle &expected, Ledger &ledger, std::ostream &trace) {
  for (unsigned count = 0; count < maximum_steps; ++count) {
    try {
      step(machine, link, expected, ledger, trace);
    } catch (const Fault &error) { return error.what(); }
  }
  throw Fault(
      "Original record loader exhausted its instruction budget without the strict boundary");
}

void verify(DspMachine &machine,
            Nand &nand,
            Link &link,
            const Oracle &expected,
            const Ledger ledger,
            const std::string &failure) {
  require(failure.contains("Unqualified NAND write-enable") &&
              failure.contains("dsp-pc=" + hex(next_write_edge, 6)) &&
              failure.contains("X:" + hex(port_c_data, 6)) &&
              failure.contains("omr=" + hex(memory_mode, 6)),
          "The original next refill must fail visibly at its first unsupported write strobe: " +
              failure);
  require(ledger.stores == expected.writes.size() && ledger.patches == expected.records.size() &&
              ledger.counts == expected_counts && link.reads() == Nand::page_bytes &&
              nand.column() == Nand::page_bytes && !nand.ready().has_value(),
          "Native original record counts cannot manufacture another page or command success");
  for (const auto &word : expected.writes) {
    require(machine.bus().read({.space = word.space, .address = word.address}) == word.value,
            "Each original destination retains its actual written payload");
  }
  const auto state = machine.cpu().state();
  const auto &last = expected.records.back();
  require(state.r[0] == tail_guard && state.r[4] == buffer + page_words &&
              state.r[7] == last.address + last.available &&
              state.lc == last.count - last.available && state.sp == 3,
          "The original partial-record continuation retains the real cursor/count/call frames");
  for (const auto guard : {DspAddress{.space = DspSpace::x, .address = 0},
                           DspAddress{.space = DspSpace::x, .address = 5},
                           DspAddress{.space = DspSpace::y, .address = 4},
                           DspAddress{.space = DspSpace::x, .address = tail_guard}}) {
    require(machine.bus().read(guard) == marker, "P/X/Y destination guards must remain unchanged");
  }
  unknown_program_guard(machine.bus());
}

void run(const Firmware &firmware, const Page &page, std::ostream &out) {
  const auto expected = oracle(page);
  DspMachine machine(contract());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  install_stage(machine.bus(), firmware);
  setup(machine.bus());
  unknown_program_guard(machine.bus());
  Nand nand;
  prepare(nand, page);
  std::ostringstream trace;
  Link link(machine, nand, trace);
  Ledger ledger;
  std::optional<Joint> saved;
  std::string failure;
  for (unsigned count = 0; failure.empty(); ++count) {
    require(count < maximum_steps, "Original record-loader instruction budget exceeded");
    try {
      step(machine, link, expected, ledger, trace);
    } catch (const Fault &error) { failure = error.what(); }
    constexpr unsigned first_y_patch = 6;
    if (failure.empty() && !saved && ledger.patches == first_y_patch) {
      saved.emplace(Joint{.machine = machine.checkpoint(),
                          .nand = nand,
                          .link = link.state(),
                          .ledger = ledger,
                          .debug = machine.cpu().debug_state(),
                          .prefix = trace.str()});
    }
  }
  if (!saved) { throw Fault("Original loader did not reach the selected live Y-store patch"); }
  verify(machine, nand, link, expected, ledger, failure);
  const auto result = machine.cpu().debug_state();
  DspMachine replay_machine(contract());
  replay_machine.restore(saved->machine);
  Nand peer = saved->nand;
  std::ostringstream repeated;
  Link replay(replay_machine, peer, repeated, saved->link);
  require(replay_machine.cpu().debug_state() == saved->debug,
          "Restored self-modified code and loop state survive fresh peripheral wiring");
  auto copied = saved->ledger;
  const auto replay_failure = until_fault(replay_machine, replay, expected, copied, repeated);
  verify(replay_machine, peer, replay, expected, copied, replay_failure);
  require(replay_failure == failure && replay_machine.cpu().debug_state() == result &&
              trace.str().starts_with(saved->prefix) &&
              repeated.str() == trace.str().substr(saved->prefix.size()),
          "Fresh restoration retains every native store/template/time row and the exact failure "
          "prefix");
  out << trace.str() << "verified records=" << ledger.patches << " stores=" << ledger.stores
      << " P=" << ledger.counts[0] << " X=" << ledger.counts[1] << " Y=" << ledger.counts[2]
      << " restored-suffix-bytes=" << repeated.str().size() << " failure=" << failure << '\n';
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 4) {
      throw blofeld::Fault("Usage: original-nand-records firmware.zip private.page private.trace");
    }
    const auto firmware = blofeld::Firmware::load(std::filesystem::path(arguments[1]));
    const auto page = blofeld::test::nand_records::read_page(std::filesystem::path(arguments[2]));
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    blofeld::test::nand_records::run(firmware, page, trace);
    std::cout << "Original runtime page:131 P/X/Y stores,15 real template patches and restored "
                 "continuation. Full boot/refill remains unqualified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
