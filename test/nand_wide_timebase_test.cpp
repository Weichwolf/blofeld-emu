// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_interval.hpp"
#include "storage/nand_operation_clock.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_write_medium.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
constexpr auto counter_max = std::numeric_limits<std::uint64_t>::max();
constexpr std::uint32_t million = 1000000, billion = 1000000000;
constexpr auto interval_max = std::numeric_limits<std::uint32_t>::max();
constexpr NandByte payload{.value = 0xa5, .known = 0xff};
constexpr unsigned byte_mask = 0xff;
constexpr std::uint8_t program_command = 0x80, program_confirm = 0x10;
constexpr std::uint8_t erase_command = 0x60, erase_confirm = 0xd0;
constexpr std::uint8_t status_command = 0x70, status_mask = 0xc1, pass_status = 0xc0;

// Fixed integer ceilings of K9F1208U0C Rev1.1 timing-table entries. These are
// arithmetic/admission vectors, not a calibration of the board oscillator.
struct Bounds {
  std::uint64_t hz;
  std::uint64_t cea, rea, oh, chz;
  std::uint64_t cs, wp, hold, cycle, whr, control;
  std::uint64_t program, erase;
};

constexpr std::array rates{Bounds{.hz = 1080000000,
                                  .cea = 38,
                                  .rea = 33,
                                  .oh = 17,
                                  .chz = 22,
                                  .cs = 34,
                                  .wp = 23,
                                  .hold = 6,
                                  .cycle = 46,
                                  .whr = 65,
                                  .control = 11,
                                  .program = 540000,
                                  .erase = 3240000},
                           Bounds{.hz = 211680000000,
                                  .cea = 7409,
                                  .rea = 6351,
                                  .oh = 3176,
                                  .chz = 4234,
                                  .cs = 6563,
                                  .wp = 4446,
                                  .hold = 1059,
                                  .cycle = 8891,
                                  .whr = 12701,
                                  .control = 2117,
                                  .program = 105840000,
                                  .erase = 635040000},
                           Bounds{.hz = 211680000001,
                                  .cea = 7409,
                                  .rea = 6351,
                                  .oh = 3176,
                                  .chz = 4234,
                                  .cs = 6563,
                                  .wp = 4446,
                                  .hold = 1059,
                                  .cycle = 8891,
                                  .whr = 12701,
                                  .control = 2117,
                                  .program = 105840001,
                                  .erase = 635040001},
                           Bounds{.hz = counter_max,
                                  .cea = 645636042580,
                                  .rea = 553402322212,
                                  .oh = 276701161106,
                                  .chz = 368934881475,
                                  .cs = 571849066285,
                                  .wp = 387381625548,
                                  .hold = 92233720369,
                                  .cycle = 774763251096,
                                  .whr = 1106804644423,
                                  .control = 184467440738,
                                  .program = 9223372036854776,
                                  .erase = 55340232221128655}};

static_assert(nand_interval_ticks<million>(counter_max, million) == counter_max);
static_assert(!nand_interval_ticks<million>(counter_max, million + 1));
static_assert(nand_interval_ticks<interval_max>(counter_max, interval_max) == counter_max);
static_assert(nand_interval_ticks<billion>(counter_max, 0) == 0);
static_assert(nand_interval_ticks<million>(0, interval_max) == 0);
static_assert(nand_interval_ticks<billion>(1, 1) == 1);

void require(const bool condition, const std::string &reason) {
  if (!condition) { throw Fault(reason); }
}

void reject(const auto &action, const std::string &reason) {
  try {
    action();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains(reason),
            "Unexpected fault: " + std::string(error.what()));
    return;
  }
  throw Fault("Expected NAND refusal: " + reason);
}

void read_output(const Bounds &row) {
  const auto clearance = row.whr * 2;
  NandReadTiming model(row.hz);
  model.observe(0, {.ce_n = true, .re_n = true});
  model.observe(clearance, {.ce_n = false, .re_n = true});
  unsigned reads = 0;
  model.observe(clearance, {.ce_n = false, .re_n = false}, [&] {
    ++reads;
    return payload;
  });
  require(reads == 1 && model.next_transition() == clearance + row.cea,
          "One genuine source read; exact wide CE access deadline");
  require(model.output(clearance + row.cea - 1) == NandReadOutput{} &&
              model.output(clearance + row.cea) == NandReadOutput{.driven = true, .data = payload},
          "Independent wide CE access boundary");
  auto ce = model;
  const auto release = clearance + row.cea;
  model.observe(release, {.ce_n = false, .re_n = true});
  ce.observe(release, {.ce_n = true, .re_n = false});
  for (auto *branch : {&model, &ce}) {
    require(branch->output(release + row.oh - 1) ==
                    NandReadOutput{.driven = true, .data = payload} &&
                branch->output(release + row.oh) == NandReadOutput{},
            "Exact output-hold boundary survives an owner copy");
  }
  require(model.output(release + row.rea - 1) == NandReadOutput{} &&
              model.output(release + row.rea) == NandReadOutput{.driven = false} &&
              ce.output(release + row.chz - 1) == NandReadOutput{} &&
              ce.output(release + row.chz) == NandReadOutput{.driven = false},
          "Independent RE/CE release maxima");
  model.qualify_ce_window(release + row.rea);
  model.resume_ce_window(release + row.rea, release + row.rea + 1, false);
  const auto fall = release + row.rea + 1;
  model.observe(fall, {.ce_n = false, .re_n = false}, [] { return payload; });
  require(model.output(fall + row.cea - 1) == NandReadOutput{} &&
              model.output(fall + row.cea) == NandReadOutput{.driven = true, .data = payload},
          "Opaque CE history retains its latest selection/access bound");
}

void setup_signal(NandReadSetupPins &pins, const unsigned field, const bool settled) {
  switch (field) {
    case 0: pins.we_n = settled; break;
    case 1: pins.ale = !settled; break;
    case 2: pins.cle = !settled; break;
    case 3: pins.ready = settled; break;
    default: throw Fault("Invalid read-setup field");
  }
}

void read_setup(const Bounds &row) {
  constexpr std::array names{"tWHR", "tAR", "tCLR", "tRR"};
  const std::array bounds{row.whr, row.control, row.control, row.chz};
  const auto changed = row.whr * 2;
  for (unsigned field = 0; field < names.size(); ++field) {
    NandReadSetup model(row.hz);
    NandReadSetupPins pins{
        .ce_n = false, .re_n = true, .we_n = true, .ale = false, .cle = false, .ready = true};
    setup_signal(pins, field, false);
    require(!model.observe(0, pins), "Initial setup observation is not a read");
    setup_signal(pins, field, true);
    require(!model.observe(changed, pins), "A setup change cannot manufacture a read");
    const auto due = changed + bounds[field];
    const auto saved = model;
    reject([&] { model.qualify_ce_window(due - 1); }, "setup constraints");
    require(model == saved, "Failed CE-window qualification retains all anchors");
    auto window = model;
    window.resume_ce_window(due, due + 1, true);
    pins.re_n = false;
    reject([&] { static_cast<void>(model.observe(due - 1, pins)); }, names[field]);
    require(model == saved && model.observe(due, pins),
            "Exact wide read setup and failure rollback");
    pins.re_n = true;
    require(!model.observe(due, pins), "Rising RE cannot consume a byte");
    pins.ce_n = true;
    require(!model.observe(due, pins), "Selected read exits on the actual CE edge");
    pins.cle = true;
    const auto before_hold = model;
    reject([&] { static_cast<void>(model.observe(due + row.control - 1, pins)); }, "tCSD");
    require(model == before_hold && !model.observe(due + row.control, pins),
            "Control hold uses the full frequency after a selected read exit");
  }
}

void write_signal(NandWritePins &pins, const unsigned field, const bool after) {
  switch (field) {
    case 0: pins.ce_n = after; break;
    case 1: pins.cle = after; break;
    case 2: pins.ale = !after; break;
    case 3: pins.data.value = after ? payload.value : 0; break;
    default: throw Fault("Invalid write-setup field");
  }
}

void no_latch(NandWriteTiming &model, const std::uint64_t tick, const NandWritePins &pins) {
  require(!model.observe(tick, pins), "No rising WE, no input latch");
}

void write_setup(const Bounds &row) {
  constexpr std::array setups{"tCS", "tCLS", "tALS", "tDS"};
  constexpr std::array holds{"tCH", "tCLH", "tALH", "tDH"};
  constexpr std::array kinds{
      NandWriteKind::command, NandWriteKind::data, NandWriteKind::address, NandWriteKind::command};
  const std::array bounds{row.cs, row.wp, row.wp, row.chz};
  const auto clearance = row.whr * 2;
  for (unsigned field = 0; field < setups.size(); ++field) {
    NandWriteTiming model(row.hz);
    NandWritePins pins{.ce_n = field == 0,
                       .re_n = true,
                       .we_n = true,
                       .cle = field != 2,
                       .ale = false,
                       .data = payload};
    no_latch(model, 0, pins);
    pins.ce_n = false;
    no_latch(model, clearance, pins);
    pins.we_n = false;
    no_latch(model, clearance, pins);
    const auto changed = clearance + row.wp;
    write_signal(pins, field, false);
    no_latch(model, changed, pins);
    const auto due = (field == 0 ? clearance : changed) + bounds[field];
    const auto saved = model;
    pins.we_n = true;
    reject([&] { static_cast<void>(model.observe(due - 1, pins)); }, setups[field]);
    require(model == saved && model.observe(due, pins) ==
                                  NandWriteLatch{.kind = kinds[field], .value = pins.data.value},
            "Wide input setup admits one exact protocol latch");
    write_signal(pins, field, true);
    const auto before_hold = model;
    reject([&] { no_latch(model, due + row.hold - 1, pins); }, holds[field]);
    require(model == before_hold, "Rejected input hold retains the complete owner");
    no_latch(model, due + row.hold, pins);
  }
}

void write_pulse(const Bounds &row) {
  NandWriteTiming model(row.hz);
  NandWritePins pins{
      .ce_n = false, .re_n = true, .we_n = true, .cle = true, .ale = false, .data = payload};
  const auto fall = row.whr * 2;
  no_latch(model, 0, pins);
  pins.we_n = false;
  no_latch(model, fall, pins);
  const auto saved = model;
  pins.we_n = true;
  reject([&] { static_cast<void>(model.observe(fall + row.wp - 1, pins)); }, "tWP");
  require(model == saved && model.observe(fall + row.wp, pins).has_value(),
          "Exact wide write pulse");
  pins.we_n = false;
  reject([&] { no_latch(model, fall + row.wp + row.oh - 1, pins); }, "tWH");
  reject([&] { no_latch(model, fall + row.cycle - 1, pins); }, "tWC");
  no_latch(model, fall + row.cycle, pins);
  pins.we_n = true;
  require(model.observe(fall + row.cycle + row.wp, pins).has_value(), "Exact wide write cycle");
}

NandWriteMedium medium(const Bounds &row, const bool erasing) {
  NandImage image;
  NandArray::Page page;
  page.cells.fill(
      {.value = static_cast<std::uint8_t>(erasing ? 0 : byte_mask), .known = byte_mask});
  page.programs = erasing ? NandProgramCount{.main = 1, .spare = 1} : NandProgramCount{};
  for (unsigned index = 0; index <= Nand::block_pages; ++index) {
    image.pages.emplace(index, page);
  }
  image.blocks.emplace(
      0, NandBlock{.valid = true, .program_protected = false, .erase_protected = false});
  Nand nand(image, NandReadContinuation::k9f1208_p);
  nand.observe_wp(true);
  NandWriteMedium result(
      nand, row.hz, {.contract = NandCellContract::reference_pulse_inhibit, .cells = {}});
  result.observe_power_ready();
  return result;
}

void begin_write(NandWriteMedium &model, const bool erasing) {
  model.command(erasing ? erase_command : program_command);
  for (unsigned index = 0; index < (erasing ? 3U : 4U); ++index) { model.address(0); }
  if (!erasing) { model.write(payload.value); }
  model.command(erasing ? erase_confirm : program_confirm);
}

void write_medium(const Bounds &row) {
  for (const bool erasing : {false, true}) {
    const auto duration = erasing ? row.erase : row.program;
    const auto operation = erasing ? NandPending::erase : NandPending::program;
    for (const auto start : {std::uint64_t{0}, counter_max - duration}) {
      auto model = medium(row, erasing);
      require(!model.advance(start), "Idle controller has no write completion");
      begin_write(model, erasing);
      const NandCompletionEvent expected{.ticks = start + duration, .operation = operation};
      require(model.next_event() == expected && !model.advance(expected.ticks - 1) &&
                  model.device().ready() == false,
              "Independent wide write duration stays busy until due");
      auto copy = model;
      require(model.advance(expected.ticks) == expected &&
                  copy.advance(expected.ticks) == expected && model.device().ready() == true &&
                  !model.next_event(),
              "Copied genuine cell operation completes exactly once");
      require(model.device().cell({.page = 0, .column = 0}) ==
                  NandByte{.value = static_cast<std::uint8_t>(erasing ? byte_mask : payload.value),
                           .known = byte_mask},
              "Ready derives from actual program/erase verification");
      require(model.device().cell({.page = Nand::block_pages, .column = 0}).value ==
                  (erasing ? 0 : byte_mask),
              "Neighboring block remains untouched");
      model.command(status_command);
      require(model.read(status_mask) == pass_status,
              "Only the cell model's verified result produces pass status");
    }
    auto overflow = medium(row, erasing);
    require(!overflow.advance(counter_max - duration + 1),
            "Empty counter can reach the overflow boundary");
    reject([&] { begin_write(overflow, erasing); }, "completion overflow");
    require(overflow.faulted() && overflow.device().ready() == true &&
                overflow.device().cell({.page = 0, .column = 0}).known == byte_mask,
            "Overflow fails before withdrawing cells or inventing completion");
  }
}

void deadline_overflow(const Bounds &row) {
  NandReadTiming read(row.hz);
  read.observe(0, {.ce_n = true, .re_n = true});
  const auto start = counter_max - row.cea + 1;
  read.observe(start, {.ce_n = false, .re_n = true});
  const auto saved = read;
  unsigned calls = 0;
  reject(
      [&] {
        read.observe(start, {.ce_n = false, .re_n = false}, [&] {
          ++calls;
          return payload;
        });
      },
      "reference deadline overflow");
  require(read == saved && calls == 0,
          "Read overflow retains the owner and never consumes protocol data");
  NandWriteTiming write(row.hz);
  NandWritePins pins{
      .ce_n = true, .re_n = true, .we_n = true, .cle = true, .ale = false, .data = payload};
  no_latch(write, 0, pins);
  pins.ce_n = false;
  no_latch(write, counter_max - row.cs + 1, pins);
  pins.we_n = false;
  no_latch(write, counter_max - row.cs + 1, pins);
  const auto saved_write = write;
  pins.we_n = true;
  reject([&] { static_cast<void>(write.observe(counter_max, pins)); },
         "reference deadline overflow");
  require(write == saved_write, "Write overflow retains the complete owner without latching");
}
}

int main() {
  try {
    for (const auto &row : rates) {
      read_output(row);
      read_setup(row);
      write_setup(row);
      write_pulse(row);
      write_medium(row);
      deadline_overflow(row);
    }
    std::cout << "PASS NAND_WIDE_TIMEBASE rates=" << rates.size() << "\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL NAND_WIDE_TIMEBASE " << error.what() << '\n';
    return 1;
  }
}
