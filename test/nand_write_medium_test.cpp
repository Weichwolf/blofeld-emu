// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "nand_native_tests.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_operation_clock.hpp"
#include "storage/nand_write_medium.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned program_command = 0x80;
constexpr unsigned program_confirm = 0x10;
constexpr unsigned erase_command = 0x60;
constexpr unsigned erase_confirm = 0xd0;
constexpr unsigned reset_command = 0xff;
constexpr unsigned status_command = 0x70;
constexpr unsigned status_mask = 0xc1;
constexpr unsigned pass_status = 0xc0;
constexpr unsigned fail_status = 0xc1;
constexpr unsigned timebase = 1080000000;
constexpr std::uint64_t program_duration = 540000;
constexpr std::uint64_t erase_duration = 3240000;
constexpr std::uint64_t write_reset_duration = 10800;
constexpr std::uint64_t erase_reset_duration = 540000;
constexpr std::array<std::uint8_t, 3> pattern{0xa2, 0xc8, 0xed};

void require(const bool condition, const std::string &reason) {
  if (!condition) { throw Fault(reason); }
}

void reject(const auto &action, const std::string &part) {
  try {
    action();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains(part),
            "Unexpected refusal: " + std::string(error.what()));
    return;
  }
  throw Fault("Expected refusal: " + part);
}

NandCellProfile profile(const std::uint8_t program = 0, const std::uint8_t erase = 0) {
  NandCellProfile result{.contract = NandCellContract::reference_pulse_inhibit, .cells = {}};
  result.cells.emplace(0, NandCellInhibit{.program = program, .erase = erase});
  return result;
}

Nand device(const bool erased = true) {
  NandImage image;
  NandArray::Page page;
  page.cells.fill(
      {.value = static_cast<std::uint8_t>(erased ? byte_mask : 0U), .known = byte_mask});
  page.programs = erased ? NandProgramCount{} : NandProgramCount{.main = 1, .spare = 1};
  for (unsigned row = 0; row <= Nand::block_pages; ++row) { image.pages.emplace(row, page); }
  image.blocks.emplace(
      0, NandBlock{.valid = true, .program_protected = false, .erase_protected = false});
  Nand result(image, NandReadContinuation::k9f1208_p);
  result.observe_wp(true);
  return result;
}

NandWriteMedium medium(const bool erased = true, const NandCellProfile &physics = profile()) {
  NandWriteMedium result(device(erased), timebase, physics);
  result.observe_power_ready();
  return result;
}

void start_program(NandWriteMedium &model, const std::array<std::uint8_t, 3> &bytes = pattern) {
  model.command(program_command);
  for (unsigned i = 0; i < 4; ++i) { model.address(0); }
  for (const auto byte : bytes) { model.write(byte); }
  model.command(program_confirm);
}

void start_erase(NandWriteMedium &model) {
  model.command(erase_command);
  for (unsigned i = 0; i < 3; ++i) { model.address(0); }
  model.command(erase_confirm);
}

struct OracleInput {
  unsigned data;
  unsigned inhibit;
  bool erasing;
};

NandCellResult oracle(const NandByte before, const OracleInput input) {
  const auto data = input.data;
  const auto inhibit = input.inhibit;
  const auto erasing = input.erasing;
  NandByte after{};
  bool wrong = false;
  bool unknown = false;
  for (unsigned bit = 0; bit < 8; ++bit) {
    const unsigned mask = 1U << bit;
    bool known = (before.known & mask) != 0;
    bool value = (before.value & mask) != 0;
    const bool selected = erasing || (data & mask) == 0;
    if (selected && (inhibit & mask) == 0) {
      known = true;
      value = erasing;
    }
    if (known) { after.known = static_cast<std::uint8_t>(after.known | mask); }
    if (known && value) { after.value = static_cast<std::uint8_t>(after.value | mask); }
    if (selected && known && value != erasing) { wrong = true; }
    if (selected && !known) { unknown = true; }
  }
  std::optional<bool> verified = true;
  if (wrong) {
    verified = false;
  } else if (unknown) {
    verified.reset();
  }
  return {.after = after, .verified = verified};
}

void cell_vectors() {
  constexpr std::array before{NandByte{.value = 0, .known = 0},
                              NandByte{.value = 0, .known = byte_mask},
                              NandByte{.value = byte_mask, .known = byte_mask},
                              NandByte{.value = 0x11, .known = 0x55}};
  unsigned count = 0;
  for (const auto cell : before) {
    for (unsigned inhibit = 0; inhibit <= byte_mask; ++inhibit) {
      const auto erased = NandCellModel::erase(
          cell, NandCellInhibit{.program = 0, .erase = static_cast<std::uint8_t>(inhibit)});
      const auto erase_expected = oracle(cell, {.data = 0, .inhibit = inhibit, .erasing = true});
      require(erased.after == erase_expected.after && erased.verified == erase_expected.verified,
              "Independent per-bit erase/verification expectation");
      for (unsigned data = 0; data <= byte_mask; ++data) {
        const auto actual = NandCellModel::program(
            cell,
            static_cast<std::uint8_t>(data),
            NandCellInhibit{.program = static_cast<std::uint8_t>(inhibit), .erase = 0});
        const auto expected = oracle(cell, {.data = data, .inhibit = inhibit, .erasing = false});
        require(actual.after == expected.after && actual.verified == expected.verified,
                "Independent per-bit program/verification expectation");
        ++count;
      }
    }
  }
  const auto unchanged = NandCellModel::program({.value = 0, .known = byte_mask},
                                                byte_mask,
                                                NandCellInhibit{.program = byte_mask, .erase = 0})
                             .verified;
  require(unchanged && *unchanged,
          "Input ones are inhibited;existing zero is not a programming verification failure");
  std::cout << "PASS CELL_MODEL program-vectors=" << count << " erase-vectors=1024\n";
}

void timed_program(const NandCellProfile &physics,
                   const bool fails,
                   const std::array<std::uint8_t, 3> &bytes = pattern) {
  auto model = medium(true, physics);
  start_program(model, bytes);
  require(model.next_event() ==
              NandCompletionEvent{.ticks = program_duration, .operation = NandPending::program},
          "Program500us maximum converted independently to540000ticks");
  require(!model.advance(program_duration - 1) && model.device().ready() == false &&
              model.device().cell({.page = 0, .column = 0}).known == 0,
          "No completed page or Ready before its own deadline");
  auto isolated = model;
  const auto event = model.advance(program_duration);
  require(
      event == NandCompletionEvent{.ticks = program_duration, .operation = NandPending::program} &&
          model.device().ready() == true && !model.next_event(),
      "Genuine program/verify completion");
  model.command(status_command);
  require(model.read(status_mask) == (fails ? fail_status : pass_status),
          "Status derives from actual requested zero verification");
  for (unsigned i = 0; i < Nand::page_bytes; ++i) {
    unsigned expected = byte_mask;
    if (i < bytes.size()) { expected = bytes[i]; }
    if (i == 0 && fails) { expected |= 1U; }
    require(model.device().cell({.page = 0, .column = i}) ==
                NandByte{.value = static_cast<std::uint8_t>(expected), .known = byte_mask},
            "Complete actual page,including programmed and inhibited cell bits");
  }
  require(isolated.device().ready() == false && isolated.ticks() == program_duration - 1,
          "Copied pending controller is isolated from completion");
  require(isolated.advance(program_duration).has_value(),
          "Copied pending operation retains its true outcome/deadline");
  auto strict = model;
  reject([&] { static_cast<void>(strict.read()); }, "unknown status bits");
  require(strict.faulted(), "Undefined status pins stay unknown and fail full-byte requests");
  std::cout << "PASS TIMED_PROGRAM fail=" << fails << " deadline=" << program_duration << '\n';
}

void timed_erase(const bool fails) {
  auto model = medium(false, profile(0, fails ? 1 : 0));
  start_erase(model);
  require(model.next_event() ==
              NandCompletionEvent{.ticks = erase_duration, .operation = NandPending::erase},
          "Erase3ms maximum converted independently to3240000ticks");
  require(!model.advance(erase_duration - 1) && model.device().ready() == false,
          "Erase remains genuinely busy");
  require(model.advance(erase_duration).has_value(),
          "Actual erase verification reaches its own deadline");
  model.command(status_command);
  require(model.read(status_mask) == (fails ? fail_status : pass_status),
          "Erase failure is computed from inhibited zero");
  for (unsigned row = 0; row < Nand::block_pages; ++row) {
    for (unsigned col = 0; col < Nand::page_bytes; ++col) {
      const auto expected = fails && row == 0 && col == 0 ? byte_mask - 1 : byte_mask;
      require(model.device().cell({.page = row, .column = col}) ==
                  NandByte{.value = static_cast<std::uint8_t>(expected), .known = byte_mask},
              "All32 actual erased pages have independently expected physical bits");
    }
  }
  require(model.device().cell({.page = Nand::block_pages, .column = 0}) ==
              NandByte{.value = 0, .known = byte_mask},
          "Erase cannot change the neighboring block");
  std::cout << "PASS TIMED_ERASE fail=" << fails << " deadline=" << erase_duration << '\n';
}

void interruption() {
  for (const bool erasing : {false, true}) {
    auto model = medium();
    if (erasing) {
      start_erase(model);
    } else {
      start_program(model);
    }
    const auto deadline = erasing ? erase_duration : program_duration;
    require(!model.advance(deadline - 1), "Explicit cut precedes physical completion");
    auto lost = model;
    lost.power_loss();
    require(!lost.next_event() && !lost.advance(deadline) && !lost.device().ready(),
            "Power loss cancels completion and leaves unknown readiness");
    model.command(reset_command);
    const auto duration = erasing ? erase_reset_duration : write_reset_duration;
    const auto reset_at = deadline - 1 + duration;
    require(model.next_event() ==
                NandCompletionEvent{.ticks = reset_at, .operation = NandPending::reset},
            "Reset replaces program/erase deadline using the actual preceding operation");
    require(!model.advance(deadline) && model.device().ready() == false,
            "Canceled write cannot publish late success");
    require(model.advance(reset_at).has_value() && model.device().ready() == true &&
                model.device().cell({.page = 0, .column = 0}).known == 0,
            "Reset readiness is separate from lost target data knowledge");
  }
  auto no_model = NandWriteMedium(device(), timebase, NandCellProfile{});
  no_model.observe_power_ready();
  reject([&] { start_program(no_model); }, "explicit cell model");
  require(no_model.faulted(), "Unselected medium never silently enables writes");
  std::cout << "PASS WRITE_INTERRUPTION canceled-program-erase=4 unselected-model-refused=1\n";
}

void incomplete_medium() {
  auto nand = device();
  constexpr unsigned incomplete = 0xfe;
  nand.observe_cell({.page = 0, .column = 0}, {.value = incomplete, .known = incomplete});
  NandWriteMedium unknown(nand, timebase, profile(1));
  unknown.observe_power_ready();
  start_program(unknown);
  reject([&] { static_cast<void>(unknown.advance(program_duration)); },
         "verification depends on unknown inhibited bits");
  require(unknown.faulted() && unknown.device().ready() == false &&
              unknown.device().pending() == NandPending::program &&
              (unknown.device().status().known & 1U) == 0,
          "Unknown verification neither creates an outcome nor releases Ready");
  reject([&] { static_cast<void>(unknown.advance(program_duration + 1)); }, "coordinated recovery");
  auto mixed = profile(1);
  mixed.cells.emplace(1, NandCellInhibit{.program = 1, .erase = 0});
  NandWriteMedium failed(nand, timebase, mixed);
  failed.observe_power_ready();
  start_program(failed);
  require(failed.advance(program_duration).has_value(),
          "Known wrong cell independently establishes failure despite another unknown cell");
  failed.command(status_command);
  require(failed.read(status_mask) == fail_status &&
              failed.device().cell({.page = 0, .column = 0}).known == incomplete,
          "Known failure status retains the independently unknown inhibited bit");
  std::cout << "PASS INCOMPLETE_MEDIUM unknown-guard=1 known-failure=1\n";
}

void deadline_bounds() {
  constexpr unsigned odd_rate = 1000001;
  for (const bool erasing : {false, true}) {
    NandWriteMedium model(device(), odd_rate, profile());
    model.observe_power_ready();
    if (erasing) {
      start_erase(model);
    } else {
      start_program(model);
    }
    const std::uint64_t due = erasing ? 3001 : 501;
    require(model.next_event() == NandCompletionEvent{.ticks = due,
                                                      .operation = erasing ? NandPending::erase
                                                                           : NandPending::program},
            "Fractional maximum durations round upward without changing the timebase");
    require(!model.advance(due - 1) && model.advance(due).has_value(),
            "Rounded completion is admitted exactly at its bound");
  }
  auto overflow = medium();
  constexpr auto near_end = std::numeric_limits<std::uint64_t>::max() - 100;
  require(!overflow.advance(near_end), "Empty reference counter can approach its bound");
  reject([&] { start_program(overflow); }, "completion overflow");
  require(overflow.faulted() && overflow.device().ready() == true &&
              overflow.device().cell({.page = 0, .column = 0}).known == byte_mask,
          "Overflow refuses before withdrawing the page or fabricating a deadline");
  std::cout << "PASS DEADLINE_BOUNDS fractional-program-erase=2 overflow=1\n";
}
}

int main() {
  try {
    cell_vectors();
    timed_program(profile(), false);
    timed_program(profile(1), true);
    auto ones = pattern;
    ones[0] = static_cast<std::uint8_t>(ones[0] | 1U);
    timed_program(profile(1), false, ones);
    timed_erase(false);
    timed_erase(true);
    interruption();
    incomplete_medium();
    deadline_bounds();
    blofeld::test::native_medium_tests();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL WRITE_MEDIUM " << error.what() << '\n';
    return 1;
  }
}
