// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using blofeld::Nand;
using blofeld::NandByte;
using blofeld::NandLocation;
using blofeld::NandOutcome;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned status_mask = 0xc1;
constexpr unsigned ready_bit = 0x40;
constexpr unsigned status_command = 0x70;
constexpr unsigned program_command = 0x80;
constexpr unsigned program_confirm = 0x10;
constexpr unsigned erase_command = 0x60;
constexpr unsigned erase_confirm = 0xd0;
constexpr unsigned spare_command = 0x50;
constexpr unsigned reset_command = 0xff;
constexpr unsigned status_pass = 0xc0;
constexpr unsigned status_fail = 0xc1;
constexpr unsigned pattern_a = 0xa5;
constexpr unsigned pattern_b = 0x5a;
constexpr unsigned payload = 0x42;
constexpr unsigned neighbor_cookie = 0x57;
constexpr unsigned reference_page = 37;
constexpr unsigned reference_block = reference_page / Nand::block_pages;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &reason) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    const std::string message = fault.what();
    require(message.contains(reason) && message.contains("page=") && message.contains("column=") &&
                message.contains("command="),
            "NAND rejection lost address/command context: " + message);
    return;
  }
  throw blofeld::Fault("Unqualified NAND operation succeeded: " + reason);
}

std::uint8_t byte(const unsigned value) {
  return static_cast<std::uint8_t>(value & byte_mask);
}

void row(Nand &nand, const unsigned page) {
  nand.address(byte(page));
  nand.address(byte(page >> 8U));
  nand.address(byte(page >> 16U));
}

void setup(Nand &nand) {
  nand.observe_power_ready();
  nand.observe_wp(true);
  nand.observe_block(reference_block,
                     {.valid = true, .program_protected = false, .erase_protected = false});
}

void begin_erase(Nand &nand, const unsigned page = reference_page) {
  nand.command(erase_command);
  row(nand, page);
  nand.command(erase_confirm);
}

void erased(Nand &nand) {
  begin_erase(nand);
  nand.observe_completion(NandOutcome::pass);
}

void begin_program(Nand &nand, const NandLocation start) {
  const bool spare = start.column >= Nand::main_bytes;
  nand.command(spare ? spare_command : 0);
  nand.command(program_command);
  nand.address(byte(spare ? start.column - Nand::main_bytes : start.column));
  row(nand, start.page);
}

NandByte oracle(const NandByte before, const std::uint8_t input) {
  NandByte result;
  for (unsigned bit = 0; bit < 8; ++bit) {
    const unsigned mask = 1U << bit;
    bool can_zero = false;
    bool can_one = false;
    for (unsigned old = 0; old < 2; ++old) {
      if ((before.known & mask) != 0 && old != ((before.value >> bit) & 1U)) { continue; }
      if (old != 0 && (input & mask) != 0) {
        can_one = true;
      } else {
        can_zero = true;
      }
    }
    if (can_zero != can_one) { result.known = byte(result.known | mask); }
    if (can_one && !can_zero) { result.value = byte(result.value | mask); }
  }
  return result;
}

void test_program_knowledge() {
  unsigned cases = 0;
  for (unsigned known = 0; known <= byte_mask; ++known) {
    for (unsigned value = 0; value <= byte_mask; ++value) {
      if ((value & ~known) != 0) { continue; }
      const NandByte before{.value = byte(value), .known = byte(known)};
      for (unsigned input = 0; input <= byte_mask; ++input) {
        require(blofeld::NandArray::programmed(before, byte(input)) == oracle(before, byte(input)),
                "NAND bit knowledge must equal the set of all consistent physical 1-to-0 outcomes");
        ++cases;
      }
    }
  }
  constexpr unsigned expected = 1679616;
  require(cases == expected, "All ternary byte observations and eight-bit inputs must be covered");
  std::cout << cases << " independent NAND bit-knowledge vectors passed\n";
}

void test_erase_program_and_status() {
  Nand nand;
  setup(nand);
  constexpr NandLocation outside{.page = Nand::block_pages * 2, .column = 0};
  nand.observe_cell(outside, {.value = neighbor_cookie, .known = byte_mask});
  begin_erase(nand, (reference_block * Nand::block_pages) + Nand::block_pages - 1);
  require(nand.ready() == false, "Erase confirmation must start a pending operation");
  reject([&] { nand.observe_completion(); }, "writes require an observed outcome");
  nand.command(status_command);
  require(nand.read(ready_bit) == 0, "Status must expose busy before observed completion");
  reject([&] { static_cast<void>(nand.read()); }, "unknown status bits");
  nand.observe_completion(NandOutcome::pass);
  require(nand.read(status_mask) == status_pass,
          "Observed successful erase must expose ready/pass/WP");
  for (unsigned page = reference_block * Nand::block_pages;
       page < (reference_block + 1) * Nand::block_pages;
       ++page) {
    for (unsigned column = 0; column < Nand::page_bytes; ++column) {
      require(nand.cell({.page = page, .column = column}) ==
                  NandByte{.value = byte_mask, .known = byte_mask},
              "Completed erase must establish all32main/spare pages, not only the addressed page");
    }
  }
  require(nand.cell(outside) == NandByte{.value = neighbor_cookie, .known = byte_mask},
          "Block erase must preserve the neighboring block");
  constexpr NandLocation target{.page = reference_page, .column = 0};
  nand.observe_cell(target, {}); // Withdraw cell knowledge without inventing fresh history.
  begin_program(nand, target);
  nand.write(0);
  nand.command(program_confirm);
  require(nand.cell(target).known == 0 && nand.ready() == false,
          "Busy program cells must not be reported as complete");
  nand.command(status_command);
  nand.observe_completion(NandOutcome::pass);
  require(nand.cell(target) == NandByte{.known = byte_mask} &&
              nand.read(status_mask) == status_pass,
          "Programming zero must establish a known zero even over unknown preimage bits");
  nand.command(reset_command);
  nand.observe_completion();
  nand.command(status_command);
  require(nand.read() == status_pass,
          "The explicitly specified post-reset C0 status is qualified with WP high");
  nand.observe_wp(std::nullopt);
  reject([&] { static_cast<void>(nand.read()); }, "unknown status bits");
}

void test_program_ce_pause() {
  for (const auto profile :
       {blofeld::NandReadContinuation::k9f1208_p, blofeld::NandReadContinuation::k9f1208_j}) {
    Nand nand(profile);
    setup(nand);
    begin_erase(nand);
    nand.observe_read_end(blofeld::NandReadEnd::ce_high);
    require(nand.pending() == blofeld::NandPending::erase && nand.ready() == false,
            "CE deselection cannot cancel or finish a busy erase");
    nand.observe_completion(NandOutcome::pass);
    begin_program(nand, {.page = reference_page, .column = 0});
    nand.observe_read_end(blofeld::NandReadEnd::ce_high);
    nand.write(pattern_a);
    nand.observe_read_end(blofeld::NandReadEnd::ce_high);
    nand.write(pattern_b);
    nand.observe_read_end(blofeld::NandReadEnd::ce_high);
    require(nand.column() == 2 && nand.ready() == true,
            "CE pause must retain the data cursor without starting a program");
    nand.command(program_confirm);
    nand.observe_read_end(blofeld::NandReadEnd::ce_high);
    require(nand.pending() == blofeld::NandPending::program && nand.ready() == false,
            "CE deselection cannot cancel or finish a busy program");
    nand.observe_completion(NandOutcome::pass);
    require(nand.cell({.page = reference_page, .column = 0}) ==
                    NandByte{.value = pattern_a, .known = byte_mask} &&
                nand.cell({.page = reference_page, .column = 1}) ==
                    NandByte{.value = pattern_b, .known = byte_mask},
            "Programming after CE pauses must preserve both staged bytes");
  }
}

void test_partial_program_limits() {
  Nand nand;
  setup(nand);
  erased(nand);
  begin_program(nand, {.page = reference_page, .column = 0});
  nand.write(pattern_a);
  nand.command(program_confirm);
  nand.observe_completion(NandOutcome::pass);
  begin_program(nand, {.page = reference_page, .column = 1});
  nand.write(pattern_b);
  reject([&] { nand.command(program_confirm); }, "main partial-program limit");
  require(nand.cell({.page = reference_page, .column = 1}).value == byte_mask,
          "Rejected main reprogram must preserve cells");
  nand.command(reset_command);
  nand.observe_completion();
  constexpr unsigned spare = Nand::main_bytes + 15;
  for (const std::uint8_t value : std::array<std::uint8_t, 2>{0xf0, 0x0f}) {
    begin_program(nand, {.page = reference_page, .column = spare});
    nand.write(value);
    nand.command(program_confirm);
    nand.observe_completion(NandOutcome::pass);
  }
  require(nand.cell({.page = reference_page, .column = spare}) == NandByte{.known = byte_mask},
          "Two spare programs must AND bits instead of restoring programmed zeros to ones");
  begin_program(nand, {.page = reference_page, .column = spare});
  nand.write(byte_mask);
  reject([&] { nand.command(program_confirm); }, "spare partial-program limit");
}

void test_history_buffer_and_pointer() {
  Nand nand;
  setup(nand);
  reject([&] { nand.command(program_confirm); }, "no staged program data");
  reject([&] { nand.command(erase_confirm); }, "no complete erase address");
  nand.observe_cell({.page = reference_page}, {.value = byte_mask, .known = byte_mask});
  begin_program(nand, {.page = reference_page, .column = 0});
  nand.write(payload);
  reject([&] { nand.command(program_confirm); }, "unknown program history");
  require(nand.cell({.page = reference_page}).value == byte_mask && nand.ready() == true,
          "Observed FF data must not invent fresh programming history or launch a rejected job");
  nand.command(reset_command);
  nand.observe_completion();
  erased(nand);
  begin_program(nand, {.page = reference_page, .column = 0});
  for (unsigned index = 0; index < Nand::page_bytes; ++index) {
    nand.write(static_cast<std::uint8_t>(index & byte_mask));
  }
  reject([&] { nand.write(0); }, "program data exceeds page");
  reject([&] { nand.address(0); }, "no qualified read command/address phase");
  reject([&] { nand.command(0); }, "replacement of a staged write sequence");
  nand.command(program_confirm);
  nand.observe_completion(NandOutcome::pass);
  for (unsigned index = 0; index < Nand::page_bytes; ++index) {
    require(nand.cell({.page = reference_page, .column = index}) ==
                NandByte{.value = byte(index), .known = byte_mask},
            "Full main/spare programming must preserve each byte after rejected overflow/address "
            "inputs");
  }
  nand.command(1);
  nand.command(program_command);
  nand.address(0);
  row(nand, reference_page + 1);
  nand.write(pattern_a);
  nand.command(program_confirm);
  nand.observe_completion(NandOutcome::pass);
  nand.command(program_command); // The one-cycle second-half pointer must have returned to A.
  nand.address(0);
  row(nand, reference_page + 2);
  nand.write(pattern_b);
  nand.command(program_confirm);
  nand.observe_completion(NandOutcome::pass);
  constexpr unsigned half = 256;
  require(
      nand.cell({.page = reference_page + 1, .column = half}).value == pattern_a &&
          nand.cell({.page = reference_page + 2}).value == pattern_b &&
          nand.cell({.page = reference_page + 2, .column = half}).value == byte_mask,
      "Second-half selection must affect only one program cycle and preserve unrelated columns");
}

void test_unknown_permissions_and_failure() {
  Nand nand;
  nand.observe_power_ready();
  reject([&] { nand.command(program_command); }, "WP high must be explicitly observed");
  nand.observe_wp(false);
  reject([&] { nand.command(erase_command); }, "WP high must be explicitly observed");
  nand.observe_wp(true);
  nand.command(erase_command);
  row(nand, reference_page);
  reject([&] { nand.command(erase_confirm); }, "unknown/invalid block condition");
  nand.observe_block(reference_block, {.valid = true});
  reject([&] { nand.command(erase_confirm); }, "unknown block protection");
  nand.observe_block(reference_block, {.valid = true, .erase_protected = true});
  reject([&] { nand.command(erase_confirm); }, "protected-block write outcome");
  nand.observe_block(reference_block,
                     {.valid = true, .program_protected = true, .erase_protected = false});
  nand.command(erase_confirm);
  reject([&] { nand.observe_wp(false); }, "WP change during write sequence");
  nand.observe_completion(NandOutcome::pass); // Erase also clears eraseable program protection.
  begin_program(nand, {.page = reference_page, .column = 0});
  nand.write(payload);
  nand.command(program_confirm);
  auto interrupted = nand;
  nand.command(status_command);
  nand.observe_completion(NandOutcome::fail);
  require(nand.read(status_mask) == status_fail && nand.cell({.page = reference_page}).known == 0,
          "Failed program must expose failure and leave cells unqualified");
  interrupted.command(reset_command);
  interrupted.observe_completion();
  interrupted.command(status_command);
  require(interrupted.read() == status_pass &&
              interrupted.cell({.page = reference_page}).known == 0,
          "A cleared reset status must not manufacture success for the interrupted program");
}
}

int main() {
  try {
    test_program_knowledge();
    test_erase_program_and_status();
    test_program_ce_pause();
    test_partial_program_limits();
    test_history_buffer_and_pointer();
    test_unknown_permissions_and_failure();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
