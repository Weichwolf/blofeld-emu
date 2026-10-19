// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/nand.hpp"
#include "blofeld/nand_array.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using blofeld::Nand;
using blofeld::NandByte;
using blofeld::NandOutcome;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned block = 1;
constexpr unsigned page = (block * Nand::block_pages) + 5;
constexpr unsigned outside = (block + 1) * Nand::block_pages;
constexpr unsigned neighbor_cookie = 0x42;
constexpr unsigned old_cookie = 0x91;
constexpr unsigned reset_command = 0xff;
constexpr unsigned program_command = 0x80;
constexpr unsigned program_confirm = 0x10;
constexpr unsigned erase_command = 0x60;
constexpr unsigned erase_confirm = 0xd0;
constexpr unsigned program_prefix = 6;
constexpr unsigned confirm_index = program_prefix + Nand::page_bytes;
constexpr unsigned program_actions = confirm_index + 2;
constexpr unsigned erase_actions = 6;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::uint8_t byte(const unsigned value) {
  return static_cast<std::uint8_t>(value & byte_mask);
}

std::uint8_t payload(const unsigned index) {
  constexpr unsigned multiplier = 73;
  return byte((index * multiplier) ^ (index >> 8U));
}

void setup(Nand &nand) {
  nand.observe_power_ready();
  nand.observe_wp(true);
  nand.observe_block(block, {.valid = true, .program_protected = false, .erase_protected = false});
  nand.observe_cell({.page = outside}, {.value = neighbor_cookie, .known = byte_mask});
}

void erase_step(Nand &nand, const unsigned step) {
  switch (step) {
    case 0: nand.command(erase_command); break;
    case 1: nand.address(byte(page)); break;
    case 2:
    case 3: nand.address(0); break;
    case 4: nand.command(erase_confirm); break;
    case 5: nand.observe_completion(NandOutcome::pass); break;
    default: throw blofeld::Fault("Own NAND erase reference step exceeds its bound");
  }
}

void program_step(Nand &nand, const unsigned step) {
  switch (step) {
    case 0: nand.command(0); break;
    case 1: nand.command(program_command); break;
    case 2: nand.address(0); break;
    case 3: nand.address(byte(page)); break;
    case 4:
    case 5: nand.address(0); break;
    default:
      if (step < confirm_index) {
        nand.write(payload(step - program_prefix));
      } else if (step == confirm_index) {
        nand.command(program_confirm);
      } else if (step == confirm_index + 1) {
        nand.observe_completion(NandOutcome::pass);
      } else {
        throw blofeld::Fault("Own NAND program reference step exceeds its bound");
      }
  }
}

void interrupt(Nand &nand, const bool power) {
  if (power) {
    nand.power_loss();
    require(!nand.ready().has_value(),
            "Power loss must withdraw readiness rather than infer reset completion");
    nand.observe_power_ready();
  } else {
    nand.command(reset_command);
    require(nand.ready() == false, "Reset requires its own explicit completion event");
    nand.observe_completion();
  }
}

void check_program(const Nand &nand, const unsigned prefix) {
  for (unsigned index = 0; index < Nand::page_bytes; ++index) {
    NandByte expected{.value = byte_mask, .known = byte_mask};
    if (prefix == confirm_index + 1) { expected = {}; }
    if (prefix == program_actions) { expected.value = payload(index); }
    require(nand.cell({.page = page, .column = index}) == expected,
            "Program cut must preserve staging, invalidate active page or retain completed data");
    for (const unsigned adjacent : {page - 1, page + 1}) {
      require(nand.cell({.page = adjacent, .column = index}) ==
                  NandByte{.value = byte_mask, .known = byte_mask},
              "Interrupted program must not invalidate neighboring pages");
    }
  }
  require(nand.cell({.page = outside}) == NandByte{.value = neighbor_cookie, .known = byte_mask},
          "Program interruption must preserve the neighboring block");
}

void retry_after_cut(Nand &nand, const unsigned prefix) {
  nand.observe_wp(true); // New explicit observation after power-up, not a reset default.
  for (unsigned step = 0; step < program_prefix; ++step) { program_step(nand, step); }
  nand.write(payload(0));
  if (prefix < confirm_index + 1) {
    nand.command(program_confirm);
    nand.observe_completion(NandOutcome::pass);
    require(nand.cell({.page = page}) == NandByte{.value = payload(0), .known = byte_mask},
            "Discarded staging must not consume a partial-program allowance");
    return;
  }
  bool rejected = false;
  try {
    nand.command(program_confirm);
  } catch (const blofeld::Fault &fault) {
    const std::string message = fault.what();
    require(
        message.contains(prefix == program_actions ? "main partial-program limit"
                                                   : "unknown/invalid block condition"),
        "Post-cut program rejection must distinguish completed history from unknown active state");
    rejected = true;
  }
  require(rejected, "A cut must not erase completed history or qualify an interrupted active page");
}

void test_all_program_cuts() {
  Nand base;
  setup(base);
  for (unsigned step = 0; step < erase_actions; ++step) { erase_step(base, step); }
  unsigned cases = 0;
  for (const bool power : {false, true}) {
    for (unsigned prefix = 0; prefix <= program_actions; ++prefix) {
      auto branch = base;
      for (unsigned step = 0; step < prefix; ++step) { program_step(branch, step); }
      auto independent = branch;
      interrupt(branch, power);
      check_program(branch, prefix);
      check_program(independent, prefix); // Independent branch retains pre-cut reference state.
      if (prefix == confirm_index + 1) {
        independent.observe_completion(NandOutcome::pass);
        check_program(independent, program_actions);
        check_program(branch, prefix);
      }
      retry_after_cut(branch, prefix);
      ++cases;
    }
  }
  std::cout << cases
            << " NAND reset/power cuts across every program address/data/confirm/completion "
               "boundary passed\n";
}

void check_erase(const Nand &nand, const unsigned prefix) {
  for (unsigned index = 0; index < Nand::block_pages; ++index) {
    for (unsigned column = 0; column < Nand::page_bytes; ++column) {
      NandByte expected{.value = old_cookie, .known = byte_mask};
      if (prefix == erase_actions - 1) { expected = {}; }
      if (prefix == erase_actions) { expected.value = byte_mask; }
      require(nand.cell({.page = (block * Nand::block_pages) + index, .column = column}) ==
                  expected,
              "Erase cut must preserve pre-confirm cells, invalidate the whole active block or "
              "retain erased cells");
    }
  }
  require(nand.cell({.page = outside}) == NandByte{.value = neighbor_cookie, .known = byte_mask},
          "Erase interruption must preserve the neighboring block");
}

void test_all_erase_cuts_and_failed_completion() {
  Nand base;
  setup(base);
  std::array<std::uint8_t, Nand::page_bytes> contents{};
  contents.fill(old_cookie);
  for (unsigned index = 0; index < Nand::block_pages; ++index) {
    base.observe_bytes((block * Nand::block_pages) + index, 0, contents);
  }
  unsigned cases = 0;
  for (const bool power : {false, true}) {
    for (unsigned prefix = 0; prefix <= erase_actions; ++prefix) {
      auto branch = base;
      for (unsigned step = 0; step < prefix; ++step) { erase_step(branch, step); }
      interrupt(branch, power);
      check_erase(branch, prefix);
      ++cases;
    }
  }
  for (unsigned step = 0; step < erase_actions - 1; ++step) { erase_step(base, step); }
  base.observe_completion(NandOutcome::fail);
  check_erase(base, erase_actions - 1);
  std::cout << cases << " NAND erase reset/power cuts and failed erase completion passed\n";
}
}

int main() {
  try {
    test_all_program_cuts();
    test_all_erase_cuts_and_failed_completion();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
