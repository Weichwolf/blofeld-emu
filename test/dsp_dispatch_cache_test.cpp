// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_dispatch_cache.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56kEmu/assembler.h"
#include "dsp_norm_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
using blofeld::test::require;
constexpr unsigned boot = 0xff0000;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned branch_target = 0x100;
constexpr unsigned boundary = 0x3ff;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned first_value = 0x123456;
constexpr unsigned second_value = 0x234567;
constexpr unsigned word_bits = 24;
constexpr std::size_t maximum_storage_bytes = std::size_t{1024} * 1024;

struct Target {
  unsigned value = 0;

  void resolve(const unsigned input) { value = input; }

  void move(const unsigned input) { value = input + 1; }

  void alu(const unsigned input) { value = input + 2; }
};

using Function = void (Target::*)(unsigned);

struct Entry {
  Function op = nullptr;
  Function move = nullptr;
  Function alu = nullptr;
  bool operator==(const Entry &) const = default;
};

using Cache = blofeld::DspDispatchCache<Entry>;
constexpr Entry cold{.op = &Target::resolve};
constexpr Entry warm{.op = &Target::alu, .move = &Target::move, .alu = &Target::resolve};

template <typename Error> void rejects(const auto &operation) {
  try {
    operation();
  } catch (const Error &) { return; }
  throw std::runtime_error("Expected cache storage rejection");
}

void sparse_storage() {
  Cache cache;
  cache.reset(Cache::maximum_words, cold);
  require(cache.size() == Cache::maximum_words && cache.resident_pages() == 0 &&
              cache.storage_bytes() < maximum_storage_bytes,
          "The complete 24-bit dispatch space starts with a bounded empty directory");
  const auto empty_bytes = cache.storage_bytes();
  for (std::size_t at = 0; at < Cache::maximum_words; at += Cache::page_words) {
    require(cache.find(at) == nullptr, "Cold invalidation must never populate a page");
  }
  require(cache.storage_bytes() == empty_bytes, "Cold probes have no allocation effects");
  cache[0] = warm;
  auto *saved = &cache.at(0);
  for (const std::size_t at :
       {Cache::page_words - 1, Cache::page_words, std::size_t{boot}, std::size_t{word_mask}}) {
    require(cache.at(at) == cold, "New pages retain all three exact initial member pointers");
  }
  require(cache.resident_pages() == 4 && cache.storage_bytes() < maximum_storage_bytes &&
              saved == &cache.at(0) && *saved == warm,
          "Low/high/boundary pages preserve sparse storage and existing entry references");
  Target target;
  (target.*saved->op)(first_value);
  require(target.value == first_value + 2, "Stored host dispatch pointers retain their identity");
  rejects<std::out_of_range>([&] { static_cast<void>(cache.at(Cache::maximum_words)); });
  rejects<std::length_error>([&] { cache.reset(Cache::maximum_words + 1, cold); });
  require(cache.size() == Cache::maximum_words && cache.resident_pages() == 4 &&
              cache.at(0) == warm && cache.find(Cache::maximum_words) == nullptr,
          "Rejected sizes/addresses preserve the existing cache");
  cache.reset(Cache::page_words + 1, cold);
  require(cache.resident_pages() == 0 && cache.at(Cache::page_words) == cold &&
              cache.resident_pages() == 1 && cache.find(0) == nullptr,
          "Reset discards old pages; a partial last page retains its logical bound");
  rejects<std::out_of_range>([&] { static_cast<void>(cache.at(Cache::page_words + 1)); });
  cache.reset(0, cold);
  require(cache.size() == 0 && cache.resident_pages() == 0 && cache.storage_bytes() == 0,
          "An empty reset releases both directory and resident entries");
}

std::vector<std::uint32_t> assemble(const char *instruction) {
  const dsp56k::Assembler assembler;
  const auto encoded = assembler.assemble(instruction);
  require(encoded.success(), "Own decoder-boundary diagnostic assembles");
  std::vector<std::uint32_t> words;
  words.reserve(encoded.wordCount);
  for (unsigned i = 0; i < encoded.wordCount; ++i) { words.push_back(encoded.word[i]); }
  return words;
}

void last_program_address() {
  blofeld::DspMachine machine;
  auto &bus = machine.bus();
  blofeld::test::DspProgram start;
  start.immediate("r0", branch_target);
  start.word(absolute_jump);
  start.word(word_mask);
  start.install(bus);
  const auto indirect = assemble("jmp (r0)");
  require(indirect.size() == 1, "Last-address jump uses no invented extension word");
  bus.install_rom({.start = word_mask, .words = indirect});
  machine.cpu().step();
  machine.cpu().step();
  require(machine.cpu().state().pc == word_mask,
          "The native interpreter reaches the last actual 24-bit P address");
  machine.cpu().step();
  require(machine.cpu().state().pc == branch_target && machine.cpu().state().r[0] == branch_target,
          "The highest decoder page executes its real indirect jump");
}

void boundary_mutation_and_restore() {
  blofeld::DspMachine machine;
  auto &bus = machine.bus();
  blofeld::test::DspProgram start;
  start.word(absolute_jump);
  start.word(boundary);
  start.install(bus);
  const auto move = assemble("move #>$123456,x0");
  require(move.size() == 2, "Own immediate crosses an actual decoder page boundary");
  bus.write({.address = boundary, .value = move[0]});
  bus.write({.address = boundary + 1, .value = move[1]});
  bus.write({.address = boundary + 2, .value = absolute_jump});
  bus.write({.address = boundary + 3, .value = boundary});
  machine.cpu().step();
  machine.cpu().step();
  const auto before = machine.cpu().debug_state();
  const auto saved = machine.checkpoint();
  require((before.cpu.x & word_mask) == first_value && before.cpu.pc == boundary + 2,
          "The initial cross-page operand executes with independent register expectations");
  bus.write({.address = boundary + 1, .value = second_value});
  machine.cpu().step();
  machine.cpu().step();
  require((machine.cpu().state().x & word_mask) == second_value,
          "A changed extension across pages cannot reuse a stale operand");
  const auto replacement = assemble("move #>$234567,x1");
  bus.write({.address = boundary, .value = replacement[0]});
  machine.cpu().step();
  machine.cpu().step();
  require(machine.cpu().state().x == (std::uint64_t{second_value} << word_bits) + second_value,
          "A changed decoded destination retains the earlier X0 and updates only X1");
  machine.restore(saved);
  require(machine.cpu().debug_state() == before,
          "Host cache replacement does not alter restored architectural/debug state");
  machine.cpu().step();
  machine.cpu().step();
  require(machine.cpu().state().x == first_value,
          "Trusted restore rebuilds dispatch from the restored program and extension");
}
}

int main() {
  try {
    sparse_storage();
    last_program_address();
    boundary_mutation_and_restore();
    std::cout
        << "Sparse host dispatch retains 24-bit P access, page-crossing mutation and restore.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
