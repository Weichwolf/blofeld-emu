// SPDX-License-Identifier: GPL-3.0-only
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dispatch_cache.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <tuple>

namespace {
using blofeld::test::esai::require;
namespace native = blofeld::test::dma_test;
constexpr unsigned boot = 0xff0000;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned branch_target = 0x100;
constexpr unsigned boundary = 0x3ff;
constexpr unsigned absolute_jump = 0x0af080;
// FM13 JMP (Rn), MOVE #imm,R0 and MOVE #imm,X0/X1; literal authored instructions.
constexpr unsigned indirect_jump = 0x0ae080, immediate_r0 = 0x60f400;
constexpr unsigned immediate_x0 = 0x44f400, immediate_x1 = 0x45f400;
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

auto observe(const blofeld::OwnDspMachine &machine) {
  return std::tuple{machine.cpu().registers(),
                    machine.cpu().progress(),
                    machine.cpu().fault(),
                    machine.cpu().processing_state(),
                    machine.clock().elapsed_ticks()};
}

void last_program_address() {
  blofeld::OwnDspMachine machine(native::native_clocks);
  auto &bus = machine.bus();
  native::NativeProgram start;
  start.word(immediate_r0);
  start.word(branch_target);
  start.word(absolute_jump);
  start.word(word_mask);
  native::reset_native(machine, start);
  const std::array indirect{indirect_jump};
  bus.install_rom({.start = word_mask, .words = indirect});
  native::step_native(machine);
  native::step_native(machine);
  require(machine.cpu().registers().pc == word_mask,
          "The native interpreter reaches the last actual 24-bit P address");
  const auto before = observe(machine);
  auto expected = before;
  std::get<0>(expected).pc = branch_target;
  ++std::get<1>(expected).instructions;
  constexpr unsigned indirect_jump_units = 3; // FM13 JMP EA.
  std::get<1>(expected).estimated_cycles += indirect_jump_units;
  native::step_native(machine);
  require(observe(machine) == expected && machine.cpu().registers().r[0] == branch_target,
          "The highest decoder page executes its real indirect jump");
}

void boundary_mutation_and_restore() {
  blofeld::OwnDspMachine machine(native::native_clocks);
  auto &bus = machine.bus();
  native::NativeProgram start;
  start.word(absolute_jump);
  start.word(boundary);
  native::reset_native(machine, start);
  bus.write({.address = boundary, .value = immediate_x0});
  bus.write({.address = boundary + 1, .value = first_value});
  bus.write({.address = boundary + 2, .value = absolute_jump});
  bus.write({.address = boundary + 3, .value = boundary});
  native::step_native(machine);
  native::step_native(machine);
  const auto before = observe(machine);
  const auto saved = machine.checkpoint();
  require(std::get<0>(before).x == first_value && std::get<0>(before).pc == boundary + 2,
          "The initial cross-page operand executes with independent register expectations");
  bus.write({.address = boundary + 1, .value = second_value});
  native::step_native(machine);
  native::step_native(machine);
  require(machine.cpu().registers().x == second_value,
          "A changed extension across pages cannot reuse a stale operand");
  bus.write({.address = boundary, .value = immediate_x1});
  native::step_native(machine);
  native::step_native(machine);
  require(machine.cpu().registers().x == (std::uint64_t{second_value} << word_bits) + second_value,
          "A changed decoded destination retains the earlier X0 and updates only X1");
  machine.restore(saved);
  require(observe(machine) == before,
          "Host cache replacement does not alter restored architectural/debug state");
  native::step_native(machine);
  native::step_native(machine);
  require(machine.cpu().registers().x == first_value,
          "Trusted restore rebuilds dispatch from the restored program and extension");
  const auto suffix = observe(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    blofeld::OwnDspMachine branch(native::native_clocks);
    branch.restore(saved);
    require(observe(branch) == before, "Fresh owners retain the complete native source state");
    native::step_native(branch);
    native::step_native(branch);
    require(
        observe(branch) == suffix,
        "Both fresh suffixes fetch restored words with identical registers, flags and progress");
  }
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
