// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::require;
constexpr unsigned operand = 0x407;
constexpr unsigned canary = 0x654321;
constexpr unsigned normal_omr = 0x300;
constexpr unsigned short_mask = 0xffff;
constexpr unsigned settle = 3;
constexpr unsigned address_registers = 8;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned r2 = 2;
constexpr std::array values{0U, 0x7fffffU, 0x800000U, 0xffffffU};
constexpr std::array flag_patterns{0x55U, 0xaaU};
unsigned windows = 0;
unsigned successes = 0;
unsigned refusals = 0;
unsigned restores = 0;
unsigned clones = 0;
unsigned retry_restores = 0;
unsigned retry_clones = 0;
unsigned read_prefixes = 0;

struct Input {
  unsigned index = 0;
  unsigned value = 0;
  unsigned flags = 0;
  bool n = false;
  bool y = false;
  bool compatibility = false;
  bool load = false;
};

std::string name(const Input sample) {
  return std::string(sample.n ? "n" : "r") + std::to_string(sample.index);
}

unsigned status(const bool compatibility, const unsigned flags) {
  return test::masked_status | flags | (compatibility ? test::compatibility : 0U);
}

void seed(test::DspProgram &code, const Input sample) {
  code.immediate("omr", normal_omr);
  code.immediate("sz", 0);
  for (unsigned i = 0; i < address_registers; ++i) {
    code.immediate("r" + std::to_string(i), canary + i);
    code.immediate("n" + std::to_string(i), i + 1);
    code.immediate("m" + std::to_string(i), word_mask);
  }
  code.immediate("x0", canary);
  code.immediate("x1", canary);
  code.immediate(name(sample), sample.load ? canary : sample.value);
}

void nops(test::DspProgram &code, const unsigned count) {
  for (unsigned i = 0; i < count; ++i) { code.word(0); }
}

void install(DspMachine &machine, const test::DspProgram &code, const Input sample) {
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = DspSpace::x, .address = operand, .value = canary});
  machine.bus().write({.space = DspSpace::y, .address = operand, .value = canary});
  if (sample.load) {
    machine.bus().write(
        {.space = sample.y ? DspSpace::y : DspSpace::x, .address = operand, .value = sample.value});
  }
}

void emit(test::DspProgram &code, const Input sample) {
  if (sample.load) {
    code.emit(std::string("move ") + (sample.y ? "y:>" : "x:>") + test::number(operand) + ',' +
              name(sample));
  } else {
    code.emit("move " + name(sample) + ",x0");
  }
}

DspDebugState expected(const DspDebugState &before, const Input sample) {
  auto result = before;
  const unsigned value = sample.value & (sample.compatibility ? short_mask : word_mask);
  if (sample.load) {
    (sample.n ? result.cpu.n : result.cpu.r).at(sample.index) = value;
  } else {
    result.cpu.x = (result.cpu.x & ~std::uint64_t{word_mask}) | value;
  }
  const unsigned words = sample.load ? 2U : 1U;
  result.cpu.pc += words;
  ++result.native_instructions;
  result.native_cycles += words;
  return result;
}

void cells(DspMachine &machine, const Input sample) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    const auto value = sample.load && (space == DspSpace::y) == sample.y ? sample.value : canary;
    require(machine.bus().read({.space = space, .address = operand}) == value,
            "Scalar AGU MOVE must preserve both actual source cells");
  }
}

void stable(const Input sample) {
  DspMachine machine;
  test::DspProgram code;
  seed(code, sample);
  code.immediate("sr", status(sample.compatibility, sample.flags));
  nops(code, settle);
  const auto target = code.pc();
  emit(code, sample);
  install(machine, code, sample);
  test::checkpoint::until(machine, target);
  const auto before = machine.cpu().debug_state();
  require(before.cpu.sr == status(sample.compatibility, sample.flags),
          "Three actual NOPs establish the requested SC and literal flags");
  if (!sample.load) {
    require((sample.n ? before.cpu.n : before.cpu.r).at(sample.index) == sample.value,
            "Changing SC does not silently clear pre-existing AGU register contents");
  }
  machine.cpu().step();
  require(machine.cpu().debug_state() == expected(before, sample),
          "Settled native scalar AGU MOVE follows literal 24/16-bit and whole-state expectations");
  cells(machine, sample);
  ++windows;
}

void destinations(Input sample) {
  stable(sample);
  sample.load = true;
  stable(sample);
  sample.y = true;
  stable(sample);
}

void matrix() {
  for (unsigned i = 0; i < address_registers; ++i) {
    for (const bool n : {false, true}) {
      for (const bool compatibility : {false, true}) {
        for (const auto value : values) {
          for (const auto flags : flag_patterns) {
            destinations({.index = i,
                          .value = value,
                          .flags = flags,
                          .n = n,
                          .compatibility = compatibility});
          }
        }
      }
    }
  }
}

struct Continuation {
  unsigned target;
  unsigned gap;
};

void suffix(DspMachine &machine, const Input sample, const Continuation continuation) {
  const auto target = continuation.target;
  const auto gap = continuation.gap;
  test::checkpoint::until(machine, target);
  const auto before = machine.cpu().debug_state();
  unsigned reads = 0;
  const DspNativeAccessObserver observer{.observe = [&](const DspNativeAccess &access) {
    if (access.kind == DspNativeAccessKind::read && access.stage == DspNativeAccessStage::after) {
      require(access.target.space == (sample.y ? DspSpace::y : DspSpace::x) &&
                  access.target.address == operand && access.value == sample.value &&
                  access.pc == target && access.omr == normal_omr,
              "Native SC transition retains its real completed source-read prefix");
      ++reads;
    }
  }};
  if (gap == settle) {
    machine.cpu().step(observer);
    require(machine.cpu().debug_state() == expected(before, sample),
            "Restored settled SC history retains literal native register,flag and counter effects");
    ++successes;
  } else {
    const auto fault = test::checkpoint::rejects([&] { machine.cpu().step(observer); },
                                                 "Unqualified SC transition must not succeed");
    const auto after = machine.cpu().debug_state();
    require(fault.contains("Unqualified native scalar AGU MOVE during SC transition") &&
                fault.contains("pc=" + hex(target, 6)) &&
                fault.contains("omr=" + hex(normal_omr, 6)) &&
                after.native_instructions == before.native_instructions &&
                after.native_cycles == before.native_cycles &&
                after.cpu.r.at(r2) == before.cpu.r.at(r2),
            "Unqualified transition reports context without publishing the AGU destination or "
            "retirement");
    ++refusals;
  }
  require(reads == 1,
          "One genuine source read survives both settled success and transition refusal");
  read_prefixes += reads;
  cells(machine, sample);
}

void transition(const bool compatibility, const unsigned gap) {
  const Input sample{.index = r2,
                     .value = values.at(1),
                     .flags = flag_patterns.at(0),
                     .compatibility = compatibility,
                     .load = true};
  DspMachine machine;
  test::DspProgram code;
  seed(code, sample);
  code.immediate("sr", status(!compatibility, sample.flags));
  nops(code, settle);
  const auto change = code.pc();
  code.immediate("sr", status(compatibility, sample.flags));
  nops(code, gap);
  const auto target = code.pc();
  emit(code, sample);
  install(machine, code, sample);
  test::checkpoint::until(machine, change);
  std::vector<DspCheckpoint> cuts;
  while (machine.cpu().state().pc != target) {
    cuts.push_back(machine.checkpoint());
    machine.cpu().step();
  }
  cuts.push_back(machine.checkpoint());
  suffix(machine, sample, {.target = target, .gap = gap});
  for (const auto &cut : cuts) {
    machine.restore(cut);
    suffix(machine, sample, {.target = target, .gap = gap});
    ++restores;
    DspMachine clone;
    clone.restore(cut);
    suffix(clone, sample, {.target = target, .gap = gap});
    ++clones;
  }
}

void finish_pending(DspMachine &machine) {
  constexpr unsigned bound = 16;
  for (unsigned i = 0; i < bound; ++i) {
    const auto pending = machine.reference_pending();
    if (!pending) { throw Fault("Reference continuation lost its genuine pending effect"); }
    const auto progress = machine.advance_reference_access(pending->ticket);
    if (progress.retired) { return; }
  }
  throw Fault("Scalar AGU reference instruction exceeded its effect bound");
}

void reference_instruction(DspMachine &machine) {
  const auto progress =
      machine.begin_reference_instruction(DspReferenceExecution::access_and_retirement_boundaries);
  if (!progress.retired) { finish_pending(machine); }
}

void retry_tail(DspMachine &machine, const DspDebugState &wanted, const Input sample) {
  for (unsigned i = 0; i < settle + 1; ++i) { reference_instruction(machine); }
  require(
      machine.cpu().debug_state() == wanted,
      "Real reference retries and restored pending SC writes retain literal final native state");
  cells(machine, sample);
}

void retries() {
  const Input sample{.index = r2,
                     .value = values.at(1),
                     .flags = flag_patterns.at(0),
                     .compatibility = true,
                     .load = true};
  DspMachine machine;
  test::DspProgram code;
  seed(code, sample);
  code.immediate("sr", status(false, sample.flags));
  const auto change = code.pc();
  code.immediate("sr", status(true, sample.flags));
  nops(code, settle);
  const auto target = code.pc();
  emit(code, sample);
  install(machine, code, sample);
  test::checkpoint::until(machine, change);
  auto wanted = machine.cpu().debug_state();
  wanted.cpu.pc = target + 2;
  wanted.cpu.sr = status(true, sample.flags);
  wanted.cpu.r.at(r2) = sample.value & short_mask;
  constexpr unsigned instructions = 5;
  constexpr unsigned cycles = 7;
  wanted.native_instructions += instructions;
  wanted.native_cycles += cycles;
  auto progress =
      machine.begin_reference_instruction(DspReferenceExecution::access_and_retirement_boundaries);
  std::vector<DspCheckpoint> cuts;
  while (!progress.retired) {
    const auto pending = progress.pending;
    if (!pending) { throw Fault("SC write lost its pending native access/retirement"); }
    cuts.push_back(machine.checkpoint());
    progress = machine.advance_reference_access(pending->ticket);
  }
  retry_tail(machine, wanted, sample);
  for (const auto &cut : cuts) {
    machine.restore(cut);
    finish_pending(machine);
    retry_tail(machine, wanted, sample);
    ++retry_restores;
    DspMachine clone;
    clone.restore(cut);
    finish_pending(clone);
    retry_tail(clone, wanted, sample);
    ++retry_clones;
  }
}

}

int main() {
  try {
    matrix();
    for (const bool compatibility : {false, true}) {
      for (unsigned gap = 0; gap <= settle; ++gap) { transition(compatibility, gap); }
    }
    retries();
    std::cout << "PASS SC_MOVES windows=" << windows << " successes=" << successes
              << " refusals=" << refusals << " restores=" << restores << " clones=" << clones
              << " retry-restores=" << retry_restores << " retry-clones=" << retry_clones
              << " read-prefixes=" << read_prefixes << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "SC_MOVES windows=" << windows << " successes=" << successes
              << " refusals=" << refusals << ": " << error.what() << '\n';
    return 1;
  }
}
