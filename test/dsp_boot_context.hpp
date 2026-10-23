// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_reference_execution.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <string_view>

namespace blofeld::test::boot_context {
enum class Phase : std::uint8_t { reset, loaded, clear, frontier };
inline constexpr unsigned word_mask = 0xffffff;
inline constexpr unsigned bootstrap_entry = 0xff0000;
inline constexpr unsigned clear_pc = 0x72;
inline constexpr unsigned word_bits = 24;
// FM Figure 5-4/Table 5-1 explicitly set CP1/CP0 as well as I1/I0.
inline constexpr unsigned reset_sr = 0xc00300;
inline constexpr unsigned clear_sr = 0xc00314;
inline constexpr unsigned stage = 0x6a;
inline constexpr unsigned stage_words = 94;
inline constexpr unsigned memory_mode = 0x600389;
inline constexpr unsigned clear_count = 0x20000;
inline constexpr unsigned ram_end = 0x2000;

inline void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

// DSP56300FM 4.3.4, 5.4.1.2, 5.4.3 and CLR/MOVE/REP definitions; the
// document-derived SPI prefix never writes Mn or the upper SR mode bytes.
// Undefined reset data/offset/address registers are deliberately not an oracle.
inline bool valid(const Phase phase, const DspDebugState &state) {
  const auto &cpu = state.cpu;
  if (cpu.sp != 0 || state.sc != 0 || cpu.vba != 0 ||
      !std::ranges::all_of(cpu.m, [](const auto value) { return value == word_mask; })) {
    return false;
  }
  switch (phase) {
    case Phase::reset: return cpu.pc == bootstrap_entry && cpu.omr == 5 && cpu.sr == reset_sr;
    case Phase::loaded:
      return cpu.pc == stage && cpu.omr == 5 && cpu.sr == reset_sr && cpu.a == stage_words &&
             cpu.r[0] == stage + stage_words && cpu.r[1] == stage && cpu.r[5] == 0;
    case Phase::clear:
    case Phase::frontier: {
      const auto address = phase == Phase::clear ? 0U : ram_end;
      return cpu.pc == clear_pc && cpu.omr == memory_mode && cpu.sr == clear_sr && cpu.a == 0 &&
             cpu.r[4] == address && cpu.lc == clear_count - address &&
             (cpu.x & word_mask) == clear_count && (cpu.x >> word_bits) == memory_mode;
    }
    default: return false;
  }
}

inline void negative_controls(const Phase phase, const DspDebugState &state) {
  auto changed = state;
  changed.cpu.m[4] = ram_end - 1;
  require(!valid(phase, changed), "A guessed modulo M4 must fail the boot-context oracle");
  constexpr std::array changed_status_bits{0x20000U, 0x2000U, 0x400U, 0x800U};
  for (const auto mask : changed_status_bits) {
    changed = state;
    changed.cpu.sr ^= mask;
    require(!valid(phase, changed), "Arithmetic/compatibility/scaling mode changes must fail");
  }
  if (phase == Phase::clear || phase == Phase::frontier) {
    changed = state;
    ++changed.cpu.r[4];
    require(!valid(phase, changed), "A changed clear cursor must fail the independent oracle");
    changed = state;
    changed.cpu.lc = 2;
    require(!valid(phase, changed), "An integer-two REP count must fail the independent oracle");
    changed = state;
    changed.cpu.x = (changed.cpu.x & ~static_cast<std::uint64_t>(word_mask)) | 2U;
    require(!valid(phase, changed), "Integer reinterpretation of short X0 must fail");
    changed = state;
    changed.cpu.a = 1;
    require(!valid(phase, changed), "A nonzero low accumulator half must fail");
    changed = state;
    changed.cpu.omr ^= 0x80U;
    require(!valid(phase, changed), "Changing the original memory mode must fail");
  }
}

inline void record(const Phase phase, const DspDebugState &state, std::ostream &trace) {
  const auto &cpu = state.cpu;
  constexpr std::array names{"reset", "loaded-stage", "clear-entry", "strict-frontier"};
  trace << names.at(static_cast<std::size_t>(phase)) << " pc=" << hex(cpu.pc, 6)
        << " sr=" << hex(cpu.sr, 6) << " omr=" << hex(cpu.omr, 6) << " sp=" << hex(cpu.sp, 6)
        << " sc=" << hex(state.sc, 6) << " vba=" << hex(cpu.vba, 6) << " a=" << hex(cpu.a, 14)
        << " b=" << hex(cpu.b, 14) << " x=" << hex(cpu.x, 12) << " y=" << hex(cpu.y, 12)
        << " lc=" << hex(cpu.lc, 6) << " la=" << hex(cpu.la, 6) << " ep=" << hex(state.ep, 6)
        << " sz=" << hex(state.sz, 6) << " native-instructions=" << state.native_instructions
        << " native-cycles=" << state.native_cycles << '\n';
  for (std::size_t index = 0; index < cpu.m.size(); ++index) {
    trace << " r" << index << '=' << hex(cpu.r[index], 6) << " n" << index << '='
          << hex(cpu.n[index], 6) << " m" << index << '=' << hex(cpu.m[index], 6) << '\n';
  }
  require(valid(phase, state), "Native boot context differs from documented register expectations");
  negative_controls(phase, state);
}

inline std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected a visible original bootstrap rejection");
}

inline DspReferenceTicket pending_ticket(const DspReferenceProgress &progress) {
  if (!progress.pending || progress.retired) { throw Fault("Original REP lost its pending store"); }
  return progress.pending->ticket;
}

// These are explicitly supplied reference switch cycles, not elapsed chip time.
// They change only the saved branch's bus map, never its CPU/register state.
inline void check_switch(const DspMachine &frontier, std::ostream &trace) {
  const auto saved = frontier.checkpoint();
  const auto state = frontier.cpu().debug_state();
  DspMachine branch;
  constexpr unsigned switched_program = 0x1000;
  constexpr unsigned switch_cycles = 3;
  for (const auto elapsed : {0U, 2U, switch_cycles}) {
    branch.restore(saved);
    branch.bus().advance_instruction_cycles(elapsed);
    const DspMemoryAccess target{.target = {.address = switched_program},
                                 .operation = DspMemoryOperation::write};
    if (elapsed < switch_cycles) {
      const auto error = rejected([&] { static_cast<void>(branch.bus().memory_resource(target)); });
      require(error.find("requires elapsed switch cycles: P:" + hex(switched_program, 6)) !=
                  std::string::npos,
              "The affected program window must still reject before the switch deadline");
    } else {
      const auto resource = branch.bus().memory_resource(target);
      require(resource.bank == DspSpace::x && !resource.rom && resource.word == 0x10000,
              "The supplied three-cycle boundary really commits the documented X-to-P switch");
    }
    require(branch.cpu().debug_state() == state,
            "Advancing reference map time cannot replace a CPU instruction or register");
    auto progress = branch.begin_reference_instruction(DspReferenceExecution::access_boundaries);
    require(progress.pending && !progress.retired, "The original REP has a pending X store");
    progress = branch.advance_reference_access(pending_ticket(progress));
    require(progress.pending && !progress.retired,
            "The original X store completes before the pending Y store");
    require(branch.cpu().debug_state() == state &&
                branch.bus().read({.space = DspSpace::x, .address = ram_end}) == 0,
            "Only the actual X zero store commits before the reserved Y access");
    const auto error = rejected(
        [&] { static_cast<void>(branch.advance_reference_access(pending_ticket(progress))); });
    require(
        error.find("write at Y:" + hex(ram_end, 6)) != std::string::npos &&
            error.find("pc=" + hex(clear_pc, 6)) != std::string::npos &&
            error.find("omr=" + hex(memory_mode, 6)) != std::string::npos &&
            branch.cpu().debug_state() == state,
        "Original reserved-Y failure persists before and after the actual reference map switch");
    trace << "switch-reference-cycles=" << elapsed << " error=" << error << '\n';
  }
  require(frontier.cpu().debug_state() == state,
          "Boot-context branches leave the original frontier owner unchanged");
}
}
