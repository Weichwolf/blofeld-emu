// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_core_fixture.hpp"
#include "dsp_rom_program.hpp"
#include "firmware/firmware.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace {
void require(const bool condition, const std::string &reason) {
  if (!condition) { throw blofeld::Fault(reason); }
}

auto native_state(const blofeld::DspMachine &machine) {
  const auto &cpu = machine.cpu();
  return std::tuple{
      cpu.registers(), cpu.progress(), cpu.fault(), cpu.processing_state(), cpu.debug_status()};
}
using enum blofeld::DspSpace;
constexpr unsigned entry = 0x2e00;
constexpr unsigned alias_offset = 0xf000, memory_mode = 0x600389;
constexpr unsigned marker_address = 0x14477;
constexpr unsigned word_mask = 0xffffff, absolute_call = 0x0bf080;
constexpr unsigned source_cookie = 0x789, cursor_cookie = 0x456;
constexpr unsigned priority_register = 0xfffffe, priority_value = 0x100;
constexpr unsigned instruction_allowance = 128;
constexpr unsigned post_identification_sr = 0x300;

struct Layout {
  unsigned helper_words, helper_block, payload_block, base, first, count;
  unsigned start, decision, accepted, forwarding;
  bool secondary;
};

constexpr Layout main_layout{.helper_words = 400,
                             .helper_block = 5,
                             .payload_block = 4,
                             .base = 0xd000,
                             .first = 0,
                             .count = 0x7473,
                             .start = entry,
                             .decision = 0x2e16,
                             .accepted = 0x2e19,
                             .forwarding = 0x2e4f,
                             .secondary = false};
// Original secondary checksum begins after GPIO/identification and its delay.
constexpr Layout secondary_layout{.helper_words = 388,
                                  .helper_block = 3,
                                  .payload_block = 2,
                                  .base = 0x10000,
                                  .first = 1,
                                  .count = 0xb01,
                                  .start = 0x2e0f,
                                  .decision = 0x2e47,
                                  .accepted = 0x2e1a,
                                  .forwarding = 0x2e47,
                                  .secondary = true};
constexpr std::array<std::string_view, 13> cases{"original",
                                                 "bad-marker",
                                                 "bad-first",
                                                 "bad-last",
                                                 "changed-prefetch",
                                                 "balanced",
                                                 "wrap-zero",
                                                 "wrap-nonzero",
                                                 "missing-marker",
                                                 "missing-first",
                                                 "missing-last",
                                                 "missing-prefetch",
                                                 "short-circuit"};

constexpr std::array<std::string_view, 12> secondary_cases{"original",
                                                           "bad-first",
                                                           "bad-last",
                                                           "changed-page",
                                                           "changed-prefetch",
                                                           "balanced",
                                                           "wrap-zero",
                                                           "wrap-nonzero",
                                                           "missing-first",
                                                           "missing-last",
                                                           "missing-prefetch",
                                                           "missing-page"};

std::vector<unsigned> words(const std::span<const std::uint8_t> bytes) {
  require(bytes.size() % 3 == 0, "Original update object has complete 24-bit words");
  std::vector<unsigned> result;
  for (std::size_t offset = 0; offset < bytes.size(); offset += 3) {
    result.push_back(
        blofeld::test::dsp_boot_word(std::span<const std::uint8_t, 3>(bytes.subspan(offset, 3))));
  }
  return result;
}

struct Input {
  std::vector<unsigned> payload;
  std::optional<unsigned> hole;
};

Input input(const std::vector<unsigned> &original,
            const std::string_view name,
            const Layout layout) {
  Input result{.payload = original, .hole = std::nullopt};
  if (name == "bad-marker" || name == "short-circuit") {
    result.payload.at(marker_address - layout.base) ^= 1U;
  }
  if (name == "bad-first") { result.payload.at(layout.first) ^= 1U; }
  if (name == "bad-last") { result.payload.at(layout.first + layout.count - 1) ^= 1U; }
  if (name == "changed-prefetch") { result.payload.at(layout.first + layout.count) ^= word_mask; }
  if (name == "changed-page") { result.payload.at(0) ^= 1U; }
  if (name == "balanced") {
    result.payload.at(layout.first) = (result.payload.at(layout.first) + 1U) & word_mask;
    result.payload.at(layout.first + 1) =
        (result.payload.at(layout.first + 1) + word_mask) & word_mask;
  }
  if (name == "wrap-zero" || name == "wrap-nonzero") {
    std::ranges::fill(std::span(result.payload).subspan(layout.first, layout.count), 0U);
    result.payload.at(layout.first) = word_mask;
    result.payload.at(layout.first + 1) = name == "wrap-zero" ? 1U : 0U;
  }
  if (name == "missing-marker") { result.hole = marker_address; }
  if (name == "missing-first" || name == "short-circuit") {
    result.hole = layout.base + layout.first;
  }
  if (name == "missing-last") { result.hole = layout.base + layout.first + layout.count - 1; }
  if (name == "missing-prefetch") { result.hole = layout.base + layout.first + layout.count; }
  if (name == "missing-page") { result.hole = layout.base; }
  return result;
}

void install(blofeld::DspBus &bus,
             const std::vector<unsigned> &helper,
             const Input &data,
             const Layout layout) {
  for (unsigned index = 0; index < helper.size(); ++index) {
    bus.write({.space = x, .address = entry + alias_offset + index, .value = helper.at(index)});
  }
  for (unsigned index = 0; index < data.payload.size(); ++index) {
    const auto address = layout.base + index;
    if (data.hole != address) {
      bus.write({.space = y, .address = address, .value = data.payload.at(index)});
    }
  }
}

void caller(blofeld::DspBus &bus, blofeld::DspMachine &cpu, const Layout layout) {
  blofeld::test::core::Program program;
  program.immediate(blofeld::test::DspRomRegister::r0, source_cookie);
  program.immediate(blofeld::test::DspRomRegister::r1, cursor_cookie);
  if (layout.secondary) {
    // Explicit post-identification caller state, not successful NAND setup.
    program.immediate(blofeld::test::DspRomRegister::r7, source_cookie);
    program.immediate(blofeld::test::DspRomRegister::sr, post_identification_sr);
    program.immediate(blofeld::test::DspRomRegister::m1, word_mask);
    program.immediate(blofeld::test::DspRomRegister::m2, word_mask);
  }
  program.immediate(blofeld::test::DspRomRegister::omr, memory_mode);
  for (unsigned index = 0; index < 3; ++index) { program.emit(0); }
  const auto mapped = program.pc();
  program.emit(absolute_call, layout.start);
  program.install(bus);
  const blofeld::test::core::Driver reset(cpu);
  require(reset.state().pc == blofeld::test::DspRomProgram::boot,
          "Native reset selects the caller");
  for (unsigned steps = 0; cpu.cpu().registers().pc != layout.start; ++steps) {
    require(steps < 32, "Own native caller must reach the original updater");
    if (cpu.cpu().registers().pc == mapped) { bus.advance_instruction_cycles(3); }
    blofeld::test::dma_test::step_native(cpu);
  }
  require(cpu.cpu().registers().sp == 1, "Own native JSR supplies the actual updater call frame");
}

void state(std::ostream &trace, const std::string_view label, const dsp56300::Registers &value) {
  trace << label << " pc=" << value.pc << " sr=" << value.sr << " omr=" << value.omr
        << " a=" << value.a << " r1=" << value.r[1] << " r7=" << value.r[7] << " x=" << value.x
        << " y=" << value.y << " sp=" << value.sp << '\n';
}

void verify_memory(blofeld::DspBus &bus,
                   const std::vector<unsigned> &helper,
                   const Input &data,
                   const Layout layout) {
  for (unsigned index = 0; index < helper.size(); ++index) {
    require(bus.inspect_program(entry + index) == helper.at(index),
            "Original updater executable words remain unchanged");
  }
  for (unsigned index = 0; index < data.payload.size(); ++index) {
    const auto address = layout.base + index;
    if (data.hole != address) {
      require(bus.read({.space = y, .address = address}) == data.payload.at(index),
              "Original validation changes no supplied update word");
    }
  }
  if (data.hole) {
    bool unknown = false;
    try {
      static_cast<void>(bus.read({.space = y, .address = *data.hole}));
    } catch (const blofeld::Fault &error) {
      unknown = std::string_view(error.what()).contains("Unknown DSP memory read");
    }
    require(unknown, "Missing update words remain unknown after original validation");
  }
}

void verify_quarantine(blofeld::DspMachine &cpu, std::ostream &trace) {
  require(cpu.cpu().fault().kind == dsp56300::FaultKind::memory_read && cpu.io().failure(),
          "Unknown update data retains the actual native read fault");
  const auto stopped = native_state(cpu);
  bool rejected = false;
  try {
    static_cast<void>(cpu.begin_fetch());
  } catch (const std::logic_error &error) {
    rejected = std::string_view(error.what()).contains("healthy stopped running state");
  }
  require(rejected && native_state(cpu) == stopped,
          "Missing native update data quarantines all further execution until reset");
  trace << "QUARANTINE unchanged=1\n";
}

struct Observations {
  std::ostream &trace;
  bool secondary;
  unsigned writes = 0;

  void read(const blofeld::DspIoReadAccess &access) const {
    if (access.address.space == p) { return; }
    require(access.address.space == y, "Original validation operand reads require Y words");
    trace << "READ address=" << access.address.address << " value=" << access.value
          << " pc=" << access.pc << '\n';
  }

  void write(const blofeld::DspStore &access) {
    require(!secondary && access.space == x && access.address == priority_register &&
                access.value == priority_value,
            "Original validation only writes its native interrupt priority");
    ++writes;
  }
};

void step(blofeld::DspMachine &machine, Observations &observations) {
  const blofeld::DspIoReadObserver reads{
      .context = &observations,
      .completed = [](void *context, const blofeld::DspIoReadAccess &access) {
        static_cast<Observations *>(context)->read(access);
      }};
  const blofeld::DspIoWriteObserver writes{
      .context = &observations,
      .completed = [](void *context, const blofeld::DspStore &access, unsigned) {
        static_cast<Observations *>(context)->write(access);
      }};
  const auto before = machine.cpu().progress();
  const auto ticks = machine.clock().elapsed_ticks();
  const auto progress = machine.run_observed_unclocked(reads, writes);
  if (progress.fault.kind != dsp56300::FaultKind::none) {
    std::string cause = "Original validation native instruction fault";
    const auto &failure = machine.io().failure();
    if (failure && failure->cause) {
      try {
        std::rethrow_exception(failure->cause);
      } catch (const std::exception &error) { cause = error.what(); }
    }
    throw blofeld::Fault(cause + " pc=" + blofeld::hex(progress.fault.pc, 6) +
                         " omr=" + blofeld::hex(machine.cpu().registers().omr, 6));
  }
  require(progress.dispatches == 1 &&
              machine.cpu().progress().instructions == before.instructions + 1 &&
              machine.clock().elapsed_ticks() == ticks,
          "One actual native instruction retires without inventing clock periods");
}

void execute(blofeld::DspMachine &cpu, std::ostream &trace, const Layout layout) {
  Observations observations{.trace = trace, .secondary = layout.secondary};
  for (unsigned steps = 0; steps < layout.count + instruction_allowance; ++steps) {
    const auto before = cpu.cpu().registers();
    if (before.pc == layout.decision || before.pc == layout.accepted) {
      state(trace, "DECISION", before);
    }
    if (before.pc == layout.forwarding || before.pc == layout.accepted) {
      state(trace, "END", before);
      trace << "COUNTS steps=" << steps << " writes=" << observations.writes << '\n';
      return;
    }
    try {
      step(cpu, observations);
    } catch (const blofeld::Fault &error) {
      const auto failed = cpu.cpu().registers();
      require(std::tuple{failed.a, failed.b, failed.x, failed.y, failed.sr} ==
                  std::tuple{before.a, before.b, before.x, before.y, before.sr},
              "Rejected operand reads retain pre-instruction ALU, destinations and status");
      trace << "FAULT " << error.what() << '\n';
      state(trace, "STOP", cpu.cpu().registers());
      trace << "COUNTS steps=" << steps << " writes=" << observations.writes << '\n';
      verify_quarantine(cpu, trace);
      return;
    }
  }
  throw blofeld::Fault("Original update validation exceeded its native instruction bound");
}

void run(const blofeld::Firmware &firmware, std::ostream &trace, const Layout layout) {
  const auto helper = words(firmware.update_blocks.at(layout.helper_block));
  const auto payload = words(firmware.update_blocks.at(layout.payload_block));
  const auto required_last =
      layout.secondary ? layout.first + layout.count : marker_address - layout.base;
  require(helper.size() == layout.helper_words && payload.size() > required_last,
          "Original updater and validation marker retain their pinned extents");
  const auto selected = layout.secondary ? std::span<const std::string_view>(secondary_cases)
                                         : std::span<const std::string_view>(cases);
  for (const auto name : selected) {
    const auto data = input(payload, name, layout);
    trace << "CASE name=" << name << '\n';
    blofeld::DspMachine cpu(blofeld::test::dma_test::native_clocks);
    auto &bus = cpu.bus();
    install(bus, helper, data, layout);
    caller(bus, cpu, layout);
    execute(cpu, trace, layout);
    verify_memory(bus, helper, data, layout);
    trace << "GUARDS program=" << helper.size() << " payload=" << payload.size()
          << " hole=" << (data.hole ? *data.hole : 0U) << " unchanged=1\n";
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3 || (count == 4 && std::string_view(arguments[3]) == "secondary"),
            "Expected original firmware, private validation trace and optional secondary mode");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    run(firmware, trace, count == 4 ? secondary_layout : main_layout);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
