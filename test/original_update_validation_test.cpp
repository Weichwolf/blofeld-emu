// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {
using blofeld::test::require;
using enum blofeld::DspSpace;
constexpr unsigned entry = 0x2e00, helper_words = 400, helper_block = 5, payload_block = 4;
constexpr unsigned alias_offset = 0xf000, memory_mode = 0x600389;
constexpr unsigned payload_begin = 0xd000, checksum_words = 0x7473;
constexpr unsigned marker_address = 0x14477, marker_index = marker_address - payload_begin;
constexpr unsigned error_selection = 0x2e16, accepted = 0x2e19, forwarding = 0x2e4f;
constexpr unsigned word_mask = 0xffffff, absolute_call = 0x0bf080;
constexpr unsigned source_cookie = 0x789, cursor_cookie = 0x456;
constexpr unsigned priority_register = 0xfffffe, priority_value = 0x100;
constexpr unsigned instruction_limit = checksum_words + 128;
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

Input input(const std::vector<unsigned> &original, const std::string_view name) {
  Input result{.payload = original, .hole = std::nullopt};
  if (name == "bad-marker" || name == "short-circuit") { result.payload.at(marker_index) ^= 1U; }
  if (name == "bad-first") { result.payload.at(0) ^= 1U; }
  if (name == "bad-last") { result.payload.at(checksum_words - 1) ^= 1U; }
  if (name == "changed-prefetch") { result.payload.at(checksum_words) ^= word_mask; }
  if (name == "balanced") {
    result.payload.at(0) = (result.payload.at(0) + 1U) & word_mask;
    result.payload.at(1) = (result.payload.at(1) + word_mask) & word_mask;
  }
  if (name == "wrap-zero" || name == "wrap-nonzero") {
    std::fill_n(result.payload.begin(), checksum_words, 0U);
    result.payload.at(0) = word_mask;
    result.payload.at(1) = name == "wrap-zero" ? 1U : 0U;
  }
  if (name == "missing-marker") { result.hole = marker_address; }
  if (name == "missing-first" || name == "short-circuit") { result.hole = payload_begin; }
  if (name == "missing-last") { result.hole = payload_begin + checksum_words - 1; }
  if (name == "missing-prefetch") { result.hole = payload_begin + checksum_words; }
  return result;
}

void install(blofeld::DspBus &bus, const std::vector<unsigned> &helper, const Input &data) {
  for (unsigned index = 0; index < helper.size(); ++index) {
    bus.write({.space = x, .address = entry + alias_offset + index, .value = helper.at(index)});
  }
  for (unsigned index = 0; index < data.payload.size(); ++index) {
    const auto address = payload_begin + index;
    if (data.hole != address) {
      bus.write({.space = y, .address = address, .value = data.payload.at(index)});
    }
  }
}

void caller(blofeld::DspBus &bus, blofeld::DspCore &cpu) {
  blofeld::test::DspProgram program;
  program.immediate("r0", source_cookie);
  program.immediate("r1", cursor_cookie);
  program.immediate("omr", memory_mode);
  for (unsigned index = 0; index < 3; ++index) { program.emit("nop"); }
  const auto mapped = program.pc();
  program.word(absolute_call);
  program.word(entry);
  program.install(bus);
  for (unsigned steps = 0; cpu.state().pc != entry; ++steps) {
    require(steps < 32, "Own native caller must reach the original updater");
    if (cpu.state().pc == mapped) { bus.advance_instruction_cycles(3); }
    cpu.step();
  }
  require(cpu.state().sp == 1, "Own native JSR supplies the actual updater call frame");
}

void state(std::ostream &trace, const std::string_view label, const blofeld::DspState &value) {
  trace << label << " pc=" << value.pc << " sr=" << value.sr << " omr=" << value.omr
        << " a=" << value.a << " r1=" << value.r[1] << " r7=" << value.r[7] << " x=" << value.x
        << " y=" << value.y << " sp=" << value.sp << '\n';
}

void verify_memory(blofeld::DspBus &bus, const std::vector<unsigned> &helper, const Input &data) {
  for (unsigned index = 0; index < helper.size(); ++index) {
    require(bus.inspect_program(entry + index) == helper.at(index),
            "Original updater executable words remain unchanged");
  }
  for (unsigned index = 0; index < data.payload.size(); ++index) {
    const auto address = payload_begin + index;
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

void verify_quarantine(blofeld::DspCore &cpu, std::ostream &trace) {
  const auto stopped = cpu.debug_state();
  bool rejected = false;
  try {
    cpu.step();
  } catch (const blofeld::Fault &error) {
    rejected =
        std::string_view(error.what()).contains("requires reset after a rejected reference step");
  }
  require(rejected && cpu.debug_state() == stopped,
          "Missing native update data quarantines all further execution until reset");
  trace << "QUARANTINE unchanged=1\n";
}

void execute(blofeld::DspCore &cpu, std::ostream &trace) {
  unsigned writes = 0;
  const blofeld::DspNativeAccessObserver observer{
      .observe = [&](const blofeld::DspNativeAccess &access) {
        if (access.stage != blofeld::DspNativeAccessStage::after) { return; }
        if (access.kind == blofeld::DspNativeAccessKind::read) {
          require(access.target.space == y && access.value.has_value(),
                  "Original validation operand reads must use known Y words");
          trace << "READ address=" << access.target.address << " value=" << *access.value
                << " pc=" << access.pc << '\n';
        }
        if (access.kind == blofeld::DspNativeAccessKind::write) {
          require(access.target.space == x && access.target.address == priority_register &&
                      access.value == priority_value,
                  "Original validation only writes its native interrupt priority");
          ++writes;
        }
      }};
  for (unsigned steps = 0; steps < instruction_limit; ++steps) {
    const auto before = cpu.state();
    if (before.pc == error_selection || before.pc == accepted) { state(trace, "DECISION", before); }
    if (before.pc == forwarding || before.pc == accepted) {
      state(trace, "END", before);
      trace << "COUNTS steps=" << steps << " writes=" << writes << '\n';
      return;
    }
    try {
      cpu.step(observer);
    } catch (const blofeld::Fault &error) {
      trace << "FAULT " << error.what() << '\n';
      state(trace, "STOP", cpu.state());
      trace << "COUNTS steps=" << steps << " writes=" << writes << '\n';
      verify_quarantine(cpu, trace);
      return;
    }
  }
  throw blofeld::Fault("Original update validation exceeded its native instruction bound");
}

void run(const blofeld::Firmware &firmware, std::ostream &trace) {
  const auto helper = words(firmware.update_blocks.at(helper_block));
  const auto payload = words(firmware.update_blocks.at(payload_block));
  require(helper.size() == helper_words && payload.size() > marker_index,
          "Original updater and validation marker retain their pinned extents");
  for (const auto name : cases) {
    const auto data = input(payload, name);
    trace << "CASE name=" << name << '\n';
    blofeld::DspBus bus;
    install(bus, helper, data);
    blofeld::DspCore cpu(bus);
    caller(bus, cpu);
    execute(cpu, trace);
    verify_memory(bus, helper, data);
    trace << "GUARDS program=" << helper.size() << " payload=" << payload.size()
          << " hole=" << (data.hole ? *data.hole : 0U) << " unchanged=1\n";
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected original firmware and private validation trace");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    run(firmware, trace);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
