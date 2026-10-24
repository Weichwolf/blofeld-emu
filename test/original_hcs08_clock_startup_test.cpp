// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/hcs08_icg.hpp"
#include "blofeld/hcs08_system_options.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
namespace app = blofeld::test::hcs08_clock_startup;
constexpr unsigned first_control = 0x28, second_control = 0x30, ref_known = 0xdf;
constexpr unsigned loaded_ccr = 0x69, loaded_zero_ccr = 0x6b, loaded_negative_ccr = 0x6d;
constexpr unsigned configured_flash_bytes = 3584, packaged_trim = 0x80, strap_mask = 0x38;
constexpr unsigned background_pc = 0xe060, zero_ccr = 0x6a;
constexpr unsigned reset_control = 0x44;

struct Saved {
  app::ApplicationMemory memory;
  Hcs08Checkpoint cpu;
  unsigned retired;
  std::size_t offset;
};

void registers(std::ostream &trace, const Hcs08State cpu) {
  trace << " pc=" << cpu.pc << " hx=" << cpu.hx << " sp=" << cpu.sp
        << " a=" << static_cast<unsigned>(cpu.a) << " ccr=" << static_cast<unsigned>(cpu.ccr)
        << " known=" << static_cast<unsigned>(cpu.ccr_known);
}

void unchanged_nvm(const app::ApplicationMemory &memory, const Hcs08FlashImage &initial) {
  require(memory.flash.image() == initial && memory.flash.state() == Hcs08FlashState{} &&
              memory.flash.image().known.count() == configured_flash_bytes &&
              memory.flash.image().fresh.none(),
          "Startup cannot alter any supplied or omitted FLASH byte/knowledge/erase history");
  require(!memory.options.written() && memory.options.read() == Hcs08SystemOptions::reset_value,
          "The original clock prefix has not disabled COP or established watchdog timing");
}

void expected_clock(const app::ApplicationMemory &memory,
                    const app::Input input,
                    const unsigned at) {
  Hcs08IcgState expected;
  expected.trim = input.trim;
  if (at >= app::trim_retired + 2) {
    expected.control1_request = first_control;
    expected.control1_known = ref_known;
    expected.control1_written = true;
    expected.filter.reset();
  }
  if (at >= app::status_retired) {
    expected.control2_request = second_control;
    expected.control2_written = true;
  }
  require(memory.icg.state() == expected,
          "Only actual original stores establish live trim and software control requests");
}

void expected_main(const app::ApplicationMemory &memory, const Hcs08 &cpu, const app::Input input) {
  const unsigned at = memory.retired;
  auto trim_ccr = loaded_ccr;
  if (input.trim == 0) {
    trim_ccr = loaded_zero_ccr;
  } else if ((input.trim & packaged_trim) != 0) {
    trim_ccr = loaded_negative_ccr;
  }
  const auto flags = at <= app::trim_retired + 1 ? trim_ccr : loaded_ccr;
  constexpr std::array<unsigned, 4> pcs{
      app::trim_pc, app::control1_pc, app::control2_pc, app::status_pc};
  require(at >= app::trim_retired && at <= app::status_retired,
          "Independent original clock prefix has four exact boundaries");
  require(cpu.state() ==
                  Hcs08State{.pc = static_cast<std::uint16_t>(pcs.at(at - app::trim_retired)),
                             .hx = app::expected_hx,
                             .sp = app::stack - 8,
                             .a = input.trim,
                             .ccr_known = app::ccr_known,
                             .ccr = static_cast<std::uint8_t>(flags)} &&
              !cpu.faulted(),
          "Independent full CPU state retains original CMP carry, unknown H and stack frame");
  app::ram_contents(memory, true);
  if (at > app::trim_retired) { expected_clock(memory, input, at); }
}

void capture(const app::ApplicationMemory &memory,
             const Hcs08 &cpu,
             const std::ostringstream &trace,
             std::vector<Saved> *const cuts) {
  if (cuts != nullptr) {
    cuts->push_back({.memory = memory,
                     .cpu = cpu.checkpoint(),
                     .retired = memory.retired,
                     .offset = trace.str().size()});
  }
}

void progress(app::ApplicationMemory &memory,
              Hcs08 &cpu,
              const app::Input input,
              std::ostringstream &trace,
              std::vector<Saved> *const cuts) {
  if (memory.retired < app::trim_retired) {
    while (memory.retired < app::trim_retired) { cpu.step(); }
    expected_main(memory, cpu, input);
    trace << "LOAD retired=" << memory.retired;
    registers(trace, cpu.state());
    trace << '\n';
  }
  while (memory.retired < app::status_retired) {
    cpu.step();
    expected_main(memory, cpu, input);
    trace << "STORE retired=" << memory.retired;
    registers(trace, cpu.state());
    const auto clock = memory.icg.state();
    if (!clock.trim.has_value()) { throw Fault("Actual original store must establish live trim"); }
    trace << " trim=" << static_cast<unsigned>(*clock.trim)
          << " control1=" << static_cast<unsigned>(clock.control1_request)
          << " control1-known=" << static_cast<unsigned>(clock.control1_known)
          << " control2=" << static_cast<unsigned>(clock.control2_request)
          << " writes=" << memory.peripheral_accesses.size() << '\n';
    if (memory.retired == app::trim_retired + 1 || memory.retired == app::status_retired) {
      capture(memory, cpu, trace, cuts);
    }
  }
}

void finish(app::ApplicationMemory &memory,
            Hcs08 &cpu,
            const app::Input input,
            std::ostringstream &trace,
            std::vector<Saved> *const cuts) {
  if (input.observation < 0) {
    const auto failure = rejected([&] { cpu.step(); });
    require(cpu.faulted() && memory.retired == app::input_retired &&
                failure.contains("GPIO read address=0x0010 pc=0xe04e") &&
                failure.contains("Detached"),
            "Unknown straps still stop at the genuine native GPIO read");
    trace << "UNKNOWN " << failure << '\n';
    return;
  }
  if ((static_cast<unsigned>(input.observation) & strap_mask) == 0) {
    while (memory.retired < app::background_retired) { cpu.step(); }
    require(cpu.background_requested() &&
                cpu.state() == Hcs08State{.pc = background_pc,
                                          .hx = app::expected_hx,
                                          .sp = app::stack,
                                          .a = static_cast<std::uint8_t>(input.observation | 3),
                                          .ccr_known = app::ccr_known,
                                          .ccr = zero_ccr},
            "Zero strap mask follows the unchanged original overlapping BGND path");
    app::ram_contents(memory, false);
    trace << "BACKGROUND retired=" << memory.retired;
    registers(trace, cpu.state());
    trace << '\n';
    return;
  }
  progress(memory, cpu, input, trace, cuts);
  const auto before = memory.icg.state();
  const auto failure = rejected([&] { cpu.step(); });
  require(cpu.faulted() && memory.retired == app::status_retired &&
              failure.contains("ICG read address=0x004a pc=0xe1b0 ccr=0x0069") &&
              failure.contains("live clock/status evolution is unqualified") &&
              memory.icg.state() == before,
          "Actual original ICGS1 load refuses without a fabricated stable/ready clock");
  trace << "STATUS " << failure << '\n';
}

void flash_reads(const app::ApplicationMemory &memory, const bool main, const app::Input input) {
  constexpr unsigned vector_high = 0xfffe;
  constexpr unsigned vector_low = 0xffff;
  const auto &reads = memory.flash_reads;
  require(reads.size() == (main ? 4U : 2U) && reads[0].address == vector_high &&
              reads[0].value == (app::entry >> 8U) && reads[1].address == vector_low &&
              reads[1].value == 0 && !reads[0].write && !reads[1].write,
          "Real normal reset reads only actual FFFE/FFFF, not synthetic vector state");
  if (main) {
    require(reads[2].address == app::entry && !reads[2].write &&
                reads[3].address == app::factory_trim && reads[3].value == input.trim &&
                !reads[3].write,
            "Original code checks its actual first program byte and loads actual NVM calibration");
  }
}

void replays(const app::ApplicationMemory &memory,
             const Hcs08 &cpu,
             const app::Input input,
             const std::string &baseline,
             const std::vector<Saved> &cuts,
             std::ostream &trace) {
  for (const auto &saved : cuts) {
    for (unsigned index = 0; index < 2; ++index) {
      app::ApplicationMemory restored(saved.memory);
      restored.bind(input.observation);
      Hcs08 other(restored, BLOFELD_HCS08_SLA);
      other.restore_checkpoint(saved.cpu);
      std::ostringstream actual;
      finish(restored, other, input, actual, nullptr);
      require(actual.str() == baseline.substr(saved.offset) && other.state() == cpu.state() &&
                  other.faulted() == cpu.faulted() &&
                  other.background_requested() == cpu.background_requested(),
              "Fresh original CPU continuation reproduces the exact suffix and native endpoint");
      app::same_endpoint(restored, memory);
      trace << "REPLAY cut=" << saved.retired << " index=" << index << " identical=1\n";
    }
  }
}

void run(const std::span<const std::uint8_t> records, const app::Input input, std::ostream &trace) {
  app::ApplicationMemory memory(records, input);
  memory.warm_reset();
  const auto initial = memory.flash.image();
  const auto warm = app::warm_state(input.seed);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(warm);
  cpu.reset();
  require(cpu.state() == Hcs08State{.pc = app::entry,
                                    .hx = static_cast<std::uint16_t>(warm.hx & byte_mask),
                                    .sp = byte_mask,
                                    .a = warm.a,
                                    .ccr_known = flags_fixed + flag_i,
                                    .ccr = flags_fixed + flag_i} &&
              memory.known.none() && !memory.resetting &&
              memory.icg.state().control1_request == reset_control &&
              memory.icg.state().control2_request == 0,
          "Real normal warm reset establishes vector/registers but no RAM/clock/trim history");
  if (input.seed == 0) {
    require(!memory.icg.state().trim, "Unknown live trim remains unknown after warm reset");
  } else {
    require(memory.icg.state().trim == app::prior_live_trim,
            "Known prior live trim survives warm reset without being mistaken for NVICGTRM");
  }
  const auto retained =
      input.seed == 0 ? std::nullopt : std::optional<std::uint8_t>(app::prior_live_trim);
  require(memory.icg.state() == Hcs08IcgState{.trim = retained},
          "Warm reset restores every ICG request/knowledge/first-write field independently of NVM");
  std::ostringstream baseline;
  baseline << "CASE seed=" << input.seed << " trim=" << static_cast<unsigned>(input.trim)
           << " observation=" << input.observation << '\n';
  baseline << "RESET reads=" << memory.flash_reads.size() << " ram-known=" << memory.known.count()
           << " flash-known=" << memory.flash.image().known.count()
           << " live-known=" << memory.icg.state().trim.has_value()
           << " live=" << static_cast<unsigned>(memory.icg.state().trim.value_or(0));
  registers(baseline, cpu.state());
  baseline << '\n';
  for (unsigned n = 0; n < app::input_retired; ++n) { cpu.step(); }
  app::configured(memory, cpu);
  baseline << "GPIO retired=" << memory.retired << " writes=" << memory.peripheral_accesses.size();
  registers(baseline, cpu.state());
  baseline << '\n';
  std::vector<Saved> cuts;
  cuts.reserve(3);
  const bool selected =
      input.seed == 0 && input.trim == packaged_trim &&
      (input.observation == -1 || input.observation == 0 || input.observation == 8);
  auto *const selected_cuts = selected ? &cuts : nullptr;
  capture(memory, cpu, baseline, selected_cuts);
  finish(memory, cpu, input, baseline, selected_cuts);
  const bool main =
      input.observation >= 0 && (static_cast<unsigned>(input.observation) & strap_mask) != 0;
  flash_reads(memory, main, input);
  unchanged_nvm(memory, initial);
  trace << baseline.str();
  replays(memory, cpu, input, baseline.str(), cuts, trace);
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned original pages and private trace");
    std::vector<std::uint8_t> records(app::records_bytes);
    std::ifstream input(arguments[1], std::ios::binary);
    input.read(reinterpret_cast<char *>(records.data()),
               static_cast<std::streamsize>(records.size()));
    require(input.gcount() == static_cast<std::streamsize>(records.size()) &&
                input.peek() == std::char_traits<char>::eof(),
            "Original seven-page records retain their exact extent");
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    constexpr std::array<std::uint8_t, 4> trims{packaged_trim, 0, 0x5a, byte_mask};
    for (unsigned seed = 0; seed < 2; ++seed) {
      for (const auto trim : trims) {
        run(records, {.seed = seed, .trim = trim, .observation = -1}, trace);
        for (unsigned bits = 0; bits < 16; ++bits) {
          run(records,
              {.seed = seed, .trim = trim, .observation = static_cast<int>(bits << 2U)},
              trace);
        }
      }
    }
    std::cout << "Original HCS08 GPIO/FLASH/ICG startup reaches strict clock status with unknown "
                 "rates.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
