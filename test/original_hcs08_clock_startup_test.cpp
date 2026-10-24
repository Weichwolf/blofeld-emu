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
constexpr unsigned reset_control = 0x44, reset_hx_known = 0xff00;
constexpr unsigned before_a_pc = 0xe02e, defined_a_pc = 0xe030, reset_flags = 0x68,
                   load_known = 0xee;

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
  if (cpu.a_known != byte_mask || cpu.hx_known != word_mask) {
    trace << " a-known=" << static_cast<unsigned>(cpu.a_known) << " hx-known=" << cpu.hx_known;
  }
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

constexpr unsigned before_a = 6164, defined_a = 6165;

bool cold_boundary(const unsigned at, const Hcs08State state) {
  constexpr std::array<unsigned, 4> pcs{0xe003, 0xe004, 0xe006, 0xe007};
  constexpr std::array<unsigned, 4> hxs{0x47e, 0x47e, 0x480, 0x80};
  constexpr std::array<unsigned, 4> flags{0x68, 0x68, 0x6c, 0x6a};
  require(state.hx_known == word_mask && state.a_known == (at < defined_a ? 0 : byte_mask),
          "Only actual first LDHX and later LDA establish cold register knowledge");
  if (at <= pcs.size()) {
    const auto index = at - 1;
    require(state ==
                Hcs08State{.pc = static_cast<std::uint16_t>(pcs[index]),
                           .hx = static_cast<std::uint16_t>(hxs[index]),
                           .sp = static_cast<std::uint16_t>(index == 0 ? byte_mask : app::stack),
                           .ccr_known = load_known,
                           .ccr = static_cast<std::uint8_t>(flags[index]),
                           .a_known = 0},
            "Independent first four cold instructions define H/X, stack and flags");
    return true;
  }
  if (at != before_a && at != defined_a) { return false; }
  require(state == Hcs08State{.pc = static_cast<std::uint16_t>(at == before_a ? before_a_pc
                                                                              : defined_a_pc),
                              .hx = app::expected_hx,
                              .sp = app::stack,
                              .a = static_cast<std::uint8_t>(at == before_a ? 0 : 3),
                              .ccr_known = app::ccr_known,
                              .ccr = reset_flags,
                              .a_known = static_cast<std::uint8_t>(at == before_a ? 0 : byte_mask)},
          "RAM clearing/calibration preserve unknown A until the original LDA immediate");
  return true;
}

void startup(app::ApplicationMemory &memory,
             Hcs08 &cpu,
             const bool cold,
             std::ostringstream &trace,
             std::vector<Saved> *const cuts) {
  if (memory.retired >= app::input_retired) { return; }
  while (memory.retired < app::input_retired) {
    cpu.step();
    if (!cold) { continue; }
    if (cold_boundary(memory.retired, cpu.state())) {
      trace << "COLD retired=" << memory.retired;
      registers(trace, cpu.state());
      trace << '\n';
    }
    if (memory.retired == before_a) { capture(memory, cpu, trace, cuts); }
  }
  app::configured(memory, cpu);
  trace << "GPIO retired=" << memory.retired << " writes=" << memory.peripheral_accesses.size();
  registers(trace, cpu.state());
  trace << '\n';
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
      startup(restored, other, true, actual, nullptr);
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

void run(const std::span<const std::uint8_t> records,
         const app::Input input,
         std::ostream &trace,
         const bool cold) {
  app::ApplicationMemory memory(records, input);
  memory.warm_reset();
  if (cold) { memory.icg.reset(Hcs08IcgReset::power_on); }
  const auto initial = memory.flash.image();
  const auto warm = app::warm_state(input.seed);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  if (!cold) { cpu.set_state(warm); }
  cpu.reset(Hcs08ResetEntry::normal, cold ? Hcs08ResetCause::power_on : Hcs08ResetCause::warm);
  require(cpu.state() ==
                  Hcs08State{.pc = app::entry,
                             .hx = static_cast<std::uint16_t>(cold ? 0 : warm.hx & byte_mask),
                             .sp = byte_mask,
                             .a = static_cast<std::uint8_t>(cold ? 0 : warm.a),
                             .ccr_known = flags_fixed + flag_i,
                             .ccr = flags_fixed + flag_i,
                             .a_known = static_cast<std::uint8_t>(cold ? 0 : byte_mask),
                             .hx_known =
                                 static_cast<std::uint16_t>(cold ? reset_hx_known : word_mask)} &&
              memory.known.none() && !memory.resetting &&
              memory.icg.state().control1_request == reset_control &&
              memory.icg.state().control2_request == 0,
          "Real normal warm reset establishes vector/registers but no RAM/clock/trim history");
  if (cold) {
    require(memory.icg.state().trim == packaged_trim,
            "Power-on live trim resets to80 independently of the declared factory NVM data");
  } else if (input.seed == 0) {
    require(!memory.icg.state().trim, "Unknown live trim remains unknown after warm reset");
  } else {
    require(memory.icg.state().trim == app::prior_live_trim,
            "Known prior live trim survives warm reset without being mistaken for NVICGTRM");
  }
  std::optional<std::uint8_t> retained;
  if (cold) {
    retained = packaged_trim;
  } else if (input.seed != 0) {
    retained = app::prior_live_trim;
  }
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
  std::vector<Saved> cuts;
  cuts.reserve(5);
  const bool selected =
      input.seed == 0 && input.trim == packaged_trim &&
      (input.observation == -1 || input.observation == 0 || input.observation == 8);
  auto *const selected_cuts = selected ? &cuts : nullptr;
  if (cold) { capture(memory, cpu, baseline, selected_cuts); }
  startup(memory, cpu, cold, baseline, selected_cuts);
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
    require(count == 3 || (count == 4 && std::string(arguments[3]) == "power-on"),
            "Expected pinned original pages, private trace and optional explicit power-on");
    const bool cold = count == 4;
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
    for (unsigned seed = 0; seed < (cold ? 1U : 2U); ++seed) {
      for (const auto trim : trims) {
        run(records, {.seed = seed, .trim = trim, .observation = -1}, trace, cold);
        for (unsigned bits = 0; bits < 16; ++bits) {
          run(records,
              {.seed = seed, .trim = trim, .observation = static_cast<int>(bits << 2U)},
              trace,
              cold);
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
