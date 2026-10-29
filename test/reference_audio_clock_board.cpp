// SPDX-License-Identifier: GPL-3.0-only
#include "audio/serial_pcm.hpp"
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "board/reference_board.hpp"
#include "board/reference_square_clock.hpp"
#include "common/bus.hpp"
#include "dax_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"
#include "esai_fixture.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using blofeld::test::esai::rejects;
using blofeld::test::esai::require;
namespace reference = blofeld::reference;
namespace dax = blofeld::test::dax_test;
constexpr unsigned oscillator = 12000000;
constexpr std::uint64_t timebase = 211680000000;
constexpr unsigned audio_hz = 11289600;
constexpr std::uint64_t half_period = 9375;
constexpr unsigned entry = 0x9000;
constexpr unsigned operations = 1200;
constexpr unsigned release_ticks = 176400;

blofeld::McuFlashImage image() {
  std::array<std::uint8_t, blofeld::McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  constexpr unsigned byte_mask = 0xff;
  bytes.fill(nop);
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return blofeld::McuFlashImage::from_known(bytes);
}

std::vector<std::uint32_t> program(const bool prime = true) {
  blofeld::test::DspProgram code;
  // Own PLL command:12MHz *9/2=54MHz under the declared boundary convention.
  code.emit("movep #>$016109,x:<<$fffffd");
  code.emit("movep #>$7d00,y:<<$ffff95");
  code.emit("movep #>$640301,y:<<$ffff96");
  code.emit("movep #>$838,y:<<$ffff9f");
  code.emit("movep #>$838,y:<<$ffff9e");
  code.emit("movep #>$800001,y:<<$ffff80");
  code.emit("movep #>$7d01,y:<<$ffff95");
  if (prime) { code.emit("movep #>0,x:<<$ffffd1"); }
  code.emit("movep #>$800001,x:<<$ffffd2");
  code.emit("movep #>$123456,x:<<$ffffd2");
  code.emit("movep #>8,x:<<$ffffd0");
  code.emit("movep #>3,x:<<$ffffd7");
  code.emit("movep #>3,x:<<$ffffd6");
  const auto loop = code.pc();
  code.emit("nop");
  constexpr unsigned absolute_jump = 0x0af080;
  code.word(absolute_jump);
  code.word(loop);
  // Own mapped ROM includes real speculative reads after the loop instruction.
  constexpr unsigned padding = 16;
  for (unsigned i = 0; i < padding; ++i) { code.emit("nop"); }
  blofeld::DspBus memory;
  code.install(memory);
  std::vector<std::uint32_t> words;
  for (unsigned address = blofeld::test::boot; address < code.pc(); ++address) {
    words.push_back(memory.peek_program(address));
  }
  return words;
}

reference::Config config(const bool pipeline) {
  reference::Config result;
  result.mcu = {.clock = {.oscillator_hz = oscillator,
                          .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
                .memory_timing = blofeld::McuRunMemory::unstretched_internal};
  result.dsp = {.clock = {.extal_hz = oscillator,
                          .timebase_hz = timebase,
                          .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
                .timing = blofeld::DspRunTiming::native_internal_estimates};
  result.schedule = {
      .dsp = {.observation = blofeld::DspStepObservation::reference_completion_boundary},
      .shared_write_order = blofeld::McuDspTieOrder::dsp_before_mcu,
      .maximum_dsp_steps_per_cycle = 64,
      .wait_order = blofeld::McuDspWaitOrder::dsp_before_mcu};
  if (pipeline) {
    result.pipeline = reference::PipelineTiming{
        .power_on = {3},
        .reset_pad = blofeld::McuResetPad::high,
        .schedule = {.dsp = {.extal_hz = oscillator,
                             .timebase_hz = timebase,
                             .timing = blofeld::DspFetchTiming::reference_cycle_order,
                             .execution =
                                 blofeld::DspReferenceExecution::staged_register_boundaries,
                             .pipeline = blofeld::DspFetchPipeline::automatic_memory,
                             .resets = blofeld::DspFetchResetTiming::reference_cycle_boundary},
                     .shared_write_order = blofeld::McuDspTieOrder::dsp_before_mcu,
                     .maximum_dsp_cycles_per_period = 64,
                     .external_write_order = blofeld::McuExternalWriteOrder::external_before_mcu,
                     .wait_order = blofeld::McuDspWaitOrder::dsp_before_mcu,
                     .reset_order = blofeld::McuDspResetOrder::dsp_before_mcu,
                     .initial_reset = blofeld::McuDspFetchInitialReset{.mode_known = 15,
                                                                       .mode_levels = 5,
                                                                       .pinit = false,
                                                                       .recovery_cycles = 2}},
        .dma_grant = blofeld::DspDmaGrant::available};
  }
  result.release_ticks = release_ticks;
  result.operations = operations;
  result.select_n = 2;
  result.clock = 4;
  result.mosi = 16;
  constexpr unsigned serial_pins = 0x16;
  result.initial = {{.port = blofeld::McuPort::b, .drive = {.mask = serial_pins, .levels = 2}},
                    {.port = blofeld::McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  result.dsp_input_order = blofeld::DspExternalOrder::external_before_dsp;
  result.maximum_dsp_inputs_per_advance = 64;
  result.audio_clock = {
      .frequency_hz = audio_hz, .epoch_ticks = pipeline ? 0U : release_ticks, .destinations = 6};
  constexpr unsigned pcm_after = 3000000;
  constexpr unsigned audio_word_bits = 24;
  result.pcm = {.instance = 1,
                .lane = 0,
                .after_ticks = pcm_after,
                .format = {.slot_bits = 32, .word_bits = audio_word_bits, .channels = 2}};
  return result;
}

struct Fixture {
  explicit Fixture(const reference::Config &configuration, const bool prime = true)
      : flash(image()),
        rom(program(prime)),
        execution{.image = flash,
                  .rom = rom,
                  .specification = BLOFELD_DEFAULT_SLA,
                  .flash_out = {},
                  .trace = &trace,
                  .instructions = operations},
        board(configuration, execution) {}

  blofeld::McuFlashImage flash;
  std::vector<std::uint32_t> rom;
  std::ostringstream trace;
  reference::Execution execution;
  reference::Board board;
};

void steps(reference::Board &board, const unsigned count) {
  for (unsigned i = 0; i < count; ++i) { board.advance(); }
}

std::uint64_t field(const std::string &line, const std::string &key) {
  const auto start = line.find(' ' + key + '=');
  if (start == std::string::npos) { throw blofeld::Fault("Missing trace field: " + key); }
  return std::stoull(line.substr(start + key.size() + 2), nullptr, 0);
}

void verify_trace(const std::string &trace, const std::uint64_t epoch) {
  std::istringstream lines(trace);
  std::uint64_t clock_rows = 0;
  unsigned rises = 0;
  std::string bmc;
  for (std::string line; std::getline(lines, line);) {
    if (line.starts_with("DSP-AUDIO-CLOCK ")) {
      const auto at = epoch + (clock_rows * half_period);
      require(field(line, "requested") == at && field(line, "actual") == at &&
                  field(line, "baseline") == (clock_rows == 0 ? 1U : 0U) &&
                  field(line, "level") == clock_rows % 2,
              "Every native Board source event matches independent absolute times and polarity");
      ++clock_rows;
    }
    if (!line.starts_with("DSP-DAX ")) { continue; }
    if (field(line, "driven") == 0) {
      rises = 0;
      continue;
    }
    if (field(line, "aci") == 0) { continue; }
    ++rises;
    if (rises % 2 == 0) { bmc.push_back(field(line, "level") != 0 ? '1' : '0'); }
  }
  constexpr unsigned subframe = 64;
  constexpr std::array preambles{'Z', 'Y', 'X', 'Y'};
  require(bmc.size() >= subframe * preambles.size(), "Real native DAX emits two complete frames");
  for (std::size_t i = 0; i < preambles.size(); ++i) {
    const auto decoded =
        dax::decode(std::string_view(bmc).substr(i * subframe, subframe), preambles.at(i));
    require(decoded == dax::Decoded{.audio = i % 2 == 0 ? dax::first_sample : dax::other_sample,
                                    .flags = 0},
            "Native MOVEP priming reaches the independent BMC payload/parity receiver");
  }
}

void runtime() {
  for (const bool pipeline : {false, true}) {
    const auto configuration = config(pipeline);
    Fixture f(configuration);
    while (!f.board.finished()) { f.board.advance(); }
    const auto state = f.board.status();
    if (state.faulted || !state.audio_clock || !state.audio_clock_started || !state.pcm ||
        !state.last_pcm || !configuration.audio_clock) {
      throw blofeld::Fault("Native initialization did not produce source/PCM state");
    }
    require(state.pcm->completed_frames() >= 2,
            "Actual native initialization and both Board schedulers produce real serial PCM");
    constexpr std::int32_t signed_sample = -8388607;
    const auto &frame = state.last_pcm.value();
    require(frame.channels == 2 && frame.samples[0] == signed_sample &&
                frame.samples[1] == signed_sample && frame.uniform_clock &&
                frame.bit_ticks == half_period * 8,
            "Independent signed PCM and exact bit periods follow original format fields");
    const auto source = state.audio_clock.value();
    require(source.current().ticks <= state.common_ticks &&
                source.next().ticks > state.common_ticks &&
                source.config().frequency_hz == audio_hz,
            "External source time never follows the PLL or native instruction count");
    require(f.trace.str().contains("core-hz=54000000") &&
                f.trace.str().contains("baseline=1 level=0 destinations=0x06"),
            "Declared DAX core rate follows the real PLL write; source baseline is explicit");
    verify_trace(f.trace.str(), configuration.audio_clock.value().epoch_ticks);
    require(f.trace.str().contains("mcu-pc=unknown-reset") == pipeline &&
                f.trace.str().contains("mcu-pc=0x"),
            "Reset observations mark MCU PC unknown; released native CPU contexts stay explicit");
    std::cout << "Native Board pipeline=" << pipeline
              << " frames=" << state.pcm.value().completed_frames()
              << " source-end=" << source.current().ticks << '\n';
  }
}

void replay_difference(const reference::Status &expected,
                       const reference::Status &actual,
                       const std::string_view expected_trace,
                       const std::string_view actual_trace,
                       const std::string_view context) {
  std::cerr << context << " status-equal=" << (expected == actual)
            << " trace-equal=" << (expected_trace == actual_trace)
            << " dsp-equal=" << (expected.dsp == actual.dsp)
            << " source-equal=" << (expected.audio_clock == actual.audio_clock)
            << " reset-equal=" << (expected.mcu_reset == actual.mcu_reset)
            << " nvm-equal=" << (expected.cold_nvm == actual.cold_nvm)
            << " pcm-equal=" << (expected.pcm == actual.pcm)
            << " stamp-equal=" << (expected.pcm_stamp == actual.pcm_stamp)
            << " frame-equal=" << (expected.last_pcm == actual.last_pcm) << '\n';
  std::cerr << "expected ticks=" << expected.common_ticks << " dsp-pc=" << expected.dsp.cpu.pc
            << " dsp-steps=" << expected.dsp_steps << " dsp-cycles=" << expected.dsp_cycles
            << " native=" << expected.dsp.native_instructions << '/' << expected.dsp.native_cycles
            << " ties=" << expected.ties << " ordered=" << expected.ordered_pairs << '\n';
  std::cerr << "actual ticks=" << actual.common_ticks << " dsp-pc=" << actual.dsp.cpu.pc
            << " dsp-steps=" << actual.dsp_steps << " dsp-cycles=" << actual.dsp_cycles
            << " native=" << actual.dsp.native_instructions << '/' << actual.dsp.native_cycles
            << " ties=" << actual.ties << " ordered=" << actual.ordered_pairs << '\n';
  std::size_t offset = 0;
  while (offset < expected_trace.size() && offset < actual_trace.size() &&
         expected_trace[offset] == actual_trace[offset]) {
    ++offset;
  }
  constexpr unsigned excerpt = 256;
  std::cerr << "first trace difference=" << offset
            << " expected=" << expected_trace.substr(offset, excerpt)
            << "\nactual=" << actual_trace.substr(offset, excerpt) << '\n';
}

void restore() {
  constexpr std::array cuts{0U, 1U, 3U, 9U, 100U, 194U, 195U, 250U};
  constexpr unsigned suffix = 160;
  for (const bool pipeline : {false, true}) {
    for (const auto cut : cuts) {
      Fixture f(config(pipeline));
      f.board.initialize();
      steps(f.board, cut);
      const auto saved = f.board.checkpoint();
      const auto prefix = f.board.status();
      f.trace.str({});
      steps(f.board, suffix);
      const auto expected = f.board.status();
      const auto trace = f.trace.str();
      for (unsigned replay = 0; replay < 2; ++replay) {
        f.board.restore(saved);
        require(f.board.status() == prefix, "Joint restore retains source cursor and baseline");
        f.trace.str({});
        steps(f.board, suffix);
        const auto actual = f.board.status();
        const auto actual_trace = f.trace.str();
        if (actual != expected || actual_trace != trace) {
          replay_difference(expected,
                            actual,
                            trace,
                            actual_trace,
                            std::string(pipeline ? "pipeline=automatic" : "pipeline=completion") +
                                " cut=" + std::to_string(cut) +
                                " replay=" + std::to_string(replay));
        }
        require(actual == expected && actual_trace == trace,
                "Both native schedules replay real source/peripheral/PCM suffixes exactly");
      }
    }
  }
  std::cout << "16 native Board cuts;32 restored suffixes\n";
}

void guards() {
  blofeld::DspGpio port(blofeld::DspPort::e);
  constexpr unsigned hckt = 0x20;
  port.set_drive({.mask = 1, .levels = 1, .floating = 2, .unknown = 4});
  port.update_drive(hckt, {.mask = hckt, .levels = hckt});
  port.write({.reg = blofeld::DspGpioRegister::control, .value = hckt | 7U});
  require(port.read(blofeld::DspGpioRegister::data, hckt | 3U) == (hckt | 1U) &&
              port.observe_pads(4).known == 0,
          "A clock pad update preserves unrelated driven, floating and unknown inputs");
  port.write({.reg = blofeld::DspGpioRegister::control, .value = 7});
  port.write({.reg = blofeld::DspGpioRegister::direction, .value = hckt});
  port.write({.value = hckt});
  require(rejects([&] { port.update_drive(hckt, {.mask = hckt}); }).contains("contention") &&
              port.read(blofeld::DspGpioRegister::data, hckt | 1U) == (hckt | 1U),
          "Opposing a real GPIO output fails atomically without dropping another input");
  static_cast<void>(rejects([&] { port.update_drive(hckt, {.mask = 1}); }));
  constexpr unsigned reserved_pin = 0x1000;
  static_cast<void>(rejects([&] { port.update_drive(reserved_pin, {}); }));
  for (const auto destinations : {0U, 8U}) {
    auto configuration = config(false);
    if (!configuration.audio_clock) { throw blofeld::Fault("Missing test clock declaration"); }
    configuration.audio_clock.value().destinations = destinations;
    static_cast<void>(rejects([&] { const Fixture f(configuration); }));
  }
  auto collision = config(false);
  collision.dsp_events = {{.ticks = release_ticks, .level = false}};
  Fixture f(collision);
  const auto error = rejects([&] { f.board.advance(); });
  require(error.contains("Simultaneous") && f.board.status().faulted &&
              !f.board.status().audio_clock_started,
          "Independent same-time inputs are refused before a composite clock observation");
  auto failed = config(false);
  failed.pcm.reset();
  Fixture g(failed, false);
  g.board.initialize();
  const auto saved = g.board.checkpoint();
  std::optional<reference::Status> expected;
  std::string expected_error;
  for (unsigned replay = 0; replay < 2; ++replay) {
    g.board.restore(saved);
    const auto failure = rejects([&] {
      while (!g.board.finished()) { g.board.advance(); }
    });
    const auto state = g.board.status();
    require(state.faulted && failure.contains("XNADR") && failure.contains("mcu-pc=") &&
                failure.contains("dsp-pc=") && failure.contains("at="),
            "Unprimed DAX stops the actual composite edge with native execution context");
    require(state.audio_clock && state.audio_clock->next().ticks == field(failure, "at"),
            "A failed fanout leaves the external source cursor before the failing transition");
    require(!expected || (*expected == state && expected_error == failure),
            "A coordinated restore reproduces the same partial-fanout refusal");
    expected = state;
    expected_error = failure;
    const auto before = g.board.status();
    static_cast<void>(rejects([&] { g.board.advance(); }));
    static_cast<void>(rejects([&] { static_cast<void>(g.board.checkpoint()); }));
    require(g.board.status() == before, "A fault cannot advance or become a trusted checkpoint");
  }
}
}

int main(const int argc, char **argv) {
  try {
    if (argc != 2) { throw blofeld::Fault("Expected runtime, restore or guards"); }
    const std::string_view mode(argv[1]);
    if (mode == "runtime") {
      runtime();
    } else if (mode == "restore") {
      restore();
    } else if (mode == "guards") {
      guards();
    } else {
      throw blofeld::Fault("Unknown audio Board test mode");
    }
    std::cout << "Native Board audio clock " << mode << " passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
