// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_midi.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "blofeld/midi_serial.hpp"
#include "blofeld/usbip_runtime.hpp"
#include "dsp_norm_fixture.hpp"
#include "host_audio_fixture.hpp"
#include "reference_board.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <ios>
#include <iostream>
#include <limits>
#include <optional>
#include <source_location>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
using blofeld::test::require;
namespace reference = blofeld::reference;
constexpr unsigned entry = 0x9000;
constexpr unsigned nop = 0xa7;
constexpr unsigned oscillator = 12000000;
constexpr unsigned timebase = 540000000;
constexpr unsigned epoch = 450;
constexpr unsigned absolute_jump = 0x0af080;
constexpr std::uint64_t nop_ticks = 45;
constexpr std::uint64_t mcu_period = 90;
constexpr unsigned extended_load = 0xb6;
constexpr unsigned unmapped_page = 0x30;
constexpr unsigned serial_pins = 0x16;
constexpr unsigned first_cursor = 0x101;
constexpr unsigned suffix_operations = 20;
constexpr unsigned midi_operations = 12000;

std::string rejects(const auto &operation,
                    const std::source_location location = std::source_location::current()) {
  try {
    operation();
  } catch (const std::exception &error) { return error.what(); }
  throw blofeld::Fault("Expected rejected reference-board operation at line " +
                       std::to_string(location.line()));
}

blofeld::McuFlashImage image(const bool fault) {
  std::array<std::uint8_t, blofeld::McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  constexpr unsigned byte_mask = 0xff;
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  if (fault) {
    constexpr unsigned code = entry - 0x8000 + 3;
    bytes[code] = extended_load; // Own LDAA extended, required unknown operand after three NOPs.
    bytes[code + 1] = unmapped_page;
    bytes[code + 2] = 0;
  }
  return blofeld::McuFlashImage::from_known(bytes);
}

reference::Config config(const std::uint64_t release,
                         const std::optional<reference::MidiRoute> midi = std::nullopt,
                         const std::optional<reference::DspMidiRoute> dsp_midi = std::nullopt) {
  reference::Config result;
  result.mcu = {.clock = {.oscillator_hz = oscillator,
                          .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
                .memory_timing = blofeld::McuRunMemory::internal_flash_reads};
  result.dsp = {.clock = {.extal_hz = oscillator,
                          .timebase_hz = timebase,
                          .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
                .timing = blofeld::DspRunTiming::native_internal_estimates};
  result.schedule = {
      .dsp = {.observation = blofeld::DspStepObservation::reference_completion_boundary},
      .shared_write_order = blofeld::McuDspTieOrder::dsp_before_mcu,
      .maximum_dsp_steps_per_cycle = 16,
      .wait_order = blofeld::McuDspWaitOrder::dsp_before_mcu};
  result.release_ticks = release;
  result.operations = (midi || dsp_midi) ? midi_operations : 32;
  result.midi_dsp = dsp_midi;
  if (dsp_midi) {
    result.dsp_input_order = blofeld::DspExternalOrder::external_before_dsp;
    result.maximum_dsp_inputs_per_advance = 4;
  }
  result.midi = midi;
  result.select_n = 2;
  result.clock = 4;
  result.mosi = 16;
  result.initial = {{.port = blofeld::McuPort::b, .drive = {.mask = serial_pins, .levels = 2}},
                    {.port = blofeld::McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  return result;
}

struct Fixture {
  Fixture(const std::filesystem::path &specification,
          const bool fault = true,
          const std::uint64_t release = epoch,
          std::vector<std::uint32_t> program = {0, absolute_jump, blofeld::test::boot},
          const std::optional<reference::MidiRoute> midi = std::nullopt,
          const std::optional<reference::DspMidiRoute> dsp_midi = std::nullopt)
      : flash(image(fault)),
        rom(std::move(program)),
        configuration(config(release, midi, dsp_midi)),
        execution{.image = flash,
                  .rom = rom,
                  .specification = specification,
                  .flash_out = {},
                  .trace = &trace,
                  .instructions = configuration.operations},
        board(configuration, execution) {}

  blofeld::McuFlashImage flash;
  std::vector<std::uint32_t> rom;
  std::ostringstream trace;
  reference::Config configuration;
  reference::Execution execution;
  reference::Board board;
};

void expect_prefix(const reference::Board &board,
                   const unsigned operations,
                   const unsigned dsp_units) {
  const auto state = board.status();
  require(state.mcu.pc == entry + operations && state.instructions == operations &&
              state.operations == operations && state.mcu_cycles == 5 + operations &&
              state.common_ticks == (5 + operations) * mcu_period && state.dsp_units == dsp_units &&
              state.dsp_ticks == dsp_units * nop_ticks && !state.faulted,
          "Independent five-reset plus one-cycle MCU NOPs and DSP NOP1/JMP4 expectations");
}

void test_pending_deadline_fault_and_joint_restore(const std::filesystem::path &specification) {
  Fixture f(specification);
  f.board.advance();
  expect_prefix(f.board, 1, 1);
  const auto frozen = f.board.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.board.restore(frozen);
    f.trace.str({});
    f.trace.clear();
    f.board.advance();
    f.board.advance();
    expect_prefix(f.board, 3, 6);
    require(f.board.status().dsp_steps == 3,
            "Pending four-unit JMP and next NOP complete exactly twice after the captured NOP");
    const auto error = rejects([&] { f.board.advance(); });
    require(error.contains("0x3000") && error.contains("0x9003") && f.board.status().faulted,
            "Actual required unmapped MCU operand fails and quarantines the composed board");
    const auto failed = f.board.status();
    static_cast<void>(rejects([&] { f.board.advance(); }));
    static_cast<void>(rejects([&] { static_cast<void>(f.board.checkpoint()); }));
    require(f.board.status() == failed,
            "A fault cannot silently advance or become a trusted whole-board checkpoint");
  }
  f.board.restore(frozen);
  expect_prefix(f.board, 1, 1);
}

void test_held_reset_and_foreign_owner(const std::filesystem::path &specification) {
  constexpr unsigned release = 720;
  Fixture f(specification, false, release);
  f.board.advance();
  expect_prefix(f.board, 1, 0);
  require(f.board.status().held_reset, "No DSP step precedes its caller-declared release");
  const auto frozen = f.board.checkpoint();
  Fixture other(specification, false, release);
  const auto unchanged = other.board.status();
  require(rejects([&] { other.board.restore(frozen); }).contains("another owner") &&
              other.board.status() == unchanged,
          "Another board identity cannot import this native/wiring checkpoint");
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.board.restore(frozen);
    f.board.advance();
    f.board.advance();
    expect_prefix(f.board, 3, 0);
    require(!f.board.status().held_reset,
            "Native reset release recurs at720 with a zero-step DSP and fresh schedule");
  }
}

void test_failed_preparation_leaves_live_board_unchanged(
    const std::filesystem::path &specification) {
  Fixture f(specification);
  f.board.advance();
  const auto frozen = f.board.checkpoint();
  f.board.advance();
  const auto unchanged = f.board.status();
  const auto backup = specification.string() + ".temporarily-unavailable";
  std::filesystem::rename(specification, backup);
  const auto error = rejects([&] { f.board.restore(frozen); });
  std::filesystem::rename(backup, specification);
  require(!error.empty() && f.board.status() == unchanged,
          "Replacement preparation failure leaves both live owners and all counters untouched");
  expect_prefix(f.board, 2, 1);
  f.board.restore(frozen);
  expect_prefix(f.board, 1, 1);
}

class CallbackTrace final : public std::stringbuf {
public:
  std::function<void()> observe;
  bool markers_only = false;
  bool flush_only = false;

private:
  void call() {
    auto callback = std::exchange(observe, {});
    if (callback) { callback(); }
  }

  std::streamsize xsputn(const char *data, const std::streamsize count) override {
    if (!flush_only &&
        (!markers_only ||
         (count >= 6 &&
          std::string_view(data, static_cast<std::size_t>(count)).starts_with("REPLAY")))) {
      call();
    }
    return std::stringbuf::xsputn(data, count);
  }

  int_type overflow(const int_type value) override {
    if (!markers_only && !flush_only) { call(); }
    return std::stringbuf::overflow(value);
  }

  int sync() override {
    if (flush_only) { call(); }
    return std::stringbuf::sync();
  }
};

void test_active_control_rejections(const std::filesystem::path &specification) {
  const auto flash = image(false);
  const std::vector<std::uint32_t> rom{0, absolute_jump, blofeld::test::boot};
  CallbackTrace buffer;
  std::ostream trace(&buffer);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  const auto settings = config(epoch);
  const reference::Execution execution{.image = flash,
                                       .rom = rom,
                                       .specification = specification,
                                       .flash_out = {},
                                       .trace = &trace,
                                       .instructions = 10};
  reference::Board board(settings, execution);
  board.initialize();
  const auto saved = board.checkpoint();
  bool called = false;
  buffer.observe = [&] {
    called = true;
    static_cast<void>(rejects([&] { static_cast<void>(board.checkpoint()); }));
    static_cast<void>(rejects([&] { board.restore(saved); }));
    static_cast<void>(rejects([&] { board.advance(); }));
    static_cast<void>(rejects([&] { static_cast<void>(board.run()); }));
  };
  board.advance();
  require(called, "The real native board trace invokes the active-operation observer");
  expect_prefix(board, 1, 1);
}

void test_replay_marker_rejections(const std::filesystem::path &specification) {
  const auto flash = image(false);
  const std::vector<std::uint32_t> rom{0, absolute_jump, blofeld::test::boot};
  CallbackTrace buffer;
  buffer.markers_only = true;
  std::ostream trace(&buffer);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  auto settings = config(epoch);
  settings.checkpoint = {.operation = 1, .replays = 1};
  const reference::Execution execution{.image = flash,
                                       .rom = rom,
                                       .specification = specification,
                                       .flash_out = {},
                                       .trace = &trace,
                                       .instructions = 2};
  reference::Board board(settings, execution);
  const auto saved = board.checkpoint();
  bool called = false;
  buffer.observe = [&] {
    called = true;
    static_cast<void>(rejects([&] { static_cast<void>(board.checkpoint()); }));
    static_cast<void>(rejects([&] { board.restore(saved); }));
    static_cast<void>(rejects([&] { board.advance(); }));
    static_cast<void>(rejects([&] { static_cast<void>(board.run()); }));
  };
  require(board.run() == 0 && called, "Replay-marker observation cannot reenter board control");
  expect_prefix(board, 2, 1);
}

void test_trace_flush_rejections(const std::filesystem::path &specification) {
  const auto flash = image(false);
  const std::vector<std::uint32_t> rom{0, absolute_jump, blofeld::test::boot};
  CallbackTrace buffer;
  buffer.flush_only = true;
  std::ostream trace(&buffer);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  const auto settings = config(epoch);
  const reference::Execution execution{.image = flash,
                                       .rom = rom,
                                       .specification = specification,
                                       .flash_out = {},
                                       .trace = &trace,
                                       .instructions = 2};
  reference::Board board(settings, execution);
  const auto saved = board.checkpoint();
  bool called = false;
  buffer.observe = [&] {
    called = true;
    static_cast<void>(rejects([&] { static_cast<void>(board.checkpoint()); }));
    static_cast<void>(rejects([&] { board.restore(saved); }));
    static_cast<void>(rejects([&] { board.advance(); }));
    static_cast<void>(rejects([&] { static_cast<void>(board.run()); }));
  };
  require(board.run() == 0 && called, "Final trace flush cannot change the completed board");
  expect_prefix(board, 2, 1);
}

std::vector<std::uint32_t> repeat_program() {
  blofeld::test::DspProgram program;
  program.immediate("r0", 0x100);
  program.immediate("x0", 7);
  program.emit("rep #5");
  program.emit("move x0,y:(r0)+");
  program.emit("move y:>$100,x1");
  program.immediate("y0", 9);
  program.emit("move y0,y:>$100");
  program.emit("move y:>$100,x0");
  const auto loop = program.pc();
  program.emit("nop");
  program.word(absolute_jump);
  program.word(loop);
  blofeld::DspBus bus;
  program.install(bus);
  std::vector<std::uint32_t> words;
  for (unsigned address = blofeld::test::boot; address < program.pc(); ++address) {
    words.push_back(bus.peek_program(address));
  }
  return words;
}

void test_latched_rep_and_written_memory(const std::filesystem::path &specification) {
  Fixture f(specification, false, epoch, repeat_program());
  for (unsigned i = 0; i < 5; ++i) { f.board.advance(); }
  const auto first = f.board.status();
  require(first.mcu.pc == entry + 5 && first.dsp_units == 10 && first.dsp_steps == 4 &&
              first.dsp.cpu.r[0] == first_cursor,
          "Two two-unit immediates, five-unit REP and first one-unit store precede capture");
  const auto saved = f.board.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.board.restore(saved);
    while (f.board.status().operations < suffix_operations) { f.board.advance(); }
    constexpr unsigned completed_cursor = 0x105;
    constexpr std::uint64_t expected_x = (std::uint64_t{7} << 24U) | 9U;
    require(
        f.board.status().dsp.cpu.r[0] == completed_cursor &&
            f.board.status().dsp.cpu.x == expected_x,
        "Four remaining native REP stores and real old/new RAM reads survive each joint restore");
  }
}

void test_midi_wire_checkpoint_and_capacity(const std::filesystem::path &specification) {
  constexpr unsigned frame_cycles = 1920;
  constexpr unsigned partial_cycles = 317;
  constexpr unsigned complete_cycles = 3 * frame_cycles;
  constexpr unsigned bus_hz = 6000000;
  for (const auto pin : {blofeld::SciReceivePin::pr2, blofeld::SciReceivePin::ps4}) {
    Fixture f(specification,
              false,
              std::numeric_limits<std::uint64_t>::max(),
              {0, absolute_jump, blofeld::test::boot},
              reference::MidiRoute{.bus_hz = bus_hz, .pin = pin});
    require(rejects([&] { f.board.enqueue_midi({}); }).contains("precedes"),
            "External bytes cannot enter before the configured MIDI source is armed");
    f.board.initialize();
    const auto initial = f.board.status();
    const std::array<std::uint8_t, 3> bytes{0x90, 64, 127};
    f.board.enqueue_midi(bytes);
    require(f.board.midi_window().available == blofeld::MidiSerialInput::capacity - bytes.size(),
            "The public input window accounts for every accepted but unconsumed byte");
    std::array<std::uint8_t, blofeld::MidiSerialInput::capacity> excess{};
    const auto before = f.board.status();
    static_cast<void>(rejects([&] { f.board.enqueue_midi(excess); }));
    require(f.board.status() == before, "An overflowing enqueue is rejected atomically");
    for (unsigned cycle = 0; cycle < partial_cycles; ++cycle) { f.board.advance(); }
    const auto saved = f.board.checkpoint();
    std::optional<reference::Status> expected;
    std::string expected_trace;
    for (unsigned replay = 0; replay < 2; ++replay) {
      f.board.restore(saved);
      f.trace.str({});
      f.trace.clear();
      for (unsigned cycle = partial_cycles; cycle < complete_cycles; ++cycle) { f.board.advance(); }
      const auto observed = f.board.status();
      require(observed.midi_bytes == bytes.size() && observed.midi_queued == 0 &&
                  observed.midi_started && observed.mcu_cycles == 5 + complete_cycles &&
                  observed.common_ticks == (5 + complete_cycles) * mcu_period &&
                  observed.cold_nvm == initial.cold_nvm,
              "Native NOP periods drain exactly three 8-N-1 frames with unchanged NVM");
      if (expected) {
        require(
            observed == *expected && f.trace.str() == expected_trace,
            "Joint MCU/board/source snapshots retain the partial bit, queue and complete trace");
      } else {
        expected = observed;
        expected_trace = f.trace.str();
      }
    }
  }
}

void test_midi_rate_failure(const std::filesystem::path &specification) {
  constexpr unsigned mismatched_bus_hz = 30000000;
  Fixture f(specification,
            false,
            std::numeric_limits<std::uint64_t>::max(),
            {0, absolute_jump, blofeld::test::boot},
            reference::MidiRoute{.bus_hz = mismatched_bus_hz});
  f.board.initialize();
  const auto before = f.board.status();
  const auto error = rejects([&] { f.board.advance(); });
  require(error.contains("fixed bus rate") && error.contains("address=0x0268") &&
              error.contains("pc=0x9000") && f.board.status().faulted &&
              f.board.status().mcu_cycles == before.mcu_cycles,
          "A supplied MIDI rate cannot silently override native source/divider or advance time");
}

void test_midi_trace_failure(const std::filesystem::path &specification) {
  constexpr unsigned bus_hz = 6000000;
  Fixture f(specification,
            false,
            std::numeric_limits<std::uint64_t>::max(),
            {0, absolute_jump, blofeld::test::boot},
            reference::MidiRoute{.bus_hz = bus_hz});
  f.board.initialize();
  const auto saved = f.board.checkpoint();
  const std::array<std::uint8_t, 1> byte{0x90};
  f.trace.setstate(std::ios::badbit);
  require(rejects([&] { f.board.enqueue_midi(byte); }).contains("trace output failed") &&
              f.board.status().faulted && f.board.status().midi_bytes == 1,
          "Trace failure retains the actual accepted prefix and quarantines board/source together");
  static_cast<void>(rejects([&] { static_cast<void>(f.board.checkpoint()); }));
  f.board.restore(saved);
  f.trace.clear();
  f.board.enqueue_midi(byte);
  require(!f.board.status().faulted && f.board.status().midi_bytes == 1,
          "Trusted joint restore removes the failed unrecorded input prefix before retry");
}

void test_dsp_midi_idle_deadline_and_joint_restore(const std::filesystem::path &specification) {
  constexpr std::uint64_t first = 1000;
  constexpr unsigned partial = 600;
  constexpr unsigned idle_operations = 20;
  constexpr unsigned complete = 6000;
  constexpr unsigned bit_ticks = 17280;
  constexpr unsigned expected_dsp_steps = 4800;
  Fixture f(specification,
            false,
            epoch,
            {0, absolute_jump, blofeld::test::boot},
            std::nullopt,
            reference::DspMidiRoute{.after_ticks = first});
  static_cast<void>(rejects([&] { f.board.enqueue_midi({}); }));
  f.board.initialize();
  const auto initial = f.board.status();
  for (unsigned op = 0; op < idle_operations; ++op) { f.board.advance(); }
  require(f.board.status().midi_dsp_events == 1 && f.board.status().midi_dsp_frames == 0,
          "The bound DSP source observes idle even before host bytes arrive");
  const std::array<std::uint8_t, 2> prefix{0xe0, 0};
  f.board.enqueue_midi(prefix);
  for (unsigned op = idle_operations; op < partial; ++op) { f.board.advance(); }
  const auto saved = f.board.checkpoint();
  const std::array<std::uint8_t, 1> suffix{0x40};
  std::optional<reference::Status> expected;
  std::string expected_trace;
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.board.restore(saved);
    f.trace.str({});
    f.trace.clear();
    f.board.enqueue_midi(suffix);
    for (unsigned op = partial; op < complete; ++op) { f.board.advance(); }
    const auto observed = f.board.status();
    const auto end = (5 + complete) * mcu_period;
    require(
        observed.midi_bytes == 3 && observed.midi_queued == 0 && observed.midi_dsp_frames == 3 &&
            observed.midi_dsp_events == ((end - first) / bit_ticks) + 1 &&
            observed.mcu_cycles == 5 + complete && observed.common_ticks == end &&
            observed.dsp_units == std::uint64_t{2} * complete &&
            observed.dsp_steps == expected_dsp_steps && observed.cold_nvm == initial.cold_nvm,
        "Independent MCU NOP/DSP NOP1-JMP4 clocks and 31250-baud grid retain all three frames/NVM");
    if (expected) {
      require(observed == *expected && f.trace.str() == expected_trace,
              "Partial DSP byte/source/schedule snapshots reproduce actual CPU and full trace");
    } else {
      expected = observed;
      expected_trace = f.trace.str();
    }
  }
}

void test_usb_transport_epoch_does_not_rewind(const std::filesystem::path &specification) {
  Fixture f(specification, false, std::numeric_limits<std::uint64_t>::max());
  const blofeld::UsbipRuntimeOptions options{
      .tcp = {},
      .location = {.path = "/own/usb/1-1", .busid = "1-1", .bus = 1, .device = 1},
      .after_ticks = std::numeric_limits<std::uint64_t>::max(),
      .period_ticks = 1,
      .probe_timeout_ticks = 100};
  blofeld::UsbipRuntime external(options);
  f.board.initialize();
  f.board.poll_usb(external);
  const auto saved = f.board.checkpoint();
  f.board.advance();
  f.board.restore(saved);
  require(
      rejects([&] { f.board.poll_usb(external); }).contains("board owner/restore/time changed") &&
          external.faulted() && f.board.status().faulted,
      "Actual coordinated restore retires its live host session before another USB event");
  f.board.restore(saved);
  blofeld::UsbipRuntime fresh(options);
  f.board.poll_usb(fresh);
  require(!fresh.started() && !fresh.faulted() && !f.board.status().faulted,
          "A new adapter can wait before its declared admission after a trusted board restore");
}
}

int main(const int argc, char **argv) {
  try {
    if (argc == 3 && std::string_view(argv[1]) == "--host-audio") {
      blofeld::test::reference_host_audio(argv[2]);
      return 0;
    }
    if (argc != 2) { throw blofeld::Fault("Usage: reference-checkpoint artifact-directory"); }
    const std::filesystem::path root(argv[1]);
    std::filesystem::create_directories(root);
    const auto specification = root / "checkpoint-HC12.sla";
    std::filesystem::copy_file(
        BLOFELD_DEFAULT_SLA, specification, std::filesystem::copy_options::overwrite_existing);
    test_pending_deadline_fault_and_joint_restore(specification);
    test_held_reset_and_foreign_owner(specification);
    test_failed_preparation_leaves_live_board_unchanged(specification);
    test_active_control_rejections(specification);
    test_replay_marker_rejections(specification);
    test_trace_flush_rejections(specification);
    test_latched_rep_and_written_memory(specification);
    test_midi_wire_checkpoint_and_capacity(specification);
    test_midi_rate_failure(specification);
    test_midi_trace_failure(specification);
    test_dsp_midi_idle_deadline_and_joint_restore(specification);
    test_usb_transport_epoch_does_not_rewind(specification);
    std::cout
        << "Joint reference-board checkpoints, atomic restore, real REP/RAM and guards passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
