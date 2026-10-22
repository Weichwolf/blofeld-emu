// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_esai.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/host_audio.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "host_audio_fixture.hpp"
#include "reference_board.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace blofeld::test {
namespace {
namespace ref = blofeld::reference;
constexpr unsigned oscillator = 12000000;
constexpr std::uint64_t common = 15360000000;
constexpr unsigned rate = 48000;
constexpr unsigned frames = 64;

void check(const bool valid, const std::string_view reason) {
  if (!valid) { throw Fault(std::string(reason)); }
}

void rejects(const auto &operation, const std::string_view reason, const bool context = false) {
  try {
    operation();
  } catch (const Fault &error) {
    check(std::string_view(error.what()).contains(reason), "Unexpected board audio diagnostic");
    if (context) {
      const std::string_view detail(error.what());
      check(detail.contains("mcu-pc=") && detail.contains("dsp-pc="),
            "Native audio failure must retain both execution contexts");
    }
    return;
  }
  throw Fault("Expected board audio boundary rejection");
}

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  bytes.fill(nop);
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned byte_mask = 0xff;
  bytes.at(protection) = byte_mask;
  constexpr unsigned unsecured = 0xfe;
  bytes.at(security) = unsecured;
  constexpr unsigned reset_high = 0x7ffe;
  constexpr unsigned reset_page = 0x90;
  bytes.at(reset_high) = reset_page;
  bytes.back() = 0;
  // Own LDS #$2000; WAI. Both cores continue under the actual shared scheduler.
  constexpr std::array<std::uint8_t, 4> caller{0xcf, 0x20, 0, 0x3e};
  constexpr unsigned entry_offset = 0x1000;
  for (unsigned i = 0; i < caller.size(); ++i) { bytes.at(entry_offset + i) = caller.at(i); }
  return McuFlashImage::from_known(bytes);
}

ref::Config config(const unsigned instance, const unsigned frame_count = frames) {
  ref::Config result;
  result.mcu = {.clock = {.oscillator_hz = oscillator,
                          .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                .memory_timing = McuRunMemory::internal_flash_reads};
  result.dsp = {.clock = {.extal_hz = oscillator,
                          .timebase_hz = common,
                          .switch_timing = DspClockSwitch::reference_instruction_boundary},
                .timing = DspRunTiming::native_internal_estimates};
  result.schedule = {.dsp = {.observation = DspStepObservation::reference_completion_boundary},
                     .shared_write_order = McuDspTieOrder::dsp_before_mcu,
                     .maximum_dsp_steps_per_cycle = 16,
                     .wait_order = McuDspWaitOrder::dsp_before_mcu};
  constexpr unsigned release = 12800;
  result.release_ticks = release;
  constexpr unsigned cycles_per_frame = 125;
  constexpr unsigned trailing_operations = 1000;
  result.operations = (frame_count * cycles_per_frame) + trailing_operations;
  result.select_n = 2;
  result.clock = 4;
  result.mosi = 16;
  constexpr unsigned serial_pins = 0x16;
  result.initial = {{.port = McuPort::b, .drive = {.mask = serial_pins, .levels = 2}},
                    {.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  result.dsp_initial = {
      {.kind = ref::DspInputKind::esai,
       .index = instance,
       .esai = {.high_clock = false, .timing = DspEsaiClockTiming::reference_reset_edges}}};
  result.dsp_input_order = DspExternalOrder::external_before_dsp;
  result.maximum_dsp_inputs_per_advance = 16;
  constexpr unsigned clock_start = 100000;
  constexpr unsigned half_period = 625;
  constexpr unsigned capture_start = 60000;
  result.dsp_clocks = {{.start_ticks = clock_start,
                        .half_period_ticks = half_period,
                        .edges = (frame_count * 512) + 4,
                        .index = instance}};
  result.pcm = ref::PcmRoute{.instance = instance, .after_ticks = capture_start, .format = {}};
  return result;
}

void case_audio(const char *server, const unsigned instance) {
  const auto flash = image();
  const auto program = host_audio_native_program(instance != 0);
  const auto configuration = config(instance);
  constexpr unsigned instruction_budget = 10000;
  const ref::Execution execution{.image = flash,
                                 .rom = program,
                                 .specification = BLOFELD_DEFAULT_SLA,
                                 .flash_out = {},
                                 .instructions = instruction_budget,
                                 .live_audio = true};
  ref::Board board(configuration, execution);
  const HostAudioTarget target{.server = server, .sink = {}};
  HostAudio old({}, {.timebase_hz = common, .sample_rate = rate}, target);
  rejects([&] { board.poll_audio(old); }, "initialized board");
  board.initialize();
  while (true) {
    const auto state = board.status();
    if (!state.pending_audio) { throw Fault("Missing configured native audio FIFO"); }
    if (state.pending_audio->size() != 0) { break; }
    board.advance();
  }
  const auto saved = board.checkpoint();
  const auto prefix = board.status();
  board.poll_audio(old);
  auto after = board.status();
  after.pending_audio = prefix.pending_audio;
  check(after == prefix && old.queued_frames() == 1 && old.frames_written() == 0,
        "Output extraction cannot change any native/clock/peripheral/NVM state");
  board.restore(saved);
  const auto restored = board.status();
  rejects([&] { board.poll_audio(old); }, "board owner/restore/time changed");
  check(old.faulted() && old.frames_written() == 0 && board.status() == restored,
        "Restore retires old playback before I/O and preserves the entire restored queue/state");
  HostAudio audio({}, {.timebase_hz = common, .sample_rate = rate}, target);
  while (!board.finished()) {
    board.advance();
    board.poll_audio(audio);
  }
  board.poll_audio(audio);
  const auto completed = board.status();
  check(completed.pcm && completed.pending_audio && completed.pcm->completed_frames() == frames &&
            completed.pending_audio->size() == 0,
        "The two real cores and finite ESAI clocks produce all 64 frames into the host queue");
  audio.finish();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!audio.finished()) {
    check(std::chrono::steady_clock::now() < deadline, "Board audio drain timed out");
    board.poll_audio(audio);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  check(audio.frames_written() == frames && !audio.faulted(),
        "All native board frames were submitted");
  std::cout << "BOARD " << instance << ' ' << audio.frames_written() << '\n' << std::flush;
}

void overflow() {
  const auto flash = image();
  const auto program = host_audio_native_program(false);
  constexpr unsigned generated_frames = 300;
  const auto configuration = config(0, generated_frames);
  constexpr unsigned instruction_budget = 10000;
  const ref::Execution execution{.image = flash,
                                 .rom = program,
                                 .specification = BLOFELD_DEFAULT_SLA,
                                 .flash_out = {},
                                 .instructions = instruction_budget,
                                 .live_audio = true};
  ref::Board board(configuration, execution);
  const auto nvm = board.status().cold_nvm;
  rejects(
      [&] {
        while (!board.finished()) { board.advance(); }
      },
      "live PCM queue exhausted",
      true);
  const auto state = board.status();
  constexpr unsigned retained = 256;
  constexpr std::int32_t first_left = -5913697;
  constexpr std::int32_t right = 1193046;
  check(state.pending_audio && state.pcm && state.faulted &&
            state.pending_audio->size() == retained &&
            state.pcm->completed_frames() == retained + 1 && state.cold_nvm == nvm &&
            state.pending_audio->peek()->samples[0] == first_left &&
            state.pending_audio->peek(retained - 1)->samples[1] == right,
        "Unconsumed native output fails on frame257 and retains its complete bounded prefix/NVM");
  std::cout << "OVERFLOW 256\n" << std::flush;
}
}

void reference_host_audio(const char *server) {
  case_audio(server, 0);
  case_audio(server, 1);
  overflow();
}
}
