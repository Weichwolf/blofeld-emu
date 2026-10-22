// SPDX-License-Identifier: GPL-3.0-only
#include "audio_runtime_fixture.hpp"
#include "blofeld/audio_runtime.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/flash_file.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/pcm_sink.hpp"
#include "blofeld/serial_pcm.hpp"
#include "reference_board.hpp"
#include "serial_pcm_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace blofeld::test::audio_runtime {
namespace {
struct Probe final : PcmFrameSink {
  void accept(const SerialPcmFrame &frame) override {
    check();
    if (frames.size() == capacity) { throw Fault("Injected output capacity failure"); }
    frames.push_back(frame);
  }

  void check() const override {
    if (retired) { throw Fault("Probe PCM output retired"); }
  }

  void invalidate() noexcept override { retired = true; }

  std::vector<SerialPcmFrame> frames;
  std::size_t capacity = 2;
  bool retired = false;
};

struct Machine {
  Machine(reference::Config configuration,
          std::vector<std::uint32_t> program,
          const McuFlashImage &image)
      : config(std::move(configuration)),
        rom(std::move(program)),
        execution{.image = image,
                  .rom = rom,
                  .specification = BLOFELD_DEFAULT_SLA,
                  .flash_out = {},
                  .trace = &trace,
                  .instructions = instruction_budget},
        board(config, execution) {}

  static constexpr unsigned instruction_budget = 1000;
  reference::Config config;
  std::vector<std::uint32_t> rom;
  std::ostringstream trace;
  reference::Execution execution;
  reference::Board board;
};

void complete(reference::Board &board) {
  while (!board.finished()) { board.advance(); }
}

void expected_frames(const Probe &probe) {
  constexpr std::int32_t first = -5913697;
  constexpr std::int32_t next = 1193046;
  constexpr std::uint64_t first_tick_expected = 568250;
  constexpr std::uint64_t first_end = 3403250;
  constexpr std::uint64_t next_tick = 3448250;
  constexpr std::uint64_t next_end = 6283250;
  pcm::require(probe.frames.size() == 2 && probe.frames[0].samples[0] == first &&
                   probe.frames[0].samples[1] == next && probe.frames[1].samples[0] == next &&
                   probe.frames[1].samples[1] == next &&
                   probe.frames[0].first_tick == first_tick_expected &&
                   probe.frames[0].last_tick == first_end &&
                   probe.frames[1].first_tick == next_tick && probe.frames[1].last_tick == next_end,
               "Actual coupled MCU/DSP/ESAI output must retain literal serial words and ticks");
}

AudioRuntimeOptions output_options(const reference::Config &config, const std::string &server) {
  if (!config.pcm) { throw Fault("Audio board fixture lacks a PCM route"); }
  return {.output = {.pcm = config.pcm->format,
                     .timebase_hz = config.dsp.clock.timebase_hz,
                     .sample_rate = sample_rate,
                     .server = server}};
}

void restore_and_rebind(Machine &machine, AudioRuntime &output) {
  const auto checkpoint = machine.board.checkpoint();
  while (!machine.board.status().last_pcm) { machine.board.advance(); }
  pcm::require(output.status().accepted == 1, "The live prefix is external to modeled checkpoints");
  machine.board.restore(checkpoint);
  pcm::require(pcm::rejects([&] { output.finish(); }).find("retired") != std::string::npos,
               "Restoring native owners retires rather than rewinds the host stream");
  const auto failure = pcm::rejects([&] { machine.board.advance(); });
  pcm::require(failure.find("retired") != std::string::npos &&
                   failure.find("mcu-pc=") != std::string::npos &&
                   failure.find("dsp-pc=") != std::string::npos,
               "A stale external binding fails with the inactive board's execution context");
  machine.board.restore(checkpoint);
}

void controls(Machine &machine, Probe &probe, const std::string &mode) {
  if (mode == "board-reject") { probe.capacity = 0; }
  machine.board.attach_pcm_output(probe);
  const auto failure = pcm::rejects([&] { complete(machine.board); });
  pcm::require(probe.frames.empty() && probe.retired && machine.board.status().faulted,
               "Unknown source data and output rejection retire the sink without invented frames");
  pcm::require(failure.find("clock=") != std::string::npos &&
                   failure.find("data=") != std::string::npos &&
                   failure.find("mcu-pc=") != std::string::npos &&
                   failure.find("dsp-pc=") != std::string::npos,
               "PCM failures retain register operation, native PCs and execution context");
}

void binding_controls(const reference::Config &config,
                      const std::vector<std::uint32_t> &program,
                      const McuFlashImage &image) {
  Probe probe;
  auto missing = config;
  missing.pcm.reset();
  Machine without_route(missing, program, image);
  pcm::require(pcm::rejects([&] {
                 without_route.board.attach_pcm_output(probe);
               }).find("decoded PCM route") != std::string::npos,
               "An external sink cannot invent a board output route");
  auto replay = config;
  replay.checkpoint = reference::ReplayPlan{.operation = 1, .replays = 1};
  Machine automatic(replay, program, image);
  pcm::require(pcm::rejects([&] {
                 automatic.board.attach_pcm_output(probe);
               }).find("automatic checkpoint replay") != std::string::npos,
               "Unselected checkpoint branches cannot escape as irreversible audio");
  Machine valid(config, program, image);
  valid.board.attach_pcm_output(probe);
  pcm::require(pcm::rejects([&] { valid.board.attach_pcm_output(probe); }).find("already bound") !=
                       std::string::npos &&
                   !probe.retired,
               "A duplicate binding rejects without retiring its still-valid sink");
  Probe invalid;
  invalid.invalidate();
  pcm::require(pcm::rejects([&] { valid.board.attach_pcm_output(invalid); }).find("retired") !=
                       std::string::npos &&
                   !probe.retired,
               "An invalid replacement must preserve the old healthy binding");
}
}

void board_cases(const BoardInputs &inputs) {
  const auto &mode = inputs.mode;
  const auto &server = inputs.server;
  auto config = reference::load_config(inputs.configuration);
  const auto program = reference::load_rom(inputs.rom);
  const auto image = load_flash_image(inputs.flash);
  if (!config.pcm) { throw Fault("Audio board fixture lacks a PCM route"); }
  if (mode == "board-binding") {
    binding_controls(config, program, image);
    return;
  }
  if (mode == "board-unknown") { config.pcm->lane = 1; }
  if (mode == "board-unknown" || mode == "board-reject") {
    Probe probe;
    Machine machine(config, program, image);
    controls(machine, probe, mode);
    return;
  }
  std::unique_ptr<AudioRuntime> replacement;
  AudioRuntime initial(output_options(config, server));
  Machine machine(config, program, image);
  Probe probe;
  Machine baseline(config, program, image);
  baseline.board.attach_pcm_output(probe);
  complete(baseline.board);
  expected_frames(probe);
  machine.board.attach_pcm_output(initial);
  if (mode == "board-restore") {
    restore_and_rebind(machine, initial);
    replacement = std::make_unique<AudioRuntime>(output_options(config, server));
    machine.board.attach_pcm_output(*replacement);
    complete(machine.board);
    replacement->finish();
    pcm::require(replacement->status().written == 2,
                 "A new external epoch delivers the replay once");
  } else {
    complete(machine.board);
    initial.finish();
    pcm::require(initial.status().written == 2, "The host receives actual board-decoded PCM");
  }
  pcm::require(machine.board.status() == baseline.board.status(),
               "The live worker cannot change native clocks, instructions, state or decoded PCM");
  std::cout << "board frames=2 exact-native-state=1 original-firmware-audio=unqualified\n";
}
}
