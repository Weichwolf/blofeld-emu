// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/reference_board.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu_flash_live_fixture.hpp"
#include "storage/flash_image.hpp"
#include "storage/private_file.hpp"
#include "storage/virtual_device_state.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned common_hz = 540000000, operation_limit = 10000;
constexpr unsigned observation_interval = 17, benchmark_reads = 128;
constexpr unsigned serial_pins = 0x16, expected_marker_shift = 8;

reference::Config settings() {
  reference::Config result;
  result.mcu = live::contract();
  result.dsp_clock = {.extal_hz = live::oscillator_hz,
                      .timebase_hz = common_hz,
                      .switch_timing = DspClockSwitch::reference_cycle_boundary};
  result.dsp = {.timing =
                    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
                .maximum_cycles_per_period = 16,
                .writes = McuWriteOrder::unqualified,
                .external_writes = McuWriteOrder::unqualified,
                .wait_dsp_first = true,
                .reset_dsp_first = true,
                .initial_reset = OwnDspResetInputs{
                    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  result.dma_grant = DspDmaGrant::unknown;
  result.power_on = reference::PowerOnTiming{.inputs = {3}, .reset_pad = McuResetPad::high};
  result.release_ticks = std::numeric_limits<std::uint64_t>::max();
  result.operations = operation_limit;
  result.select_n = 2;
  result.clock = 4;
  result.mosi = 16;
  result.initial = {{.port = McuPort::b, .drive = {.mask = serial_pins, .levels = 2}},
                    {.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  result.dsp_input_order = DspPeerOrder::peer_first;
  result.maximum_dsp_inputs_per_advance = 1;
  return result;
}

std::string bytes(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  live::require(input.good(), "Owned test unit remains readable");
  return {std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
}

struct Unit {
  Unit(const std::filesystem::path &path,
       const live::Code &algorithm,
       const live::Scenario scenario)
      : initial(live::image(algorithm, scenario)),
        config(settings()),
        owner(std::make_shared<VirtualDeviceState>(path, initial, [] { return "EMUSTAT0001"; })),
        installed(owner->image()),
        execution{.image = installed,
                  .rom = rom,
                  .specification = BLOFELD_DEFAULT_SLA,
                  .flash_out = {},
                  .trace = &trace,
                  .instructions = operation_limit,
                  .device_state = owner},
        board(config, execution),
        file(path) {
    owner->set_observer([this](const PrivateFilePhase phase) {
      if (phase == PrivateFilePhase::created) { ++publications; }
    });
  }

  McuFlashImage initial;
  reference::Config config;
  std::vector<std::uint32_t> rom;
  std::ostringstream trace;
  unsigned publications = 0;
  std::shared_ptr<VirtualDeviceState> owner;
  McuFlashImage installed;
  reference::Execution execution;
  reference::Board board;
  std::filesystem::path file;
};

void observation(Unit &unit) {
  const auto full = unit.board.status();
  const auto original_file = bytes(unit.file);
  const auto original_trace = unit.trace.str();
  const auto publications = unit.publications;
  auto expected = full;
  live::require(!expected.cold_nvm.empty(),
                "Existing status retains its complete cold flash image");
  expected.cold_nvm.clear();
  for (unsigned index = 0; index < 4; ++index) {
    const auto runtime = unit.board.runtime_status();
    live::require(runtime.cold_nvm.empty() && runtime == expected,
                  "Runtime observation preserves every field while explicitly omitting cold NVM");
  }
  live::require(unit.board.status() == full && unit.trace.str() == original_trace &&
                    unit.publications == publications && bytes(unit.file) == original_file,
                "Observation changes no native state, trace, cold cut or live file publication");
}

void completed(const Unit &unit, const unsigned idle_pc) {
  const auto state = unit.board.status();
  const auto image = decode_flash_image(state.cold_nvm);
  constexpr unsigned low_byte = 0xff;
  const auto offset = live::target - live::entry;
  live::require(state.mcu_state_known && state.mcu.pc == idle_pc && !state.waiting &&
                    state.held_reset && state.dsp_steps == 0 && !state.faulted &&
                    (state.mcu.d >> expected_marker_shift) == live::signature,
                "Actual authored POR/flash helper reaches its own final native branch");
  live::require(image.known[offset] && image.known[offset + 1] &&
                    image.bytes[offset] == (live::word_data >> expected_marker_shift) &&
                    image.bytes[offset + 1] == (live::word_data & low_byte) &&
                    unit.publications != 0,
                "The genuine guest flash command completes its independently specified word");
}

void benchmark(Unit &unit) {
  using Clock = std::chrono::steady_clock;
  const auto before = unit.board.status();
  std::uint64_t full_operations = 0;
  std::uint64_t runtime_operations = 0;
  std::uint64_t serialized_bytes = 0;
  const auto full_start = Clock::now();
  for (unsigned index = 0; index < benchmark_reads; ++index) {
    const auto state = unit.board.status();
    full_operations += state.operations;
    serialized_bytes += state.cold_nvm.size();
  }
  const auto full_end = Clock::now();
  for (unsigned index = 0; index < benchmark_reads; ++index) {
    const auto state = unit.board.runtime_status();
    live::require(state.cold_nvm.empty(), "Runtime query never returns an invented cold image");
    runtime_operations += state.operations;
  }
  const auto runtime_end = Clock::now();
  live::require(full_operations == runtime_operations && unit.board.status() == before,
                "Both host-only observation loops leave every emulated counter unchanged");
  std::cout << "OBSERVATION reads=" << benchmark_reads << " serialized-bytes=" << serialized_bytes
            << " full-ns="
            << std::chrono::duration_cast<std::chrono::nanoseconds>(full_end - full_start).count()
            << " runtime-ns="
            << std::chrono::duration_cast<std::chrono::nanoseconds>(runtime_end - full_end).count()
            << '\n';
}

void exercise(const std::filesystem::path &root, const bool wait) {
  const live::Scenario scenario{.wait = wait};
  const auto algorithm = live::helper(scenario);
  const auto idle_pc = live::ram + static_cast<unsigned>(algorithm.size()) - 2;
  const char *const prefix = wait ? "wait" : "poll";
  Unit observed(root / (std::string(prefix) + "-observed.bfnvm"), algorithm, scenario);
  Unit control(root / (std::string(prefix) + "-control.bfnvm"), algorithm, scenario);
  observation(observed);
  observed.board.initialize();
  control.board.initialize();
  observation(observed);
  bool seen_wait = false;
  bool finished = false;
  for (unsigned operation = 0; operation < operation_limit; ++operation) {
    observed.board.advance();
    control.board.advance();
    const auto runtime = observed.board.runtime_status();
    seen_wait = seen_wait || runtime.waiting;
    if (operation % observation_interval == 0) { observation(observed); }
    if (runtime.mcu_state_known && runtime.mcu.pc == idle_pc) {
      finished = true;
      break;
    }
  }
  live::require(finished && (!wait || seen_wait),
                "The actual bounded program reaches completion, including its declared WAI");
  completed(observed, idle_pc);
  completed(control, idle_pc);
  observation(observed);
  live::require(observed.board.status() == control.board.status() &&
                    observed.trace.str() == control.trace.str(),
                "Frequent runtime observations preserve the entire unobserved guest execution");
  benchmark(observed);
  std::cout << "COMPLETE wait=" << wait << " actual-flash-word=" << live::word_data
            << " state-and-trace-identical=1\n";
}
}

int main(const int count, char **arguments) {
  try {
    live::require(count == 2, "Expected a fresh owned artifact directory");
    const std::filesystem::path base(arguments[1]);
    std::filesystem::create_directories(base);
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto root = base / ("run-" + std::to_string(stamp));
    live::require(std::filesystem::create_directory(root),
                  "Each test execution owns a fresh directory");
    exercise(root, false);
    exercise(root, true);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
