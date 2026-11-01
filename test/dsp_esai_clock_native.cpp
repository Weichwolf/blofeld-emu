// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "dsp_rom_program.hpp"
#include "esai_clock_fixture.hpp"
#include "esai_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>

namespace blofeld::test::esai_clock {
namespace {
using esai::rejects;
using esai::require;
constexpr unsigned secondary_base = 0xffff80;
constexpr unsigned mux = 0x838, port_control = 31, port_direction = 30;
constexpr std::uint64_t common_hz = 138240000000, unit_ticks = 11520, half_period = 5625;
constexpr std::uint64_t reset_hold = 2 * unit_ticks, reset_end = 4 * unit_ticks;
// First immediate MOVEP writes in period4, then publishes PCU and retires.
// Each following immediate takes3 periods: direction writes in period13 at
// 195840. Rising HCKT196875/208125/219375/230625 then toggle SCKT twice.
// TE writes in period19 at264960; frame0 has started, so data waits for frame1.
constexpr std::uint64_t first_unenabled_launch = 230625, first_data = 3110625;
constexpr std::uint64_t data_period = 45000;
constexpr std::uint64_t capture = first_data + (6 * data_period) + (2 * half_period) + 1;
constexpr std::uint64_t finish = first_data + (63 * data_period) + (4 * half_period);
constexpr unsigned padding = 128, budget = 1024, maximum_events = 2048;
constexpr auto execution =
    OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = common_hz,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};
enum class ClockOrder : std::uint8_t { native_first, input_first };

DspRomProgram program(const bool second, const unsigned delay) {
  // FM immediate low-X/low-Y MOVEP and split-qq x0 status-read encodings.
  constexpr unsigned primary_write = 0x07f420;
  constexpr unsigned secondary_write = 0x07b480;
  constexpr unsigned primary_status = 0x0444d3;
  constexpr unsigned secondary_status = 0x044433;
  const auto put = second ? secondary_write : primary_write;
  DspRomProgram result;
  result.append(std::array{put + esai::control,
                           word32,
                           put + esai::clock,
                           base_clock | clock_polarity,
                           put + port_control,
                           mux,
                           put + port_direction,
                           mux,
                           put,
                           esai::other,
                           put + esai::control,
                           word32 | 1U});
  for (unsigned i = 0; i < delay; ++i) { result.word(0); }
  result.word(second ? secondary_status : primary_status);
  result.append(std::array{put, esai::input});
  const auto loop = result.pc();
  constexpr unsigned jump_absolute = 0x0af080;
  result.append(std::array{0U, jump_absolute, loop});
  return result;
}

struct Wire {
  DspEsaiInstance instance;
  bool enabled;
  std::uint64_t cursor = 0;
  std::string bits;
  bool disabled_launch = false;

  void observe(OwnDspMachine &machine, const std::uint64_t ticks) {
    const auto previous = machine.bus().esai_clock_outputs(instance);
    const auto registers = machine.cpu().registers();
    const auto progress = machine.cpu().progress();
    const auto elapsed = machine.clock().elapsed_ticks();
    machine.bus().observe_esai_transmit(instance,
                                        {.high_clock = cursor % 2 != 0, .timing = timing});
    const auto outputs = machine.bus().esai_clock_outputs(instance);
    if (!disabled_launch && (previous.known & outputs.known & 1U) != 0 &&
        (previous.levels & 1U) != 0 && (outputs.levels & 1U) == 0) {
      require(ticks == first_unenabled_launch && outputs.driven == 3 && outputs.levels == 2 &&
                  machine.bus().esai_transmit_outputs(instance).driven == 0,
              "Native mux/direction start a complete frame before the real TE write");
      disabled_launch = true;
    }
    // TCKP1 launches falling; an independent receiver samples rising.
    if ((previous.known & outputs.known & 1U) != 0 && (previous.levels & 1U) == 0 &&
        (outputs.levels & 1U) != 0) {
      const auto data = machine.bus().esai_transmit_outputs(instance);
      if ((data.driven & 1U) != 0) {
        require(data.known == 1, "The receiver requires an explicitly known data word");
        bits.push_back(esai::sample(data, 0));
        const auto expected = first_data + (4 * half_period) + ((bits.size() - 1) * data_period);
        require(ticks == expected,
                "Every captured bit retains the independently derived divider/frame phase");
      }
    }
    require(machine.cpu().registers() == registers && machine.cpu().progress() == progress &&
                machine.clock().elapsed_ticks() == elapsed,
            "HCKT and output observations cannot execute native instructions or charge time");
    ++cursor;
  }

  void drain(OwnDspMachine &machine, const std::uint64_t through, const bool inclusive = true) {
    while (enabled) {
      const auto next = cursor * half_period;
      if (next > through || (!inclusive && next == through)) { return; }
      require(cursor < maximum_events, "The explicit clock source has a finite event budget");
      observe(machine, next);
    }
  }
};

struct ClockSnapshot {
  OwnDspTimelineCheckpoint timeline;
  Wire wire;
  std::uint64_t cycles, horizon;
};

class ClockRun {
public:
  ClockRun(const bool second,
           const ClockOrder order,
           const bool source = true,
           const unsigned delay = padding)
      : wire_{.instance = second ? DspEsaiInstance::secondary : DspEsaiInstance::primary,
              .enabled = source,
              .bits = {}},
        order_(order) {
    const auto code = program(second, delay);
    machine.bus().install_rom({.start = DspRomProgram::boot, .words = code.words()});
    timeline = std::make_unique<OwnDspTimeline>(
        machine,
        execution,
        0,
        OwnDspResetInputs{
            .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
    wire_.drain(machine, reset_hold);
    require(timeline->advance_reset_to(reset_hold) == reset_hold, "Actual reset hold completes");
    timeline->release_hardware_reset();
    wire_.drain(machine, reset_end);
    require(timeline->advance_reset_to(reset_end) == reset_end - reset_hold &&
                !machine.clock().hardware_reset_active(),
            "Actual reset recovery completes before any native instruction");
  }

  void advance_to(const std::uint64_t horizon) {
    require(horizon >= horizon_, "The native clock experiment cannot rewind common time");
    for (unsigned count = 0; count < budget; ++count) {
      const auto deadline = timeline->next_deadline();
      const auto through = std::min(deadline, horizon);
      wire_.drain(machine, through, order_ == ClockOrder::input_first || deadline > horizon);
      const auto port = wire_.instance == DspEsaiInstance::primary ? DspPort::c : DspPort::e;
      const auto selected = machine.bus().gpio(port).outputs().peripheral;
      const auto result = timeline->advance_one(through,
                                                OwnDspBoundary::through,
                                                {.cycle = cycles_,
                                                 .timing = DspDmaTiming::reference_sampled_cycles,
                                                 .grant = DspDmaGrant::available});
      if (!result) {
        horizon_ = horizon;
        return;
      }
      if (selected == 0 && machine.bus().gpio(port).outputs().peripheral != 0) {
        constexpr unsigned direction_period = 13;
        require(deadline == reset_end + (direction_period * unit_ticks) &&
                    machine.bus().gpio(port).outputs().peripheral == mux,
                "The actual native direction write retains its independently expected period");
      }
      ++cycles_;
      horizon_ = deadline;
      if (result->native && result->native->fault.kind != dsp56300::FaultKind::none) {
        native_fault(result->native->fault);
      }
      if (order_ == ClockOrder::native_first) { wire_.drain(machine, through); }
    }
    throw Fault("Native ESAI clock experiment exceeded its period budget");
  }

  [[nodiscard]] ClockSnapshot checkpoint() const {
    return {
        .timeline = timeline->checkpoint(), .wire = wire_, .cycles = cycles_, .horizon = horizon_};
  }

  void restore(const ClockSnapshot &saved) {
    timeline = std::make_unique<OwnDspTimeline>(machine, saved.timeline);
    wire_ = saved.wire;
    cycles_ = saved.cycles;
    horizon_ = saved.horizon;
  }

  [[nodiscard]] const Wire &wire() const { return wire_; }

  [[nodiscard]] OwnDspMachine &chip() { return machine; }

  [[nodiscard]] std::uint64_t next_deadline() const { return timeline->next_deadline(); }

  [[nodiscard]] bool faulted() const { return timeline->faulted(); }

private:
  OwnDspMachine machine{clocks};
  std::unique_ptr<OwnDspTimeline> timeline;

  [[noreturn]] void native_fault(const dsp56300::Fault fault) const {
    std::string cause;
    const auto &failure = machine.io().failure();
    if (failure && failure->cause) {
      try {
        std::rethrow_exception(failure->cause);
      } catch (const std::exception &error) { cause = error.what(); }
    }
    constexpr std::array spaces{"P:", "X:", "Y:"};
    const bool read = fault.kind == dsp56300::FaultKind::memory_read ||
                      fault.kind == dsp56300::FaultKind::io_read;
    const auto *operation = read ? "read" : "native";
    throw Fault("Native ESAI " + std::string(operation) + " " +
                spaces.at(static_cast<unsigned>(fault.space)) + hex(fault.address, 6) +
                " pc=" + hex(fault.pc, 6) +
                " tick=" + std::to_string(machine.clock().elapsed_ticks()) + ": " + cause);
  }

  Wire wire_;
  ClockOrder order_;
  std::uint64_t cycles_ = 0, horizon_ = reset_end;
};

void run_native(const bool second, const ClockOrder order) {
  ClockRun run(second, order);
  run.advance_to(capture);
  require(run.wire().disabled_launch && run.wire().bits == "000100" &&
              run.chip().cpu().registers().x == esai::empty_even_frame &&
              run.next_deadline() > capture,
          "Actual status/refill occurs in the first native word before a partial-phase snapshot");
  const auto saved = run.checkpoint();

  struct CpuView {
    dsp56300::Registers registers;
    dsp56300::Progress progress;
    bool operator==(const CpuView &) const = default;
  };

  std::optional<CpuView> expected;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved);
    run.advance_to(finish);
    require(run.wire().bits == word(false, true) + word(true, true) &&
                run.wire().cursor == finish / half_period + 1,
            "Native/timeline/divider/source restoration reproduces all64 independent bits");
    // Actual reference periods, including four reset periods, end strictly
    // before this final external edge; no pending future period may be charged.
    constexpr unsigned periods = 514;
    constexpr std::uint64_t completed = 5967360;
    constexpr std::uint64_t next = 5978880;
    require(run.chip().clock().native_cycle_units() == periods &&
                run.chip().clock().elapsed_ticks() == completed && run.next_deadline() == next,
            "The independent reference-period ledger preserves the final partial native period");
    const CpuView actual{.registers = run.chip().cpu().registers(),
                         .progress = run.chip().cpu().progress()};
    if (!expected) { expected = actual; }
    require(actual == expected.value(),
            "Partial native execution and exact external phase restore complete CPU state");
  }
}

void pad_contracts() {
  DspBus bus;
  constexpr auto instance = DspEsaiInstance::secondary;
  const auto put = [&](const unsigned offset, const unsigned value) {
    bus.write({.space = DspSpace::y, .address = secondary_base + offset, .value = value});
  };
  const auto observe = [&](const bool clock) {
    bus.observe_esai_transmit(instance, {.high_clock = clock, .timing = timing});
  };
  observe(false);
  put(esai::clock, base_clock);
  put(esai::control, word32 | 1U);
  put(0, esai::other);
  constexpr unsigned without_high_clock = 0x818;
  put(port_control, without_high_clock);
  put(port_direction, without_high_clock);
  require(rejects([&] { observe(true); }).contains("HCKT"),
          "A divided clock requires the actual high-clock input pad selected as a peripheral");
  put(port_control, mux);
  put(port_direction, mux);
  constexpr unsigned bit_pad = 8;
  bus.gpio(DspPort::e).set_drive({.mask = bit_pad, .levels = bit_pad});
  require(rejects([&] { observe(true); }).contains("Y:0xffff9d"),
          "Opposed external SCKT drive fails against the selected generated output");
  bus.gpio(DspPort::e).set_drive({});
  observe(true);
  observe(false);
  constexpr unsigned frame_pad = 16;
  bus.gpio(DspPort::e).set_drive({.mask = frame_pad});
  static_cast<void>(rejects([&] { observe(true); }));
  bus.gpio(DspPort::e).set_drive({});
  observe(true);
  require(bus.esai_clock_outputs(instance).levels == 3 &&
              esai::sample(bus.esai_transmit_outputs(instance), 0) == '0',
          "Rejected clock/frame contention commits neither the divider edge nor the first bit");
}

void missing_clock_recovery() {
  constexpr std::uint64_t setup_end = reset_end + (21 * unit_ticks);
  // SAISR read, destination publication, PCU publication, then retirement.
  constexpr unsigned read_periods = 25;
  constexpr std::uint64_t read_end = reset_end + (read_periods * unit_ticks);
  for (const bool second : {false, true}) {
    ClockRun run(second, ClockOrder::input_first, false, 0);
    run.advance_to(setup_end);
    const auto saved = run.checkpoint();
    const auto error = rejects([&] { run.advance_to(read_end); });
    require(error.contains("pc=0xff000c") && error.contains(second ? "Y:0xffff93" : "X:0xffffb3") &&
                error.contains("timing") && run.faulted(),
            "The actual native status read retains missing-clock operation/address/PC/time");
    run.restore(saved);
    run.chip().bus().observe_esai_transmit(second ? DspEsaiInstance::secondary
                                                  : DspEsaiInstance::primary,
                                           {.high_clock = false, .timing = timing});
    run.advance_to(read_end);
    constexpr unsigned instructions = 7;
    constexpr unsigned base_cost = 13;
    require(run.chip().cpu().registers().x == 0 &&
                run.chip().cpu().progress().instructions == instructions &&
                run.chip().cpu().progress().estimated_cycles == base_cost &&
                run.chip().clock().native_cycle_units() == read_periods,
            "Trusted restore and an explicit frozen baseline recover only real reset status");
  }
}
}

void native_clock() {
  for (const auto order : {ClockOrder::native_first, ClockOrder::input_first}) {
    run_native(false, order);
    run_native(true, order);
  }
  pad_contracts();
  missing_clock_recovery();
}
}
