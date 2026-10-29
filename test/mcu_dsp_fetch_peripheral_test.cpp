// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_fetch_schedule.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned oscillator = 12000000, common_hz = 540000000, byte_mask = 0xff;
constexpr unsigned nop = 0xa7, protection = 0x7f0d, security = 0x7f0f, unsecured = 0xfe;
constexpr unsigned entry = 0x8000, marker = 0x42, expected_ccr = 0xd0;
constexpr std::uint64_t collision = 720, later = 900, native_epoch = 450;
constexpr unsigned dsp_nops = 32, initial_status = 0xc00300;

void require(const bool value, const std::string_view reason) {
  if (!value) { throw Fault(std::string(reason)); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected native MCU/peripheral refusal");
}

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  // Own independent HCS12 LDAB #42; STAB direct PORTB (one plus two periods).
  const std::array<std::uint8_t, 4> program{0xc6, marker, 0x5b, 1};
  bytes.fill(nop);
  for (unsigned i = 0; i < program.size(); ++i) { bytes.at(i) = program.at(i); }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract contract() {
  return {.clock = {.oscillator_hz = oscillator,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

McuDspFetchInputs settings(const McuExternalWriteOrder order,
                           const bool running = false,
                           const McuDspTieOrder native = McuDspTieOrder::dsp_before_mcu) {
  McuDspFetchInputs result{
      .dsp = {.extal_hz = oscillator,
              .timebase_hz = common_hz,
              .timing = DspFetchTiming::reference_cycle_order,
              .execution = DspReferenceExecution::staged_register_boundaries,
              .pipeline = DspFetchPipeline::automatic_memory,
              .resets = DspFetchResetTiming::reference_cycle_boundary},
      .epoch_ticks = running ? native_epoch : 0,
      .shared_write_order = native,
      .maximum_dsp_cycles_per_period = 16,
      .external_write_order = order,
      .initial_reset = McuDspFetchInitialReset{
          .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  if (running) { result.initial_reset.reset(); }
  return result;
}

DspFetchCycleInputs pins() {
  return {.known_requests = 15, .grant = DspDmaGrant::available};
}

void prepare(DspMachine &dsp) {
  test::DspProgram program;
  for (unsigned i = 0; i < dsp_nops; ++i) { program.emit("nop"); }
  program.install(dsp.bus());
  dsp.bus().pll().observe_pinit(false);
  dsp.cpu().reset_spi_boot();
  static_cast<void>(
      dsp.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
}

struct Sources {
  std::shared_ptr<const DspFetchEventIdentity> inherited_id =
      std::make_shared<const DspFetchEventIdentity>();
  std::shared_ptr<const PeripheralEventIdentity> peripheral_id =
      std::make_shared<const PeripheralEventIdentity>();
  std::optional<std::uint64_t> pending = collision;
  std::optional<bool> level;
  bool include_inherited = true;
  std::vector<char> trace;

  [[nodiscard]] DspFetchExternalEvents external() const {
    if (!include_inherited) { return {}; }
    // Nonempty inherited input is deliberately later and must stay pinned.
    return {.identity = inherited_id,
            .next_ticks = []() -> std::optional<std::uint64_t> { return later; },
            .observe = [](std::uint64_t) { throw Fault("Inherited input must remain pending"); },
            .order = DspExternalOrder::external_before_dsp,
            .maximum_events = 4};
  }

  DspFetchPeripheralEvents peripheral() {
    return {.identity = peripheral_id,
            .next_ticks = [this] { return pending; },
            .observe =
                [this](const std::uint64_t ticks) {
                  require(pending == ticks && ticks == collision, "Own event consumes literal720");
                  trace.push_back('P');
                  pending.reset();
                  level = true;
                },
            .order = DspExternalOrder::external_before_dsp,
            .maximum_events = 4};
  }
};

struct Fixture {
  explicit Fixture(const McuExternalWriteOrder order,
                   const bool running = false,
                   const McuDspTieOrder native = McuDspTieOrder::dsp_before_mcu)
      : mcu(image(), BLOFELD_DEFAULT_SLA, contract()) {
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    mcu.bus().write(hc12::portb, 0);
    mcu.bus().write(hc12::ddrb, byte_mask);
    source.include_inherited = !running;
    if (running) { prepare(dsp); }
    schedule = std::make_unique<McuDspFetchSchedule>(
        settings(order, running, native), mcu, dsp, source.external(), source.peripheral());
    attach();
  }

  Fixture(const McuDspFetchCheckpoint &checkpoint, Sources staged)
      : mcu(image(), BLOFELD_DEFAULT_SLA, contract(), McuPowerOnInputs{3}),
        source(std::move(staged)) {
    schedule = std::make_unique<McuDspFetchSchedule>(
        checkpoint, mcu, dsp, source.external(), source.peripheral());
    attach();
  }

  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;

  ~Fixture() {
    mcu.set_cycle_observer({});
    mcu.bus().set_observer({});
  }

  void attach() {
    const auto after = [this](const DspFetchCycleResult &cycle) {
      if (cycle.completion_ticks + native_epoch == collision) { source.trace.push_back('D'); }
    };
    mcu.set_cycle_observer([this, after](const Hc12BusCycle cycle) {
      schedule->begin_cycle(cycle, cycle.kind == 'w' && cycle.address == hc12::portb, pins, after);
    });
    mcu.bus().set_observer([this, after](const BusEvent &event) {
      if (!event.write || event.address != hc12::portb) { return; }
      schedule->observe_write(
          event,
          [this] {
            const auto pads = mcu.bus().observe_pads(McuPort::b, byte_mask);
            require(schedule->end_ticks() == collision && pads.known == byte_mask &&
                        pads.levels == marker,
                    "Actual native STAB owns the shared write and pads");
            sample = source.level;
            source.trace.push_back('M');
            ++writes;
          },
          pins,
          after);
    });
  }

  void step() {
    require(mcu.cpu().step().instruction_executed, "Every own MCU instruction actually executes");
    schedule->end_instruction();
  }

  McuMachine mcu;
  DspMachine dsp;
  Sources source;
  std::unique_ptr<McuDspFetchSchedule> schedule;
  std::optional<bool> sample;
  unsigned writes = 0;
};

auto registers(const McuMachine &mcu) {
  const auto cpu = mcu.cpu().state();
  return std::tuple{cpu.pc, cpu.d, cpu.x, cpu.y, cpu.sp, cpu.ccr};
}

void endpoint(const Fixture &f, const McuExternalWriteOrder policy) {
  const auto cpu = f.mcu.cpu().state();
  const auto calendar = f.schedule->peripheral_state();
  const auto expected_sample =
      policy == McuExternalWriteOrder::external_before_mcu ? std::optional{true} : std::nullopt;
  const std::vector<char> expected_trace = policy == McuExternalWriteOrder::external_before_mcu
                                               ? std::vector<char>{'P', 'M'}
                                               : std::vector<char>{'M', 'P'};
  require(!f.schedule->faulted() && f.sample == expected_sample && f.writes == 1 &&
              f.source.trace == expected_trace && f.source.level == true && !f.source.pending &&
              f.schedule->end_ticks() == collision && f.mcu.run_clock().bus_cycles() == 8 &&
              cpu.pc == entry + 4 && cpu.d == marker && cpu.ccr == expected_ccr &&
              f.schedule->run().completed_cycles() == 0 &&
              f.schedule->timeline_state().next_external == later &&
              f.schedule->timeline_state().external_events == 0 && calendar &&
              calendar->causes == 2 && calendar->peripheral_events == 1 &&
              calendar->last_peripheral == collision && calendar->last_peer == collision &&
              !calendar->last_native && !calendar->last_external && !calendar->pending,
          "Independent complete native/cursor/peer/peripheral endpoints obey the declared order");
}

void tie(const McuExternalWriteOrder policy) {
  Fixture f(policy);
  const auto held = f.dsp.cpu().debug_state();
  f.step();
  if (policy == McuExternalWriteOrder::unqualified) {
    require(rejects([&] { f.step(); }).contains("Unqualified simultaneous external event") &&
                f.schedule->faulted() && f.writes == 0 && f.mcu.bus().read(hc12::portb) == 0 &&
                f.source.pending == collision && !f.source.level && f.source.trace.empty() &&
                f.mcu.run_clock().bus_cycles() == 7 && f.dsp.cpu().debug_state() == held,
            "Unknown tie refuses before MCU store and peripheral dispatch without DSP work");
  } else {
    f.step();
    endpoint(f, policy);
    require(f.dsp.cpu().debug_state() == held, "Held DSP executes no hidden native instruction");
  }
}

void snapshots() {
  constexpr auto policy = McuExternalWriteOrder::mcu_before_external;
  Fixture f(policy);
  f.step();
  const auto checkpoint = f.schedule->checkpoint();
  const auto staged = f.source;
  const auto cpu_at_cut = registers(f.mcu);
  const auto dsp_at_cut = f.dsp.cpu().debug_state();
  const auto calendar_at_cut = f.schedule->peripheral_state();
  f.step();
  endpoint(f, policy);
  const auto final = registers(f.mcu);
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.source = staged;
    f.writes = 0;
    f.sample.reset();
    f.schedule->restore(checkpoint);
    require(registers(f.mcu) == cpu_at_cut && f.dsp.cpu().debug_state() == dsp_at_cut &&
                f.schedule->peripheral_state() == calendar_at_cut,
            "Coordinated native restoration retains the complete peer/peripheral history");
    f.attach();
    f.step();
    endpoint(f, policy);
    require(registers(f.mcu) == final, "Actual restored MCU STAB reproduces every native register");
  }
  Fixture cloned(checkpoint, staged);
  require(registers(cloned.mcu) == cpu_at_cut &&
              cloned.schedule->peripheral_state() == calendar_at_cut,
          "Trusted joint clone retains native and peripheral state on distinct owners");
  cloned.step();
  endpoint(cloned, policy);
  require(registers(cloned.mcu) == final && cloned.dsp.cpu().debug_state() == dsp_at_cut,
          "Cloned real native suffix retains both owner endpoints");
}

void running() {
  struct Policy {
    McuDspTieOrder native;
    McuExternalWriteOrder external;
    std::vector<char> trace;
  };

  const std::array<Policy, 3> policies{
      Policy{.native = McuDspTieOrder::dsp_before_mcu,
             .external = McuExternalWriteOrder::external_before_mcu,
             .trace = {'P', 'D', 'M'}},
      Policy{.native = McuDspTieOrder::mcu_before_dsp,
             .external = McuExternalWriteOrder::external_before_mcu,
             .trace = {'P', 'M', 'D'}},
      Policy{.native = McuDspTieOrder::mcu_before_dsp,
             .external = McuExternalWriteOrder::mcu_before_external,
             .trace = {'M', 'P', 'D'}}};
  for (const auto &policy : policies) {
    Fixture f(policy.external, true, policy.native);
    f.step();
    f.step();
    require(
        f.source.trace == policy.trace && f.writes == 1 && f.source.level == true &&
            f.schedule->run().completed_cycles() == 6 &&
            f.schedule->run().elapsed_ticks() == collision - native_epoch &&
            f.schedule->run().timer_clk2_edges() == 3 && f.schedule->ties() == 1 &&
            f.dsp.cpu().state().sr == initial_status && f.dsp.cpu().state().a == 0 &&
            !f.schedule->timeline_state().external_identity &&
            f.mcu.run_clock().bus_cycles() == 8 && f.mcu.cpu().state().pc == entry + 4,
        "All three acyclic MCU/peripheral/native orders retain independent NOP flags and clocks");
  }
  Fixture cyclic(McuExternalWriteOrder::mcu_before_external, true, McuDspTieOrder::dsp_before_mcu);
  cyclic.step();
  require(rejects([&] { cyclic.step(); }).contains("Cyclic MCU/DSP/external") &&
              cyclic.schedule->faulted() && cyclic.writes == 0 && cyclic.source.trace.empty() &&
              cyclic.source.pending == collision && cyclic.schedule->run().completed_cycles() == 5,
          "Peripheral-before-DSP, DSP-before-MCU, MCU-before-peripheral cycle rejects before all "
          "tied actions");
}

void query_guard() {
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, contract());
  DspMachine dsp;
  Sources source;
  auto peripheral = source.peripheral();
  peripheral.next_ticks = [&]() -> std::optional<std::uint64_t> {
    static_cast<void>(mcu.cpu().step());
    return collision;
  };
  require(rejects([&] {
            const McuDspFetchSchedule schedule(settings(McuExternalWriteOrder::external_before_mcu),
                                               mcu,
                                               dsp,
                                               source.external(),
                                               peripheral);
          }).contains("changed native MCU state") &&
              mcu.run_clock().bus_cycles() == 6 && mcu.cpu().state().pc == entry + 2 &&
              source.trace.empty(),
          "A real unobserved native MCU instruction inside a peripheral query fails visibly");
}
}

int main(const int argc, const char *const *argv) {
  try {
    require(argc == 2, "Choose external-first,mcu-first,unqualified,state,guards or running");
    const std::string_view mode(argv[1]);
    if (mode == "external-first") {
      tie(McuExternalWriteOrder::external_before_mcu);
    } else if (mode == "mcu-first") {
      tie(McuExternalWriteOrder::mcu_before_external);
    } else if (mode == "unqualified") {
      tie(McuExternalWriteOrder::unqualified);
    } else if (mode == "state") {
      snapshots();
    } else if (mode == "guards") {
      query_guard();
    } else if (mode == "running") {
      running();
    } else {
      throw Fault("Unknown native MCU/peripheral scope");
    }
    std::cout << "PASS actual MCU/separate-peripheral: " << mode << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
