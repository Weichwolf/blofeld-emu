// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
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
#include <functional>
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
constexpr std::uint64_t collision = 720, later = 900, recovered = 540;
constexpr unsigned boot = 0xff0000;
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

McuDspRuntimeInputs settings(const McuWriteOrder order,
                             const McuWriteOrder native = McuWriteOrder::before_mcu) {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .writes = native,
          .external_writes = order,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
}

DspDmaInputs pins() {
  return {.known_requests = 15,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

void prepare(OwnDspMachine &dsp) {
  const std::array<std::uint32_t, dsp_nops> code{}; // Authored FM NOP words.
  dsp.bus().install_rom({.space = DspSpace::p, .start = boot, .words = code});
}

dsp56300::Registers expected_nops(const unsigned pc_words) {
  constexpr unsigned reset_omr = 0x305;
  dsp56300::Registers expected{.pc = boot + pc_words, .sr = initial_status, .omr = reset_omr};
  expected.m.fill(dsp56300::word_mask);
  return expected;
}

struct Sources {
  std::shared_ptr<const bool> inherited_id = std::make_shared<const bool>(false);
  std::shared_ptr<const PeripheralEventIdentity> peripheral_id =
      std::make_shared<const PeripheralEventIdentity>();
  std::optional<std::uint64_t> pending = collision;
  std::optional<std::uint64_t> inherited_pending = later;
  std::optional<bool> level;
  bool include_inherited = true;
  bool consume_inherited = false;
  bool forbidden = false;
  unsigned queries = 0;
  std::vector<std::uint64_t> observed;
  std::vector<char> trace;

  [[nodiscard]] DspRuntimeEvents external() {
    if (!include_inherited) { return {}; }
    // Nonempty inherited input is deliberately later and must stay pinned.
    return {.identity = inherited_id,
            .next_ticks =
                [this] {
                  require(!forbidden, "Clone must not query inherited payload while staging");
                  ++queries;
                  return inherited_pending;
                },
            .observe =
                [this](const std::uint64_t ticks) {
                  require(
                      !forbidden && consume_inherited && inherited_pending == ticks,
                      "Inherited input must remain pinned until an actual declared observation");
                  trace.push_back('E');
                  inherited_pending.reset();
                },
            .order = DspPeerOrder::peer_first,
            .maximum_events_per_period = 4};
  }

  DspRuntimePeripheralEvents peripheral() {
    return {.identity = peripheral_id,
            .next_ticks =
                [this] {
                  require(!forbidden, "Clone must not query peripheral payload while staging");
                  ++queries;
                  return pending;
                },
            .observe =
                [this](const std::uint64_t ticks) {
                  require(!forbidden && pending == ticks,
                          "Own event consumes its actual advertised deadline");
                  observed.push_back(ticks);
                  trace.push_back('P');
                  pending.reset();
                  level = true;
                },
            .order = DspPeerOrder::peer_first,
            .maximum_events_per_period = 4};
  }
};

struct Scenario {
  bool running = false;
  McuWriteOrder native = McuWriteOrder::before_mcu;
  std::optional<std::uint64_t> deadline = collision;
  std::optional<std::uint64_t> inherited = later;
  unsigned budget = 4;
  bool consume_inherited = false;
  bool inherited_lane = true;
};

struct Fixture {
  explicit Fixture(const McuWriteOrder order, const Scenario &scenario = {})
      : mcu(image(), BLOFELD_DEFAULT_SLA, contract()) {
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    mcu.bus().write(hc12::portb, 0);
    mcu.bus().write(hc12::ddrb, byte_mask);
    source.pending = scenario.deadline;
    source.inherited_pending = scenario.inherited;
    source.consume_inherited = scenario.consume_inherited;
    source.include_inherited = scenario.inherited_lane;
    if (scenario.running) { prepare(dsp); }
    auto inherited = source.external();
    auto peripheral = source.peripheral();
    if (inherited.identity) { inherited.maximum_events_per_period = scenario.budget; }
    peripheral.maximum_events_per_period = scenario.budget;
    schedule = std::make_unique<McuDspRuntime>(
        settings(order, scenario.native), mcu, dsp, inherited, peripheral);
    if (scenario.running) { schedule->release_dsp_reset(); }
    attach();
  }

  Fixture(const McuDspRuntimeCheckpoint &checkpoint, Sources staged)
      : mcu(image(), BLOFELD_DEFAULT_SLA, contract(), McuPowerOnInputs{3}) {
    restore(checkpoint, std::move(staged));
  }

  void restore(const McuDspRuntimeCheckpoint &checkpoint, Sources staged) {
    schedule.reset();
    source = std::move(staged);
    const auto queries = source.queries;
    source.forbidden = true;
    schedule = std::make_unique<McuDspRuntime>(
        checkpoint, mcu, dsp, source.external(), source.peripheral());
    require(source.queries == queries, "Native staging queried live input or peripheral state");
    source.forbidden = false;
    writes = 0;
    sample.reset();
    attach();
  }

  Fixture(const Fixture &) = delete;
  Fixture &operator=(const Fixture &) = delete;

  ~Fixture() {
    mcu.set_cycle_observer({});
    mcu.bus().set_observer({});
  }

  void attach() {
    const auto after = [this](const DspRuntimeCycle &cycle) {
      if (cycle.completion_ticks == collision) { source.trace.push_back('D'); }
      if (on_cycle) { on_cycle(cycle); }
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
            if (on_write) { on_write(); }
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
  OwnDspMachine dsp{{.extal_hz = oscillator,
                     .timebase_hz = common_hz,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  Sources source;
  std::unique_ptr<McuDspRuntime> schedule;
  std::optional<bool> sample;
  unsigned writes = 0;
  std::function<void()> on_write;
  McuDspRuntime::Observer on_cycle;
};

auto registers(const McuMachine &mcu) {
  const auto cpu = mcu.cpu().state();
  return std::tuple{cpu.pc, cpu.d, cpu.x, cpu.y, cpu.sp, cpu.ccr};
}

void endpoint(const Fixture &f, const McuWriteOrder policy) {
  const auto cpu = f.mcu.cpu().state();
  const auto calendar = f.schedule->peripheral_state();
  const auto expected_sample =
      policy == McuWriteOrder::before_mcu ? std::optional{true} : std::nullopt;
  const std::vector<char> expected_trace = policy == McuWriteOrder::before_mcu
                                               ? std::vector<char>{'P', 'M'}
                                               : std::vector<char>{'M', 'P'};
  require(!f.schedule->faulted() && f.sample == expected_sample && f.writes == 1 &&
              f.source.trace == expected_trace && f.source.level == true && !f.source.pending &&
              f.schedule->end_ticks() == collision && f.mcu.run_clock().bus_cycles() == 8 &&
              cpu.pc == entry + 4 && cpu.d == marker && cpu.ccr == expected_ccr &&
              f.source.inherited_pending == later && f.schedule->completed_cycles() == 0 &&
              calendar && calendar->causes == 2 && calendar->peripheral_events == 1 &&
              calendar->last_peripheral == collision && calendar->last_peer == collision &&
              !calendar->last_native && !calendar->last_external && !calendar->pending,
          "Independent complete native/cursor/peer/peripheral endpoints obey the declared order");
  require(f.dsp.cpu().registers() == expected_nops(0) &&
              f.dsp.cpu().progress() == dsp56300::Progress{} &&
              f.dsp.clock().elapsed_ticks() == collision &&
              f.dsp.clock().hardware_reset_ticks() == collision &&
              f.dsp.clock().timer_clk2_edges() == 0,
          "Both input lanes preserve full held DSP state and actual reset clocks");
}

void tie(const McuWriteOrder policy) {
  Fixture f(policy);
  const auto held = f.dsp.cpu().registers();
  f.step();
  if (policy == McuWriteOrder::unqualified) {
    require(rejects([&] { f.step(); }).contains("Unqualified simultaneous external event") &&
                f.schedule->faulted() && f.writes == 0 && f.mcu.bus().read(hc12::portb) == 0 &&
                f.source.pending == collision && !f.source.level && f.source.trace.empty() &&
                f.mcu.run_clock().bus_cycles() == 7 && f.dsp.cpu().registers() == held,
            "Unknown tie refuses before MCU store and peripheral dispatch without DSP work");
  } else {
    f.step();
    endpoint(f, policy);
    require(f.dsp.cpu().registers() == held, "Held DSP executes no hidden native instruction");
  }
}

void snapshots() {
  constexpr auto policy = McuWriteOrder::after_mcu;
  Fixture f(policy);
  f.step();
  const auto checkpoint = f.schedule->checkpoint();
  const auto staged = f.source;
  const auto cpu_at_cut = registers(f.mcu);
  const auto dsp_at_cut = f.dsp.cpu().registers();
  const auto calendar_at_cut = f.schedule->peripheral_state();
  f.step();
  endpoint(f, policy);
  const auto final = registers(f.mcu);
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.restore(checkpoint, staged);
    require(registers(f.mcu) == cpu_at_cut && f.dsp.cpu().registers() == dsp_at_cut &&
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
  require(registers(cloned.mcu) == final && cloned.dsp.cpu().registers() == dsp_at_cut,
          "Cloned real native suffix retains both owner endpoints");
}

void running() {
  struct Policy {
    McuWriteOrder native;
    McuWriteOrder external;
    std::vector<char> trace;
  };

  const std::array<Policy, 3> policies{Policy{.native = McuWriteOrder::before_mcu,
                                              .external = McuWriteOrder::before_mcu,
                                              .trace = {'P', 'D', 'M'}},
                                       Policy{.native = McuWriteOrder::after_mcu,
                                              .external = McuWriteOrder::before_mcu,
                                              .trace = {'P', 'M', 'D'}},
                                       Policy{.native = McuWriteOrder::after_mcu,
                                              .external = McuWriteOrder::after_mcu,
                                              .trace = {'M', 'P', 'D'}}};
  for (const auto &policy : policies) {
    Fixture f(policy.external, {.running = true, .native = policy.native});
    f.step();
    f.step();
    require(
        f.source.trace == policy.trace && f.writes == 1 && f.source.level == true &&
            f.schedule->completed_cycles() == 4 && f.dsp.clock().elapsed_ticks() == collision &&
            f.dsp.clock().hardware_reset_ticks() == recovered &&
            f.dsp.clock().timer_clk2_edges() == 2 && f.schedule->ties() == 1 &&
            f.dsp.cpu().registers() == expected_nops(1) &&
            f.dsp.cpu().progress() ==
                dsp56300::Progress{.instructions = 1, .estimated_cycles = 1} &&
            f.mcu.run_clock().bus_cycles() == 8 && f.mcu.cpu().state().pc == entry + 4,
        "All three acyclic MCU/peripheral/native orders retain independent NOP flags and clocks");
  }
  Fixture cyclic(McuWriteOrder::after_mcu, {.running = true});
  cyclic.step();
  require(rejects([&] { cyclic.step(); }).contains("Cyclic MCU/DSP/external") &&
              cyclic.schedule->faulted() && cyclic.writes == 0 && cyclic.source.trace.empty() &&
              cyclic.source.pending == collision && cyclic.schedule->completed_cycles() == 3,
          "Peripheral-before-DSP, DSP-before-MCU, MCU-before-peripheral cycle rejects before all "
          "tied actions");
}

void query_guard() {
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, contract());
  OwnDspMachine dsp{{.extal_hz = oscillator,
                     .timebase_hz = common_hz,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  Sources source;
  auto peripheral = source.peripheral();
  peripheral.next_ticks = [&]() -> std::optional<std::uint64_t> {
    static_cast<void>(mcu.cpu().step());
    return collision;
  };
  require(rejects([&] {
            const McuDspRuntime schedule(
                settings(McuWriteOrder::before_mcu), mcu, dsp, source.external(), peripheral);
          }).contains("peripheral query changed CPU state or time") &&
              mcu.run_clock().bus_cycles() == 6 && mcu.cpu().state().pc == entry + 2 &&
              source.trace.empty(),
          "A real unobserved native MCU instruction inside a peripheral query fails visibly");
}

void absent_deadline_pin() {
  Fixture f(McuWriteOrder::before_mcu, {.deadline = std::nullopt});
  const auto held = f.dsp.cpu().registers();
  f.source.pending = collision;
  const auto error = rejects([&] { f.step(); });
  require(error.contains("source identity or pinned pending deadline changed") &&
              f.schedule->faulted() && f.mcu.run_clock().bus_cycles() == 5 &&
              f.source.trace.empty() && f.writes == 0 && !f.source.level &&
              f.dsp.cpu().registers() == held && f.schedule->completed_cycles() == 0,
          "An absent peripheral deadline cannot appear without an actual native/input/peer cause");
}

void inherited_deadline_pin() {
  Fixture f(McuWriteOrder::before_mcu);
  f.step();
  f.step();
  endpoint(f, McuWriteOrder::before_mcu);
  const auto before = registers(f.mcu);
  const auto cycles = f.mcu.run_clock().bus_cycles();
  f.source.inherited_pending = collision + 1;
  const auto error = rejects([&] { f.step(); });
  require(error.contains("source changed its pinned pending deadline") && f.schedule->faulted() &&
              f.mcu.run_clock().bus_cycles() == cycles && f.writes == 1 &&
              f.source.observed == std::vector<std::uint64_t>{collision} &&
              std::get<1>(registers(f.mcu)) == std::get<1>(before),
          "Peripheral dispatch cannot replace or release the independent inherited900 deadline");
}

void peer_birth(const bool qualified, const bool immediate) {
  constexpr std::uint64_t following_period = 810;
  const std::uint64_t event = immediate ? collision : collision + 1;
  const auto order = qualified ? McuWriteOrder::after_mcu : McuWriteOrder::before_mcu;
  Fixture f(order, {.deadline = std::nullopt});
  f.on_write = [&] { f.source.pending = qualified ? event : collision; };
  f.step();
  if (qualified) {
    f.step();
    if (!immediate) {
      require(f.source.pending == event && f.source.trace == std::vector<char>{'M'} &&
                  f.source.observed.empty() && !f.sample && f.writes == 1,
              "The accepted MCU write creates a strictly future peripheral deadline");
      f.step();
    }
    const auto end = immediate ? collision : following_period;
    const unsigned cycles = immediate ? 8 : 9;
    const auto calendar = f.schedule->peripheral_state();
    require(f.source.observed == std::vector<std::uint64_t>{event} &&
                f.source.trace == std::vector<char>{'M', 'P'} && f.source.level == true &&
                f.source.inherited_pending == later && f.schedule->end_ticks() == end &&
                f.mcu.run_clock().bus_cycles() == cycles && f.schedule->completed_cycles() == 0 &&
                f.dsp.clock().hardware_reset_ticks() == end && calendar && calendar->causes == 2 &&
                calendar->last_peer == collision && calendar->last_peripheral == event,
            "A born exact or sub-edge deadline dispatches once without unpinning inherited input");
  } else {
    const auto error = rejects([&] { f.step(); });
    require(error.contains("new deadline precedes its actual cause") && f.schedule->faulted() &&
                f.writes == 1 && f.mcu.bus().read(hc12::portb) == marker &&
                f.mcu.run_clock().bus_cycles() == 8 && f.source.pending == collision &&
                f.source.trace == std::vector<char>{'M'} && f.source.observed.empty() &&
                !f.source.level,
            "Late same-tick peripheral input cannot precede its already accepted MCU write");
  }
}

void native_birth(const bool qualified) {
  constexpr std::uint64_t first = recovered + 45;
  constexpr std::uint64_t future = first + 90;
  Fixture f(McuWriteOrder::before_mcu, {.running = true, .deadline = std::nullopt});
  f.on_cycle = [&](const DspRuntimeCycle &cycle) {
    if (cycle.completion_ticks == first) { f.source.pending = qualified ? future : first; }
  };
  f.step();
  if (qualified) {
    f.step();
    const auto calendar = f.schedule->peripheral_state();
    require(!f.schedule->faulted() && f.source.observed == std::vector<std::uint64_t>{future} &&
                f.source.trace == std::vector<char>{'P', 'D', 'M'} &&
                f.source.inherited_pending == later && f.source.level == true && f.sample == true &&
                f.schedule->completed_cycles() == 4 &&
                f.dsp.cpu().registers() == expected_nops(1) && calendar &&
                calendar->last_peripheral == future && calendar->last_native == collision &&
                calendar->last_peer == collision && calendar->causes == 6,
            "An actual DSP clock creates an earlier peripheral deadline without unpinning "
            "inherited input");
  } else {
    const auto error = rejects([&] { f.step(); });
    require(error.contains("new deadline precedes its actual cause") && f.schedule->faulted() &&
                f.schedule->completed_cycles() == 1 && f.dsp.clock().elapsed_ticks() == first &&
                f.mcu.run_clock().bus_cycles() == 6 && f.source.pending == first && f.writes == 0 &&
                f.source.observed.empty() && f.source.trace.empty(),
            "A born event cannot precede the genuine DSP cycle that already created it");
  }
}

void shared_budget(const unsigned budget) {
  Fixture f(McuWriteOrder::before_mcu,
            {.inherited = collision, .budget = budget, .consume_inherited = true});
  f.step();
  if (budget == 2) {
    f.step();
    const auto calendar = f.schedule->peripheral_state();
    require(
        !f.schedule->faulted() && f.source.trace == std::vector<char>{'E', 'P', 'M'} &&
            !f.source.inherited_pending && !f.source.pending && f.sample == true &&
            f.source.observed == std::vector<std::uint64_t>{collision} && f.writes == 1 &&
            calendar && calendar->last_external == collision &&
            calendar->last_peripheral == collision && calendar->last_peer == collision &&
            calendar->causes == 3 && f.schedule->completed_cycles() == 0,
        "Both equal-time input lanes fit the exact shared budget and precede the actual MCU write");
  } else {
    const auto error = rejects([&] { f.step(); });
    require(error.contains("input sources exceeded their shared period budget") &&
                f.schedule->faulted() && f.source.trace == std::vector<char>{'E'} &&
                !f.source.inherited_pending && f.source.pending == collision &&
                f.source.observed.empty() && f.writes == 0 && f.mcu.run_clock().bus_cycles() == 7 &&
                f.mcu.bus().read(hc12::portb) == 0,
            "A shared budget failure preserves the accepted inherited input and refuses "
            "peripheral/MCU work");
  }
}

void standalone_peripheral() {
  for (const auto order : {McuWriteOrder::before_mcu, McuWriteOrder::after_mcu}) {
    Fixture f(order, {.budget = 1, .inherited_lane = false});
    f.step();
    f.step();
    endpoint(f, order);
    require(!f.source.include_inherited,
            "The peripheral-only lane uses its own positive budget without inherited callbacks");
  }
}

void malformed_sources() {
  for (unsigned kind = 0; kind < 8; ++kind) {
    McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, contract());
    OwnDspMachine dsp({.extal_hz = oscillator,
                       .timebase_hz = common_hz,
                       .switch_timing = DspClockSwitch::reference_cycle_boundary});
    Sources source;
    auto inherited = source.external();
    auto peripheral = source.peripheral();
    switch (kind) {
      case 0: peripheral.identity.reset(); break;
      case 1: peripheral.next_ticks = {}; break;
      case 2: peripheral.observe = {}; break;
      case 3: peripheral.order = DspPeerOrder::unqualified; break;
      case 4: peripheral.maximum_events_per_period = 0; break;
      case 5: inherited.order = DspPeerOrder::dsp_first; break;
      case 6: ++peripheral.maximum_events_per_period; break;
      case 7:
        peripheral = {};
        peripheral.order = DspPeerOrder::peer_first;
        break;
      default: throw Fault("Invalid fixed peripheral binding control");
    }
    const auto before = registers(mcu);
    const auto chip = dsp.cpu().registers();
    const auto error = rejects([&] {
      const McuDspRuntime rejected(
          settings(McuWriteOrder::before_mcu), mcu, dsp, inherited, peripheral);
    });
    const bool combined = kind == 5 || kind == 6;
    require(error.contains(combined ? "combined input sources" : "peripheral source requires") &&
                registers(mcu) == before && dsp.cpu().registers() == chip &&
                mcu.run_clock().bus_cycles() == 5 && dsp.clock().elapsed_ticks() == 0 &&
                source.queries == 0 && source.trace.empty(),
            "Malformed/unsupported source contracts reject before queries or either native machine "
            "changes");
  }
}

void observation_guard() {
  constexpr std::uint64_t initial = 450;
  McuMachine mcu(image(), BLOFELD_DEFAULT_SLA, contract());
  OwnDspMachine dsp({.extal_hz = oscillator,
                     .timebase_hz = common_hz,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary});
  Sources source;
  source.pending = initial;
  auto peripheral = source.peripheral();
  peripheral.observe = [&](std::uint64_t) { static_cast<void>(mcu.cpu().step()); };
  require(rejects([&] {
            const McuDspRuntime rejected(
                settings(McuWriteOrder::before_mcu), mcu, dsp, source.external(), peripheral);
          }).contains("peripheral observation changed CPU state or time") &&
              mcu.run_clock().bus_cycles() == 6 && mcu.cpu().state().pc == entry + 2 &&
              dsp.clock().native_cycle_units() == 0 && source.observed.empty(),
          "An actual MCU instruction inside peripheral observation is detected and cannot publish "
          "a joint owner");
}

void snapshot_bindings() {
  Fixture f(McuWriteOrder::after_mcu);
  f.step();
  const auto checkpoint = f.schedule->checkpoint();
  const auto cpu = registers(f.mcu);
  const auto chip = f.dsp.cpu().registers();
  const auto queries = f.source.queries;
  f.source.forbidden = true;
  for (unsigned kind = 0; kind < 5; ++kind) {
    McuMachine target(image(), BLOFELD_DEFAULT_SLA, contract(), McuPowerOnInputs{3});
    OwnDspMachine dsp({.extal_hz = oscillator,
                       .timebase_hz = common_hz,
                       .switch_timing = DspClockSwitch::reference_cycle_boundary});
    auto peripheral = f.source.peripheral();
    switch (kind) {
      case 0: peripheral.identity = std::make_shared<const PeripheralEventIdentity>(); break;
      case 1: peripheral.order = DspPeerOrder::dsp_first; break;
      case 2: ++peripheral.maximum_events_per_period; break;
      case 3: peripheral.next_ticks = {}; break;
      case 4: peripheral.observe = {}; break;
      default: throw Fault("Invalid fixed restored peripheral binding");
    }
    require(rejects([&] {
              const McuDspRuntime rejected(
                  checkpoint, target, dsp, f.source.external(), peripheral);
            }).contains("matching peripheral identity and contract") &&
                target.run_clock().elapsed_ticks() == 0 && dsp.clock().elapsed_ticks() == 0 &&
                registers(f.mcu) == cpu && f.dsp.cpu().registers() == chip &&
                f.source.queries == queries,
            "A mismatched peripheral snapshot binding rejects without input queries or native "
            "mutation");
  }
  f.source.forbidden = false;
  f.step();
  endpoint(f, McuWriteOrder::after_mcu);
}

void stale_snapshot_payload(const bool absent) {
  Fixture f(McuWriteOrder::after_mcu,
            {.deadline = absent ? std::nullopt : std::optional{collision}});
  f.step();
  const auto checkpoint = f.schedule->checkpoint();
  auto staged = f.source;
  staged.pending = collision + 1;
  const auto cpu = registers(f.mcu);
  const auto chip = f.dsp.cpu().registers();
  Fixture clone(checkpoint, staged);
  require(rejects([&] {
            clone.step();
          }).contains("source identity or pinned pending deadline changed") &&
              clone.schedule->faulted() && clone.mcu.run_clock().bus_cycles() == 6 &&
              clone.writes == 0 && clone.source.observed.empty() && registers(f.mcu) == cpu &&
              f.dsp.cpu().registers() == chip && !f.schedule->faulted(),
          "Present/absent saved deadlines are verified only on advancement,without changing the "
          "source clone");
  f.step();
  if (!absent) {
    endpoint(f, McuWriteOrder::after_mcu);
  } else {
    require(!f.schedule->faulted() && f.writes == 1 && !f.source.pending &&
                f.source.trace == std::vector<char>{'M'} && f.source.observed.empty(),
            "The original absent source continues without inventing a peripheral event");
  }
}

}

int main(const int argc, const char *const *argv) {
  try {
    require(
        argc == 2,
        "Choose external-first,mcu-first,unqualified,state,guards,running,births,sources,budget or "
        "restore");
    const std::string_view mode(argv[1]);
    if (mode == "external-first") {
      tie(McuWriteOrder::before_mcu);
    } else if (mode == "mcu-first") {
      tie(McuWriteOrder::after_mcu);
    } else if (mode == "unqualified") {
      tie(McuWriteOrder::unqualified);
    } else if (mode == "state") {
      snapshots();
    } else if (mode == "guards") {
      query_guard();
    } else if (mode == "running") {
      running();
    } else if (mode == "births") {
      peer_birth(true, true);
      peer_birth(true, false);
      peer_birth(false, true);
      native_birth(true);
      native_birth(false);
    } else if (mode == "sources") {
      absent_deadline_pin();
      inherited_deadline_pin();
      malformed_sources();
      observation_guard();
      standalone_peripheral();
    } else if (mode == "budget") {
      shared_budget(1);
      shared_budget(2);
    } else if (mode == "restore") {
      snapshot_bindings();
      stale_snapshot_payload(false);
      stale_snapshot_payload(true);
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
