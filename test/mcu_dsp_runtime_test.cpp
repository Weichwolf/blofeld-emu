// SPDX-License-Identifier: GPL-3.0-only
#include "board/detail/mcu_dsp_boundary.hpp"
#include "board/mcu_dsp_runtime.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
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
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned extal = 12000000;
constexpr unsigned common_hz = 540000000;
constexpr unsigned write_ticks = 720;
constexpr unsigned all_pins = 0xff;
constexpr unsigned port_b = 1;
constexpr unsigned value = 0x42;
constexpr unsigned dsp_boot = 0xff0000;
constexpr DspRunClockInputs clocks{.extal_hz = extal,
                                   .timebase_hz = common_hz,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const std::exception &error) { return error.what(); }
  throw std::runtime_error("Expected MCU/DSP runtime refusal");
}

McuFlashImage image() {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  constexpr unsigned nop = 0xa7;
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  constexpr unsigned entry_hi = 0x80;
  // Own HCS12 fixture: LDAB #42, STAB PORTB, then NOP padding.
  constexpr std::array<std::uint8_t, 4> code{0xc6, value, 0x5b, port_b};
  bytes.fill(nop);
  for (unsigned i = 0; i < code.size(); ++i) { bytes.at(i) = code[i]; }
  bytes[protection] = all_pins;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry_hi;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

McuRunContract mcu_contract() {
  return {.clock = {.oscillator_hz = extal,
                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = McuRunMemory::unstretched_internal};
}

struct Case {
  McuWriteOrder native;
  McuWriteOrder external;
  DspPeerOrder source;
  std::string_view expected;
};

McuDspRuntimeInputs inputs(const Case spec) {
  return {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          .maximum_cycles_per_period = 16,
          .writes = spec.native,
          .external_writes = spec.external,
          .wait_dsp_first = true,
          .reset_dsp_first = true,
          .initial_reset = OwnDspResetInputs{
              .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 3}};
}

class Fixture {
public:
  explicit Fixture(const Case spec, const bool unknown_read = false)
      : mcu(image(), BLOFELD_DEFAULT_SLA, mcu_contract()), source_order(spec.source) {
    mcu.bus().set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    mcu.bus().write(hc12::portb, 0);
    mcu.bus().write(hc12::ddrb, all_pins);
    std::array<std::uint32_t, 128> program{};
    if (unknown_read) {
      // Independently encoded MOVE X:$2000,X0; this RAM word is deliberately unknown.
      constexpr unsigned load_x0_absolute = 0x44f000;
      program[0] = load_x0_absolute;
      program[1] = 0x2000;
    }
    dsp.bus().install_rom({.space = DspSpace::p, .start = dsp_boot, .words = program});
    runtime = std::make_unique<McuDspRuntime>(inputs(spec), mcu, dsp, external_events());
    runtime->release_dsp_reset();
    bind();
  }

  DspRuntimeEvents external_events() {
    return {.identity = identity,
            .next_ticks = [this]() -> std::optional<std::uint64_t> {
              return external_done ? std::nullopt : std::optional<std::uint64_t>{write_ticks};
            },
            .observe =
                [this](const std::uint64_t ticks) {
                  require(ticks == write_ticks, "External source observed the wrong common tick");
                  events += 'E';
                  external_done = true;
                },
            .order = source_order,
            .maximum_events_per_period = 16};
  }

  static DspDmaInputs cycle_inputs() {
    return {.known_requests = 15,
            .timing = DspDmaTiming::reference_sampled_cycles,
            .grant = DspDmaGrant::available};
  }

  McuDspRuntime::Observer observer() {
    return [this](const DspRuntimeCycle &cycle) {
      if (cycle.completion_ticks != write_ticks) { return; }
      events += 'D';
      rejects([&] { static_cast<void>(runtime->checkpoint()); });
      rejected_reentry = true;
    };
  }

  void bind() {
    mcu.set_cycle_observer([this](const Hc12BusCycle &cycle) {
      const bool shared =
          (cycle.kind == 'w' || cycle.kind == 'W') && cycle.address && *cycle.address == port_b;
      runtime->begin_cycle(cycle, shared, cycle_inputs, observer());
    });
    mcu.bus().set_observer([this](const BusEvent &event) {
      if (!event.write || event.address != port_b || drop_write) { return; }
      runtime->observe_write(
          event,
          [this, event] {
            require(event.value == value, "Native MCU store delivered the wrong byte");
            const auto pins = mcu.bus().observe_pads(McuPort::b, all_pins);
            require(pins.known == all_pins && pins.levels == value,
                    "Observer preceded the actual MCU GPIO write");
            events += 'W';
          },
          cycle_inputs,
          observer());
    });
  }

  void step() {
    require(mcu.cpu().step().instruction_executed, "Synthetic MCU instruction did not retire");
    runtime->end_instruction();
  }

  McuMachine mcu;
  OwnDspMachine dsp{clocks};
  DspPeerOrder source_order;
  std::shared_ptr<const bool> identity = std::make_shared<const bool>(true);
  std::unique_ptr<McuDspRuntime> runtime;
  std::string events;
  bool external_done = false;
  bool drop_write = false;
  bool rejected_reentry = false;
};

void ordering_matrix() {
  using enum McuWriteOrder;
  constexpr std::array cases{Case{.native = before_mcu,
                                  .external = before_mcu,
                                  .source = DspPeerOrder::peer_first,
                                  .expected = "EDW"},
                             Case{.native = before_mcu,
                                  .external = before_mcu,
                                  .source = DspPeerOrder::dsp_first,
                                  .expected = "DEW"},
                             Case{.native = before_mcu,
                                  .external = after_mcu,
                                  .source = DspPeerOrder::dsp_first,
                                  .expected = "DWE"},
                             Case{.native = before_mcu,
                                  .external = after_mcu,
                                  .source = DspPeerOrder::peer_first,
                                  .expected = ""},
                             Case{.native = after_mcu,
                                  .external = before_mcu,
                                  .source = DspPeerOrder::peer_first,
                                  .expected = "EWD"},
                             Case{.native = after_mcu,
                                  .external = before_mcu,
                                  .source = DspPeerOrder::dsp_first,
                                  .expected = ""},
                             Case{.native = after_mcu,
                                  .external = after_mcu,
                                  .source = DspPeerOrder::peer_first,
                                  .expected = "WED"},
                             Case{.native = after_mcu,
                                  .external = after_mcu,
                                  .source = DspPeerOrder::dsp_first,
                                  .expected = "WDE"}};
  for (const auto spec : cases) {
    Fixture fixture(spec);
    fixture.step();
    if (spec.expected.empty()) {
      const auto message = rejects([&] { fixture.step(); });
      require(message.contains("Cyclic MCU/DSP/external shared-write ordering at 0x0001") &&
                  message.contains("mcu-pc=0x8002") && message.contains("dsp-pc=0xff0000") &&
                  message.contains("tick=720") && fixture.runtime->faulted(),
              "Cyclic ordering lost native PCs/address/time or was not quarantined");
    } else {
      fixture.step();
      require(fixture.events == spec.expected && fixture.runtime->end_ticks() == write_ticks &&
                  fixture.dsp.clock().elapsed_ticks() == write_ticks &&
                  fixture.runtime->completed_cycles() == 3 && fixture.runtime->ties() == 1 &&
                  fixture.rejected_reentry,
              "Independent native/external/shared-write permutation or literal clocks differ");
    }
  }
}

void checkpoint_and_missing_write() {
  const Case spec{.native = McuWriteOrder::before_mcu,
                  .external = McuWriteOrder::before_mcu,
                  .source = DspPeerOrder::peer_first,
                  .expected = "EDW"};
  Fixture source(spec);
  source.step();
  const auto snapshot = source.runtime->checkpoint();
  Fixture clone(spec);
  clone.identity = source.identity;
  clone.runtime =
      std::make_unique<McuDspRuntime>(snapshot, clone.mcu, clone.dsp, clone.external_events());
  clone.bind();
  source.step();
  clone.step();
  constexpr unsigned final_pc = 0x8004;
  require(source.events == spec.expected && clone.events == spec.expected &&
              detail::mcu_boundary(source.mcu) == detail::mcu_boundary(clone.mcu) &&
              source.mcu.cpu().state().pc == final_pc && source.mcu.cpu().state().d == value &&
              source.dsp.cpu().registers() == clone.dsp.cpu().registers() &&
              clone.runtime->end_ticks() == write_ticks,
          "Joint native MCU/DSP recovery differs from literal store/time and replay expectations");
  Fixture missing(spec);
  missing.drop_write = true;
  missing.step();
  rejects([&] { missing.step(); });
  require(missing.runtime->faulted(), "Dropped native write did not quarantine the coordinator");
}

void native_failure_context() {
  const Case spec{.native = McuWriteOrder::before_mcu,
                  .external = McuWriteOrder::before_mcu,
                  .source = DspPeerOrder::peer_first,
                  .expected = "EDW"};
  Fixture fixture(spec, true);
  std::string message;
  try {
    constexpr unsigned bound = 32;
    for (unsigned count = 0; count < bound; ++count) { fixture.step(); }
  } catch (const Fault &error) { message = error.what(); }
  require(message.contains("read X:0x002000") && message.contains("pc=0xff0000") &&
              message.contains("mcu-pc=0x800") && message.contains("tick=") &&
              message.contains("Unknown DSP memory") && fixture.runtime->faulted(),
          "Genuine unknown native operand lost operation, address, PCs, time or underlying cause");
}
}

int main() {
  try {
    ordering_matrix();
    checkpoint_and_missing_write();
    native_failure_context();
    std::cout << "MCU/dsp56300 runtime ordering and joint checkpoint controls passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
