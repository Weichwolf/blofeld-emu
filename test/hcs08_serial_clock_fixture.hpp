// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_spi_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <span>
#include <vector>

namespace blofeld::test::hcs08_serial_clock {
using namespace blofeld::test::hcs08;
namespace app = blofeld::test::hcs08_clock_startup;
namespace wire = blofeld::test::hcs08_spi_test;
namespace adc = hcs08_adc;
constexpr unsigned poll = 0xe33a, stack = 0x47f, cursor = 0x97, tx = 0x100, partial = 0xef;
constexpr unsigned conversion_periods = 176, firmware_stack = app::stack - 8;
constexpr unsigned reference_uv = 1536000, admitted_strap = 8;
constexpr Hcs08AdcReference reference{
    .bus_hz = 6000000, .supply_uv = 3300000, .low_uv = 0, .high_uv = 3072000};

struct Setup {
  Hcs08SpiRunContract order;
  Hcs08SpiLaunchContract launch;
  unsigned seed;
  Hcs08PrefetchContract program;
  bool scanner = false;
  bool reconnect_gpio = false;
  std::optional<unsigned> pads;
};

class NoMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override { return false; }

  std::uint8_t read(std::uint16_t /*address*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected original serial-clock BDC memory read");
  }

  void write(ByteWrite /*access*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected original serial-clock BDC memory write");
  }
};

struct Event {
  unsigned period;
  Access access;
  bool operator==(const Event &) const = default;
};

inline app::ApplicationMemory startup_ram(const std::span<const std::uint8_t> pages,
                                          std::ostream &trace) {
  app::ApplicationMemory memory(pages, {.seed = 0, .trim = 0x80, .observation = admitted_strap});
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(app::warm_state(0));
  memory.warm_reset();
  cpu.reset();
  while (memory.retired < app::status_retired) { cpu.step(); }
  require(cpu.state().pc == app::status_pc && cpu.state().sp == firmware_stack && !cpu.faulted(),
          "Actual original reset/JSR/AIS establishes the main frame before strict ICGS1");
  app::ram_contents(memory, true);
  trace << "STARTUP retired=" << memory.retired << " pc=" << cpu.state().pc
        << " sp=" << cpu.state().sp << " ram_known=" << memory.known.count() << '\n';
  return memory;
}

struct Machine {
  app::ApplicationMemory memory;
  Hcs08 cpu;
  const Setup setup;
  NoMemory mapping;
  std::unique_ptr<Hcs08Bdc> bdc;
  std::unique_ptr<Hcs08SpiClock> spi;
  std::unique_ptr<Hcs08SpiRunClock> clock;
  std::vector<Event> events, ram_events;
  unsigned transmitted = 0, samples = 0;

  Machine(const std::span<const std::uint8_t> pages,
          const Setup input,
          const app::ApplicationMemory *startup = nullptr)
      : memory(startup != nullptr
                   ? *startup
                   : app::ApplicationMemory(pages, {.seed = 0, .trim = 0x80, .observation = -1})),
        cpu(memory, BLOFELD_HCS08_SLA, Hcs08RtiIrqContract::reference_immediate),
        setup(input) {
    if (setup.reconnect_gpio) { connect_pads(); }
    require(input.scanner == (startup != nullptr),
            "Scanner caller requires actual original startup RAM");
    if (!input.scanner) { memory.warm_reset(); }
    memory.retired = 0;
    memory.accesses.clear();
    memory.flash_reads.clear();
    memory.peripheral_accesses.clear();
    constexpr std::array<std::uint8_t, 5> cursors{0, 3, 0, 0, 0};
    memory.install(cursor, cursors);
    memory.install(tx, wire::event_bytes);
    if (input.program != Hcs08PrefetchContract::reference_peripheral_program_cycles) {
      const std::array<std::uint8_t, 1> expiry{static_cast<std::uint8_t>(input.seed)};
      constexpr unsigned expiry_index = 0xa8;
      memory.install(expiry_index, expiry);
    }
    wire::configure(memory.spi);
    memory.adc.observe_reference(reference);
    memory.adc.observe_input(0, reference_uv);
    memory.adc.write({.address = adc::pins, .value = 7});
    constexpr unsigned preset = 0xc2;
    memory.adc.write({.address = adc::control, .value = preset});
    memory.adc.write({.address = adc::status, .value = 0});
    require(memory.adc.state().active && memory.adc.state().remaining == conversion_periods,
            "Declared real conversion remains busy while the original foreground polls its status");
    cpu.set_state({.pc = poll,
                   .hx = static_cast<std::uint16_t>((input.seed << 8U) | (byte_mask - input.seed)),
                   .sp = static_cast<std::uint16_t>(input.scanner ? firmware_stack : stack),
                   .a = static_cast<std::uint8_t>(byte_mask - input.seed),
                   .ccr_known = partial,
                   .ccr = flags_fixed | flag_c});
    bdc = std::make_unique<Hcs08Bdc>(
        cpu, mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::normal);
    spi = std::make_unique<Hcs08SpiClock>(
        memory.spi, Hcs08SpiClockContract::reference_half_period_and_final_edge);
    clock =
        std::make_unique<Hcs08SpiRunClock>(*bdc, *spi, input.order, input.launch, input.program);
  }

  void connect_pads(const bool include_a = true) {
    memory.bind(admitted_strap);
    if (!include_a || !setup.pads) { return; }
    const auto levels = *setup.pads;
    memory.gpio.attach_pad_source(Hcs08GpioPort::a, [levels](const unsigned mask) {
      return GpioPadSample{.known = mask, .levels = levels & mask};
    });
  }

  void tick() {
    if (setup.reconnect_gpio) {
      require(!clock->faulted(), "Incomplete CPU/SPI/ADC operation requires coordinated recovery");
    }
    const auto before = cpu.state();
    memory.adc.context({.pc = before.pc, .ccr = before.ccr});
    memory.adc.advance_bus(
        1); // Explicit board tie: old ADC work precedes both common-clock stages.
    const auto peripheral = memory.spi.state();
    const unsigned bit = peripheral.active && peripheral.samples < wire::samples
                             ? wire::samples - peripheral.samples - 1
                             : wire::samples - 1;
    const bool miso = ((static_cast<unsigned>(wire::reply_bytes[0]) >> bit) & 1U) != 0;
    const auto begin = memory.peripheral_accesses.size();
    const auto ram_begin = memory.accesses.size();
    const auto pins = clock->advance(miso);
    const auto period = static_cast<unsigned>(clock->state().completed_periods);
    for (auto n = begin; n < memory.peripheral_accesses.size(); ++n) {
      events.push_back({.period = period, .access = memory.peripheral_accesses[n]});
    }
    for (auto n = ram_begin; n < memory.accesses.size(); ++n) {
      ram_events.push_back({.period = period, .access = memory.accesses[n]});
    }
    if (pins && !pins->clock) {
      if (!pins->mosi) { throw Fault("Missing original MOSI at a real trailing sampling edge"); }
      transmitted = (transmitted << 1U) | static_cast<unsigned>(*pins->mosi);
      ++samples;
    }
    require(clock->state().spi_periods == period && spi->state().offered_periods == period &&
                clock->state().irq_periods + clock->state().program_periods == period &&
                memory.adc.state().remaining == conversion_periods - period &&
                memory.adc.state().conversions ==
                    static_cast<unsigned>(period == conversion_periods),
            "One reference BUSCLK advances genuine CPU/SPI/ADC owners once, never poll-driven "
            "readiness");
  }
};

struct Snapshot {
  app::ApplicationMemory memory;
  Hcs08Checkpoint cpu;
  Hcs08BdcCheckpoint bdc;
  Hcs08SpiClockCheckpoint spi;
  Hcs08SpiRunCheckpoint clock;
  std::vector<Event> events, ram_events;
  unsigned transmitted, samples;
};

inline Snapshot capture(const Machine &original) {
  const auto state = original.cpu.state();
  return {.memory = original.memory,
          .cpu = original.cpu.checkpoint(),
          .bdc = original.bdc->checkpoint(),
          .spi = original.spi->checkpoint({.pc = state.pc, .ccr = state.ccr}),
          .clock = original.clock->checkpoint(),
          .events = original.events,
          .ram_events = original.ram_events,
          .transmitted = original.transmitted,
          .samples = original.samples};
}

inline void restore(Machine &m, const Snapshot &s, const bool reconnect_a = true) {
  m.memory = s.memory;
  if (m.setup.reconnect_gpio) { m.connect_pads(reconnect_a); }
  m.cpu.restore_checkpoint(s.cpu);
  m.bdc->restore_checkpoint(s.bdc);
  const auto state = m.cpu.state();
  m.spi->restore_checkpoint(s.spi, {.pc = state.pc, .ccr = state.ccr});
  m.clock->restore_checkpoint(s.clock);
  m.events = s.events;
  m.ram_events = s.ram_events;
  m.transmitted = s.transmitted;
  m.samples = s.samples;
}

}
