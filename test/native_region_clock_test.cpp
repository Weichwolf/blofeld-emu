// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_dma_registers.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi_registers.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
// Integral test timebase: 12MHz EXTAL gives 45 ticks/cycle, PLL 9/2 gives 10.
constexpr unsigned extal = 12000000;
constexpr std::uint64_t timebase = 540000000;
constexpr std::uint64_t period = 45;
constexpr std::uint64_t pll_period = 10;
constexpr unsigned pll_9_over_2 = 0x02e112;
constexpr unsigned csr = 0xffff8f, load = 0xffff8e, compare = 0xffff8d, count = 0xffff8c;
constexpr unsigned timer_enable_compare_irq = 5;
constexpr unsigned timer_compare_vector = 0x54;
constexpr unsigned tx_irq = 0x800, tx_vector = 0x40;
constexpr unsigned sample = 0x123456;
constexpr unsigned observed_pc = 0x123;
constexpr unsigned dma_source = 0xffffef, dma_destination = 0xffffee, dma_count = 0xffffed;
constexpr unsigned dma_control = 0xffffec, dma_word_copy = 0x880244;
constexpr std::uint64_t counter_range = 0x1000000;
std::uint64_t assertions = 0;

void require(const bool condition, const std::string &message) {
  ++assertions;
  if (!condition) { throw blofeld::Fault(message); }
}

std::string refuses(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected a visible native-region refusal");
}

struct Span {
  unsigned cycles;
  unsigned requests = 0;
};

struct Device {
  blofeld::DspBus bus;
  blofeld::DspRunClock clock{
      {.extal_hz = extal,
       .timebase_hz = timebase,
       .switch_timing = blofeld::DspClockSwitch::functional_region_boundary}};

  Device() {
    bus.pll().observe_pinit(false);
    bus.reset();
    require(clock.next_native_boundary_ticks(bus) == period,
            "An unsampled DMA controller requires a real first cycle");
    require(!cycle(), "The first real DMA baseline has no fabricated word");
    require(bus.native_dma_quiescent() && !clock.next_native_boundary_ticks(bus),
            "A sampled reset device has no internal event before input changes");
  }

  [[nodiscard]] blofeld::DspDmaInputs inputs(const Span span) const {
    const auto previous = bus.last_dma_cycle();
    return {.cycle = previous ? *previous + span.cycles : 0,
            .known_requests = 15,
            .requests = span.requests,
            .timing = blofeld::DspDmaTiming::reference_sampled_cycles,
            .grant = blofeld::DspDmaGrant::available};
  }

  void begin() { clock.begin_step(bus, {.pc = observed_pc, .omr = 0}); }

  [[nodiscard]] std::optional<blofeld::DspDmaWord> cycle(const unsigned requests = 0) {
    begin();
    return clock.finish_native_cycle(bus, inputs({.cycles = 1, .requests = requests}));
  }

  void span(const Span span) {
    begin();
    clock.finish_native_region(bus, span.cycles, inputs(span));
  }

  void timer(const unsigned match) {
    bus.timers().write({.address = load, .value = 3});
    bus.timers().write({.address = compare, .value = match});
    bus.timers().write({.address = csr, .value = timer_enable_compare_irq});
  }
};

void test_timer_phase_and_exact_boundary() {
  Device device;
  device.timer(7);
  require(device.clock.next_native_boundary_ticks(device.bus) == 9 * period,
          "Five CLK/2 edges from an odd native phase take nine cycles");
  device.span({.cycles = 8});
  require(device.clock.native_cycle_units() == 9 && device.clock.elapsed_ticks() == 9 * period &&
              device.clock.timer_clk2_edges() == 4 && device.bus.timers().read(count) == 6 &&
              device.bus.timers().interrupt_vectors().empty() && device.bus.last_dma_cycle() == 8,
          "An aggregate region retains actual time, timer phase and consecutive DMA cursor");
  require(device.clock.next_native_boundary_ticks(device.bus) == period,
          "One final native cycle remains before the exact compare");
  device.span({.cycles = 1});
  require(device.clock.native_cycle_units() == 10 && device.clock.timer_clk2_edges() == 5 &&
              device.bus.timers().read(count) == 7 &&
              device.bus.timers().interrupt_vectors() == std::vector{timer_compare_vector} &&
              !device.clock.next_native_boundary_ticks(device.bus),
          "The exact final edge latches the IRQ without blocking on an already-pending source");
  device.bus.timers().acknowledge_interrupt(timer_compare_vector);
  require(device.clock.next_native_boundary_ticks(device.bus) == 2 * counter_range * period,
          "A future full-counter horizon is not truncated to the per-run capacity");
}

void test_event_crossing_and_unknown_inputs() {
  Device crossing;
  crossing.timer(4);
  const auto ticks = crossing.clock.elapsed_ticks();
  const auto cursor = crossing.bus.last_dma_cycle();
  const auto failure = refuses([&] { crossing.span({.cycles = 4}); });
  require(failure.contains("pc=0x000123") && failure.contains("crossed a device event") &&
              crossing.clock.faulted() && crossing.clock.elapsed_ticks() == ticks &&
              crossing.bus.last_dma_cycle() == cursor && crossing.bus.timers().read(count) == 0,
          "A region cannot consume DMA or timer clocks past its known event");

  Device unknown;
  unknown.bus.timers().write({.address = csr, .value = 1});
  require(unknown.clock.next_native_boundary_ticks(unknown.bus) == period,
          "An unknown timer preload retains its next genuine clock check");
  const auto missing = refuses([&] { unknown.span({.cycles = 1}); });
  require(missing.contains("Unknown DSP timer load") && missing.contains("0xffff8e") &&
              missing.contains("pc=0x000123") && unknown.clock.faulted(),
          "Bulk execution retains the actual unsupported-input address and CPU context");

  Device changed;
  const auto before = changed.bus.last_dma_cycle();
  const auto request = refuses([&] { changed.span({.cycles = 1, .requests = 1}); });
  require(request.contains("held-request baseline") && changed.clock.faulted() &&
              changed.bus.last_dma_cycle() == before,
          "A new request edge must take the actual DMA path instead of an idle span");
}

void test_mapping_and_shi_boundaries() {
  Device mapping;
  constexpr unsigned data_alias = 0x10000;
  constexpr unsigned code_alias = 0x1000;
  constexpr unsigned switched_omr = 0x600085;
  mapping.bus.write({.space = blofeld::DspSpace::x, .address = data_alias, .value = sample});
  mapping.bus.observe_omr(switched_omr);
  const auto revision = mapping.bus.program_revision();
  require(mapping.clock.next_native_boundary_ticks(mapping.bus) == 3 * period,
          "The pending memory map retains its three real cycle units");
  mapping.span({.cycles = 2});
  require(mapping.clock.next_native_boundary_ticks(mapping.bus) == period &&
              !mapping.bus.inspect_known_program(code_alias) &&
              mapping.bus.program_revision() == revision,
          "A partial region cannot prematurely expose switched code");
  mapping.span({.cycles = 1});
  require(mapping.bus.inspect_known_program(code_alias) == sample &&
              mapping.bus.program_revision() == revision + 1 &&
              !mapping.clock.next_native_boundary_ticks(mapping.bus),
          "The final cycle commits the actual map and invalidates the code view");

  Device serial;
  serial.bus.shi().observe_spi({.selected = false, .clock = false, .mosi = false});
  serial.bus.shi().write({.address = blofeld::shi::hcsr, .value = blofeld::shi::hen | tx_irq});
  serial.bus.shi().write({.address = blofeld::shi::hcsr, .value = blofeld::shi::hen});
  require(serial.clock.next_native_boundary_ticks(serial.bus) == period &&
              serial.bus.shi().interrupt_vectors() == std::vector{tx_vector},
          "Removing SHI IRQ enable still retains its actual one-cycle delay");
  serial.span({.cycles = 1});
  require(serial.bus.shi().interrupt_vectors().empty() &&
              !serial.clock.next_native_boundary_ticks(serial.bus),
          "The next real cycle applies the delayed mask");
  serial.bus.shi().write({.address = blofeld::shi::hcsr, .value = 0});
  require(serial.clock.next_native_boundary_ticks(serial.bus) == period,
          "Individual SHI reset supplies a separate one-cycle boundary");
  serial.span({.cycles = 1});
  require(!serial.bus.shi().next_cycle_boundary() &&
              !serial.clock.next_native_boundary_ticks(serial.bus),
          "Bulk clocks complete the real SHI reset without inventing further delay");
}

void test_actual_dma_then_idle_region() {
  Device device;
  constexpr unsigned source = 0x80;
  constexpr unsigned destination = 0x90;
  device.bus.write({.space = blofeld::DspSpace::x, .address = source, .value = sample});
  for (const auto store :
       {blofeld::DspStore{.space = blofeld::DspSpace::x, .address = dma_source, .value = source},
        {.space = blofeld::DspSpace::x, .address = dma_destination, .value = destination},
        {.space = blofeld::DspSpace::x, .address = dma_count, .value = 0},
        {.space = blofeld::DspSpace::x, .address = dma_control, .value = dma_word_copy}}) {
    device.bus.write(store);
  }
  for (unsigned cycle = 0; cycle < blofeld::dma::settling_cycles; ++cycle) {
    require(device.clock.next_native_boundary_ticks(device.bus) == period && !device.cycle(),
            "Enable settling uses actual single-cycle DMA observations");
  }
  require(device.bus.has_active_dma() && device.bus.native_dma_quiescent(),
          "The real enabled controller can wait for an actual request without per-cycle replay");
  device.span({.cycles = 64});
  require(!device.cycle(1) && !device.bus.native_dma_quiescent() &&
              device.clock.next_native_boundary_ticks(device.bus) == period,
          "A genuine rising request consumes only the source phase");
  const auto result = device.cycle(1);
  require(result && result->value == sample && device.bus.native_dma_quiescent() &&
              device.bus.read({.space = blofeld::DspSpace::y, .address = destination}) == sample,
          "The next actual cycle commits the original RAM word through the bus");
  device.bus.write({.space = blofeld::DspSpace::y, .address = destination, .value = 0});
  device.span({.cycles = 64, .requests = 1});
  constexpr std::uint64_t expected_cycles =
      134; // Baseline + settle3 + idle64 + transfer2 + idle64.
  require(device.bus.read({.space = blofeld::DspSpace::y, .address = destination}) == 0 &&
              device.clock.native_cycle_units() == expected_cycles &&
              device.clock.elapsed_ticks() == expected_cycles * period &&
              device.bus.last_dma_cycle() == expected_cycles - 1,
          "Held requests cannot retrigger a completed word while aggregate clocks stay exact");
}

void test_pll_change_at_completed_boundary() {
  Device device;
  device.span({.cycles = 2});
  device.bus.pll().write(pll_9_over_2);
  device.span({.cycles = 3});
  constexpr std::uint64_t expected_ticks = (3 * period) + (3 * pll_period);
  require(device.clock.elapsed_ticks() == expected_ticks &&
              device.clock.native_cycle_units() == 6 && device.clock.timer_clk2_edges() == 3,
          "A completed PLL command changes the following region without losing timer phase");
}
}

int main() {
  try {
    test_timer_phase_and_exact_boundary();
    test_event_crossing_and_unknown_inputs();
    test_mapping_and_shi_boundaries();
    test_actual_dma_then_idle_region();
    test_pll_change_at_completed_boundary();
    std::cout << "Native-region clock/peripherals: " << assertions << " assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
