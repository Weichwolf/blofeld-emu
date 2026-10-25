// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_run_clock.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "storage/private_file.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <sstream>
#include <string>

namespace {
using namespace blofeld;
using test::hcs08::rejected;
using test::hcs08::require;
namespace app = test::hcs08_clock_startup;
namespace hp = test::hcs08_programmer;
namespace adc = hcs08_adc;
constexpr unsigned setup = 0xe1b4, setup_end = 0xe1bc, poll = 0xe33a, repeat = 0xe7ae;
constexpr unsigned stack = 0x475, caller_hx = 0x5aa5, caller_a = 0xa5;
constexpr unsigned flags = 0x68, unknown_half = 0xef, all_known = 0xff, define_half = 0xe152;
constexpr unsigned conversion = 176, setup_periods = 13, ready_periods = 156, poll_periods = 5;
constexpr unsigned ready_instructions = 53, initial_sum = 0x12f0, next_remaining = 135;
constexpr unsigned frame_bound = 200, trim = 0x80, microvolts_per_step = 3000;
constexpr std::array<unsigned, 3> sums{0xb8, 0xba, 0xbc}, counters{0xbf, 0xbe, 0xc0};
constexpr Hcs08AdcReference reference{
    .bus_hz = 6000000, .supply_uv = 3300000, .low_uv = 0, .high_uv = 3072000};

class NoBdcMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override { return false; }

  std::uint8_t read(std::uint16_t /*address*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected ADC component BDC memory read");
  }

  void write(ByteWrite /*access*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected ADC component BDC memory write");
  }
};

struct Machine {
  app::ApplicationMemory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};
  NoBdcMemory mapping;
  std::unique_ptr<Hcs08Bdc> bdc;
  std::unique_ptr<Hcs08RunClock> clock;
  unsigned edges = 0;

  Machine(const std::span<const std::uint8_t> records,
          const unsigned sample,
          const bool known_inputs = true)
      : memory(records, {.seed = 0, .trim = trim, .observation = -1}) {
    memory.warm_reset();
    memory.adc.observe_reference(reference);
    for (unsigned channel = 0; channel < sums.size(); ++channel) {
      if (known_inputs) { memory.adc.observe_input(channel, sample * microvolts_per_step); }
    }
    cpu.set_state({.pc = setup,
                   .hx = caller_hx,
                   .sp = stack,
                   .a = caller_a,
                   .ccr_known = unknown_half,
                   .ccr = flags});
    bdc = std::make_unique<Hcs08Bdc>(
        cpu, mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
    clock = std::make_unique<Hcs08RunClock>(
        memory, *bdc, Hcs08RunContract::reference_instruction_retirement);
    command(hcs08_bdc::go);
  }

  void tick() {
    // Declared reference tie: old ADC work advances before native effects commit.
    const auto current = cpu.state();
    memory.adc.context({.pc = current.pc, .ccr = current.ccr});
    memory.adc.advance_bus(1);
    clock->advance();
    ++edges;
    require(clock->state().offered_periods == edges,
            "One common reference BUSCLK is offered to both genuine owners");
  }

  void command(const std::uint8_t byte) const {
    bdc->receive(byte);
    bdc->service(Hcs08BdcGrant::reference_instruction_boundary);
  }
};

void prefix(Machine &m) {
  for (unsigned period = 1; period <= setup_periods; ++period) {
    m.tick();
    unsigned retired = 0;
    for (const unsigned boundary : {4U, 8U, setup_periods}) {
      retired += static_cast<unsigned>(period >= boundary);
    }
    require(m.memory.retired == retired && m.cpu.state().ccr_known == unknown_half &&
                m.memory.adc.state().conversions == 0 && !m.memory.adc.state().result,
            "Literal4/4/5-period original MOV/MOV/CLR boundaries never create an ADC result");
    require(m.memory.adc.state().active == (period == setup_periods),
            "Original status CLR starts ADC only at its actual native retirement boundary");
  }
  require(m.cpu.state().pc == setup_end && m.memory.adc.state().remaining == conversion &&
              m.clock->state().executing_periods == setup_periods && m.memory.known.none(),
          "Original setup completes in thirteen periods with176 real ADC periods outstanding");
  const std::array<std::uint8_t, 2> sum{
      static_cast<std::uint8_t>(initial_sum >> 8U),
      static_cast<std::uint8_t>(initial_sum & test::hcs08::byte_mask)};
  constexpr std::array<std::uint8_t, 1> count{1};
  for (unsigned channel = 0; channel < sums.size(); ++channel) {
    m.memory.install(sums[channel], sum);
    m.memory.install(counters[channel], count);
  }
}

void enter_poll(Machine &m, const unsigned phase) {
  require(m.clock->state().remaining_periods == 0 && !m.cpu.instruction_reserved(),
          "An explicit component caller can select its entry only at an unowned boundary");
  auto state = m.cpu.state();
  state.pc = poll;
  m.cpu.set_state(state); // Declared scanner entry, not a boot-origin foreground loop.
  m.command(hcs08_bdc::background);
  const unsigned elapsed = conversion - m.memory.adc.state().remaining;
  require(phase >= elapsed && phase <= conversion, "Declared ADC phase never rewinds real work");
  for (unsigned period = elapsed; period < phase; ++period) { m.tick(); }
  require(
      m.bdc->state().active && m.cpu.state().pc == poll &&
          m.memory.adc.state().remaining == conversion - phase,
      "Actual held BDC periods advance ADC without a CPU fetch, result injection or retirement");
  m.command(hcs08_bdc::go);
}

void suffix(Machine &m) {
  unsigned periods = 0;
  auto known = m.cpu.state().ccr_known;
  while (m.cpu.state().pc != repeat) {
    require(++periods <= frame_bound, "Bounded original ADC window must reach repeat_scan");
    const auto before = m.cpu.state();
    const auto retired = m.memory.retired;
    const bool defines_half = before.pc == define_half;
    m.tick();
    if (m.memory.retired != retired) {
      if (defines_half) { known = all_known; }
    } else {
      require(m.cpu.state() == before, "Incomplete native budget cannot mutate CPU registers");
    }
    require(m.cpu.state().ccr_known == known,
            "Shared-clock execution retains unknown H until the actual ADD retires");
  }
}

struct Observation {
  unsigned sample, phase;
};

void endpoint(const Machine &m, const unsigned channel, const Observation observation) {
  const auto sample = observation.sample;
  const auto value = initial_sum + sample;
  require(m.cpu.state() == Hcs08State{.pc = repeat,
                                      .hx = static_cast<std::uint16_t>(sums[channel]),
                                      .sp = stack,
                                      .a = 1,
                                      .ccr = flags} &&
              m.memory.bytes[sums[channel]] == value >> 8U &&
              m.memory.bytes[sums[channel] + 1] == (value & test::hcs08::byte_mask) &&
              m.memory.bytes[counters[channel]] == 2 &&
              m.memory.adc.state().conversions == channel + 1 &&
              (m.memory.adc.state().status & adc::channel_mask) == (channel + 1) % sums.size() &&
              m.memory.adc.state().remaining == next_remaining && m.memory.adc.state().active &&
              m.memory.adc.state().result == sample && !m.memory.adc.interrupt_requested(),
          "Independent ADC endpoint channel=" + std::to_string(channel) +
              " sample=" + std::to_string(sample) + " pc=" + hex(m.cpu.state().pc) +
              " hx=" + hex(m.cpu.state().hx) + " sp=" + hex(m.cpu.state().sp) +
              " a=" + hex(m.cpu.state().a) + " ccr=" + hex(m.cpu.state().ccr) + " known=" +
              hex(m.cpu.state().ccr_known) + " sum-hi=" + hex(m.memory.bytes[sums[channel]]) +
              " sum-lo=" + hex(m.memory.bytes[sums[channel] + 1]) +
              " counter=" + std::to_string(m.memory.bytes[counters[channel]]) +
              " status=" + hex(m.memory.adc.state().status) +
              " remaining=" + std::to_string(m.memory.adc.state().remaining) +
              " conversions=" + std::to_string(m.memory.adc.state().conversions) + " result=" +
              std::to_string(m.memory.adc.state().result.value_or(test::hcs08::word_mask)) +
              " active=" + std::to_string(static_cast<unsigned>(m.memory.adc.state().active)));
}

void window(Machine &m,
            const Observation observation,
            const unsigned channel,
            std::ostream &trace) {
  const auto sample = observation.sample;
  const auto phase = observation.phase;
  enter_poll(m, phase);
  if (phase < conversion - poll_periods) {
    const auto before = m.edges;
    const auto retired = m.memory.retired;
    const auto known = m.cpu.state().ccr_known;
    suffix(m);
    require(m.edges - before == 9 && m.memory.retired - retired == 2 &&
                m.cpu.state().ccr_known == known &&
                m.memory.adc.state().conversions == channel + 1 &&
                m.memory.adc.state().remaining == 0 && m.memory.adc.state().result == sample &&
                (m.memory.adc.read(adc::status) & adc::complete) != 0,
            "Completion after BRSET retirement stays pending across the genuine JMP miss path");
    trace << "MISS channel=" << channel << " periods=9 instructions=2 pending=1\n";
    enter_poll(m, conversion);
  }
  const auto before = m.edges;
  const auto retired = m.memory.retired;
  suffix(m);
  const auto periods = ready_periods + (channel == 2 ? 1U : 0U);
  require(m.edges - before == periods && m.memory.retired - retired == ready_instructions,
          "Independent primary instruction sums give156/156/157 periods and53 native retirements");
  endpoint(m, channel, observation);
  trace << "READY channel=" << channel << " periods=" << periods
        << " instructions=53 sum=" << initial_sum + sample
        << " counter=2 remaining=135 known=255\n";
}

void replays(const std::span<const std::uint8_t> records, std::ostream &trace) {
  constexpr unsigned sample = 512;
  constexpr unsigned phase = 171;
  constexpr std::array<unsigned, 12> cuts{0, 1, 4, 5, 8, 9, 30, 31, 65, 100, 115, 155};
  Machine first(records, sample);
  prefix(first);
  enter_poll(first, phase);
  const auto initial_memory = first.memory;
  const auto initial_cpu = first.cpu.checkpoint();
  const auto initial_bdc = first.bdc->checkpoint();
  const auto initial_clock = first.clock->checkpoint();
  const auto initial_edges = first.edges;
  suffix(first);
  endpoint(first, 0, {.sample = sample, .phase = phase});
  for (const auto cut : cuts) {
    Machine partial(records, sample);
    partial.memory = initial_memory;
    partial.cpu.restore_checkpoint(initial_cpu);
    partial.bdc->restore_checkpoint(initial_bdc);
    partial.clock->restore_checkpoint(initial_clock);
    partial.edges = initial_edges;
    for (unsigned edge = 0; edge < cut; ++edge) { partial.tick(); }
    const auto cpu = partial.cpu.checkpoint();
    const auto bdc = partial.bdc->checkpoint();
    const auto clock = partial.clock->checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      Machine fresh(records, sample);
      fresh.memory = partial.memory;
      fresh.cpu.restore_checkpoint(cpu);
      fresh.bdc->restore_checkpoint(bdc);
      fresh.clock->restore_checkpoint(clock);
      fresh.edges = partial.edges;
      suffix(fresh);
      endpoint(fresh, 0, {.sample = sample, .phase = phase});
      app::same_endpoint(fresh.memory, first.memory);
      require(
          fresh.cpu.state() == first.cpu.state() && fresh.bdc->state() == first.bdc->state() &&
              fresh.clock->state() == first.clock->state() && fresh.edges == first.edges,
          "Fresh CPU/RAM/ADC/BDC/clock graphs preserve a real partial instruction and conversion");
      trace << "REPLAY cut=" << cut << " index=" << replay << " identical=1\n";
    }
  }
}

void unknown_input(const std::span<const std::uint8_t> records, std::ostream &trace) {
  Machine m(records, 0, false);
  prefix(m);
  const auto cpu = m.cpu.state();
  const auto clock = m.clock->state();
  const auto adc_state = m.memory.adc.state();
  const auto ram = m.memory.bytes;
  const auto error = rejected([&] { m.tick(); });
  require(
      error.contains("sample address=0x0051 pc=0xe1bc ccr=0x006a") &&
          error.contains("remaining=176 channel=0: unknown enabled analog pin=0") &&
          m.cpu.state() == cpu && m.clock->state() == clock && m.memory.adc.state() == adc_state &&
          m.memory.adc.faulted() && m.edges == setup_periods && m.memory.bytes == ram &&
          m.memory.retired == 3,
      "Unknown actual pin quarantines ADC before a CPU period, RAM effect or fabricated sample");
  trace << "UNKNOWN " << error << '\n';
}

void run(const std::span<const std::uint8_t> records, std::ostream &trace) {
  constexpr std::array<unsigned, 5> samples{0, 1, 511, 512, 1023};
  constexpr std::array<unsigned, 5> phases{170, 171, 172, 175, 176};
  for (const auto sample : samples) {
    for (const auto phase : phases) {
      Machine m(records, sample);
      prefix(m);
      const auto image = m.memory.flash.image();
      trace << "CASE sample=" << sample << " phase=" << phase << " setup=13 known=239\n";
      for (unsigned channel = 0; channel < sums.size(); ++channel) {
        window(m, {.sample = sample, .phase = phase}, channel, trace);
      }
      require(m.memory.flash.image() == image && !m.memory.options.written() &&
                  m.memory.spi.state() == Hcs08SpiState{} && !m.cpu.faulted() &&
                  !m.clock->faulted(),
              "Actual timed component leaves all original FLASH and unrelated SPI/SOPT untouched");
      trace << "END edges=" << m.edges << " executing=" << m.clock->state().executing_periods
            << " held=" << m.edges - m.clock->state().executing_periods
            << " retired=" << m.memory.retired
            << " conversions=3 remaining=135 flash-unchanged=1\n";
    }
  }
  replays(records, trace);
  unknown_input(records, trace);
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned pages and private original ADC common-clock trace");
    const auto records = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    run(records, trace);
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original ADC shared-clock retirement boundaries and joint snapshots pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
