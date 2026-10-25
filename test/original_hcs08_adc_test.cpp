// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "storage/private_file.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <sstream>

namespace {
using namespace blofeld;
using test::hcs08::rejected;
using test::hcs08::require;
namespace app = test::hcs08_clock_startup;
namespace hp = test::hcs08_programmer;
namespace adc = hcs08_adc;
constexpr unsigned setup = 0xe1b4, setup_end = 0xe1bc, poll = 0xe33a, repeat = 0xe7ae;
constexpr unsigned stack = 0x475, foreground_hx = 0x5aa5, foreground_a = 0xa5;
constexpr unsigned flags = 0x68, half_carry = 0x10, trim = 0x80;
constexpr unsigned all_known = 0xff, unknown_half = 0xef, define_half = 0xe152;
constexpr unsigned adc_control = 0xc2, pin_mask = 7, single_periods = 176;
constexpr unsigned initial_sum = 0x12f0, work = 0xa3, instruction_count = 55, bound = 64;
constexpr unsigned microvolts_per_step = 3000, total_retired = 168;
constexpr std::array<unsigned, 3> sums{0xb8, 0xba, 0xbc}, counters{0xbf, 0xbe, 0xc0};
constexpr Hcs08AdcReference reference{
    .bus_hz = 6000000, .supply_uv = 3300000, .low_uv = 0, .high_uv = 3072000};

void original_pages(const app::ApplicationMemory &memory,
                    const std::span<const std::uint8_t> records) {
  const auto &image = memory.flash.image();
  for (unsigned page = 0; page < app::pages.size(); ++page) {
    for (unsigned n = 0; n < app::hf::page_bytes; ++n) {
      const auto offset = app::pages[page] - app::hf::first + n;
      require(image.known[offset] &&
                  image.data[offset] == records[(page * hp::record_bytes) + 2 + n],
              "All supplied original instruction/data/calibration bytes stay unchanged");
    }
  }
}

void foreground(Hcs08 &cpu, const unsigned pc) {
  auto state = cpu.state();
  state.pc = static_cast<std::uint16_t>(pc);
  cpu.set_state(state);
}

void prefix(app::ApplicationMemory &memory,
            Hcs08 &cpu,
            const unsigned initial_flags,
            const unsigned initial_known) {
  cpu.set_state({.pc = setup,
                 .hx = foreground_hx,
                 .sp = stack,
                 .a = foreground_a,
                 .ccr_known = static_cast<std::uint8_t>(initial_known),
                 .ccr = static_cast<std::uint8_t>(initial_flags)});
  for (unsigned n = 0; n < 3; ++n) { cpu.step(); }
  require(cpu.state() == Hcs08State{.pc = setup_end,
                                    .hx = foreground_hx,
                                    .sp = stack,
                                    .a = foreground_a,
                                    .ccr_known = static_cast<std::uint8_t>(initial_known),
                                    .ccr = static_cast<std::uint8_t>(initial_flags | 2U)} &&
              memory.retired == 3 && memory.adc.read(adc::control) == adc_control &&
              memory.adc.read(adc::pins) == pin_mask && memory.adc.read(adc::status) == 0 &&
              memory.adc.state().remaining == single_periods && !memory.adc.state().result &&
              memory.known.none(),
          "Three actual original MOV/MOV/CLR stores start channel0 without inventing a result");
}

void initialize_ram(app::ApplicationMemory &memory) {
  const std::array<std::uint8_t, 2> sum{
      static_cast<std::uint8_t>(initial_sum >> 8U),
      static_cast<std::uint8_t>(initial_sum & test::hcs08::byte_mask)};
  const std::array<std::uint8_t, 1> count{1};
  for (unsigned n = 0; n < sums.size(); ++n) {
    memory.install(sums[n], sum);
    memory.install(counters[n], count);
  }
}

void consume(app::ApplicationMemory &memory,
             Hcs08 &cpu,
             const unsigned channel,
             const unsigned sample,
             std::ostream &trace) {
  const auto before = memory.retired;
  const auto initial_known = cpu.state().ccr_known;
  memory.adc.advance_bus(memory.adc.state().remaining - 1U);
  const auto unready = memory.adc.state();
  const auto ram = memory.bytes;
  foreground(cpu, poll);
  cpu.step();
  cpu.step();
  require(cpu.state().pc == repeat && memory.retired == before + 2 && memory.bytes == ram &&
              memory.adc.state() == unready && cpu.state().ccr_known == initial_known,
          "Actual original BRSET/JMP skips processing while CCF is clear and cannot advance ADC");
  memory.adc.advance_bus(1);
  require(memory.adc.read(adc::status) == (adc::complete | channel) &&
              memory.adc.state().result == sample,
          "Only the final supplied BUSCLK exposes the actual selected stable analog sample");
  foreground(cpu, poll);
  bool defined_half = false;
  while (cpu.state().pc != repeat) {
    require(memory.retired - before < bound, "Original bounded ADC path must reach repeat_scan");
    const bool defines_half = cpu.state().pc == define_half;
    require(cpu.state().ccr_known == (defined_half ? all_known : initial_known),
            "Native poll and inline dispatch preserve unknown H until the actual ADD operand");
    cpu.step();
    defined_half = defined_half || defines_half;
    require(cpu.state().ccr_known == (defined_half ? all_known : initial_known),
            "Only the completed original ADD defines H; subsequent native code retains it");
  }
  require(defined_half, "The original inline dispatch must execute the first H-defining ADD");
  const auto value = initial_sum + sample;
  require(
      cpu.state() == Hcs08State{.pc = repeat,
                                .hx = static_cast<std::uint16_t>(sums[channel]),
                                .sp = stack,
                                .a = 1,
                                .ccr = flags} &&
          memory.retired - before == instruction_count && memory.bytes[work] == sample >> 8U &&
          memory.bytes[work + 1] == (sample & test::hcs08::byte_mask) &&
          memory.bytes[sums[channel]] == value >> 8U &&
          memory.bytes[sums[channel] + 1] == (value & test::hcs08::byte_mask) &&
          memory.bytes[counters[channel]] == 2,
      "Independent CPU/stack/sum/counter oracle checks native dispatch and byte carry arithmetic");
  const auto next = (channel + 1) % sums.size();
  require(
      memory.adc.read(adc::status) == next && memory.adc.state().active &&
          memory.adc.state().remaining == single_periods &&
          memory.adc.state().conversions == channel + 1 && !memory.adc.interrupt_requested(),
      "Original handler starts the genuine next channel, clears CCF and keeps its IRQ disabled");
  trace << "SAMPLE channel=" << channel << " next=" << next << " code=" << sample
        << " sum=" << value << " counter=2 instructions=" << instruction_count
        << " conversions=" << memory.adc.state().conversions << " remaining=" << single_periods
        << " pc=" << cpu.state().pc << " sp=" << cpu.state().sp << " hx=" << cpu.state().hx
        << " a=1 ccr=" << static_cast<unsigned>(cpu.state().ccr)
        << " initial-known=" << static_cast<unsigned>(initial_known)
        << " ccr-known=" << static_cast<unsigned>(cpu.state().ccr_known)
        << " h-defined=" << defined_half << '\n';
}

void replays(const app::ApplicationMemory &memory,
             const Hcs08 &cpu,
             const unsigned sample,
             std::ostream &trace) {
  constexpr std::array<unsigned, 3> cuts{0, 1, single_periods - 1};
  auto expected_memory = memory;
  Hcs08 expected_cpu(expected_memory, BLOFELD_HCS08_SLA);
  expected_cpu.restore_checkpoint(cpu.checkpoint());
  std::ostringstream expected;
  consume(expected_memory, expected_cpu, 0, sample, expected);
  for (const auto cut : cuts) {
    auto pending = memory;
    pending.adc.advance_bus(cut);
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto fresh = pending;
      Hcs08 restored(fresh, BLOFELD_HCS08_SLA);
      restored.restore_checkpoint(cpu.checkpoint());
      std::ostringstream actual;
      consume(fresh, restored, 0, sample, actual);
      app::same_endpoint(fresh, expected_memory);
      require(
          actual.str() == expected.str() && restored.state() == expected_cpu.state(),
          "Fresh joint CPU/RAM/ADC owners preserve partial conversion and actual native suffix");
      trace << "REPLAY cut=" << cut << " index=" << replay << " identical=1\n";
    }
  }
}

void run(const std::span<const std::uint8_t> records,
         const unsigned sample,
         const unsigned initial_flags,
         const unsigned initial_known,
         std::ostream &trace) {
  app::ApplicationMemory memory(records, {.seed = 0, .trim = trim, .observation = -1});
  memory.warm_reset();
  original_pages(memory, records);
  const auto image = memory.flash.image();
  memory.adc.observe_reference(reference);
  for (unsigned n = 0; n < sums.size(); ++n) {
    memory.adc.observe_input(n, sample * microvolts_per_step);
  }
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  prefix(memory, cpu, initial_flags, initial_known);
  trace << "CASE code=" << sample << " flags=" << initial_flags << " known=" << initial_known
        << " prefix=3 remaining=" << memory.adc.state().remaining << '\n';
  initialize_ram(memory);
  replays(memory, cpu, sample, trace);
  for (unsigned channel = 0; channel < sums.size(); ++channel) {
    consume(memory, cpu, channel, sample, trace);
  }
  require(memory.flash.image() == image && !memory.options.written() && !cpu.faulted() &&
              !memory.adc.faulted() && memory.spi.state() == Hcs08SpiState{} &&
              memory.retired == total_retired,
          "Three original handlers preserve all FLASH and unrelated SPI/SOPT without a host event");
  trace << "END retired=" << memory.retired << " conversions=3 next=0 flash-unchanged=1\n";
}

void unknown_input(const std::span<const std::uint8_t> records, std::ostream &trace) {
  app::ApplicationMemory memory(records, {.seed = 0, .trim = trim, .observation = -1});
  memory.warm_reset();
  memory.adc.observe_reference(reference);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  prefix(memory, cpu, flags, all_known);
  memory.adc.context({.pc = cpu.state().pc, .ccr = cpu.state().ccr});
  const auto before = memory.adc.state();
  const auto error = rejected([&] { memory.adc.advance_bus(1); });
  require(error.contains("sample address=0x0051 pc=0xe1bc ccr=0x006a") &&
              error.contains("unknown enabled analog pin=0") && memory.adc.state() == before &&
              memory.adc.faulted() && memory.retired == 3 && memory.known.none(),
          "Original startup with an unknown real input cannot create an observed ADC result");
  trace << "UNKNOWN " << error << '\n';
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned page records and private original ADC trace");
    const auto records = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    constexpr std::array<unsigned, 5> samples{0, 1, 511, 512, 1023};
    for (const auto sample : samples) {
      for (const auto initial : {flags, flags | half_carry}) {
        run(records, sample, initial, all_known, trace);
      }
      run(records, sample, flags, unknown_half, trace);
    }
    unknown_input(records, trace);
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original ADC setup, polling, dispatch and three native accumulators pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
