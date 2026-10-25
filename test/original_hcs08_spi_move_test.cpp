// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_serial_clock_fixture.hpp"
#include "hcs08_spi_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
namespace app = blofeld::test::hcs08_clock_startup;
namespace hp = blofeld::test::hcs08_programmer;
namespace wire = blofeld::test::hcs08_spi_test;
namespace shared = blofeld::test::hcs08_serial_clock;
using shared::Event;
using shared::Snapshot;
using shared::startup_ram;
constexpr unsigned partial = 0xef;
constexpr unsigned write_period = 45, conversion_periods = 176, half_period = 8;
constexpr unsigned irq_periods = 22, second_irq_end = 70;
constexpr unsigned stop_period = 123, word_stop = 171, move_stop = 176;
constexpr unsigned word_pc = 0xe252, move_next = 0xe255, firmware_stack = app::stack - 8;
constexpr unsigned move_read_period = 172, move_write_period = 174;
constexpr unsigned word_retired = 45, word_reads = 113, word_accesses = 45, word_events = 5;
constexpr unsigned word_native_retired = 31, partial_edges = 15, partial_samples = 7;
constexpr unsigned partial_transmitted = 0x48, source_sign = 0x80, expected_conversion = 512;
constexpr unsigned word_destination = 0xa1, sample_destination = 0x9c;
constexpr std::array<unsigned, 4> pad_observations{0, 0x55, 0x80, 0xff};
constexpr std::array<unsigned, 37> cpu_cuts{
    0,  11, 18, 26, 29, 37, 42, 44,  45,  46,  47,  49,  50,  55,  56,  59,  60,  65, 66,
    67, 70, 71, 77, 85, 88, 96, 102, 103, 104, 105, 109, 114, 115, 118, 119, 120, 123};

struct Input {
  Hcs08SpiRunContract order;
  Hcs08SpiLaunchContract launch;
  unsigned seed;
  std::optional<unsigned> pads;
};

unsigned delay(const Input input) {
  return input.launch == Hcs08SpiLaunchContract::reference_one_period ? 1U : 2U;
}

unsigned end_period(const Input input) {
  return input.pads ? move_stop : word_stop;
}

std::set<unsigned> cuts(const Input input) {
  std::set<unsigned> result(cpu_cuts.begin(), cpu_cuts.end());
  const unsigned edges = input.pads ? 16U : 15U;
  for (unsigned edge = 1; edge <= edges; ++edge) {
    const unsigned period = write_period + delay(input) + (edge * half_period) - 1;
    result.insert(period - 1);
    result.insert(period);
  }
  for (unsigned n = stop_period + 1; n <= end_period(input); ++n) { result.insert(n); }
  return result;
}

struct Machine : shared::Machine {
  Machine(const Input input, const app::ApplicationMemory &startup)
      : shared::Machine({},
                        {.order = input.order,
                         .launch = input.launch,
                         .seed = input.seed,
                         .program = Hcs08PrefetchContract::reference_scanner_move_cycles,
                         .scanner = true,
                         .reconnect_gpio = true,
                         .pads = input.pads},
                        &startup) {}
};

void scanner_accesses(const Machine &m, const Input input) {
  constexpr unsigned contact_high = 0x240;
  constexpr unsigned contact_low = 0x241;
  constexpr unsigned frame_high = firmware_stack + 1;
  constexpr unsigned frame_offset = firmware_stack + 2;
  const std::array<Event, 10> prefix{{
      {.period = 138,
       .access = {.address = contact_low + (2 * input.seed), .value = 0, .write = false}},
      {.period = 141, .access = {.address = firmware_stack, .value = 0, .write = true}},
      {.period = 144,
       .access = {.address = contact_high + (2 * input.seed), .value = 0, .write = false}},
      {.period = 148, .access = {.address = frame_high, .value = 0, .write = true}},
      {.period = 153, .access = {.address = firmware_stack - 1, .value = 0, .write = true}},
      {.period = 156, .access = {.address = frame_offset, .value = 2 * input.seed, .write = true}},
      {.period = 159, .access = {.address = firmware_stack - 1, .value = 0, .write = false}},
      {.period = 162, .access = {.address = firmware_stack, .value = 0, .write = false}},
      {.period = 165, .access = {.address = word_destination, .value = 0, .write = true}},
      {.period = 166, .access = {.address = word_destination + 1, .value = 0, .write = true}},
  }};
  const auto accesses = std::span(m.ram_events);
  const unsigned suffix = input.pads ? 1U : 0U;
  require(
      accesses.size() >= prefix.size() + suffix &&
          std::ranges::equal(
              accesses.subspan(accesses.size() - prefix.size() - suffix, prefix.size()), prefix),
      "Independent original scanner RAM/word writes retain actual periods and addresses");
  if (input.pads) {
    require(
        accesses.back() ==
            Event{.period = move_write_period,
                  .access = {.address = sample_destination, .value = *input.pads, .write = true}},
        "Original MOV writes its genuinely sampled PTA value on w174");
  }
}

void endpoint(const Machine &m, const Input input, std::ostream &trace) {
  scanner_accesses(m, input);
  const auto state = m.cpu.state();
  const auto s = m.clock->state();
  const auto q = m.cpu.prefetch_state();
  const auto serial = m.spi->state();
  const auto peripheral = m.memory.spi.state();
  const auto atd = m.memory.adc.state();
  const unsigned end = end_period(input);
  const unsigned extra = input.pads ? 1U : 0U;
  const unsigned flags =
      input.pads
          ? ((*input.pads == 0 ? flag_z : 0U) | ((*input.pads & source_sign) != 0 ? flag_n : 0U))
          : flag_z;
  const unsigned pc = input.pads ? move_next : word_pc;
  require(
      s.completed_periods == end && s.attempted_periods == end && s.spi_periods == end &&
          s.irq_periods == irq_periods && s.program_periods == end - irq_periods &&
          s.launches == 1 + extra && state.pc == pc && state.sp == firmware_stack && state.a == 0 &&
          state.hx == 0 && state.ccr == (flags_fixed | flags) && state.ccr_known == partial &&
          m.memory.retired == word_retired + extra &&
          m.memory.flash_reads.size() == word_reads + 3 * extra &&
          m.memory.accesses.size() == word_accesses + extra &&
          m.events.size() == word_events + extra && m.memory.interrupt_entries == 2 &&
          m.memory.interrupt_acknowledgements == 2 && q &&
          q->contract == Hcs08PrefetchContract::reference_scanner_move_cycles && q->head == pc &&
          q->next_read == pc + 3 && q->queued_bytes == 3 && q->ready_bytes == 2 &&
          q->program_byte_in_flight && q->completed_periods == end - second_irq_end &&
          q->retired_instructions == word_native_retired + extra && q->operand == Hcs08DataRead{},
      "Original native MOV/BEQ PC, flags, bus counts and completed queue match ISA expectations");
  if (input.pads) {
    require(
        m.memory.bytes[sample_destination] == *input.pads &&
            serial.generated_edges == wire::edges && serial.completed_transfers == 1 &&
            serial.active_periods ==
                wire::edges * half_period + move_stop - word_stop - 2 * delay(input) &&
            serial.active &&
            serial.remaining_periods == half_period - (move_stop - word_stop - 2 * delay(input)) &&
            peripheral.active && peripheral.samples == 0 && peripheral.edges == 0 &&
            peripheral.shift_in == 0 && peripheral.shift_out == wire::event_bytes[1] &&
            !peripheral.transmit && peripheral.receive_full &&
            peripheral.receive == wire::reply_bytes[0] && m.transmitted == wire::event_bytes[0] &&
            m.samples == wire::samples && !atd.active && atd.complete &&
            atd.result == expected_conversion && atd.conversions == 1 && atd.remaining == 0 &&
            !m.memory.adc.interrupt_requested(),
        "Real serial byte completion, queued-byte promotion and ADC completion remain autonomous");
    require(m.events.back() ==
                Event{.period = move_read_period,
                      .access = {.address = 0, .value = *input.pads, .write = false}},
            "Original MOV performs one genuine PTA read on r172");
  } else {
    require(serial.generated_edges == partial_edges && serial.completed_transfers == 0 &&
                serial.active_periods == word_stop - write_period - delay(input) + 1 &&
                serial.remaining_periods == delay(input) + 1 &&
                peripheral.samples == partial_samples &&
                peripheral.shift_in == wire::reply_bytes[0] &&
                peripheral.transmit == wire::event_bytes[1] && !peripheral.receive_full &&
                m.transmitted == partial_transmitted && m.samples == partial_samples &&
                atd.active && !atd.complete && atd.remaining == conversion_periods - word_stop &&
                atd.conversions == 0,
            "Missing pads retain the real pre-MOV word, partial serial byte and busy ADC");
  }
  trace << "END seed=" << input.seed << " order=" << static_cast<unsigned>(input.order)
        << " delay=" << delay(input)
        << " pads=" << (input.pads ? static_cast<int>(*input.pads) : -1) << " periods=" << end
        << " pc=" << state.pc << " sp=" << state.sp << " ccr=" << static_cast<unsigned>(state.ccr)
        << " retired=" << m.memory.retired << " reads=" << m.memory.flash_reads.size()
        << " data=" << m.memory.accesses.size() << " events=" << m.events.size()
        << " edges=" << serial.generated_edges << " active=" << serial.active_periods
        << " remaining=" << serial.remaining_periods << " transfers=" << serial.completed_transfers
        << " launches=" << s.launches << " tx=" << m.transmitted << " samples=" << m.samples
        << " adc_remaining=" << atd.remaining << " adc_conversions=" << atd.conversions << '\n';
}

void missing_pads(Machine &m, const Input input, std::ostream &trace) {
  const auto state = m.cpu.state();
  const auto accesses = m.memory.accesses;
  const auto reads = m.memory.flash_reads;
  const auto events = m.memory.peripheral_accesses;
  const auto error = rejected([&] { m.tick(); });
  const bool prior_spi = input.order == Hcs08SpiRunContract::reference_spi_before_cpu;
  const auto s = m.clock->state();
  const auto q = m.cpu.prefetch_state();
  require(
      error.contains("GPIO read") && error.contains("address=0x0000") &&
          error.contains("pc=0xe252") && error.contains("ccr=") &&
          error.contains("Detached resolved GPIO pad source") && m.clock->faulted() &&
          m.cpu.faulted() && m.cpu.state() == state && state.pc == word_pc &&
          m.memory.accesses == accesses && m.memory.flash_reads == reads &&
          m.memory.peripheral_accesses == events && m.memory.retired == word_retired &&
          s.attempted_periods == move_read_period && s.completed_periods == word_stop &&
          s.irq_periods == irq_periods && s.program_periods == word_stop - irq_periods &&
          s.spi_periods == word_stop + static_cast<unsigned>(prior_spi) &&
          m.memory.spi.state().edges == partial_edges && m.memory.adc.state().remaining == 4 &&
          !s.last_pins && q && q->instruction_bytes == 3 && q->instruction_cycle == 0 &&
          q->queued_bytes == 0 && q->ready_bytes == 0 && !q->program_byte_in_flight &&
          !q->native_executed,
      "Unknown PTA read must fail before a write/retirement, retaining real CPU/SPI/ADC prefix: " +
          error);
  static_cast<void>(rejected([&] { static_cast<void>(m.cpu.checkpoint()); }));
  static_cast<void>(rejected([&] { m.tick(); }));
  trace << "MISSING order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " attempted=" << s.attempted_periods << " completed=" << s.completed_periods
        << " spi_periods=" << s.spi_periods << " adc_remaining=4 edges=15\n";
}

Snapshot capture(const Input input, const unsigned cut, const app::ApplicationMemory &startup) {
  Machine original(input, startup);
  for (unsigned n = 0; n < cut; ++n) { original.tick(); }
  return shared::capture(original);
}

void checked(Machine &m,
             const Input input,
             const app::ApplicationMemory &startup,
             std::ostream &trace) {
  endpoint(m, input, trace);
  require(m.memory.flash.image() == startup.flash.image(),
          "All original executable pages and NVM history remain unchanged");
  if (!input.pads) { missing_pads(m, input, trace); }
}

struct MatrixCounts {
  unsigned periods = 0;
  unsigned replays = 0;
  unsigned cuts = 0;
  unsigned contexts = 0;
};

void replay_case(const Input input,
                 const app::ApplicationMemory &startup,
                 std::ostream &trace,
                 MatrixCounts &counts) {
  for (const auto cut : cuts(input)) {
    const auto saved = capture(input, cut, startup);
    counts.periods += cut;
    ++counts.cuts;
    for (unsigned replay = 0; replay < 2; ++replay) {
      Machine fresh(input, startup);
      restore(fresh, saved);
      for (unsigned n = cut; n < end_period(input); ++n) {
        fresh.tick();
        ++counts.periods;
      }
      trace << "REPLAY order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
            << " pads=" << (input.pads ? static_cast<int>(*input.pads) : -1) << " cut=" << cut
            << " index=" << replay << '\n';
      checked(fresh, input, startup, trace);
      ++counts.replays;
    }
  }
}

void run(const std::span<const std::uint8_t> pages, std::ostream &trace) {
  const auto startup = startup_ram(pages, trace);
  MatrixCounts counts;
  constexpr std::array<std::optional<unsigned>, 5> sources{std::nullopt,
                                                           pad_observations[0],
                                                           pad_observations[1],
                                                           pad_observations[2],
                                                           pad_observations[3]};
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    for (const auto launch : {Hcs08SpiLaunchContract::reference_one_period,
                              Hcs08SpiLaunchContract::reference_two_periods}) {
      for (const auto pads : sources) {
        for (unsigned seed = 0; seed < 8; ++seed) {
          const Input input{.order = order, .launch = launch, .seed = seed, .pads = pads};
          Machine m(input, startup);
          for (unsigned n = 0; n < end_period(input); ++n) {
            m.tick();
            ++counts.periods;
          }
          checked(m, input, startup, trace);
          ++counts.contexts;
        }
        replay_case(
            {.order = order, .launch = launch, .seed = 7, .pads = pads}, startup, trace, counts);
      }
      const Input input{.order = order, .launch = launch, .seed = 7, .pads = pad_observations[2]};
      const auto saved = capture(input, word_stop, startup);
      counts.periods += word_stop;
      Machine fresh(input, startup);
      restore(fresh, saved, false);
      trace << "DETACHED order=" << static_cast<unsigned>(order) << " delay=" << delay(input)
            << '\n';
      missing_pads(fresh, input, trace);
    }
  }
  trace << "SUMMARY periods=" << counts.periods << " contexts=" << counts.contexts
        << " cuts=" << counts.cuts << " replays=" << counts.replays << " detached=4\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned pages and private MOV-clock trace");
    const auto pages = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    run(pages, trace);
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout
        << "Original native MOV joins real SPI/ADC completion under declared pad observations; "
           "missing and detached pads still fail. Full boot and physical matrix remain "
           "unqualified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
