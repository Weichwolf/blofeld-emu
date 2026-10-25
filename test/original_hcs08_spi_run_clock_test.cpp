// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
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
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
namespace app = blofeld::test::hcs08_clock_startup;
namespace hp = blofeld::test::hcs08_programmer;
namespace wire = blofeld::test::hcs08_spi_test;
namespace adc = hcs08_adc;
namespace shared = blofeld::test::hcs08_serial_clock;
using shared::Event;
using shared::Snapshot;
using shared::startup_ram;
constexpr unsigned poll = 0xe33a, stack = 0x47f, cursor = 0x97, partial = 0xef;
constexpr unsigned write_period = 45, conversion_periods = 176, half_period = 8;
constexpr unsigned retired = 29, reads = 78, ram_accesses = 34, second_irq_end = 70,
                   irq_periods = 22;
constexpr unsigned stop_period = 123, partial_edges = 9, partial_samples = 4, partial_tx = 9,
                   partial_rx = 0x10;
constexpr unsigned jump_stop = 134, scan_shift = 0xe210, expiry_index = 0xa8;
constexpr unsigned jump_edges = 11, jump_samples = 5, jump_tx = 18, jump_rx = 0x10;
constexpr unsigned scanner_stop = 164, scanner_word_store = 0xe224, firmware_stack = app::stack - 8;
constexpr unsigned scanner_program_reads = 30;
constexpr unsigned scanner_edges = 14, scanner_samples = 7, scanner_tx = 0x48, scanner_rx = 0x12;
constexpr unsigned contact_high = 0x240, contact_low = 0x241, frame_high = firmware_stack + 1,
                   frame_offset = firmware_stack + 2;
constexpr unsigned current_instructions = 15, second_status = 73, second_data = 104,
                   poll_start = 119;
constexpr std::array<unsigned, 37> cpu_cuts{
    0,  11, 18, 26, 29, 37, 42, 44,  45,  46,  47,  49,  50,  55,  56,  59,  60,  65, 66,
    67, 70, 71, 77, 85, 88, 96, 102, 103, 104, 105, 109, 114, 115, 118, 119, 120, 123};

struct Input {
  Hcs08SpiRunContract order;
  Hcs08SpiLaunchContract launch;
  unsigned seed;
  bool jump = false;
  bool scan = false;
};

unsigned delay(const Input input) {
  return input.launch == Hcs08SpiLaunchContract::reference_one_period ? 1U : 2U;
}

unsigned end_period(const Input input) {
  if (input.scan) { return scanner_stop; }
  return input.jump ? jump_stop : stop_period;
}

std::set<unsigned> cuts(const Input input) {
  std::set<unsigned> result(cpu_cuts.begin(), cpu_cuts.end());
  const unsigned edges = input.scan ? scanner_edges : partial_edges;
  for (unsigned edge = 1; edge <= edges; ++edge) {
    const unsigned period = write_period + delay(input) + (edge * half_period) - 1;
    result.insert(period - 1);
    result.insert(period);
  }
  result.insert(end_period(input));
  if (input.jump || input.scan) {
    for (unsigned n = stop_period + 1; n < end_period(input); ++n) { result.insert(n); }
  }
  return result;
}

shared::Setup reference_setup(const Input input) {
  auto program = Hcs08PrefetchContract::reference_peripheral_program_cycles;
  if (input.jump) { program = Hcs08PrefetchContract::reference_jump_program_cycles; }
  if (input.scan) { program = Hcs08PrefetchContract::reference_scanner_byte_cycles; }
  return {.order = input.order,
          .launch = input.launch,
          .seed = input.seed,
          .program = program,
          .scanner = input.scan,
          .pads = std::nullopt};
}

struct Machine : shared::Machine {
  Machine(const std::span<const std::uint8_t> pages,
          const Input input,
          const app::ApplicationMemory *startup = nullptr)
      : shared::Machine(pages, reference_setup(input), startup) {}
};

void endpoint(const Machine &m, const Input input, std::ostream &trace) {
  const auto s = m.clock->state();
  const auto peripheral = m.memory.spi.state();
  const auto serial = m.spi->state();
  const auto adc_state = m.memory.adc.state();
  const auto state = m.cpu.state();
  const auto q = m.cpu.prefetch_state();
  const unsigned end = end_period(input);
  const unsigned low = byte_mask - input.seed;
  if (!q || !q->operand.value || !peripheral.transmit) {
    throw Fault("Missing real foreground data/queue observations");
  }
  require(s.completed_periods == end && s.attempted_periods == end &&
              s.irq_periods == irq_periods && s.program_periods == end - irq_periods &&
              s.spi_periods == end && s.launches == 1 && !s.launch_byte &&
              s.launch_remaining == 0 && serial.generated_edges == partial_edges &&
              serial.completed_transfers == 0 &&
              serial.active_periods == end - write_period - delay(input) + 1 &&
              serial.aborted_transfers == 0 && peripheral.active &&
              peripheral.edges == partial_edges && peripheral.samples == partial_samples &&
              !peripheral.receive_full && peripheral.receive == 0 &&
              peripheral.shift_in == partial_rx && peripheral.shift_out == wire::event_bytes[0] &&
              peripheral.transmit == wire::event_bytes[1] && !peripheral.transmit_armed &&
              !peripheral.receive_armed && m.transmitted == partial_tx &&
              m.samples == partial_samples && m.memory.bytes[cursor] == 2 &&
              m.memory.retired == retired && m.memory.interrupt_entries == 2 &&
              m.memory.interrupt_acknowledgements == 2 && m.memory.flash_reads.size() == reads &&
              m.memory.accesses.size() == ram_accesses && !adc_state.complete && adc_state.active &&
              !adc_state.result && adc_state.remaining == conversion_periods - end &&
              adc_state.conversions == 0,
          "Original ISR writes, real serial prefixes and genuine busy ADC latency match "
          "independent endpoints");
  require(
      state.pc == poll + 3 && state.sp == stack && state.a == low &&
          state.hx == ((input.seed << 8U) | low) && state.ccr == flags_fixed &&
          state.ccr_known == partial && state.a_known == byte_mask && state.hx_known == word_mask &&
          state.pc_known && m.cpu.irq_delay() == Hcs08IrqDelay::none &&
          q->completed_periods == end - second_irq_end &&
          q->retired_instructions == current_instructions && q->operand.address == adc::status &&
          *q->operand.value == 0 && q->instruction_cycle == 0 && !q->operand.in_flight &&
          q->operand.consumed && q->head == poll + 3 && q->next_read == poll + 6 &&
          q->queued_bytes == 3 && q->ready_bytes == 2 && q->program_byte_in_flight,
      "Original busy ADC BRSET selects its actual following JMP without a fabricated foreground "
      "loop");
  const auto &image = m.memory.flash.image().data;
  for (unsigned n = 0; n < q->queued_bytes; ++n) {
    require(q->queued[n] == image[poll + 3 + n - app::hf::first],
            "Actual original JMP bytes remain queued");
  }
  require(m.events ==
              std::vector<Event>{{.period = 14,
                                  .access = {.address = hcs08_spi::status,
                                             .value = hcs08_spi::transmit_empty,
                                             .write = false}},
                                 {.period = write_period,
                                  .access = {.address = hcs08_spi::data,
                                             .value = wire::event_bytes[0],
                                             .write = true}},
                                 {.period = second_status,
                                  .access = {.address = hcs08_spi::status,
                                             .value = hcs08_spi::transmit_empty,
                                             .write = false}},
                                 {.period = second_data,
                                  .access = {.address = hcs08_spi::data,
                                             .value = wire::event_bytes[1],
                                             .write = true}},
                                 {.period = poll_start,
                                  .access = {.address = adc::status, .value = 0, .write = false}}},
          "Only real native SPI writes and the real foreground ADC query occur at their reference "
          "periods");
  trace << "CLOCK seed=" << input.seed << " order=" << static_cast<unsigned>(input.order)
        << " delay=" << delay(input) << " periods=" << end << " pc=" << state.pc
        << " sp=" << state.sp << " a=" << static_cast<unsigned>(state.a) << " hx=" << state.hx
        << " ccr=" << static_cast<unsigned>(state.ccr)
        << " known=" << static_cast<unsigned>(state.ccr_known) << " retired=" << m.memory.retired
        << " irq=" << m.memory.interrupt_entries << " ack=" << m.memory.interrupt_acknowledgements
        << " irq_periods=" << s.irq_periods << " program_periods=" << s.program_periods
        << " spi_periods=" << s.spi_periods << " launches=" << s.launches
        << " edges=" << serial.generated_edges << " active_periods=" << serial.active_periods
        << " remaining=" << serial.remaining_periods << " tx=" << m.transmitted
        << " samples=" << m.samples << " shift_in=" << static_cast<unsigned>(peripheral.shift_in)
        << " rx_full=" << peripheral.receive_full
        << " queued=" << static_cast<unsigned>(*peripheral.transmit)
        << " adc_remaining=" << adc_state.remaining << " reads=" << m.memory.flash_reads.size()
        << " data_accesses=" << m.memory.accesses.size() << " events=" << m.events.size()
        << " ready=" << q->ready_bytes << " inflight=" << q->program_byte_in_flight
        << " next=" << q->next_read << " head=" << q->head << '\n';
}

void jump_endpoint(const Machine &m, const Input input, std::ostream &trace) {
  const auto state = m.cpu.state();
  const auto s = m.clock->state();
  const auto serial = m.spi->state();
  const auto peripheral = m.memory.spi.state();
  const auto q = m.cpu.prefetch_state();
  if (!q || !peripheral.transmit) {
    throw Fault("Missing actual JMP/scanner queue or serial byte");
  }
  const unsigned flags = flags_fixed | (input.seed == 0 ? flag_z : 0U);
  require(
      s.completed_periods == jump_stop && s.attempted_periods == jump_stop &&
          s.spi_periods == jump_stop && s.irq_periods == irq_periods &&
          s.program_periods == jump_stop - irq_periods && s.launches == 1 &&
          m.memory.retired == retired + 3 && m.memory.interrupt_entries == 2 &&
          m.memory.interrupt_acknowledgements == 2 && state.pc == scan_shift && state.sp == stack &&
          state.a == byte_mask - input.seed && state.hx == ((input.seed << 8U) | input.seed) &&
          state.ccr == flags && state.ccr_known == partial &&
          q->contract == Hcs08PrefetchContract::reference_jump_program_cycles &&
          q->head == scan_shift && q->next_read == scan_shift + 3 && q->queued_bytes == 3 &&
          q->ready_bytes == 2 && q->program_byte_in_flight &&
          q->retired_instructions == current_instructions + 3 &&
          q->completed_periods == jump_stop - second_irq_end &&
          q->operand.address == expiry_index && q->operand.value == input.seed &&
          q->operand.consumed && !q->operand.in_flight && !q->jump_discarded &&
          m.memory.bytes[expiry_index] == input.seed && m.memory.flash_reads.size() == reads + 10 &&
          m.memory.accesses.size() == ram_accesses + 1 && m.events.size() == 5,
      "Two native original JMPs and actual scanner LDX reach ASLX without host PC/loop injection");
  require(serial.generated_edges == jump_edges && serial.completed_transfers == 0 &&
              serial.active_periods == jump_stop - write_period - delay(input) + 1 &&
              serial.remaining_periods == 6 + delay(input) && peripheral.active &&
              peripheral.edges == jump_edges && peripheral.samples == jump_samples &&
              peripheral.shift_in == jump_rx && peripheral.transmit == wire::event_bytes[1] &&
              !peripheral.receive_full && m.transmitted == jump_tx && m.samples == jump_samples &&
              m.memory.adc.state().remaining == conversion_periods - jump_stop &&
              m.memory.adc.state().active && m.memory.adc.state().conversions == 0,
          "Original scanner continuation retains genuine serial/ADC progress exactly once");
  trace << "JUMP seed=" << input.seed << " order=" << static_cast<unsigned>(input.order)
        << " delay=" << delay(input) << " periods=" << jump_stop << " pc=" << state.pc
        << " sp=" << state.sp << " a=" << static_cast<unsigned>(state.a) << " hx=" << state.hx
        << " ccr=" << static_cast<unsigned>(state.ccr)
        << " known=" << static_cast<unsigned>(state.ccr_known) << " retired=" << m.memory.retired
        << " irq=" << m.memory.interrupt_entries << " spi_periods=" << s.spi_periods
        << " edges=" << serial.generated_edges << " active_periods=" << serial.active_periods
        << " remaining=" << serial.remaining_periods << " tx=" << m.transmitted
        << " samples=" << m.samples << " shift_in=" << static_cast<unsigned>(peripheral.shift_in)
        << " queued=" << static_cast<unsigned>(*peripheral.transmit)
        << " adc_remaining=" << m.memory.adc.state().remaining
        << " reads=" << m.memory.flash_reads.size() << " data_accesses=" << m.memory.accesses.size()
        << " events=" << m.events.size() << " ready=" << q->ready_bytes
        << " inflight=" << q->program_byte_in_flight << " next=" << q->next_read
        << " head=" << q->head << '\n';
}

void scanner_accesses(const Machine &m, const Input input) {
  const std::array<Event, 8> expected{{
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
  }};
  require(m.ram_events.size() >= expected.size() &&
              std::ranges::equal(std::span(m.ram_events).last(expected.size()), expected),
          "Actual native scanner RAM reads/stores/pulls occur once at independent addresses and "
          "periods");
}

void scanner_endpoint(const Machine &m, const Input input, std::ostream &trace) {
  scanner_accesses(m, input);
  const auto state = m.cpu.state();
  const auto s = m.clock->state();
  const auto q = m.cpu.prefetch_state();
  const auto serial = m.spi->state();
  const auto peripheral = m.memory.spi.state();
  if (!q || !peripheral.transmit) { throw Fault("Missing genuine scanner/serial state"); }
  require(s.completed_periods == scanner_stop && s.attempted_periods == scanner_stop &&
              s.spi_periods == scanner_stop && s.irq_periods == irq_periods &&
              s.program_periods == scanner_stop - irq_periods && s.launches == 1 &&
              state.pc == scanner_word_store && state.sp == firmware_stack && state.a == 0 &&
              state.hx == 0 && state.ccr == (flags_fixed | (input.seed == 0 ? flag_z : 0U)) &&
              state.ccr_known == partial && m.memory.retired == retired + 14 &&
              m.memory.flash_reads.size() == reads + scanner_program_reads &&
              m.memory.accesses.size() == ram_accesses + 9 && m.events.size() == 5 &&
              m.memory.interrupt_entries == 2 && m.memory.interrupt_acknowledgements == 2 &&
              q->contract == Hcs08PrefetchContract::reference_scanner_byte_cycles &&
              q->head == scanner_word_store && q->next_read == scanner_word_store + 3 &&
              q->queued_bytes == 3 && q->ready_bytes == 2 && q->program_byte_in_flight &&
              q->completed_periods == scanner_stop - second_irq_end &&
              q->retired_instructions == current_instructions + 14 &&
              q->operand.address == firmware_stack && q->operand.value == 0 &&
              q->operand.consumed && !q->operand.in_flight,
          "Native ASLX/SP1 stores and real stack pulls retain the original main frame and queue");
  require(m.memory.bytes[contact_high + (2 * input.seed)] == 0 &&
              m.memory.bytes[contact_low + (2 * input.seed)] == 0 &&
              m.memory.bytes[frame_high] == 0 && m.memory.bytes[frame_offset] == 2 * input.seed &&
              m.memory.known[frame_high] && m.memory.known[frame_offset] &&
              serial.generated_edges == scanner_edges && serial.completed_transfers == 0 &&
              serial.active_periods == scanner_stop - write_period - delay(input) + 1 &&
              serial.remaining_periods == delay(input) && peripheral.active &&
              peripheral.samples == scanner_samples && peripheral.shift_in == scanner_rx &&
              peripheral.transmit == wire::event_bytes[1] && !peripheral.receive_full &&
              m.transmitted == scanner_tx && m.samples == scanner_samples &&
              m.memory.adc.state().active &&
              m.memory.adc.state().remaining == conversion_periods - scanner_stop,
          "Startup-derived contact RAM, native frame stores and concurrent serial/ADC prefixes "
          "remain genuine");
  trace << "SCAN seed=" << input.seed << " order=" << static_cast<unsigned>(input.order)
        << " delay=" << delay(input) << " periods=" << scanner_stop << " pc=" << state.pc
        << " sp=" << state.sp << " a=" << static_cast<unsigned>(state.a) << " hx=" << state.hx
        << " ccr=" << static_cast<unsigned>(state.ccr)
        << " known=" << static_cast<unsigned>(state.ccr_known) << " retired=" << m.memory.retired
        << " edges=" << serial.generated_edges << " active_periods=" << serial.active_periods
        << " remaining=" << serial.remaining_periods << " tx=" << m.transmitted
        << " samples=" << m.samples << " shift_in=" << static_cast<unsigned>(peripheral.shift_in)
        << " queued=" << static_cast<unsigned>(*peripheral.transmit)
        << " adc_remaining=" << m.memory.adc.state().remaining
        << " reads=" << m.memory.flash_reads.size() << " data_accesses=" << m.memory.accesses.size()
        << " events=" << m.events.size()
        << " frame_high=" << static_cast<unsigned>(m.memory.bytes[frame_high])
        << " frame_offset=" << static_cast<unsigned>(m.memory.bytes[frame_offset])
        << " ready=" << q->ready_bytes << " inflight=" << q->program_byte_in_flight
        << " next=" << q->next_read << " head=" << q->head << '\n';
}

void frontier(Machine &m, const Input input, std::ostream &trace) {
  const auto accesses = m.memory.accesses;
  const auto reads_before = m.memory.flash_reads;
  const auto state = m.cpu.state();
  const auto error = rejected([&] { m.tick(); });
  const bool prior_spi = input.order == Hcs08SpiRunContract::reference_spi_before_cpu;
  const auto s = m.clock->state();
  const unsigned stop = end_period(input);
  unsigned instructions = retired + (input.jump ? 3U : 0U);
  if (input.scan) { instructions = retired + 14; }
  bool diagnostic = input.jump ? error.contains("opcode=0x0058") && error.contains("pc=0xe210")
                               : error.contains("opcode=0x00cc") && error.contains("pc=0xe33d");
  if (input.scan) { diagnostic = error.contains("opcode=0x0035") && error.contains("pc=0xe224"); }
  unsigned expected_edges = input.jump ? jump_edges : partial_edges;
  if (input.scan) {
    expected_edges = scanner_edges + static_cast<unsigned>(prior_spi && delay(input) == 1);
  }
  require(diagnostic && m.clock->faulted() && m.cpu.faulted() && m.cpu.state() == state &&
              m.memory.accesses == accesses && m.memory.flash_reads == reads_before &&
              m.memory.retired == instructions && s.attempted_periods == stop + 1 &&
              s.completed_periods == stop && s.irq_periods == irq_periods &&
              s.program_periods == stop - irq_periods &&
              s.spi_periods == stop + static_cast<unsigned>(prior_spi) &&
              m.memory.spi.state().edges == expected_edges &&
              m.memory.adc.state().remaining == conversion_periods - stop - 1,
          "Unqualified original foreground cycles fail before execution while retaining genuine "
          "ADC/SPI "
          "prefixes");
  const auto q = m.cpu.prefetch_state();
  require(q && q->ready_bytes == 3 && !q->program_byte_in_flight,
          "The previous actual program-byte arrival survives the refused instruction");
  if (input.scan) {
    const bool edge = prior_spi && delay(input) == 1;
    require(s.last_pins.has_value() == edge,
            "A later CPU word-store refusal retains the actual prior serial edge pins");
  }
  trace << "FRONTIER order=" << static_cast<unsigned>(input.order) << " delay=" << delay(input)
        << " pc=" << state.pc << " attempted=" << s.attempted_periods
        << " completed=" << s.completed_periods << " spi_periods=" << s.spi_periods
        << " adc_remaining=" << m.memory.adc.state().remaining;
  if (input.scan) {
    trace << " edges=" << m.memory.spi.state().edges << " pins=" << s.last_pins.has_value();
  }
  trace << '\n';
}

Snapshot capture(const std::span<const std::uint8_t> pages,
                 const Input input,
                 const unsigned cut,
                 const app::ApplicationMemory *startup) {
  Machine original(pages, input, startup);
  for (unsigned n = 0; n < cut; ++n) { original.tick(); }
  return shared::capture(original);
}

void checked_endpoint(const Machine &m, const Input input, std::ostream &trace) {
  if (input.scan) {
    scanner_endpoint(m, input, trace);
  } else if (input.jump) {
    jump_endpoint(m, input, trace);
  } else {
    endpoint(m, input, trace);
  }
}

void run(const std::span<const std::uint8_t> pages,
         std::ostream &trace,
         const bool jump,
         const bool scan) {
  std::optional<app::ApplicationMemory> startup;
  if (scan) { startup = startup_ram(pages, trace); }
  const auto *initial = startup ? &*startup : nullptr;
  unsigned periods = 0;
  unsigned replays = 0;
  unsigned cut_count = 0;
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    for (const auto launch : {Hcs08SpiLaunchContract::reference_one_period,
                              Hcs08SpiLaunchContract::reference_two_periods}) {
      for (unsigned seed = 0; seed < 8; ++seed) {
        const Input input{
            .order = order, .launch = launch, .seed = seed, .jump = jump, .scan = scan};
        Machine m(pages, input, initial);
        for (unsigned n = 0; n < end_period(input); ++n) {
          m.tick();
          ++periods;
        }
        checked_endpoint(m, input, trace);
        frontier(m, input, trace);
      }
      const Input input{.order = order, .launch = launch, .seed = 7, .jump = jump, .scan = scan};
      for (const auto cut : cuts(input)) {
        const auto saved = capture(pages, input, cut, initial);
        periods += cut;
        ++cut_count;
        for (unsigned replay = 0; replay < 2; ++replay) {
          Machine fresh(pages, input, initial);
          restore(fresh, saved);
          for (unsigned n = cut; n < end_period(input); ++n) {
            fresh.tick();
            ++periods;
          }
          trace << "REPLAY order=" << static_cast<unsigned>(order) << " delay=" << delay(input)
                << " cut=" << cut << " index=" << replay << '\n';
          checked_endpoint(fresh, input, trace);
          frontier(fresh, input, trace);
          ++replays;
        }
      }
    }
  }
  trace << "SUMMARY periods=" << periods << " contexts=32 cuts=" << cut_count
        << " replays=" << replays << '\n';
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3 || (count == 4 && (std::string_view(arguments[3]) == "--jump" ||
                                          std::string_view(arguments[3]) == "--scan")),
            "Expected pinned pages, private trace and optional --jump/--scan reference");
    const auto pages = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    run(pages,
        trace,
        count == 4 && std::string_view(arguments[3]) == "--jump",
        count == 4 && std::string_view(arguments[3]) == "--scan");
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original CPU/SPI/ADC periods preserve genuine serial prefixes and the strict "
                 "foreground frontier.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
