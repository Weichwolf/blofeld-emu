// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_shi.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_spi_fixture.hpp"
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
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::hcs08::require;
namespace app = test::hcs08_clock_startup;
namespace hp = test::hcs08_programmer;
namespace link = test::hcs08_spi_test;
namespace adc = hcs08_adc;
constexpr unsigned setup = 0xe1b4, poll = 0xe33a, repeat = 0xe7ae, scan = 0xe20e;
constexpr unsigned stack = 0x475, trim = 0x80, bound = 2048, word_mask = 0xffff;
constexpr unsigned minimum = 0x80, center = 0x82, lower_gain = 0x84, upper_gain = 0x85;
constexpr unsigned last = 0x89, maximum = 0x8b, filter = 0x8d, sum = 0xb8, counter = 0xbf;
constexpr unsigned tx_read = 0x97, tx_write = 0x98, rx_write = 0x99, rx_latest = 0x9a;
constexpr unsigned rx_previous = 0x9b, cycle_flags = 0xa0, expiry = 0xa8, calibration = 0xc1;
constexpr unsigned rx_buffer = 0x200, interrupt_vector = 0xffe0;
constexpr unsigned flags = 0x60, partial_flags = 0xef, full_flags = 0xff;
constexpr unsigned seed_center = 2048, seed_maximum = 4095, seed_counter = 16;
constexpr unsigned flag_i = 8, foreground_a = 0xa5, foreground_hx = 0x5aa5;
constexpr unsigned conversion_periods = 176, irq_handler = 0xea29, matrix_count = 68;
constexpr unsigned lower_working = 1000, upper_working = 3000, center_code = 8192,
                   maximum_code = 16383;
constexpr unsigned scaled_maximum = 4092, wrap_lower_working = 325, wrap_upper_working = 2352;
constexpr unsigned wrapping_gain = 252;
constexpr unsigned setup_periods = 13, next_start = 115, next_complete = 291;
constexpr unsigned tx_buffer = 0x100, mul_opcode = 0x42, partial_mul = word_mask;
constexpr Hcs08AdcReference analog_reference{
    .bus_hz = 6000000, .supply_uv = 3300000, .low_uv = 0, .high_uv = 3072000};

struct Input {
  unsigned working, previous, gain, cursor;
  int filter_offset = 0;
};

unsigned byte_cursor(const unsigned value) {
  return value & test::hcs08::byte_mask;
}

struct Seed {
  unsigned address, value;
};

void byte(app::ApplicationMemory &memory, const Seed seed) {
  const std::array<std::uint8_t, 1> data{static_cast<std::uint8_t>(seed.value)};
  memory.install(seed.address, data);
}

void word(app::ApplicationMemory &memory, const Seed seed) {
  const std::array<std::uint8_t, 2> data{static_cast<std::uint8_t>(seed.value >> 8U),
                                         static_cast<std::uint8_t>(seed.value & link::byte_mask)};
  memory.install(seed.address, data);
}

unsigned word(const app::ApplicationMemory &memory, const unsigned address) {
  require(memory.known[address] && memory.known[address + 1],
          "Original word must actually be known");
  return (static_cast<unsigned>(memory.bytes[address]) << 8U) | memory.bytes[address + 1];
}

void seed(app::ApplicationMemory &memory, const Input input) {
  word(memory, {.address = minimum, .value = 0});
  word(memory, {.address = center, .value = seed_center});
  word(memory, {.address = maximum, .value = seed_maximum});
  word(memory, {.address = calibration, .value = 0});
  word(memory, {.address = last, .value = input.previous});
  const int initial_filter = static_cast<int>(input.working * 4) + input.filter_offset;
  const int average = (initial_filter / static_cast<int>(seed_counter)) - input.filter_offset;
  require(initial_filter >= 0 && initial_filter <= static_cast<int>(word_mask) && average >= 0 &&
              average <= static_cast<int>(seed_maximum),
          "Own filter/accumulator inputs fit their actual word widths");
  word(memory, {.address = filter, .value = static_cast<unsigned>(initial_filter)});
  word(memory, {.address = sum, .value = static_cast<unsigned>(average) * seed_counter});
  byte(memory, {.address = lower_gain, .value = input.gain});
  byte(memory, {.address = upper_gain, .value = input.gain});
  byte(memory, {.address = counter, .value = seed_counter});
  byte(memory, {.address = tx_read, .value = input.cursor});
  byte(memory, {.address = tx_write, .value = input.cursor});
  byte(memory, {.address = rx_write, .value = 0});
  byte(memory, {.address = rx_latest, .value = 0});
  byte(memory, {.address = rx_previous, .value = 0});
  byte(memory, {.address = cycle_flags, .value = 3});
  byte(memory, {.address = expiry, .value = 0});
}

class NoBdcMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override { return false; }

  std::uint8_t read(const std::uint16_t address, const ExecutionContext context) override {
    throw Fault("Unexpected pitch component BDC read address=" + hex(address) +
                " pc=" + hex(context.pc));
  }

  void write(const ByteWrite access, const ExecutionContext context) override {
    throw Fault("Unexpected pitch component BDC write address=" + hex(access.address) +
                " pc=" + hex(context.pc));
  }
};

struct Machine {
  app::ApplicationMemory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};
  DspShi peer;
  unsigned foreground_steps = 0;
  NoBdcMemory mapping;
  std::unique_ptr<Hcs08Bdc> bdc;
  std::unique_ptr<Hcs08RunClock> clock;
  unsigned poll_periods = 0;
  bool polling = false;

  Machine(const std::span<const std::uint8_t> records,
          const Input input,
          const bool clocked = false)
      : memory(records, {.seed = 0, .trim = trim, .observation = -1}) {
    memory.warm_reset();
    memory.adc.observe_reference(analog_reference);
    for (unsigned channel = 0; channel < adc::channels; ++channel) {
      memory.adc.observe_input(channel, 0);
    }
    cpu.set_state({.pc = setup,
                   .hx = foreground_hx,
                   .sp = stack,
                   .a = foreground_a,
                   .ccr_known = partial_flags,
                   .ccr = static_cast<std::uint8_t>(flags | (clocked ? flag_i : 0U))});
    if (clocked) {
      bdc = std::make_unique<Hcs08Bdc>(
          cpu, mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
      clock = std::make_unique<Hcs08RunClock>(
          memory, *bdc, Hcs08RunContract::reference_instruction_retirement);
      command(hcs08_bdc::go);
    }
    for (unsigned n = 0; n < 3; ++n) { step(); }
    wait_for_sample();
    require(memory.adc.state().result == 0 && memory.adc.state().complete,
            "Only the real ADC conversion supplies this selected stable zero sample");
    auto state = cpu.state();
    state.pc = poll;
    cpu.set_state(state); // Declared scanner caller, never a handler/frame or output value.
    seed(memory, input);
    link::configure(memory.spi, link::quiet_control);
    link::select(peer);
    if (clock) { command(hcs08_bdc::go); }
    polling = true;
  }

  void command(const std::uint8_t value) const {
    bdc->receive(value);
    bdc->service(Hcs08BdcGrant::reference_instruction_boundary);
  }

  void wait_for_sample() {
    if (!clock) {
      memory.adc.advance_bus(conversion_periods);
      return;
    }
    require(clock->state().executing_periods == setup_periods &&
                memory.adc.state().remaining == conversion_periods,
            "Original ADC setup is exactly4+4+5 periods before any conversion work");
    command(hcs08_bdc::background);
    for (unsigned n = 0; n < conversion_periods; ++n) { tick(); }
    require(clock->state().offered_periods == setup_periods + conversion_periods &&
                clock->state().executing_periods == setup_periods,
            "Held BDC periods advance only the real ADC, without a foreground fetch");
  }

  void adc_phase() const {
    const auto state = memory.adc.state();
    const bool converting = poll_periods >= next_start && poll_periods < next_complete;
    const unsigned remaining = converting ? next_complete - poll_periods : 0;
    require(state.active == converting && state.remaining == remaining &&
                state.conversions == (poll_periods < next_complete ? 1U : 2U) &&
                state.result == 0 &&
                (poll_periods < next_start ||
                 (state.status == 1 && state.complete == (poll_periods >= next_complete))),
            "Independent native ADC start115/completion291 at poll period=" +
                std::to_string(poll_periods) + " pc=" + hex(cpu.state().pc) +
                " status=" + hex(state.status) +
                " complete=" + std::to_string(static_cast<unsigned>(state.complete)) +
                " conversions=" + std::to_string(state.conversions) +
                " remaining=" + std::to_string(state.remaining));
  }

  void tick() {
    const auto before = cpu.state();
    const auto retired = memory.retired;
    memory.adc.context({.pc = before.pc, .ccr = before.ccr});
    memory.adc.advance_bus(1); // Explicit reference tie: ADC edge precedes native retirement.
    clock->advance();
    if (memory.retired == retired) {
      require(cpu.state() == before,
              "Incomplete native budgets cannot change architectural registers");
    }
    if (polling) {
      ++poll_periods;
      require((cpu.state().ccr & flag_i) != 0 && memory.interrupt_entries == 0,
              "Declared masked caller never manufactures an IRQ admission");
      adc_phase();
    }
  }

  void step() {
    if (!clock) {
      cpu.step();
      return;
    }
    const auto retired = memory.retired;
    unsigned periods = 0;
    while (memory.retired == retired) {
      require(++periods <= 11, "One ordinary instruction has its documented finite budget");
      tick();
    }
  }
};

void interrupt(Machine &m) {
  const auto before = m.cpu.state();
  const auto retired = m.memory.retired;
  require(m.cpu.pending_interrupt() == interrupt_vector && m.cpu.service_interrupt(),
          "Only a genuine SPI request may enter the original FLASH vector");
  require(m.cpu.state().pc == irq_handler && m.cpu.state().sp == before.sp - 5 &&
              m.memory.bytes[before.sp] == (before.pc & link::byte_mask) &&
              m.memory.bytes[before.sp - 1] == before.pc >> 8U &&
              m.memory.bytes[before.sp - 2] == (before.hx & link::byte_mask) &&
              m.memory.bytes[before.sp - 3] == before.a &&
              m.memory.bytes[before.sp - 4] == before.ccr,
          "Independent five-byte hardware frame precedes actual original IRQ execution");
  while (m.cpu.state().pc != before.pc) {
    require(m.memory.retired - retired < 64, "Original SPI handler has a bounded native return");
    m.cpu.step();
  }
  require(m.cpu.state() == before && !m.cpu.faulted(),
          "Actual original RTI restores every foreground register and its CCR knowledge");
}

void produce(Machine &m, const unsigned stop_after = bound) {
  const auto before = m.memory.retired;
  while (m.cpu.state().pc != repeat && m.memory.retired - before < stop_after) {
    require(m.memory.retired < bound, "Original calibrated pitch producer must reach repeat_scan");
    if (m.cpu.pending_interrupt()) {
      interrupt(m);
    } else {
      m.step();
    }
  }
}

std::optional<unsigned> deliver(Machine &m) {
  if (!m.memory.spi.state().transmit) {
    require(m.memory.interrupt_entries == 0 && !m.cpu.pending_interrupt(),
            "A suppressed original event cannot acquire an invented SPI transmission");
    return {};
  }
  for (unsigned n = 0; n < 3; ++n) {
    require(m.memory.spi.state().transmit.has_value(), "Original ISR must supply every next byte");
    m.memory.spi.transfer_to_shifter();
    for (unsigned edge = 0; edge < link::edges; ++edge) {
      link::coupled_edge(m.memory.spi, m.peer);
    }
    m.memory.spi.complete_transfer();
    interrupt(m);
    // No source is pending while this real following instruction executes;
    // both RTI reference timings agree. JMP/LDX/ASLX stay on original FLASH.
    m.cpu.step();
    ++m.foreground_steps;
  }
  require(m.cpu.state().pc == scan + 3 && m.cpu.state().hx == 0 && m.foreground_steps == 3 &&
              m.memory.interrupt_entries == 4 && m.memory.interrupt_acknowledgements == 4 &&
              !m.cpu.pending_interrupt() &&
              m.memory.spi.read(hcs08_spi::control1) == link::quiet_control &&
              !m.memory.spi.state().transmit && !m.memory.spi.state().receive_full,
          "Original producer plus four IRQs deliver all three bytes and really quiesce SPI");
  return m.peer.read(link::hrx);
}

void trace_outcome(const Machine &m,
                   const Input input,
                   const std::optional<unsigned> packet,
                   std::ostream &trace) {
  const bool emitted = packet.has_value();
  trace << "CASE working=" << input.working << " previous=" << input.previous
        << " gain=" << input.gain << " cursor=" << input.cursor << " offset=" << input.filter_offset
        << " last=" << word(m.memory, last) << " filter=" << word(m.memory, filter)
        << " sum=" << word(m.memory, sum)
        << " counter=" << static_cast<unsigned>(m.memory.bytes[counter]) << " emitted=" << emitted
        << " packet=" << packet.value_or(0)
        << " tx=" << static_cast<unsigned>(m.memory.bytes[tx_write])
        << " rx=" << static_cast<unsigned>(m.memory.bytes[rx_write])
        << " irqs=" << m.memory.interrupt_entries << " foreground=" << m.foreground_steps
        << " pc=" << m.cpu.state().pc << " sp=" << m.cpu.state().sp << '\n';
}

void outcome(Machine &m, const Input input, std::ostream &trace) {
  const auto packet = deliver(m);
  const auto emitted = packet.has_value();
  require(word(m.memory, sum) == 0 && word(m.memory, filter) == input.working * 4 &&
              word(m.memory, minimum) == 0 && word(m.memory, center) == seed_center &&
              word(m.memory, maximum) == seed_maximum && word(m.memory, calibration) == 0 &&
              m.memory.bytes[counter] == seed_counter + 1 &&
              m.memory.bytes[tx_write] == byte_cursor(input.cursor + (emitted ? 3U : 0U)) &&
              m.memory.bytes[tx_read] == m.memory.bytes[tx_write] &&
              m.memory.bytes[rx_write] == (emitted ? 3U : 0U) &&
              m.memory.bytes[cycle_flags] == (emitted ? 2U : 3U) && m.cpu.state().sp == stack &&
              m.cpu.state().ccr_known == full_flags && m.cpu.state().a_known == full_flags &&
              m.cpu.state().hx_known == word_mask && (m.cpu.state().ccr & flag_i) == 0 &&
              !m.memory.options.written() && m.memory.adc.state().status == 1 &&
              m.memory.adc.state().conversions == 1 &&
              m.memory.adc.state().remaining == conversion_periods,
          "Independent filter/accumulator/calibration/ring/stack/ADC invariants retain original "
          "behavior");
  if (emitted) {
    for (unsigned n = 0; n < link::reply_bytes.size(); ++n) {
      require(m.memory.known[rx_buffer + n] &&
                  m.memory.bytes[rx_buffer + n] == link::reply_bytes[n],
              "Original RX code stores only the actual SHI reply bytes");
    }
    require(m.memory.bytes[rx_latest] == link::reply_bytes.back() &&
                m.memory.bytes[rx_previous] == link::reply_bytes[1],
            "Original latest/previous receive bytes follow real full-duplex transfers");
  }
  trace_outcome(m, input, packet, trace);
}

std::optional<unsigned> queued_packet(const Machine &m, const Input input) {
  const auto cursor = m.memory.bytes[tx_write];
  if (cursor == input.cursor) { return {}; }
  require(cursor == byte_cursor(input.cursor + 3),
          "Original producer advances exactly one three-byte ring record");
  unsigned result = 0;
  for (unsigned n = 0; n < 3; ++n) {
    const auto address = tx_buffer + byte_cursor(input.cursor + n);
    require(m.memory.known[address], "Every output byte must be written by the original producer");
    result = (result << 8U) | m.memory.bytes[address];
  }
  return result;
}

void clocked_outcome(const Machine &m, const Input input, std::ostream &trace) {
  const auto packet = queued_packet(m, input);
  const bool emitted = packet.has_value();
  require(
      m.cpu.state().pc == repeat && m.cpu.state().sp == stack &&
          m.cpu.state().ccr_known == full_flags && m.cpu.state().a_known == full_flags &&
          m.cpu.state().hx_known == word_mask && (m.cpu.state().ccr & flag_i) != 0 &&
          word(m.memory, sum) == 0 && word(m.memory, filter) == input.working * 4 &&
          word(m.memory, minimum) == 0 && word(m.memory, center) == seed_center &&
          word(m.memory, maximum) == seed_maximum && word(m.memory, calibration) == 0 &&
          m.memory.bytes[counter] == seed_counter + 1 && m.memory.bytes[tx_read] == input.cursor &&
          m.memory.bytes[rx_write] == 0 && m.memory.bytes[rx_latest] == 0 &&
          m.memory.bytes[rx_previous] == 0 && m.memory.bytes[cycle_flags] == (emitted ? 2U : 3U) &&
          m.memory.spi.state().control1 == (emitted ? link::control : link::quiet_control) &&
          m.memory.spi.interrupt_requested() == emitted && !m.memory.spi.state().transmit &&
          !m.memory.spi.state().receive_full && m.memory.interrupt_entries == 0 &&
          m.memory.interrupt_acknowledgements == 0 && !m.cpu.pending_interrupt() &&
          m.poll_periods >= next_complete && !m.memory.options.written(),
      "Clocked original filter/calibration/ring/stack effects retain masked SPI and real ADC work");
  m.adc_phase();
  const auto clock = m.clock->state();
  require(clock.remaining_periods == 0 &&
              clock.executing_periods == setup_periods + m.poll_periods &&
              clock.offered_periods == clock.executing_periods + conversion_periods &&
              clock.retired_instructions == m.memory.retired && !m.bdc->state().active &&
              !m.clock->faulted() && !m.cpu.faulted(),
          "Every offered ADC/CPU period retains actual native ownership and retirement counts");
  trace_outcome(m, input, packet, trace);
  trace << "CLOCK periods=" << m.poll_periods << " instructions=" << m.memory.retired - 3
        << " retired=" << clock.retired_instructions << " partial=" << clock.remaining_periods
        << " executing=" << clock.executing_periods << " offered=" << clock.offered_periods
        << " start=" << next_start << " complete=" << next_complete
        << " adc=" << static_cast<unsigned>(m.memory.adc.state().status)
        << " ccf=" << m.memory.adc.state().complete
        << " conversions=" << m.memory.adc.state().conversions
        << " remaining=" << m.memory.adc.state().remaining << " tx_read=" << input.cursor
        << " control=" << static_cast<unsigned>(m.memory.spi.state().control1) << " masked=1\n";
}

std::vector<Input> cases() {
  constexpr std::array<unsigned, 15> working{
      0, 63, 64, 65, 256, 1000, 1702, 1703, 2048, 2112, 2113, 2232, 2233, 3000, 4092};
  constexpr std::array<unsigned, 4> cursors{0, 253, 254, 255};
  std::vector<Input> result;
  const auto add = [&](const unsigned value,
                       const unsigned previous,
                       const unsigned gain,
                       const int filter_offset = 0) {
    result.push_back({.working = value,
                      .previous = previous,
                      .gain = gain,
                      .cursor = cursors[result.size() % cursors.size()],
                      .filter_offset = filter_offset});
  };
  for (const auto value : working) {
    for (const unsigned previous : {0U, 8192U, 16383U}) { add(value, previous, 5); }
  }
  for (const unsigned previous : {4680U, 4711U, 4712U, 4648U, 4647U}) {
    add(lower_working, previous, 5);
  }
  for (const unsigned previous : {12032U, 12063U, 12064U, 12000U, 11999U}) {
    add(upper_working, previous, 5);
  }
  add(0, 1, 5);
  add(seed_center, center_code - 1, 5);
  add(scaled_maximum, maximum_code - 1, 5);
  add(wrap_lower_working, 0, wrapping_gain);
  add(wrap_upper_working, 0, wrapping_gain);
  for (const unsigned value : {lower_working, upper_working}) {
    for (const int offset : std::array<int, 4>{-64, 64, -63, 63}) { add(value, 0, 5, offset); }
  }
  require(result.size() == matrix_count, "Own boundary/dynamic-filter matrix has its exact extent");
  return result;
}

void replays(const std::span<const std::uint8_t> records, std::ostream &trace) {
  constexpr Input input{
      .working = 3000, .previous = 0, .gain = 5, .cursor = 255, .filter_offset = 64};
  constexpr std::array<unsigned, 6> cuts{0, 1, 24, 60, 100, 140};
  Machine reference(records, input);
  produce(reference);
  std::ostringstream expected;
  outcome(reference, input, expected);
  for (const auto cut : cuts) {
    Machine partial(records, input);
    produce(partial, cut);
    require(
        partial.memory.interrupt_entries == 0 && !partial.memory.spi.state().transmit &&
            partial.cpu.state().pc != repeat,
        "Selected native cuts precede peer traffic, including actual filter/multiply stack state");
    const auto cpu = partial.cpu.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      Machine fresh(records, input);
      fresh.memory = partial.memory;
      fresh.cpu.restore_checkpoint(cpu);
      produce(fresh);
      std::ostringstream actual;
      outcome(fresh, input, actual);
      app::same_endpoint(fresh.memory, reference.memory);
      require(actual.str() == expected.str() && fresh.cpu.state() == reference.cpu.state(),
              "Fresh CPU/RAM/ADC/SPI owners reproduce the complete native producer/IRQ/SHI suffix");
      trace << "REPLAY cut=" << cut << " index=" << replay << " identical=1\n";
    }
  }
}

void clocked_replays(const std::span<const std::uint8_t> records, std::ostream &trace) {
  constexpr Input input{
      .working = 3000, .previous = 0, .gain = 5, .cursor = 255, .filter_offset = 64};
  constexpr std::array<unsigned, 7> cuts{0, 1, 114, 115, 290, 291, partial_mul};
  Machine reference(records, input, true);
  produce(reference);
  std::ostringstream expected;
  clocked_outcome(reference, input, expected);
  for (const auto cut : cuts) {
    Machine partial(records, input, true);
    if (cut == partial_mul) {
      while (partial.cpu.state().pc != repeat &&
             (partial.memory.peek(partial.cpu.state().pc) != mul_opcode ||
              partial.clock->state().remaining_periods != 3)) {
        require(partial.poll_periods < bound,
                "Original positive pitch path must reach a partial MUL");
        partial.tick();
      }
      require(partial.memory.peek(partial.cpu.state().pc) == mul_opcode &&
                  partial.clock->state().remaining_periods == 3,
              "Selected snapshot is genuinely inside a native five-period MUL");
    } else {
      for (unsigned n = 0; n < cut; ++n) { partial.tick(); }
    }
    require(partial.cpu.state().pc != repeat && partial.memory.interrupt_entries == 0,
            "Selected period cuts precede the actual original masked producer endpoint");
    const auto cpu = partial.cpu.checkpoint();
    const auto bdc = partial.bdc->checkpoint();
    const auto clock = partial.clock->checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      Machine fresh(records, input, true);
      fresh.memory = partial.memory;
      fresh.cpu.restore_checkpoint(cpu);
      fresh.bdc->restore_checkpoint(bdc);
      fresh.clock->restore_checkpoint(clock);
      fresh.poll_periods = partial.poll_periods;
      produce(fresh);
      std::ostringstream actual;
      clocked_outcome(fresh, input, actual);
      app::same_endpoint(fresh.memory, reference.memory);
      require(actual.str() == expected.str() && fresh.cpu.state() == reference.cpu.state() &&
                  fresh.clock->state() == reference.clock->state() &&
                  fresh.bdc->state() == reference.bdc->state(),
              "Fresh CPU/RAM/ADC/SPI/BDC/clock owners reproduce every partial-budget suffix");
      trace << "CLOCK_REPLAY cut=" << cut << " phase=" << partial.poll_periods
            << " remaining=" << partial.clock->state().remaining_periods << " index=" << replay
            << " identical=1\n";
    }
  }
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 3 || count == 4,
            "Expected pinned page records, private pitch trace and optional --clocked");
    const bool clocked = count == 4;
    if (clocked) {
      require(std::string(arguments[3]) == "--clocked", "Unknown pitch test profile");
    }
    const auto records = hp::read_input(arguments[1], app::records_bytes);
    std::ostringstream trace;
    for (const auto input : cases()) {
      Machine m(records, input, clocked);
      const auto image = m.memory.flash.image();
      produce(m);
      if (clocked) {
        clocked_outcome(m, input, trace);
      } else {
        outcome(m, input, trace);
      }
      require(m.memory.flash.image() == image && !m.cpu.faulted(),
              "All supplied original instructions and data stay unchanged through event delivery");
    }
    if (clocked) {
      clocked_replays(records, trace);
    } else {
      replays(records, trace);
    }
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout
        << (clocked ? "Original pitch filter/ring with masked IRQ and common ADC/CPU clock pass.\n"
                    : "Original pitch filter, integer mapping, event ring and SPI IRQ/SHI delivery "
                      "pass.\n");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
