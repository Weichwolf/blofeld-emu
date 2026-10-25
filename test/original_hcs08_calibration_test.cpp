// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_run_clock.hpp"
#include "storage/private_file.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::hcs08::require;
namespace app = test::hcs08_clock_startup;
constexpr unsigned setup = 0xe1b4, setup_end = 0xe1c2, poll = 0xe33a, repeat = 0xe7ae;
constexpr unsigned divide_wrapper = 0xe12a, stack = 0x475, sample_count = 4817;
constexpr unsigned frame_count = sample_count * 3, instruction_bound = 2048, flags_mask = 8;
constexpr unsigned minimum = 0x80, center = 0x82, lower_gain = 0x84, upper_gain = 0x85;
constexpr unsigned last = 0x89, maximum = 0x8b, filter = 0x8d, calibration = 0xc1;
constexpr unsigned tx_read = 0x97, tx_write = 0x98, pressure_last = 0x96, cycle_flags = 0xa0;
constexpr unsigned pitch_sum = 0xb8, pitch_counter = 0xbf, tx_buffer = 0x100;
constexpr unsigned remaining_mask = 255, startup_count = 6188, initial_calibration = 300;
constexpr unsigned conversion_periods = 176, setup_periods = 21, initial_wait = 168;
constexpr unsigned strap_levels = 0x38;
constexpr unsigned pitch_sample = 512, microvolts_per_code = 3000, default_trim = 0x80;
constexpr unsigned initial_minimum = 2047, expected_center = 2041, expected_minimum = 1541;
constexpr unsigned expected_maximum = 2541, expected_gain = 24, expected_filter = 8192;
constexpr unsigned spi_control = 0x74, next_remaining = 135, maximum_pressure = 127;
constexpr unsigned modulation_minimum = 0x86, modulation_maximum = 0x90, modulation_gain = 0x88;
constexpr unsigned modulation_counter = 0xbe, pressure_counter = 0xc0;

constexpr Hcs08AdcReference analog_reference{
    .bus_hz = 6000000, .supply_uv = 3300000, .low_uv = 0, .high_uv = 3072000};

unsigned word(const app::ApplicationMemory &memory, const unsigned address) {
  require(memory.known[address] && memory.known[address + 1],
          "Original word must actually be known");
  return (static_cast<unsigned>(memory.bytes[address]) << 8U) | memory.bytes[address + 1];
}

class NoBdcMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override { return false; }

  std::uint8_t read(std::uint16_t /*address*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected calibration BDC memory read");
  }

  void write(ByteWrite /*access*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected calibration BDC memory write");
  }
};

struct Machine {
  app::ApplicationMemory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};
  NoBdcMemory mapping;
  std::unique_ptr<Hcs08Bdc> bdc;
  std::unique_ptr<Hcs08RunClock> clock;
  unsigned windows = 0;
  bool in_frame = false;

  explicit Machine(const std::span<const std::uint8_t> records)
      : memory(records, {.seed = 0, .trim = default_trim, .observation = strap_levels}) {
    memory.warm_reset();
    memory.icg.reset(Hcs08IcgReset::power_on);
    cpu.reset(Hcs08ResetEntry::normal, Hcs08ResetCause::power_on);
    require(memory.known.none() && cpu.state().a_known == 0,
            "Actual cold vector starts with unknown RAM/A before original initialization");
    while (cpu.state().pc != app::status_pc) {
      require(memory.retired < startup_count,
              "Original startup must reach its strict ICG boundary");
      cpu.step();
    }
    require(memory.retired == startup_count && cpu.state().sp == stack &&
                word(memory, calibration) == initial_calibration &&
                word(memory, minimum) == initial_minimum && word(memory, center) == 0 &&
                word(memory, maximum) == 0 && word(memory, filter) == 0 &&
                word(memory, pitch_sum) == 0 && memory.bytes[pitch_counter] == 0 &&
                memory.bytes[lower_gain] == 0 && memory.bytes[upper_gain] == 0,
            "Only original cold RAM-clear/calibration/prologue instructions establish history");
    app::ram_contents(memory, true);
    // Explicit component handoff. Both unresolved ICG status reads remain unexecuted;
    // this is not normal board boot or a supplied ready response.
    auto state = cpu.state();
    state.pc = setup;
    cpu.set_state(state);
    memory.adc.observe_reference(analog_reference);
    for (unsigned channel = 0; channel < hcs08_adc::channels; ++channel) {
      memory.adc.observe_input(channel, channel == 0 ? pitch_sample * microvolts_per_code : 0);
    }
    bdc = std::make_unique<Hcs08Bdc>(
        cpu, mapping, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
    clock = std::make_unique<Hcs08RunClock>(
        memory, *bdc, Hcs08RunContract::reference_instruction_retirement);
    command(hcs08_bdc::go);
    while (cpu.state().pc != setup_end) { step(); }
    require(clock->state().executing_periods == setup_periods &&
                memory.adc.state().remaining == initial_wait &&
                initial_wait == conversion_periods - (setup_periods - 13) &&
                memory.bytes[modulation_counter] == 5 && memory.bytes[pressure_counter] == 10,
            "Original4/4/5/4/4-period ADC/decimator setup retains real conversion progress");
  }

  void command(const std::uint8_t value) const {
    bdc->receive(value);
    bdc->service(Hcs08BdcGrant::reference_instruction_boundary);
  }

  void tick() {
    const auto state = cpu.state();
    memory.adc.context({.pc = state.pc, .ccr = state.ccr});
    memory.adc.advance_bus(1);
    clock->advance();
  }

  void step() {
    const auto retired = memory.retired;
    unsigned periods = 0;
    while (memory.retired == retired) {
      require(++periods <= 11, "Every ordinary instruction keeps its finite reference budget");
      tick();
    }
  }

  void enter() {
    require(clock->state().remaining_periods == 0,
            "Only an unowned boundary admits a component caller");
    command(hcs08_bdc::background);
    const auto adc = memory.adc.state();
    require(adc.complete || adc.active,
            "Only a genuinely pending/completed ADC job permits the next poll");
    if (!adc.complete) {
      for (unsigned n = 0; n < adc.remaining; ++n) { tick(); }
    }
    require(memory.adc.state().complete &&
                (memory.adc.state().status & hcs08_adc::channel_mask) == windows % 3 &&
                memory.adc.state().result == (windows % 3 == 0 ? pitch_sample : 0),
            "Original channel rotation and real stable-input samples precede every native poll");
    auto state = cpu.state();
    state.pc = poll;
    cpu.set_state(state); // Declared scanner caller; no RAM/register result or channel is injected.
    command(hcs08_bdc::go);
    in_frame = true;
  }
};

struct Saved {
  app::ApplicationMemory memory;
  Hcs08Checkpoint cpu;
  Hcs08BdcCheckpoint bdc;
  Hcs08RunCheckpoint clock;
  unsigned windows;
  bool in_frame;
  std::size_t offset;
  unsigned kind;
  unsigned remaining;
};

void capture(Machine &m,
             std::ostringstream &trace,
             std::vector<Saved> &saved,
             const unsigned kind) {
  saved.push_back({.memory = m.memory,
                   .cpu = m.cpu.checkpoint(),
                   .bdc = m.bdc->checkpoint(),
                   .clock = m.clock->checkpoint(),
                   .windows = m.windows,
                   .in_frame = m.in_frame,
                   .offset = trace.str().size(),
                   .kind = kind,
                   .remaining = m.clock->state().remaining_periods});
}

void calibration_row(const Machine &m, std::ostream &trace) {
  const unsigned sample = (m.windows / 3) + 1;
  if ((sample - 1) % 16 != 0) { return; }
  trace << "CAL sample=" << sample << " batch=" << ((sample - 1) / 16) + 1
        << " remaining=" << word(m.memory, calibration) << " center=" << word(m.memory, center)
        << " min=" << word(m.memory, minimum) << " max=" << word(m.memory, maximum)
        << " lower=" << static_cast<unsigned>(m.memory.bytes[lower_gain])
        << " upper=" << static_cast<unsigned>(m.memory.bytes[upper_gain])
        << " filter=" << word(m.memory, filter) << " sum=" << word(m.memory, pitch_sum)
        << " counter=" << static_cast<unsigned>(m.memory.bytes[pitch_counter])
        << " last=" << word(m.memory, last)
        << " tx=" << static_cast<unsigned>(m.memory.bytes[tx_write]) << '\n';
}

void finish(Machine &m, std::ostream &trace) {
  const auto adc = m.memory.adc.state();
  const auto clock = m.clock->state();
  require(m.windows == frame_count && !m.in_frame && m.cpu.state().pc == repeat &&
              m.cpu.state().sp == stack && (m.cpu.state().ccr & flags_mask) != 0 &&
              m.cpu.state().ccr_known == remaining_mask &&
              m.cpu.state().a_known == remaining_mask &&
              m.cpu.state().hx_known == test::hcs08::word_mask &&
              word(m.memory, calibration) == 0 && word(m.memory, center) == expected_center &&
              word(m.memory, minimum) == expected_minimum &&
              word(m.memory, maximum) == expected_maximum &&
              m.memory.bytes[lower_gain] == expected_gain &&
              m.memory.bytes[upper_gain] == expected_gain &&
              word(m.memory, filter) == expected_filter && word(m.memory, pitch_sum) == 0 &&
              word(m.memory, last) == expected_filter && m.memory.bytes[tx_write] == 6 &&
              m.memory.bytes[tx_read] == 0 && m.memory.bytes[pressure_last] == maximum_pressure &&
              m.memory.bytes[cycle_flags] == 0 && m.memory.spi.state().control1 == spi_control &&
              !m.memory.spi.state().transmit && m.memory.spi.interrupt_requested() &&
              m.memory.interrupt_entries == 0 && m.memory.interrupt_acknowledgements == 0 &&
              !m.cpu.pending_interrupt() && !m.memory.options.written() &&
              adc.conversions == frame_count && adc.status == 0 && !adc.complete && adc.active &&
              adc.remaining == next_remaining && adc.result == 0 && clock.remaining_periods == 0 &&
              clock.retired_instructions + startup_count == m.memory.retired && !m.cpu.faulted() &&
              !m.clock->faulted(),
          "Independent full calibration/filter/gain/event/channel/stack endpoint must agree");
  constexpr std::array<unsigned, 6> packets{0xd0, 0, 127, 0xe0, 0, 64};
  for (unsigned n = 0; n < packets.size(); ++n) {
    require(m.memory.known[tx_buffer + n] && m.memory.bytes[tx_buffer + n] == packets[n],
            "Only original pressure/pitch producers write both independently expected records");
  }
  trace << "FINAL windows=" << m.windows << " conversions=" << adc.conversions
        << " remaining=" << adc.remaining << " channel=" << static_cast<unsigned>(adc.status)
        << " modulation_min=" << word(m.memory, modulation_minimum)
        << " modulation_max=" << word(m.memory, modulation_maximum)
        << " modulation_gain=" << static_cast<unsigned>(m.memory.bytes[modulation_gain])
        << " pressure=" << static_cast<unsigned>(m.memory.bytes[pressure_last])
        << " tx=" << static_cast<unsigned>(m.memory.bytes[tx_write])
        << " irqs=" << m.memory.interrupt_entries << " sp=" << m.cpu.state().sp
        << " pc=" << m.cpu.state().pc << " retired=" << clock.retired_instructions
        << " executing=" << clock.executing_periods << " offered=" << clock.offered_periods << '\n';
}

void run(Machine &m, std::ostringstream &trace, std::vector<Saved> *snapshots = nullptr) {
  while (m.windows < frame_count) {
    if (!m.in_frame) { m.enter(); }
    const auto before = m.memory.retired;
    while (m.cpu.state().pc != repeat) {
      require(m.memory.retired - before < instruction_bound,
              "Original ADC window has a bounded native endpoint");
      m.tick();
      if (snapshots != nullptr && snapshots->size() == 1 && word(m.memory, calibration) == 0 &&
          m.cpu.state().pc == divide_wrapper && m.clock->state().remaining_periods == 1) {
        capture(m, trace, *snapshots, 1);
      }
    }
    require(m.cpu.state().sp == stack && (m.cpu.state().ccr & flags_mask) != 0 &&
                m.memory.interrupt_entries == 0,
            "Every native analog window retains stack/IRQ ownership");
    if (m.windows % 3 == 0) { calibration_row(m, trace); }
    ++m.windows;
    m.in_frame = false;
    if (snapshots != nullptr && snapshots->empty() && m.windows % 3 == 1 &&
        word(m.memory, calibration) == 1) {
      capture(m, trace, *snapshots, 0);
    }
  }
  finish(m, trace);
}

void replays(const std::span<const std::uint8_t> records,
             const Machine &reference,
             const std::vector<Saved> &saved,
             const std::string &expected,
             std::ostream &trace) {
  require(saved.size() == 2 && saved[0].kind == 0 && !saved[0].in_frame && saved[1].kind == 1 &&
              saved[1].in_frame && saved[1].remaining == 1,
          "Selected late calibration and real partial divide-helper cuts both exist");
  for (const auto &cut : saved) {
    for (unsigned index = 0; index < 2; ++index) {
      Machine fresh(records);
      fresh.memory = cut.memory;
      fresh.cpu.restore_checkpoint(cut.cpu);
      fresh.bdc->restore_checkpoint(cut.bdc);
      fresh.clock->restore_checkpoint(cut.clock);
      fresh.windows = cut.windows;
      fresh.in_frame = cut.in_frame;
      std::ostringstream actual;
      run(fresh, actual);
      app::same_endpoint(fresh.memory, reference.memory);
      require(actual.str() == expected.substr(cut.offset) &&
                  fresh.cpu.state() == reference.cpu.state() &&
                  fresh.clock->state() == reference.clock->state() &&
                  fresh.bdc->state() == reference.bdc->state(),
              "Fresh complete graph reproduces native calibration gain/helper/event suffix");
      trace << "REPLAY kind=" << cut.kind << " index=" << index << " identical=1\n";
    }
  }
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned page records and private calibration trace");
    const auto records = test::hcs08_programmer::read_input(arguments[1], app::records_bytes);
    Machine machine(records);
    const auto image = machine.memory.flash.image();
    std::ostringstream trace;
    std::vector<Saved> saved;
    run(machine, trace, &saved);
    require(machine.memory.flash.image() == image, "All supplied original FLASH stays unchanged");
    const auto text = trace.str();
    replays(records, machine, saved, text, trace);
    const auto output = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(output.data()), output.size()});
    std::cout
        << "Original cold RAM,300-step pitch calibration,real ADC history and joint replay pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
