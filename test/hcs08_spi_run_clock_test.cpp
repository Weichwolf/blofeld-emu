// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
namespace reg = hcs08_spi;
constexpr unsigned handler = 0x9000, vector = 0xffe0, stack = 0x240;
constexpr unsigned nop = 0x9d, rti = 0x80, bra = 0x20, backward_two = 0xfe;
constexpr unsigned lda = 0xb6, immediate = 0xa6, sta = 0xb7, mov = 0x6e;
constexpr unsigned control = 0x74, quiet = 0x54, pattern = 0xa5, reply = 0x5a, index = 0xb73c;
constexpr unsigned write_period = 17, control_period = 21, return_period = 32;
constexpr unsigned original_baud = 0x12;
constexpr unsigned partial = 0xef, samples = 8, edges = 16, instruction_count = 5;
constexpr auto edge_contract = Hcs08SpiClockContract::reference_half_period_and_final_edge;

struct Input {
  Hcs08SpiRunContract order = Hcs08SpiRunContract::reference_cpu_before_spi;
  Hcs08SpiLaunchContract launch = Hcs08SpiLaunchContract::reference_one_period;
  unsigned data = pattern, baud = 0, next_control = quiet;
  Hcs08PrefetchContract program = Hcs08PrefetchContract::reference_peripheral_program_cycles;
};

unsigned latency(const Input input) {
  return input.launch == Hcs08SpiLaunchContract::reference_one_period ? 1U : 2U;
}

unsigned divisor(const Input input) {
  return ((input.baud >> 4U) + 1U) * (1U << ((input.baud & 7U) + 1U));
}

unsigned launch_period(const Input input) {
  return write_period + latency(input);
}

unsigned completion_period(const Input input) {
  return launch_period(input) + (samples * divisor(input)) - 1;
}

Hcs08State initial() {
  return {.pc = entry,
          .hx = index,
          .sp = stack,
          .a = pattern,
          .ccr_known = partial,
          .ccr = flags_fixed | flag_c};
}

struct AccessAt {
  unsigned period;
  Access access;
  bool operator==(const AccessAt &) const = default;
};

class MemoryOwner final : public MaskedMemory {
public:
  Hcs08Spi spi{Hcs08SpiContract::candidate_gt8a_reference_master_edges};
  unsigned period = 0;
  bool fail_store = false;
  std::vector<AccessAt> peripheral_accesses;

  std::uint8_t read(const std::uint16_t address) override {
    if (!Hcs08Spi::contains(address)) { return Memory::read(address); }
    spi.context(context);
    const auto value = spi.read(address);
    peripheral_accesses.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = false}});
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (!Hcs08Spi::contains(address)) {
      Memory::write(address, value);
      return;
    }
    spi.context(context);
    spi.write({.address = address, .value = value});
    peripheral_accesses.push_back(
        {.period = period, .access = {.address = address, .value = value, .write = true}});
    if (address == reg::data && fail_store) { fail(address, "own after-SPID refusal"); }
  }

  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const override {
    if (!spi.interrupt_requested()) { return {}; }
    return InterruptRequest{.vector = vector};
  }
};

class NoMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override { return false; }

  std::uint8_t read(std::uint16_t /*address*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected serial-clock BDC memory read");
  }

  void write(ByteWrite /*access*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected serial-clock BDC memory write");
  }
};

struct Machine {
  MemoryOwner memory;
  Hcs08 cpu;
  NoMemory mapping;
  std::unique_ptr<Hcs08Bdc> bdc;
  std::unique_ptr<Hcs08SpiClock> spi;
  std::unique_ptr<Hcs08SpiRunClock> clock;
  unsigned transmitted = 0, sample_count = 0;

  explicit Machine(const Input input,
                   const Hcs08RtiIrqContract policy = Hcs08RtiIrqContract::reference_immediate,
                   const Hcs08BdcReset background = Hcs08BdcReset::normal)
      : cpu(memory, BLOFELD_HCS08_SLA, policy) {
    memory.writable.set();
    constexpr std::array<std::uint8_t, 2> target{0x90, 0};
    constexpr std::array<std::uint8_t, 3> loop{bra, backward_two, nop};
    const std::array<std::uint8_t, 12> code{lda,
                                            reg::status,
                                            immediate,
                                            static_cast<std::uint8_t>(input.data),
                                            sta,
                                            reg::data,
                                            mov,
                                            static_cast<std::uint8_t>(input.next_control),
                                            reg::control1,
                                            rti,
                                            nop,
                                            nop};
    memory.install(vector, target);
    memory.install(handler, code);
    memory.install(entry, loop);
    memory.spi.write({.address = reg::baud, .value = static_cast<std::uint8_t>(input.baud)});
    memory.spi.write({.address = reg::control1, .value = control});
    cpu.set_state(initial());
    bdc = std::make_unique<Hcs08Bdc>(cpu, mapping, Hcs08FlashContract::candidate_gt8a, background);
    spi = std::make_unique<Hcs08SpiClock>(memory.spi, edge_contract);
    clock =
        std::make_unique<Hcs08SpiRunClock>(*bdc, *spi, input.order, input.launch, input.program);
  }

  std::optional<bool> miso() const {
    const auto s = memory.spi.state();
    const unsigned bit = s.active && s.samples < samples ? samples - s.samples - 1 : samples - 1;
    return ((reply >> bit) & 1U) != 0;
  }

  void tick(const std::optional<bool> input) {
    memory.period = static_cast<unsigned>(clock->state().attempted_periods + 1);
    const auto pins = clock->advance(input);
    if (pins && !pins->clock) {
      if (!pins->mosi) { throw Fault("Expected actual MOSI at trailing sample edge"); }
      transmitted = (transmitted << 1U) | static_cast<unsigned>(*pins->mosi);
      ++sample_count;
    }
  }

  void tick() { tick(miso()); }
};

void check_case(const Input input) {
  Machine m(input);
  const unsigned end = completion_period(input) + 3;
  const unsigned half = divisor(input) / 2;
  for (unsigned period = 1; period <= end; ++period) {
    m.tick();
    const auto s = m.clock->state();
    const unsigned elapsed = period >= launch_period(input) ? period - launch_period(input) + 1 : 0;
    const unsigned count = std::min(elapsed / half, edges);
    const auto peripheral = m.memory.spi.state();
    require(
        s.attempted_periods == period && s.completed_periods == period && s.spi_periods == period &&
            s.irq_periods == std::min(period, 11U) && s.irq_periods + s.program_periods == period &&
            s.launches == static_cast<unsigned>(period >= launch_period(input)) &&
            m.spi->state().offered_periods == period && m.spi->state().generated_edges == count &&
            m.spi->state().completed_transfers ==
                static_cast<unsigned>(period >= completion_period(input)) &&
            peripheral.active ==
                (period >= launch_period(input) && period < completion_period(input)) &&
            peripheral.receive_full == (period >= completion_period(input)),
        "Independent CPU/SPI period, automatic launch and exact divider edge oracle");
    if (period >= return_period) {
      require(m.cpu.state() == initial() && m.memory.retired >= instruction_count,
              "Native RTI restores the real saved caller while its genuinely queued BRA repeats");
    }
  }
  require(m.transmitted == input.data && m.sample_count == samples &&
              m.memory.spi.state().receive == reply &&
              m.memory.peripheral_accesses ==
                  std::vector<AccessAt>{
                      {.period = 12,
                       .access = {.address = reg::status,
                                  .value = reg::transmit_empty,
                                  .write = false}},
                      {.period = write_period,
                       .access = {.address = reg::data, .value = input.data, .write = true}},
                      {.period = control_period,
                       .access = {.address = reg::control1, .value = quiet, .write = true}}},
          "Only actual native SPIS/SPID/control transactions occur; real pins form both complete "
          "bytes");
}

struct Snapshot {
  MemoryOwner memory;
  Hcs08Checkpoint cpu;
  Hcs08BdcCheckpoint bdc;
  Hcs08SpiClockCheckpoint spi;
  Hcs08SpiRunCheckpoint clock;
  unsigned transmitted, sample_count;
};

Snapshot save(const Machine &m) {
  const auto state = m.cpu.state();
  return {.memory = m.memory,
          .cpu = m.cpu.checkpoint(),
          .bdc = m.bdc->checkpoint(),
          .spi = m.spi->checkpoint({.pc = state.pc, .ccr = state.ccr}),
          .clock = m.clock->checkpoint(),
          .transmitted = m.transmitted,
          .sample_count = m.sample_count};
}

void restore(Machine &m, const Snapshot &s) {
  m.memory = s.memory;
  m.cpu.restore_checkpoint(s.cpu);
  m.bdc->restore_checkpoint(s.bdc);
  const auto state = m.cpu.state();
  m.spi->restore_checkpoint(s.spi, {.pc = state.pc, .ccr = state.ccr});
  m.clock->restore_checkpoint(s.clock);
  m.transmitted = s.transmitted;
  m.sample_count = s.sample_count;
}

Snapshot capture(const Input input, const unsigned cut) {
  Machine original(input);
  for (unsigned n = 0; n < cut; ++n) { original.tick(); }
  return save(original);
}

void fresh_continuations() {
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    for (const auto launch : {Hcs08SpiLaunchContract::reference_one_period,
                              Hcs08SpiLaunchContract::reference_two_periods}) {
      const Input input{.order = order, .launch = launch};
      const unsigned end = completion_period(input) + 1;
      Machine baseline(input);
      for (unsigned n = 0; n < end; ++n) { baseline.tick(); }
      for (unsigned cut = 0; cut <= end; ++cut) {
        const auto saved = capture(input, cut);
        for (unsigned replay = 0; replay < 2; ++replay) {
          Machine fresh(input);
          restore(fresh, saved);
          for (unsigned n = cut; n < end; ++n) { fresh.tick(); }
          require(fresh.clock->state() == baseline.clock->state() &&
                      fresh.cpu.state() == baseline.cpu.state() &&
                      fresh.cpu.prefetch_state() == baseline.cpu.prefetch_state() &&
                      fresh.memory.spi.state() == baseline.memory.spi.state() &&
                      fresh.spi->state() == baseline.spi->state() &&
                      fresh.memory.bytes == baseline.memory.bytes &&
                      fresh.memory.accesses == baseline.memory.accesses &&
                      fresh.memory.peripheral_accesses == baseline.memory.peripheral_accesses &&
                      fresh.transmitted == input.data && fresh.sample_count == samples,
                  "Fresh joint owners retain every IRQ/launch/edge/native prefix without duplicate "
                  "time");
        }
      }
    }
  }
}

void unknown_miso() {
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    const Input input{.order = order};
    Machine m(input);
    const unsigned sample = launch_period(input) + 1;
    for (unsigned n = 1; n < sample; ++n) { m.tick(); }
    const auto saved = save(m);
    const auto before = m.clock->state();
    require(
        rejected([&] { m.tick(std::nullopt); }).contains("unknown resolved input") &&
            m.clock->faulted() && m.clock->state().completed_periods == before.completed_periods &&
            m.clock->state().attempted_periods == sample && m.memory.spi.state().edges == 1 &&
            m.clock->state().irq_periods + m.clock->state().program_periods ==
                before.irq_periods + before.program_periods +
                    static_cast<unsigned>(order == Hcs08SpiRunContract::reference_cpu_before_spi),
        "Unknown MISO fails at its actual sample edge, retaining only the chosen CPU prefix");
    require(rejected([&] { m.tick(); }).contains("coordinated recovery"),
            "Failed periods cannot silently retry");
    restore(m, saved);
    for (unsigned n = sample; n <= completion_period(input); ++n) { m.tick(); }
    require(m.transmitted == pattern && m.memory.spi.state().receive == reply,
            "Joint recovery completes each real edge once");
  }
}

void failed_cpu_prefixes() {
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    const Input input{.order = order};
    Machine m(input);
    for (unsigned n = 1; n < write_period; ++n) { m.tick(); }
    const auto saved = save(m);
    m.memory.fail_store = true;
    require(rejected([&] { m.tick(); }).contains("after-SPID") && m.clock->faulted() &&
                m.memory.spi.state().transmit == pattern &&
                m.clock->state().completed_periods == write_period - 1 &&
                m.clock->state().spi_periods ==
                    write_period - static_cast<unsigned>(
                                       order == Hcs08SpiRunContract::reference_cpu_before_spi),
            "A failed native write retains the actual byte and any prior SPI period, never host "
            "rollback");
    restore(m, saved);
    for (unsigned n = write_period; n <= completion_period(input); ++n) { m.tick(); }
    require(m.clock->state().launches == 1 && m.transmitted == pattern,
            "Joint native-write recovery launches exactly once");
  }
}

void ambiguous_irq_prefixes() {
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    const Input input{.order = order, .next_control = control};
    Machine m(input, Hcs08RtiIrqContract::unqualified);
    for (unsigned n = 0; n < return_period; ++n) { m.tick(); }
    require(rejected([&] { m.tick(); }).contains("RTI IRQ timing contract") && m.clock->faulted(),
            "The common clock cannot select an implicit RTI policy at a real source reentry");
    const bool prior_edge = order == Hcs08SpiRunContract::reference_spi_before_cpu;
    require(m.memory.spi.state().receive_full == prior_edge &&
                m.clock->state().last_pins.has_value() == prior_edge &&
                m.clock->state().completed_periods == return_period &&
                m.clock->state().spi_periods == return_period + static_cast<unsigned>(prior_edge),
            "A later CPU arbitration fault retains the real SPI completion pins and byte prefix");
  }
}

void native_disables() {
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    for (const auto launch : {Hcs08SpiLaunchContract::reference_one_period,
                              Hcs08SpiLaunchContract::reference_two_periods}) {
      const Input input{.order = order, .launch = launch, .next_control = reg::reset_control};
      Machine m(input);
      for (unsigned n = 0; n < control_period; ++n) { m.tick(); }
      const auto saved = save(m); // SPI-before-CPU retains a genuine pending abort here.
      Machine fresh(input);
      restore(fresh, saved);
      fresh.tick();
      const unsigned active =
          control_period - launch_period(input) +
          static_cast<unsigned>(order == Hcs08SpiRunContract::reference_spi_before_cpu);
      require(fresh.spi->state().aborted_transfers == 1 &&
                  fresh.spi->state().active_periods == active &&
                  fresh.spi->state().completed_transfers == 0 &&
                  !fresh.memory.spi.state().receive_full,
              "Native SPE disable and a fresh pending-abort restore preserve the selected real "
              "edge order");
    }
  }
}

void unowned_foreground() {
  for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                           Hcs08SpiRunContract::reference_spi_before_cpu}) {
    const Input input{.order = order};
    Machine m(input);
    m.memory.spi.write({.address = reg::control1, .value = quiet});
    require(rejected([&] { m.tick(); }).contains("unowned foreground") && m.clock->faulted() &&
                m.memory.retired == 0 && m.memory.accesses.empty() &&
                m.clock->state().completed_periods == 0 &&
                m.clock->state().spi_periods ==
                    static_cast<unsigned>(order == Hcs08SpiRunContract::reference_spi_before_cpu),
            "Without an actual IRQ or owned queue the clock cannot invent a successful CPU idle "
            "period");
  }
}

void foreign_owners() {
  const Input input{};
  Machine m(input);
  require(m.bdc->start_irq_bus(Hcs08IrqBusContract::reference_unstretched_sequence),
          "Own source can be admitted by an outside owner for this negative case");
  require(rejected([&] { m.tick(); }).contains("another phase owner") && m.clock->faulted() &&
              m.spi->state().offered_periods == 0 && m.memory.accesses.empty(),
          "Foreign CPU admission is detected before any clock or bus effect");
  Machine external(input);
  const auto current = external.cpu.state();
  static_cast<void>(external.spi->advance({.pc = current.pc, .ccr = current.ccr}, false));
  require(rejected([&] { external.tick(); }).contains("another phase owner") &&
              external.spi->state().offered_periods == 1 && external.memory.accesses.empty(),
          "Foreign SPI periods cannot become an extra hidden common-clock period");
  require(
      rejected([&] {
        const Machine halted(
            input, Hcs08RtiIrqContract::reference_immediate, Hcs08BdcReset::bkgd_held_low);
      }).contains("active background"),
      "Unknown active-background peripheral clock behavior cannot become a synthetic idle tick");
}

void program_contracts() {
  const Input base{};
  const Input jump{.program = Hcs08PrefetchContract::reference_jump_program_cycles};
  Machine original(base);
  const auto saved = save(original);
  Machine other(jump);
  require(rejected([&] { restore(other, saved); }).contains("program contract mismatch") &&
              other.clock->faulted(),
          "Joint clock snapshots cannot reinterpret a retained program contract");
  require(rejected([&] {
            const Machine unsupported({.program = Hcs08PrefetchContract::unqualified});
          }).contains("supported IRQ program contract"),
          "A common clock cannot choose unknown instruction-cycle behavior");
  for (unsigned n = 0; n < 12; ++n) { original.tick(); }
  require(rejected([&] {
            const Hcs08SpiRunClock mismatch(
                *original.bdc, *original.spi, base.order, base.launch, jump.program);
          }).contains("program queue contract"),
          "An existing actual queue must match the common owner's selected program reference");
}
}

int main() {
  try {
    for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                             Hcs08SpiRunContract::reference_spi_before_cpu}) {
      for (const auto launch : {Hcs08SpiLaunchContract::reference_one_period,
                                Hcs08SpiLaunchContract::reference_two_periods}) {
        for (unsigned data = 0; data < byte_values; ++data) {
          check_case({.order = order, .launch = launch, .data = data});
        }
        check_case({.order = order, .launch = launch, .baud = original_baud});
      }
    }
    fresh_continuations();
    unknown_miso();
    failed_cpu_prefixes();
    ambiguous_irq_prefixes();
    native_disables();
    foreign_owners();
    unowned_foreground();
    program_contracts();
    std::cout
        << "HCS08 CPU/SPI clock:1024 byte/order/launch cases,real IRQ/RTI,automatic serial edges,"
           "fresh joint suffixes and strict committed failure prefixes pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
