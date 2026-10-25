// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_cop.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_run_clock.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_system_options.hpp"
#include "keyboard/hcs08_tpm.hpp"
#include "storage/private_file.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
namespace hf = hcs08_flash;
constexpr auto chip = Hcs08FlashContract::candidate_gt8a;
constexpr auto periods_contract = Hcs08RunContract::reference_instruction_retirement;
constexpr unsigned entry = 0xe1c2, exit_pc = 0xe1d1, stack = 0x475, seed_hx = 0x1234;
constexpr unsigned accumulator = 0x5a, initial_ccr = 0x68, exit_ccr = 0x6a, known_flags = 0xef;
constexpr unsigned threshold = 31250, page_bytes = 512, record_bytes = 514, supplied_bytes = 3598;
constexpr unsigned modulus_low = 0x34, timer_control = 0x0f;
constexpr unsigned spi_control = 0x74, spi_divider = 0x12, flash_divider = 0x3d;
constexpr unsigned spi_periods = 13, setup_periods = 37, fault_periods = 41;
constexpr unsigned flash_fault_pc = 0xea88, return_pc = 0xe1dc, setup_instructions = 10;
constexpr unsigned setup_accumulator = 0xff, setup_flags = 0x6c, interrupt_mask = 8;
constexpr unsigned long_timeout = 262144, deadline_period = 20;
constexpr std::array pages{0xe000U, 0xe200U, 0xe400U, 0xe600U, 0xe800U, 0xea00U, 0xfe00U};

Hcs08FlashImage original_image(const std::span<const std::uint8_t> records) {
  Hcs08FlashImage image;
  for (unsigned n = 0; n < pages.size(); ++n) {
    const auto offset = n * record_bytes;
    require((static_cast<unsigned>(records[offset]) << 8U | records[offset + 1]) == pages[n],
            "All pinned original page addresses remain exact");
    for (unsigned k = 0; k < page_bytes; ++k) {
      const auto index = pages[n] - hf::first + k;
      image.data[index] = records[offset + 2 + k];
      image.known.set(index);
    }
  }
  require(image.known.count() == pages.size() * page_bytes && image.fresh.none(),
          "Supplied pages establish contents, never omitted pages or erase history");
  return image;
}

class Memory final : public blofeld::test::hcs08::Memory {
public:
  Hcs08Flash flash;
  Hcs08Tpm timer;
  Hcs08Cop cop{Hcs08CopContract::candidate_gt8a_run_bus_edges};
  Hcs08Spi spi{Hcs08SpiContract::candidate_gt8a_reference_master_edges};
  std::vector<blofeld::test::hcs08::Access> setup_writes;
  unsigned services = 0, high_reads = 0, low_reads = 0, control_writes = 0;

  Memory(const Hcs08FlashImage &image, const unsigned phase)
      : flash(image, chip), timer(Hcs08TpmContract::candidate_gt8a_run_bus_edges, phase) {
    constexpr unsigned ram_first = 0x80;
    constexpr unsigned ram_end = 0x480;
    for (unsigned address = ram_first; address < ram_end; ++address) { writable.set(address); }
  }

  std::uint8_t read(const std::uint16_t address) override {
    if (address >= hf::first) { return flash.read_array(address); }
    if (Hcs08Spi::contains(address)) { return spi.read(address); }
    if (address >= hf::divider && address <= hf::command) { return flash.read(address); }
    if (address == Hcs08Cop::status || address == Hcs08SystemOptions::address) {
      return cop.read(address);
    }
    if (address >= Hcs08Tpm::status && address <= modulus_low) {
      const auto value = timer.read(address);
      high_reads += static_cast<unsigned>(address == Hcs08Tpm::count_high);
      low_reads += static_cast<unsigned>(address == Hcs08Tpm::count_low);
      return value;
    }
    return blofeld::test::hcs08::Memory::read(address);
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (Hcs08Spi::contains(address)) {
      spi.write({.address = address, .value = value});
      setup_writes.push_back({.address = address, .value = value, .write = true});
    } else if (address >= hf::divider && address <= hf::command) {
      flash.write({.address = address, .value = value});
      setup_writes.push_back({.address = address, .value = value, .write = true});
    } else if (address == Hcs08Cop::status || address == Hcs08SystemOptions::address) {
      cop.write({.address = address, .value = value});
      services += static_cast<unsigned>(address == Hcs08Cop::status);
    } else if (address >= Hcs08Tpm::status && address <= modulus_low) {
      timer.write({.address = address, .value = value});
      ++control_writes;
    } else {
      blofeld::test::hcs08::Memory::write(address, value);
    }
  }

  void begin_instruction(const ExecutionContext value) override {
    blofeld::test::hcs08::Memory::begin_instruction(value);
    flash.context(value);
    timer.context(value);
    cop.context(value);
    spi.context(value);
  }

  std::uint8_t peek(const std::uint16_t address) const override {
    return address >= hf::first ? flash.read_array(address)
                                : blofeld::test::hcs08::Memory::peek(address);
  }

  void check_execute(const InstructionFetch fetch) const override {
    if (fetch.address < hf::first ||
        fetch.length > blofeld::test::hcs08::space_bytes - fetch.address) {
      throw Fault("HCS08 timer helper fetch address=" + hex(fetch.address) +
                  " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr) +
                  ": requires actual readable FLASH");
    }
    for (unsigned n = 0; n < fetch.length; ++n) {
      static_cast<void>(flash.read_array(static_cast<std::uint16_t>(fetch.address + n)));
    }
  }

  void bus_edge(const Hcs08State cpu) {
    const ExecutionContext current{.pc = cpu.pc, .ccr = cpu.ccr};
    cop.context(current);
    if (cop.state().reset_requested) {
      throw Fault("HCS08 timer fixture clock address=" + hex(Hcs08Cop::status) +
                  " pc=" + hex(cpu.pc) + " ccr=" + hex(cpu.ccr) + ": pending COP reset");
    }
    timer.advance_bus(1);
    require(cop.advance_bus(1) == 1, "The offered bus edge is consumed exactly once");
    if (cop.state().reset_requested) {
      throw Fault("HCS08 COP timeout address=" + hex(Hcs08Cop::status) + " pc=" + hex(cpu.pc) +
                  " ccr=" + hex(cpu.ccr) + ": coordinated reset required");
    }
    require(!timer.interrupt_requested(), "Unconnected timer IRQ cannot be silently consumed");
  }
};

// No debug traffic is supplied by this run-mode component. Unsupported debug
// transactions fail instead of inventing privileges or successful memory access.
class NoDebugger final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(const ExecutionContext context) const override {
    throw Fault("Unqualified timer-fixture BDC security query pc=" + hex(context.pc) +
                " ccr=" + hex(context.ccr));
  }

  std::uint8_t read(const std::uint16_t address, const ExecutionContext context) override {
    throw Fault("Unqualified timer-fixture BDC read address=" + hex(address) +
                " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr));
  }

  void write(const ByteWrite access, const ExecutionContext context) override {
    throw Fault("Unqualified timer-fixture BDC write address=" + hex(access.address) +
                " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr));
  }
};

struct Expected {
  unsigned phase, initial_age, periods, instructions, services, endpoint_phase;
  std::uint16_t hx_known = Hcs08State::all_hx;
};

constexpr std::array<Expected, 2> expectations{{{.phase = 0,
                                                 .initial_age = 0,
                                                 .periods = 4000013,
                                                 .instructions = 1142861,
                                                 .services = 285714,
                                                 .endpoint_phase = 9},
                                                {.phase = 127,
                                                 .initial_age = long_timeout - 22,
                                                 .periods = 3999887,
                                                 .instructions = 1142825,
                                                 .services = 285705,
                                                 .endpoint_phase = 10,
                                                 .hx_known = 0}}};

void advance_to(Memory &memory, const Hcs08 &cpu, Hcs08RunClock &clock, const unsigned periods) {
  while (clock.state().executing_periods < periods) {
    memory.bus_edge(cpu.state());
    clock.advance();
  }
}

void setup_spi(Memory &memory,
               Hcs08 &cpu,
               Hcs08RunClock &clock,
               const Expected expected,
               std::ostream &trace) {
  const auto options = memory.flash.read(hf::options);
  const auto protection = memory.flash.read(hf::protection);
  advance_to(memory, cpu, clock, expected.periods + spi_periods);
  const Hcs08SpiState configured{
      .control1 = spi_control, .baud = spi_divider, .transmit = std::nullopt, .mosi = std::nullopt};
  require(memory.spi.state() == configured && memory.spi.interrupt_requested() &&
              memory.spi.clock_divisor() == 16 && !memory.spi.faulted() &&
              (cpu.state().ccr & interrupt_mask) != 0 && memory.known.none(),
          "Original MOV/MOV/CLR configure the real idle SPI with its IRQ masked by CPU I");
  advance_to(memory, cpu, clock, expected.periods + setup_periods);
  require(cpu.state() == Hcs08State{.pc = flash_fault_pc,
                                    .hx = threshold,
                                    .sp = stack - 2,
                                    .a = setup_accumulator,
                                    .ccr_known = known_flags,
                                    .ccr = setup_flags} &&
              memory.retired == expected.instructions + setup_instructions &&
              memory.flash.state() == Hcs08FlashState{.staged = {},
                                                      .active = std::nullopt,
                                                      .divider = flash_divider,
                                                      .divider_loaded = true} &&
              memory.flash.read(hf::options) == options &&
              memory.flash.read(hf::protection) == protection,
          "Original JSR and flash setup preserve unknown H, ignore CPU FOPT/FPROT writes and "
          "load only the real write-once FCDIV before idle FSTAT");
  const std::vector<blofeld::test::hcs08::Access> writes{
      {.address = hcs08_spi::baud, .value = spi_divider, .write = true},
      {.address = hcs08_spi::control1, .value = spi_control, .write = true},
      {.address = hcs08_spi::control2, .value = 0, .write = true},
      {.address = hf::divider, .value = flash_divider, .write = true},
      {.address = hf::options, .value = 2, .write = true},
      {.address = hf::protection, .value = setup_accumulator, .write = true}};
  require(memory.setup_writes == writes && memory.known.count() == 2 && memory.known[stack] &&
              memory.known[stack - 1] &&
              memory.bytes[stack] == (return_pc & blofeld::test::hcs08::byte_mask) &&
              memory.bytes[stack - 1] == (return_pc >> 8U),
          "Exact original peripheral write order and actual low/high JSR stack bytes are retained");
  trace << "SPI phase=" << expected.phase << " periods=" << clock.state().executing_periods
        << " retired=" << memory.retired << " control=" << spi_control << " baud=" << spi_divider
        << " divisor=" << memory.spi.clock_divisor() << " irq=1 cpu-i=1 edges=0 samples=0"
        << " flash-divider=" << flash_divider << " stack-bytes=" << memory.known.count() << '\n';
  const auto error =
      rejected([&] { advance_to(memory, cpu, clock, expected.periods + fault_periods); });
  require(error.contains("register write address=" + hex(hf::status)) &&
              error.contains("pc=" + hex(flash_fault_pc)) &&
              error.contains("unqualified idle FCBEF launch") && cpu.faulted() && clock.faulted() &&
              memory.retired == expected.instructions + setup_instructions &&
              clock.state().executing_periods == expected.periods + fault_periods &&
              memory.timer.state().counter == threshold &&
              memory.timer.state().prescaler_phase == expected.endpoint_phase + fault_periods &&
              memory.cop.state().elapsed == 10 + fault_periods &&
              memory.spi.state() == configured && memory.spi.interrupt_requested() &&
              memory.setup_writes == writes && memory.known.count() == 2,
          "Actual original idle FSTAT write refuses after its full budget; no serial edge, "
          "command launch, IRQ delivery or NVM completion is fabricated");
  trace << "ERROR " << error << '\n'
        << "FAULT phase=" << expected.phase << " periods=" << clock.state().executing_periods
        << " retired=" << memory.retired << " pc=" << cpu.state().pc << '\n';
}

void finish(Memory &memory,
            Hcs08 &cpu,
            Hcs08RunClock &clock,
            const Expected expected,
            std::ostream &trace) {
  advance_to(memory, cpu, clock, expected.periods);
  require(cpu.state() == Hcs08State{.pc = exit_pc,
                                    .hx = threshold,
                                    .sp = stack,
                                    .a = accumulator,
                                    .ccr_known = known_flags,
                                    .ccr = exit_ccr} &&
              memory.retired == expected.instructions &&
              clock.state().retired_instructions == expected.instructions &&
              clock.state().offered_periods == expected.periods &&
              clock.state().remaining_periods == 0,
          "Independent complete CPU/period oracle checks the actual original threshold exit");
  require(memory.timer.state() == Hcs08TpmState{.counter = threshold,
                                                .prescaler_phase = expected.endpoint_phase,
                                                .control = timer_control} &&
              memory.cop.state() == Hcs08CopState{.elapsed = 10} && memory.control_writes == 1 &&
              memory.services == expected.services && memory.high_reads == expected.services + 1 &&
              memory.low_reads == memory.high_reads && memory.known.none(),
          "Real coherent counter reads and all original SRS services retain exact peripheral/RAM "
          "state");
  trace << "EXIT phase=" << expected.phase << " periods=" << expected.periods
        << " retired=" << memory.retired << " pc=" << cpu.state().pc << " hx=" << cpu.state().hx
        << " services=" << memory.services << " counter=" << memory.timer.state().counter
        << " divider=" << expected.endpoint_phase << " cop-age=" << memory.cop.state().elapsed
        << '\n';
  setup_spi(memory, cpu, clock, expected, trace);
}

struct Saved {
  Memory memory;
  Hcs08Checkpoint cpu;
  Hcs08BdcCheckpoint bdc;
  Hcs08RunCheckpoint clock;
};

void run(const Hcs08FlashImage &image, const Expected expected, std::ostream &trace) {
  Memory memory(image, expected.phase);
  require(memory.cop.advance_bus(expected.initial_age) == expected.initial_age,
          "Previous watchdog age is an explicit component input, separate from timer phase");
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = entry,
                 .hx = seed_hx,
                 .sp = stack,
                 .a = accumulator,
                 .ccr_known = known_flags,
                 .ccr = initial_ccr,
                 .hx_known = expected.hx_known});
  require(
      cpu.state().hx_known == expected.hx_known && cpu.state().hx == (seed_hx & expected.hx_known),
      "Declared HX knowledge is preserved; an unknown representative is canonical, not observed");
  trace << "ENTRY phase=" << expected.phase << " hx-known=" << cpu.state().hx_known
        << " a-known=" << static_cast<unsigned>(cpu.state().a_known)
        << " ccr-known=" << static_cast<unsigned>(cpu.state().ccr_known) << '\n';
  NoDebugger debugger;
  Hcs08Bdc bdc(cpu, debugger, chip, Hcs08BdcReset::normal);
  Hcs08RunClock clock(memory, bdc, periods_contract);
  advance_to(memory, cpu, clock, 10);
  require(
      cpu.state().hx_known == expected.hx_known && memory.high_reads == 0 &&
          memory.low_reads == 0 && clock.state().remaining_periods == 1,
      "MOV/BRA and the uncommitted LDHX budget cannot manufacture HX knowledge or a counter read");
  advance_to(memory, cpu, clock, 11);
  require(cpu.state().hx_known == Hcs08State::all_hx &&
              cpu.state().hx == (expected.phase == 0 ? 0 : 1) &&
              cpu.state().a_known == Hcs08State::all_a && cpu.state().ccr_known == known_flags &&
              memory.high_reads == 1 && memory.low_reads == 1,
          "Only the actual first coherent TPM word load defines HX, preserving A and unknown H");
  std::vector<Saved> cuts;
  constexpr std::array<unsigned, 2> distance{13, 4};
  constexpr std::array<unsigned, 2> remaining{3, 1};
  for (unsigned n = 0; n < distance.size(); ++n) {
    advance_to(memory, cpu, clock, expected.periods - distance[n]);
    require(clock.state().remaining_periods == remaining[n],
            "Joint snapshots occur during actual original SRS-store/CPHX budgets");
    cuts.push_back({.memory = memory,
                    .cpu = cpu.checkpoint(),
                    .bdc = bdc.checkpoint(),
                    .clock = clock.checkpoint()});
  }
  std::ostringstream reference;
  finish(memory, cpu, clock, expected, reference);
  trace << reference.str();
  for (unsigned cut = 0; cut < cuts.size(); ++cut) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      Memory restored(cuts[cut].memory);
      Hcs08 other(restored, BLOFELD_HCS08_SLA);
      other.restore_checkpoint(cuts[cut].cpu);
      NoDebugger other_debugger;
      Hcs08Bdc other_bdc(other, other_debugger, chip, Hcs08BdcReset::normal);
      other_bdc.restore_checkpoint(cuts[cut].bdc);
      Hcs08RunClock fresh(restored, other_bdc, periods_contract);
      fresh.restore_checkpoint(cuts[cut].clock);
      std::ostringstream actual;
      finish(restored, other, fresh, expected, actual);
      require(
          actual.str() == reference.str() && other.state() == cpu.state() &&
              fresh.state() == clock.state() && other_bdc.state() == bdc.state() &&
              restored.timer.state() == memory.timer.state() &&
              restored.cop.state() == memory.cop.state() && restored.flash.image() == image &&
              memory.flash.image() == image && restored.bytes == memory.bytes &&
              restored.known == memory.known && restored.services == memory.services &&
              restored.high_reads == memory.high_reads && restored.low_reads == memory.low_reads &&
              restored.control_writes == memory.control_writes,
          "Fresh joint CPU/FLASH/RAM/TPM/COP/BDC/clock owners reproduce the complete fault suffix");
      require(restored.spi.state() == memory.spi.state() &&
                  restored.spi.faulted() == memory.spi.faulted() &&
                  restored.setup_writes == memory.setup_writes &&
                  restored.flash.state() == memory.flash.state(),
              "Fresh SPI/FLASH owners retain the complete setup without shared buffers or peers");
      trace << "REPLAY phase=" << expected.phase << " cut=" << cut << " index=" << replay
            << " identical=1\n";
    }
  }
  require(bdc.state().serviced == 0, "No BDC host command supplies timer-loop behavior");
}

void expired_entry(const Hcs08FlashImage &image, std::ostream &trace) {
  Memory memory(image, 0);
  require(memory.cop.advance_bus(long_timeout - deadline_period) == long_timeout - deadline_period,
          "The failing prehistory leaves only twenty bus periods before reset");
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = entry,
                 .hx = seed_hx,
                 .sp = stack,
                 .a = accumulator,
                 .ccr_known = known_flags,
                 .ccr = initial_ccr});
  NoDebugger debugger;
  Hcs08Bdc bdc(cpu, debugger, chip, Hcs08BdcReset::normal);
  Hcs08RunClock clock(memory, bdc, periods_contract);
  const auto error = rejected([&] { advance_to(memory, cpu, clock, deadline_period + 1); });
  constexpr unsigned service_pc = 0xe1c7;
  constexpr unsigned pending_flags = 0x6d;
  require(error.contains("COP timeout address=0x1800 pc=0xe1c7 ccr=0x006d") &&
              cpu.state() == Hcs08State{.pc = service_pc,
                                        .hx = 0,
                                        .sp = stack,
                                        .a = accumulator,
                                        .ccr_known = known_flags,
                                        .ccr = pending_flags} &&
              !cpu.faulted() && !clock.faulted() &&
              clock.state().offered_periods == deadline_period - 1 &&
              clock.state().remaining_periods == 2 && memory.retired == 5 && memory.services == 0 &&
              memory.cop.state() ==
                  Hcs08CopState{.elapsed = long_timeout, .reset_requested = true} &&
              memory.timer.state().counter == 0 && memory.timer.state().prescaler_phase == 16 &&
              memory.known.none() && memory.flash.image() == image,
          "COP requests reset before the first original SRS store can retire; no success is "
          "fabricated");
  const auto timer = memory.timer.state();
  for (unsigned retry = 0; retry < 2; ++retry) {
    require(
        rejected([&] {
          advance_to(memory, cpu, clock, deadline_period + 1);
        }).contains("pending COP reset") &&
            memory.timer.state() == timer && memory.retired == 5 && memory.services == 0,
        "The board admission boundary refuses further clock/CPU work pending coordinated reset");
  }
  trace << "DEADLINE periods=" << deadline_period << " offered=" << clock.state().offered_periods
        << " retired=" << memory.retired << " pc=" << cpu.state().pc
        << " remaining=" << clock.state().remaining_periods << " services=" << memory.services
        << '\n';
  trace << "ERROR " << error << '\n';
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected supplied original page records and private trace");
    const auto records = blofeld::test::hcs08_programmer::read_input(arguments[1], supplied_bytes);
    const auto image = original_image(records);
    std::ostringstream trace;
    for (const auto expected : expectations) { run(image, expected, trace); }
    expired_entry(image, trace);
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout
        << "Original timer wait reaches its threshold through real TPM counts and COP services.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
