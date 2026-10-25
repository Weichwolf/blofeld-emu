// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_run_clock.hpp"
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
using blofeld::test::hcs08::Access;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
namespace hp = blofeld::test::hcs08_programmer;
namespace hf = hcs08_flash;
namespace hi = hcs08_icg;
constexpr unsigned byte_mask = 255, page_bytes = 512, record_bytes = 514;
constexpr unsigned application = 0xe000, gpio_pc = 0xe04e, trim_pc = 0xe1a8, status_pc = 0xe1b0;
constexpr unsigned gpio_retired = 6176, before_trim = 6185, after_trim = 6186,
                   after_controls = 6188;
constexpr unsigned startup_sp = 0x47d, after_frame_sp = 0x475, expected_hx = 0x12c;
constexpr unsigned warm_pc = 0x1234, warm_hx = 0x3322, warm_sp = 0x4567, warm_a = 0xa5;
constexpr unsigned packaged_trim = 0x80, previous_trim = 0x5a, known_flags = 0xef;
constexpr unsigned gpio_ccr = 0x6c, trim_ccr = 0x6d, request_ccr = 0x69, refs_known = 0xdf;
constexpr unsigned gpio_input = 8, supplied_bytes = 3598, reset_ccr = 0x68;
constexpr std::array pages{0xe000U, 0xe200U, 0xe400U, 0xe600U, 0xe800U, 0xea00U, 0xfe00U};
constexpr auto chip = Hcs08FlashContract::candidate_gt8a;
constexpr unsigned gpio_periods = 22630, loaded_trim_periods = 22659, trim_periods = 22662;
constexpr unsigned controls_periods = 22670, fault_periods = 22673;
constexpr auto period_contract = Hcs08RunContract::reference_instruction_retirement;

Hcs08FlashImage original_image(const std::span<const std::uint8_t> records) {
  Hcs08FlashImage image;
  for (unsigned n = 0; n < pages.size(); ++n) {
    const auto offset = n * record_bytes;
    require((static_cast<unsigned>(records[offset]) << 8U | records[offset + 1]) == pages[n],
            "Pinned original page addresses remain exact");
    for (unsigned k = 0; k < page_bytes; ++k) {
      const auto address = pages[n] - hf::first + k;
      image.data[address] = records[offset + 2 + k];
      image.known.set(address);
    }
  }
  require(image.known.count() == pages.size() * page_bytes && image.fresh.none(),
          "Only supplied pages are known; no erased history or omitted bytes are invented");
  return image;
}

class Memory final : public hp::Memory {
public:
  Hcs08Gpio gpio{Hcs08GpioContract::candidate_gt8a};
  std::vector<Access> gpio_accesses;

  Memory(const Hcs08FlashImage &image, const int seed)
      : hp::Memory(Hcs08IdleLaunch::unqualified, 5) {
    flash = Hcs08Flash(image, chip); // Discard the programmer fixture's separate6MHz input.
    bytes.fill(warm_a);
    known.reset();
    if (seed >= 0) { icg.write({.address = hi::trim, .value = static_cast<std::uint8_t>(seed)}); }
    warm_reset();
  }

  void warm_reset() {
    reset_peripherals();
    gpio.reset();
  }

  std::uint8_t read(const std::uint16_t address) override {
    if (resetting || !Hcs08Gpio::contains(address)) { return hp::Memory::read(address); }
    const auto value = gpio.read(address, context);
    gpio_accesses.push_back({.address = address, .value = value, .write = false});
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (resetting) {
      hp::Memory::write(address, value);
      return;
    }
    if (Hcs08Gpio::contains(address)) {
      gpio.write({.address = address, .value = value}, context);
      gpio_accesses.push_back({.address = address, .value = value, .write = true});
    } else {
      hp::Memory::write(address, value);
    }
  }

  void check_ram_write(const std::uint16_t address) const override {
    require(address >= hp::ram_first && address < hp::ram_end,
            "HCS08 application write address=" + hex(address) + " pc=" + hex(context.pc) +
                " ccr=" + hex(context.ccr) + ": outside declared RAM");
  }

  std::uint8_t peek(const std::uint16_t address) const override {
    return address >= hf::first ? flash.read_array(address) : hp::Memory::peek(address);
  }

  void check_execute(const InstructionFetch fetch) const override {
    if (resetting || fetch.address < hf::first ||
        fetch.length > blofeld::test::hcs08::space_bytes - fetch.address) {
      throw Fault("HCS08 application fetch address=" + hex(fetch.address) +
                  " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr) +
                  ": requires readable FLASH outside reset");
    }
    for (unsigned n = 0; n < fetch.length; ++n) {
      static_cast<void>(flash.read_array(static_cast<std::uint16_t>(fetch.address + n)));
    }
  }
};

void fetch_controls(const Hcs08FlashImage &image) {
  Memory memory(image, -1);
  memory.begin_instruction({.pc = application, .ccr = reset_ccr});
  constexpr unsigned missing = 0xec00;
  constexpr unsigned last = 0xffff;
  for (const auto fetch : std::array{InstructionFetch{.address = hp::ram_first, .length = 1},
                                     InstructionFetch{.address = missing, .length = 1},
                                     InstructionFetch{.address = last, .length = 2}}) {
    const auto error = rejected([&] { memory.check_execute(fetch); });
    require(error.contains("address=" + hex(fetch.address)) && error.contains("pc=0xe000") &&
                error.contains("ccr=0x0068") && memory.retired == 0 &&
                memory.flash.image() == image,
            "RAM-profile, omitted FLASH and wrapped fetches retain address/op/PC/context without "
            "effects");
  }
}

void bind(Memory &memory, const int observation) {
  if (observation < 0) { return; }
  memory.gpio.attach_pad_source(Hcs08GpioPort::e, [observation](const unsigned mask) {
    return GpioPadSample{.known = mask, .levels = static_cast<unsigned>(observation) & mask};
  });
}

void expect_ram(const Memory &memory, const bool entered) {
  constexpr unsigned calibration = 0xc1;
  constexpr unsigned low_calibration = 44;
  constexpr unsigned iterator_low = 0x80;
  constexpr unsigned return_high = 0xe0;
  constexpr unsigned return_low = 0x5e;
  std::array<std::uint8_t, hp::ram_end - hp::ram_first> expected{};
  expected[0] = expected[6] = 7;
  expected[1] = expected[7] = byte_mask;
  expected[calibration - hp::ram_first] = 1;
  expected[calibration - hp::ram_first + 1] = low_calibration;
  expected[expected.size() - 2] = 4;
  expected[expected.size() - 1] = iterator_low;
  if (entered) {
    expected[startup_sp - hp::ram_first] = return_low;
    expected[startup_sp - hp::ram_first - 1] = return_high;
  }
  for (unsigned n = 0; n < expected.size(); ++n) {
    require(memory.known[hp::ram_first + n] && memory.bytes[hp::ram_first + n] == expected[n],
            "Every initialized RAM, iterator, calibration and return-stack byte has an oracle");
  }
}

void expect_setup(const Memory &memory, const Hcs08 &cpu) {
  constexpr unsigned control1 = 0x28;
  constexpr unsigned control2 = 0x30;
  require(cpu.state() == Hcs08State{.pc = status_pc,
                                    .hx = expected_hx,
                                    .sp = after_frame_sp,
                                    .a = packaged_trim,
                                    .ccr_known = known_flags,
                                    .ccr = request_ccr} &&
              memory.retired == after_controls,
          "Complete independent original endpoint before live ICGS1 read");
  require(memory.icg.state() == Hcs08IcgState{.control1_request = control1,
                                              .control1_known = refs_known,
                                              .control2_request = control2,
                                              .trim = packaged_trim,
                                              .filter = std::nullopt,
                                              .control1_written = true,
                                              .control2_written = true},
          "Actual software requests retain unknown REFS/filter and establish no clock status");
  constexpr std::array<Access, 3> expected{
      {{.address = hi::trim, .value = packaged_trim, .write = true},
       {.address = hi::control1, .value = control1, .write = true},
       {.address = hi::control2, .value = control2, .write = true}}};
  std::vector<Access> writes;
  for (const auto access : memory.accesses) {
    if (access.address >= hi::control1 && access.address <= hi::trim) { writes.push_back(access); }
  }
  require(writes == std::vector<Access>(expected.begin(), expected.end()),
          "Independent native address/value/order oracle checks all three ICG writes");
  expect_ram(memory, true);
}

void finish(Memory &memory, Hcs08 &cpu, const int observation, std::ostream &trace) {
  if (observation >= 0) {
    while (memory.retired < after_controls) { cpu.step(); }
    expect_setup(memory, cpu);
  }
  const auto clock = memory.icg.state();
  const auto image = memory.flash.image();
  const auto error = rejected([&] { cpu.step(); });
  require(error.contains(observation < 0 ? "GPIO read address=0x0010 pc=0xe04e"
                                         : "ICG read address=0x004a pc=0xe1b0") &&
              cpu.faulted() && memory.icg.state() == clock && memory.flash.image() == image &&
              memory.flash.state() == Hcs08FlashState{} && memory.transfers == 0 &&
              memory.completions == 0 && memory.array_writes == 0 && memory.command_writes == 0 &&
              !memory.transfer_due && !memory.complete_due,
          "Actual unsupported input/status read cannot invent clocks or change/complete NVM");
  const auto failed = cpu.state();
  require(rejected([&] { cpu.step(); }).contains("coordinated recovery") && cpu.state() == failed,
          "Faulted native execution remains quarantined");
  trace << "ERROR " << error << '\n';
  trace << "FAULT retired=" << memory.retired << " pc=" << cpu.state().pc << '\n';
}

struct Saved {
  Memory memory;
  Hcs08Checkpoint cpu;
};

void reset_check(const Saved &saved, const Hcs08FlashImage &image) {
  Memory memory(saved.memory);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.restore_checkpoint(saved.cpu);
  const auto ram = memory.bytes;
  const auto known = memory.known;
  memory.warm_reset();
  cpu.reset();
  require(memory.flash.image() == image && memory.bytes == ram && memory.known == known &&
              memory.icg.state() == Hcs08IcgState{.trim = packaged_trim} &&
              memory.gpio.state(Hcs08GpioPort::e) == Hcs08GpioRegisters{} &&
              cpu.state() == Hcs08State{.pc = application,
                                        .hx = expected_hx & byte_mask,
                                        .sp = byte_mask,
                                        .a = packaged_trim,
                                        .ccr_known = reset_ccr,
                                        .ccr = reset_ccr} &&
              memory.retired == after_controls,
          "Coordinated warm GPIO/ICG/NVM/CPU reset retains actual trim and RAM, not old requests");
}

void run(const Hcs08FlashImage &image, const int seed, const int observation, std::ostream &trace) {
  Memory memory(image, seed);
  bind(memory, observation);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = warm_pc, .hx = warm_hx, .sp = warm_sp, .a = warm_a, .ccr = byte_mask});
  cpu.reset();
  const auto retained =
      seed < 0 ? std::nullopt : std::optional<std::uint8_t>(static_cast<std::uint8_t>(seed));
  require(memory.icg.state() == Hcs08IcgState{.trim = retained} && memory.known.none() &&
              memory.flash.image() == image && cpu.state().pc == application,
          "Normal reset reads actual FLASH vector, preserves unknown RAM and actual live trim");
  trace << "CASE seed=" << seed << " observation=" << observation << '\n';
  for (unsigned n = 0; n < gpio_retired; ++n) { cpu.step(); }
  require(cpu.state() == Hcs08State{.pc = gpio_pc,
                                    .hx = expected_hx,
                                    .sp = startup_sp,
                                    .a = 3,
                                    .ccr_known = known_flags,
                                    .ccr = gpio_ccr} &&
              memory.gpio_accesses.size() == 11,
          "Original GPIO configuration retains independent complete CPU state through FLASH fetch");
  expect_ram(memory, false);
  std::vector<Saved> cuts;
  cuts.push_back({.memory = memory, .cpu = cpu.checkpoint()});
  if (observation >= 0) {
    while (memory.retired < after_trim) { cpu.step(); }
    require(memory.icg.state() == Hcs08IcgState{.trim = packaged_trim} &&
                cpu.state().pc == trim_pc + 2 && cpu.state().ccr == trim_ccr &&
                memory.retired == before_trim + 1,
            "Only the actual trim store replaces unknown/retained live trim with packaged80");
    cuts.push_back({.memory = memory, .cpu = cpu.checkpoint()});
    while (memory.retired < after_controls) { cpu.step(); }
    expect_setup(memory, cpu);
    cuts.push_back({.memory = memory, .cpu = cpu.checkpoint()});
    reset_check(cuts.back(), image);
    trace << "RESET trim=" << packaged_trim << " pc=" << application
          << " retired=" << after_controls << '\n';
  }
  std::ostringstream expected;
  finish(memory, cpu, observation, expected);
  trace << expected.str();
  for (unsigned cut = 0; cut < cuts.size(); ++cut) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      Memory restored(cuts[cut].memory);
      bind(restored, observation);
      Hcs08 other(restored, BLOFELD_HCS08_SLA);
      other.restore_checkpoint(cuts[cut].cpu);
      std::ostringstream actual;
      finish(restored, other, observation, actual);
      require(actual.str() == expected.str() && other.state() == cpu.state() &&
                  restored.bytes == memory.bytes && restored.known == memory.known &&
                  restored.flash.image() == image && restored.icg.state() == memory.icg.state() &&
                  restored.accesses == memory.accesses &&
                  restored.gpio_accesses == memory.gpio_accesses,
              "Fresh CPU/RAM/NVM/GPIO/ICG owners reproduce every partial-knowledge fault suffix");
      trace << "REPLAY cut=" << cut << " index=" << replay << " identical=1\n";
    }
  }
}

void advance_to(Memory &memory, const Hcs08 &cpu, Hcs08RunClock &clock, const unsigned periods) {
  while (clock.state().executing_periods < periods) {
    clock.advance();
    if (clock.state().executing_periods == gpio_periods) {
      require(memory.retired == gpio_retired && cpu.state().pc == gpio_pc,
              "1024 original RAM-clear iterations and calibration/GPIO cost22630 bus periods");
      expect_ram(memory, false);
    }
    if (clock.state().executing_periods == loaded_trim_periods) {
      require(memory.retired == before_trim && cpu.state().pc == trim_pc &&
                  memory.icg.state() == Hcs08IcgState{.trim = std::nullopt},
              "Actual original trim load completes at22659; the live register is still unknown");
    }
    if (clock.state().executing_periods == trim_periods) {
      require(memory.retired == after_trim && cpu.state().pc == trim_pc + 2 &&
                  memory.icg.state() == Hcs08IcgState{.trim = packaged_trim},
              "Actual trim store commits at22662, after its full three-period budget");
    }
  }
}

void finish_periods(Memory &memory, Hcs08 &cpu, Hcs08RunClock &clock, std::ostream &trace) {
  advance_to(memory, cpu, clock, controls_periods);
  expect_setup(memory, cpu);
  const auto registers = cpu.state();
  const auto icg = memory.icg.state();
  const auto accesses = memory.accesses;
  advance_to(memory, cpu, clock, fault_periods - 1);
  require(cpu.state() == registers && memory.icg.state() == icg && memory.accesses == accesses &&
              clock.state().remaining_periods == 1,
          "Two status-load periods have no early architectural or peripheral-read effects");
  const auto error = rejected([&] { clock.advance(); });
  require(
      error.contains("ICG read address=0x004a pc=0xe1b0 ccr=0x0069") &&
          error.contains("live clock/status evolution is unqualified") && cpu.faulted() &&
          clock.faulted() && clock.state().executing_periods == fault_periods &&
          clock.state().offered_periods == fault_periods &&
          clock.state().retired_instructions == after_controls &&
          clock.state().remaining_periods == 0 && cpu.state() == registers &&
          memory.icg.state() == icg && memory.accesses == accesses &&
          rejected([&] { clock.advance(); }).contains("coordinated recovery") &&
          rejected([&] { static_cast<void>(clock.checkpoint()); }).contains("coordinated recovery"),
      "Actual ICG status read faults at22673 without retirement, mutation or successful retry");
  trace << "ERROR " << error << '\n'
        << "FAULT periods=" << fault_periods << " offered=" << clock.state().offered_periods
        << " retired=" << memory.retired << " pc=" << cpu.state().pc
        << " remaining=" << clock.state().remaining_periods << '\n';
}

struct SavedPeriods {
  Memory memory;
  Hcs08Checkpoint cpu;
  Hcs08BdcCheckpoint bdc;
  Hcs08RunCheckpoint clock;
};

void run_periods(const Hcs08FlashImage &image, std::ostream &trace) {
  Memory memory(image, -1);
  bind(memory, gpio_input);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = warm_pc, .hx = warm_hx, .sp = warm_sp, .a = warm_a});
  cpu.reset();
  blofeld::test::hcs08_bdc::Memory mapping(memory);
  Hcs08Bdc bdc(cpu, mapping, chip, Hcs08BdcReset::normal);
  Hcs08RunClock clock(memory, bdc, period_contract);
  std::vector<SavedPeriods> cuts;
  constexpr std::array<unsigned, 3> cut_periods{1, 22664, 22671};
  for (const auto period : cut_periods) {
    advance_to(memory, cpu, clock, period);
    require(clock.state().remaining_periods == 2,
            "Fresh joint cuts retain real partial LDHX/MOV/status-load budgets");
    cuts.push_back({.memory = memory,
                    .cpu = cpu.checkpoint(),
                    .bdc = bdc.checkpoint(),
                    .clock = clock.checkpoint()});
  }
  trace << "TIMED gpio=" << gpio_periods << " trim=" << trim_periods
        << " controls=" << controls_periods << '\n';
  std::ostringstream expected;
  finish_periods(memory, cpu, clock, expected);
  trace << expected.str();
  for (unsigned cut = 0; cut < cuts.size(); ++cut) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      Memory restored(cuts[cut].memory);
      bind(restored, gpio_input);
      Hcs08 other(restored, BLOFELD_HCS08_SLA);
      other.restore_checkpoint(cuts[cut].cpu);
      blofeld::test::hcs08_bdc::Memory other_mapping(restored);
      Hcs08Bdc other_bdc(other, other_mapping, chip, Hcs08BdcReset::normal);
      other_bdc.restore_checkpoint(cuts[cut].bdc);
      Hcs08RunClock fresh(restored, other_bdc, period_contract);
      fresh.restore_checkpoint(cuts[cut].clock);
      std::ostringstream actual;
      finish_periods(restored, other, fresh, actual);
      require(
          actual.str() == expected.str() && other.state() == cpu.state() &&
              fresh.state() == clock.state() && other_bdc.state() == bdc.state() &&
              restored.bytes == memory.bytes && restored.known == memory.known &&
              restored.flash.image() == image && restored.options.read() == memory.options.read() &&
              restored.options.written() == memory.options.written() &&
              restored.icg.state() == memory.icg.state() &&
              restored.gpio.state(Hcs08GpioPort::e) == memory.gpio.state(Hcs08GpioPort::e) &&
              restored.accesses == memory.accesses &&
              restored.gpio_accesses == memory.gpio_accesses,
          "Fresh CPU/RAM/FLASH/GPIO/ICG/BDC/clock owners preserve partial-period fault suffixes");
      trace << "REPLAY cut=" << cut << " index=" << replay << " periods=" << cut_periods[cut]
            << " identical=1\n";
    }
  }
  require(memory.flash.image() == image && memory.transfers == 0 && memory.completions == 0 &&
              bdc.state().serviced == 0 && !bdc.state().active,
          "Bus-period reference supplies neither a physical rate, NVM completion nor BDC command");
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 3 || (count == 4 && std::string(arguments[3]) == "periods"),
            "Expected original supplied page records, private trace and optional periods mode");
    const auto records = hp::read_input(arguments[1], supplied_bytes);
    const auto image = original_image(records);
    std::ostringstream trace;
    if (count == 4) {
      run_periods(image, trace);
    } else {
      fetch_controls(image);
      trace << "FETCH_CONTROLS count=3 retired=0\n";
      run(image, -1, -1, trace);
      run(image, -1, gpio_input, trace);
      run(image, previous_trim, gpio_input, trace);
    }
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout
        << "Original HCS08 FLASH/GPIO/ICG setup retains unknown status and actual trim history.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
