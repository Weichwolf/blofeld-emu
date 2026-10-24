// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/gpio_pads.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_gpio.hpp"
#include "hcs08_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned ram_first = 0x80, ram_end = 0x480, page_size = 512, record_size = 514;
constexpr unsigned application = 0xe000, gpio_read_pc = 0xe04e, gpio_read_retired = 6176;
constexpr unsigned background_pc = 0xe060, background_retired = 6181, trim_retired = 6185;
constexpr unsigned inputs = 0x3c, strap_mask = 0x38, port_e_address = 0x10, clock_trim = 0x4e;
constexpr unsigned trim_pc = 0xe1a8;
constexpr unsigned trim_address = 0xffbe, expected_hx = 0x12c, startup_sp = 0x47d;
constexpr unsigned previous_pc = 0x1234, previous_hx = 0x3322, previous_sp = 0x4567,
                   previous_a = 0xa5;
constexpr unsigned initialized_known = 0xef, negative_ccr = 0x6c, zero_ccr = 0x6a;
constexpr std::array pages{0xe000U, 0xe200U, 0xe400U, 0xe600U, 0xe800U, 0xea00U, 0xfe00U};

class Memory final : public blofeld::test::hcs08::Memory {
public:
  Hcs08Gpio gpio{Hcs08GpioContract::candidate_gt8a};
  std::vector<Access> gpio_accesses;

  std::uint8_t read(const std::uint16_t address) override {
    if (!Hcs08Gpio::contains(address)) { return blofeld::test::hcs08::Memory::read(address); }
    const auto value = gpio.read(address, context);
    gpio_accesses.push_back({.address = address, .value = value, .write = false});
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (!Hcs08Gpio::contains(address)) {
      blofeld::test::hcs08::Memory::write(address, value);
      return;
    }
    gpio.write({.address = address, .value = value}, context);
    gpio_accesses.push_back({.address = address, .value = value, .write = true});
  }
};

void install(Memory &memory, const std::span<const std::uint8_t> records) {
  for (unsigned n = ram_first; n < ram_end; ++n) { memory.writable.set(n); }
  for (unsigned n = 0; n < pages.size(); ++n) {
    const auto offset = n * record_size;
    require((static_cast<unsigned>(records[offset]) << 8U | records[offset + 1]) == pages[n],
            "Every pinned original page keeps its declared address");
    memory.install(pages[n], records.subspan(offset + 2, page_size));
  }
}

void ram_contents(const Memory &memory, const bool entered_main) {
  constexpr unsigned calibration = 0xc1;
  constexpr unsigned calibration_low = 44;
  constexpr unsigned return_high = 0xe0;
  constexpr unsigned return_low = 0x5e;
  std::array<std::uint8_t, ram_end - ram_first> expected{};
  expected[0] = expected[6] = 7;
  expected[1] = expected[7] = byte_mask;
  expected[calibration - ram_first] = 1;
  expected[calibration - ram_first + 1] = calibration_low;
  expected[expected.size() - 2] = 4;
  expected[expected.size() - 1] = 0x80;
  if (entered_main) {
    expected[startup_sp - ram_first] = return_low;
    expected[startup_sp - ram_first - 1] = return_high;
  }
  for (unsigned n = 0; n < expected.size(); ++n) {
    require(
        memory.known[ram_first + n] && memory.bytes[ram_first + n] == expected[n],
        "Every native initialized RAM byte, iterator, calibration and return-stack byte agrees");
  }
}

void configured(const Memory &memory) {
  constexpr std::array<Access, 11> writes{{{.address = 16, .value = 3, .write = true},
                                           {.address = 19, .value = 3, .write = true},
                                           {.address = 1, .value = 255, .write = true},
                                           {.address = 3, .value = 0, .write = true},
                                           {.address = 0, .value = 0, .write = true},
                                           {.address = 4, .value = 248, .write = true},
                                           {.address = 8, .value = 255, .write = true},
                                           {.address = 12, .value = 255, .write = true},
                                           {.address = 7, .value = 248, .write = true},
                                           {.address = 11, .value = 255, .write = true},
                                           {.address = 15, .value = 255, .write = true}}};
  constexpr std::array<Hcs08GpioRegisters, 5> expected{{{.pull_enable = 255},
                                                        {.latch = 248, .direction = 248},
                                                        {.latch = 255, .direction = 255},
                                                        {.latch = 31, .direction = 31},
                                                        {.latch = 3, .direction = 3}}};
  require(memory.gpio_accesses == std::vector<Access>(writes.begin(), writes.end()),
          "All11 original GPIO writes have independent address/value/order expectations");
  for (unsigned n = 0; n < expected.size(); ++n) {
    require(memory.gpio.state(static_cast<Hcs08GpioPort>(n)) == expected[n],
            "Every port latch/direction/pull/slew field matches original startup effects");
  }
  for (unsigned n = ram_first; n < ram_end; ++n) {
    require(memory.known[n], "Original startup establishes every initially unknown RAM byte");
  }
}

void bind(Memory &memory, const int observation) {
  if (observation >= 0) {
    memory.gpio.attach_pad_source(Hcs08GpioPort::e, [observation](const unsigned mask) {
      return GpioPadSample{.known = mask, .levels = static_cast<unsigned>(observation) & mask};
    });
  }
}

void finish(Memory &memory, Hcs08 &cpu, const int observation, std::ostream &trace) {
  if (observation < 0) {
    const auto error = rejected([&] { cpu.step(); });
    require(error.contains("GPIO read address=0x0010 pc=0xe04e") && error.contains("Detached") &&
                cpu.faulted() && memory.retired == gpio_read_retired,
            "Absent external straps refuse at actual PTE input read without inventing ready bits");
    const auto failed = cpu.state();
    const auto accesses = memory.gpio_accesses;
    require(rejected([&] { cpu.step(); }).contains("coordinated recovery") &&
                cpu.state() == failed && memory.gpio_accesses == accesses,
            "Unknown-pad failure quarantines native execution");
    trace << "UNKNOWN " << error << '\n';
    return;
  }
  const bool start = (static_cast<unsigned>(observation) & strap_mask) != 0;
  const unsigned end = start ? trim_retired : background_retired;
  while (memory.retired < end) { cpu.step(); }
  require(memory.gpio_accesses.back() == Access{.address = port_e_address,
                                                .value = static_cast<unsigned>(observation) | 3U,
                                                .write = false},
          "Actual native read combines supplied input bits with the two output latch bits");
  if (!start) {
    require(
        cpu.background_requested() &&
            cpu.state() == Hcs08State{.pc = background_pc,
                                      .hx = expected_hx,
                                      .sp = startup_sp,
                                      .a = static_cast<std::uint8_t>(observation | 3),
                                      .ccr_known = initialized_known,
                                      .ccr = zero_ccr},
        "Zero strap mask follows actual BIT/BEQ/BRA/overlapping BGND with independent full state");
    ram_contents(memory, false);
    trace << "BACKGROUND observation=" << observation << " retired=" << memory.retired << '\n';
    return;
  }
  constexpr unsigned packaged_trim = 0x80;
  constexpr unsigned loaded_trim_ccr = 0x6d;
  require(cpu.state() == Hcs08State{.pc = trim_pc,
                                    .hx = expected_hx,
                                    .sp = startup_sp - 8,
                                    .a = packaged_trim,
                                    .ccr_known = initialized_known,
                                    .ccr = loaded_trim_ccr} &&
              memory.bytes[trim_address] == packaged_trim,
          "Independent full state: native JSR/AIS stack and actual supplied packaged trim load");
  ram_contents(memory, true);
  const auto pc = cpu.state().pc;
  const auto error = rejected([&] { cpu.step(); });
  require(error.contains("write address=" + hex(clock_trim)) && error.contains("pc=" + hex(pc)) &&
              memory.retired == trim_retired && cpu.faulted(),
          "Nonzero declared straps reach the actual unmodeled clock-trim store");
  trace << "CLOCK observation=" << observation << " retired=" << memory.retired << " pc=" << pc
        << " address=" << clock_trim << '\n';
}

void run(const std::span<const std::uint8_t> records, const int observation, std::ostream &trace) {
  require(observation < 0 || (static_cast<unsigned>(observation) & ~inputs) == 0,
          "Declared original GPIO observations contain input bits only");
  Memory memory;
  install(memory, records);
  bind(memory, observation);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(
      {.pc = previous_pc, .hx = previous_hx, .sp = previous_sp, .a = previous_a, .ccr = byte_mask});
  cpu.reset();
  require(cpu.state().pc == application && cpu.state().ccr_known == flags_fixed + flag_i,
          "Actual warm reset reads the original vector and keeps unknown flags");
  for (unsigned n = 0; n < gpio_read_retired; ++n) { cpu.step(); }
  configured(memory);
  ram_contents(memory, false);
  require(cpu.state() == Hcs08State{.pc = gpio_read_pc,
                                    .hx = expected_hx,
                                    .sp = startup_sp,
                                    .a = 3,
                                    .ccr_known = initialized_known,
                                    .ccr = negative_ccr},
          "Independent complete state before first actual PTE input read, after all11 GPIO writes");
  trace << "CONFIG observation=" << observation << " retired=" << memory.retired
        << " pc=" << cpu.state().pc << " writes=" << memory.gpio_accesses.size() << '\n';
  const auto saved_cpu = cpu.checkpoint();
  const auto saved_memory = memory;
  std::ostringstream expected;
  finish(memory, cpu, observation, expected);
  trace << expected.str();
  if (observation != -1 && observation != 0 && observation != 8) { return; }
  for (unsigned replay = 0; replay < 2; ++replay) {
    Memory restored(saved_memory);
    bind(restored, observation);
    Hcs08 other(restored, BLOFELD_HCS08_SLA);
    other.restore_checkpoint(saved_cpu);
    std::ostringstream actual;
    finish(restored, other, observation, actual);
    require(actual.str() == expected.str() && other.state() == cpu.state() &&
                restored.retired == memory.retired && restored.bytes == memory.bytes &&
                restored.known == memory.known && restored.accesses == memory.accesses &&
                restored.gpio_accesses == memory.gpio_accesses,
            "Fresh native CPU/RAM/GPIO owners reproduce the complete suffix and endpoint");
    for (unsigned n = 0; n < 5; ++n) {
      const auto port = static_cast<Hcs08GpioPort>(n);
      require(restored.gpio.state(port) == memory.gpio.state(port),
              "All restored GPIO registers retain actual native effects");
    }
    trace << "REPLAY observation=" << observation << " index=" << replay << " identical=1\n";
  }
}

} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned pages and private trace");
    std::vector<std::uint8_t> records(pages.size() * record_size);
    std::ifstream input(arguments[1], std::ios::binary);
    input.read(reinterpret_cast<char *>(records.data()),
               static_cast<std::streamsize>(records.size()));
    require(input.gcount() == static_cast<std::streamsize>(records.size()) &&
                input.peek() == std::char_traits<char>::eof(),
            "Original page records retain their exact extent");
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    run(records, -1, trace);
    for (unsigned bits = 0; bits < 16; ++bits) {
      run(records, static_cast<int>(bits << 2U), trace);
    }
    std::cout << "Original HCS08 GPIO startup preserves unknown pads and actual strap branches.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
