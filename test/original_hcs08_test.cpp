// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned ram_first = 0x80, ram_end = 0x480, canary = 0xa5;
constexpr unsigned program_start = 0xa0, program_size = 132, flash_divider = 0x1820;
constexpr unsigned divider_write_pc = 0xa5, divider_value = 30, program_stack_low = 0x77;
constexpr unsigned reset_vector = 0xfffe, application_entry = 0xe000;
constexpr unsigned gpio_write_pc = 0xe030, port_e = 0x10, calibration = 0xc1;
constexpr unsigned startup_retired = 6165, expected_writes = 3084, expected_reads = 2048;
constexpr unsigned calibration_batches = 300, calibration_low = 44;
constexpr unsigned page_size = 512, record_size = page_size + 2;
constexpr std::array pages{0xe000U, 0xe200U, 0xe400U, 0xe600U, 0xe800U, 0xea00U, 0xfe00U};
constexpr unsigned reset_ccr = 0x68, initialized_ccr_known = 0xef;
enum class StartupEntry : std::uint8_t { debug, normal_reset };

std::vector<std::uint8_t> input(const char *path, const unsigned size) {
  std::ifstream stream(path, std::ios::binary);
  std::vector<std::uint8_t> data(size);
  stream.read(reinterpret_cast<char *>(data.data()), static_cast<std::streamsize>(size));
  require(stream.gcount() == static_cast<std::streamsize>(size) &&
              stream.peek() == std::char_traits<char>::eof(),
          "Pinned original HCS08 input has exactly its declared extent");
  return data;
}

void ram(Memory &bus) {
  for (unsigned n = ram_first; n < ram_end; ++n) {
    bus.writable.set(n);
    bus.known.set(n);
    bus.bytes[n] = canary;
  }
}

void quarantine(Hcs08 &cpu, Memory &bus) {
  const auto failed = cpu.state();
  const auto events = bus.accesses;
  require(cpu.faulted() && rejected([&] { cpu.step(); }).contains("coordinated recovery") &&
              cpu.state() == failed && bus.accesses == events,
          "Original target failure cannot resume or modify memory without coordinated recovery");
}

void programmer(const std::vector<std::uint8_t> &program,
                std::ostream &trace,
                const unsigned seed) {
  Memory bus;
  ram(bus);
  bus.install(program_start, program);
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = program_start, .hx = static_cast<std::uint16_t>(seed), .sp = ram_end - 4});
  for (unsigned n = 0; n < 3; ++n) { cpu.step(); }
  const auto before = cpu.state();
  require(before.pc == divider_write_pc && before.hx == (seed | (program_stack_low + 1)) &&
              before.sp == (seed | program_stack_low) && before.a == divider_value,
          "Original RAM programmer preserves H through LDX/TXS before its first FCDIV write");
  const auto error = rejected([&] { cpu.step(); });
  require(error.contains("write address=" + hex(flash_divider)) &&
              error.contains("pc=" + hex(divider_write_pc)) && bus.retired == 3 &&
              bus.accesses.empty(),
          "The original programmer stops at its actual unsupported flash divider store");
  quarantine(cpu, bus);
  for (unsigned n = 0; n < program.size(); ++n) {
    require(bus.bytes[program_start + n] == program[n], "Original RAM programmer stays exact");
  }
  trace << "PROGRAMMER seed=" << seed << " retired=3 pc=" << before.pc << " sp=" << before.sp
        << " hx=" << before.hx << " a=" << static_cast<unsigned>(before.a)
        << " address=" << flash_divider << " fault=1\n";
}

void enter_application(Hcs08 &cpu, Memory &bus, const unsigned seed, const StartupEntry mode) {
  if (mode == StartupEntry::normal_reset) {
    constexpr unsigned retained_x = 0x22;
    constexpr unsigned previous_pc = 0x1234;
    constexpr unsigned previous_sp = 0x4567;
    for (unsigned n = ram_first; n < ram_end; ++n) { bus.known.reset(n); }
    cpu.set_state({.pc = previous_pc,
                   .hx = static_cast<std::uint16_t>(seed | retained_x),
                   .sp = previous_sp,
                   .a = canary,
                   .ccr = byte_mask});
    cpu.reset();
    require(
        cpu.state() == Hcs08State{.pc = application_entry,
                                  .hx = retained_x,
                                  .sp = byte_mask,
                                  .a = canary,
                                  .ccr_known = reset_ccr,
                                  .ccr = reset_ccr} &&
            bus.accesses ==
                std::vector<Access>{
                    {.address = reset_vector, .value = application_entry >> 8U, .write = false},
                    {.address = reset_vector + 1, .value = 0, .write = false}} &&
            bus.retired == 0,
        "Native normal warm reset reads both original vector bytes without defining CCR or RAM");
    return;
  }
  const auto initial_pc =
      static_cast<std::uint16_t>(bus.read_memory({.address = reset_vector, .size = 2}));
  require(initial_pc == application_entry, "Original supplied reset vector selects E000");
  bus.accesses.clear(); // Explicit debug fixture reads are not native CPU transactions.
  cpu.set_state({.pc = initial_pc,
                 .hx = static_cast<std::uint16_t>(seed),
                 .sp = 0,
                 .a = canary,
                 .ccr = static_cast<std::uint8_t>(seed == 0 ? flags_fixed : byte_mask)});
}

void startup(const std::vector<std::uint8_t> &records,
             std::ostream &trace,
             const unsigned seed,
             const StartupEntry mode) {
  Memory bus;
  ram(bus);
  for (unsigned n = 0; n < pages.size(); ++n) {
    const auto offset = n * record_size;
    require((static_cast<unsigned>(records[offset]) << 8U | records[offset + 1]) == pages[n],
            "Original supplied page addresses remain exact; omitted memory is not filled");
    bus.install(pages[n], std::span(records).subspan(offset + 2, page_size));
  }
  Hcs08 cpu(bus, BLOFELD_HCS08_SLA);
  enter_application(cpu, bus, seed, mode);
  for (unsigned n = 0; n < startup_retired; ++n) { cpu.step(); }
  const auto before = cpu.state();
  require(before.pc == gpio_write_pc && before.sp == ram_end - 3 &&
              before.hx == calibration_batches && before.a == 3,
          "Original HCS08 startup completes its real RAM loop and calibration assignments");
  if (mode == StartupEntry::normal_reset) {
    require(before.ccr_known == initialized_ccr_known && before.ccr == reset_ccr,
            "Original CPHX establishes C, later CLR preserves it and H remains unknown");
  }
  const auto error = rejected([&] { cpu.step(); });
  require(error.contains("write address=" + hex(port_e)) &&
              error.contains("pc=" + hex(gpio_write_pc)) && bus.retired == startup_retired,
          "The original application stops at actual unmodeled GPIO, not successful scan setup");
  std::array<std::uint8_t, ram_end - ram_first> expected{};
  expected[0] = expected[6] = 7;
  expected[1] = expected[7] = byte_mask;
  expected[calibration - ram_first] = 1;
  expected[calibration - ram_first + 1] = calibration_low;
  expected[expected.size() - 2] = 4;
  expected[expected.size() - 1] = 0x80;
  for (unsigned n = 0; n < expected.size(); ++n) {
    require(bus.known[ram_first + n] && bus.bytes[ram_first + n] == expected[n],
            "Every original initialized RAM byte and final overlapping iterator agrees");
  }
  unsigned writes = 0;
  unsigned reads = 0;
  for (const auto &event : bus.accesses) { event.write ? ++writes : ++reads; }
  const unsigned reset_reads = mode == StartupEntry::normal_reset ? 2 : 0;
  require(writes == expected_writes && reads == expected_reads + reset_reads,
          "Independent loop and word-access counts match every native RAM transaction");
  quarantine(cpu, bus);
  trace << (mode == StartupEntry::normal_reset ? "WARM_STARTUP" : "STARTUP") << " seed=" << seed
        << " retired=" << bus.retired << " pc=" << before.pc << " sp=" << before.sp
        << " hx=" << before.hx << " a=" << static_cast<unsigned>(before.a) << " writes=" << writes
        << " reads=" << reads << " address=" << port_e << " fault=1";
  if (mode == StartupEntry::normal_reset) {
    trace << " ccr-known=" << static_cast<unsigned>(before.ccr_known)
          << " ccr=" << static_cast<unsigned>(before.ccr);
  }
  trace << '\n';
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Expected pinned original HCS08 programmer, supplied pages and trace");
    const auto program = input(arguments[1], program_size);
    const auto records = input(arguments[2], static_cast<unsigned>(pages.size()) * record_size);
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    for (const unsigned seed : {0U, 0x1200U}) {
      programmer(program, trace, seed);
      startup(records, trace, seed, StartupEntry::debug);
    }
    for (const unsigned seed : {0U, 0x1200U}) {
      startup(records, trace, seed, StartupEntry::normal_reset);
    }
    std::cout << "Original HCS08 programmer and startup execute to strict peripheral boundaries.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
