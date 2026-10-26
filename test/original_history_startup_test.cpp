// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_clock_startup_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "history_startup_memory.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_tpm.hpp"

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
namespace app = blofeld::test::hcs08_clock_startup;
using blofeld::test::icg_history::Memory;
constexpr unsigned after_status1 = 6189, after_status2 = 6190, endpoint = 6197;
constexpr unsigned status2_pc = 0xe1b2, adc_pc = 0xe1b4, timer_pc = 0xe1ca;
constexpr unsigned partial_byte = 0xfe, known_flags = 0xef, status_flags = 0xed;
constexpr unsigned expected_flags = 0x69, expected_stack = 0x475;
constexpr unsigned modulation = 0xbe, pressure = 0xc0, timer_control = 0xf;
constexpr unsigned adc_control = 0xc2, adc_remaining = 176;

void check_status(const Hcs08 &cpu, const unsigned retired) {
  const auto actual = cpu.state();
  const auto expected_pc = retired == after_status1 ? status2_pc : adc_pc;
  const auto expected_a_mask = retired == after_status1 ? 0U : partial_byte;
  // Both masks leave N known clear only for ICGS2. V is cleared by native LDA,
  // H remains uninitialized, and Z remains unknown for either status load.
  const auto expected_ccr_mask = retired == after_status1 ? 0xe9U : status_flags;
  require(actual.pc == expected_pc && actual.a == 0 && actual.a_known == expected_a_mask &&
              actual.ccr_known == expected_ccr_mask && actual.sp == expected_stack &&
              actual.hx == app::expected_hx,
          "Original native LDA preserves every unknown live status bit and exact frame");
}

void finish(Memory &memory, Hcs08 &cpu) {
  while (memory.data.retired < endpoint) {
    cpu.step();
    if (memory.data.retired == after_status1 || memory.data.retired == after_status2) {
      check_status(cpu, memory.data.retired);
    }
  }
  require(cpu.state() == Hcs08State{.pc = timer_pc,
                                    .hx = app::expected_hx,
                                    .sp = expected_stack,
                                    .a = 0,
                                    .ccr_known = known_flags,
                                    .ccr = expected_flags,
                                    .a_known = partial_byte},
          "Native CLR/MOV overwrite status flags, preserving partial A until real timer read");
  require(memory.data.bytes[modulation] == 5 && memory.data.known[modulation] &&
              memory.data.bytes[pressure] == 10 && memory.data.known[pressure],
          "Only actual MOV stores establish the original decimator phase bytes");
  const auto adc = memory.data.adc.state();
  require(adc.control == adc_control && adc.pins == 7 && adc.status == 0 && adc.active &&
              !adc.result && !adc.reference && adc.remaining == adc_remaining &&
              adc.conversions == 0,
          "Real ADC job starts without an invented result, reference or elapsed conversion");
  require(memory.timer.state() == Hcs08TpmState{.control = timer_control},
          "Actual timer request does not invent a prescaler phase or elapsed bus clocks");
  const auto before = cpu.state();
  const auto failure = rejected([&] { cpu.step(); });
  require(failure.contains("TPM live read address=0x0031 pc=0xe1ca") &&
              failure.contains("actual clock history and prescaler phase are unqualified") &&
              cpu.faulted() && cpu.state() == before && memory.data.retired == endpoint,
          "First actual live counter read refuses without retiring or supplying a frozen timer");
}

void run(const std::span<const std::uint8_t> pages) {
  Memory memory(pages);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.reset(Hcs08ResetEntry::normal, Hcs08ResetCause::power_on);
  while (memory.data.retired < app::status_retired) { cpu.step(); }
  require(cpu.state().pc == app::status_pc && cpu.state().a_known == byte_mask,
          "Unmodified original reset reaches both status loads through actual vector/code fetches");
  const auto flash = memory.data.flash.image();
  finish(memory, cpu);
  require(memory.data.flash.image() == flash,
          "Conservative status access never modifies original FLASH or missing-byte knowledge");
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Expected pinned original seven-page records");
    std::ifstream input(arguments[1], std::ios::binary);
    std::vector<std::uint8_t> pages(app::records_bytes);
    input.read(reinterpret_cast<char *>(pages.data()), static_cast<std::streamsize>(pages.size()));
    require(input.gcount() == static_cast<std::streamsize>(pages.size()) &&
                input.peek() == std::char_traits<char>::eof(),
            "Original pages retain their exact extent");
    run(pages);
    std::cout << "Original reset retains unknown ICG status through native ADC setup; strict TPM "
                 "frontier.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
