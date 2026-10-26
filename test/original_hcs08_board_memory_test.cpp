// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"

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
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr unsigned extent = 3598, record = 514, calibration = 0xffbe, declared_trim = 0x80;
constexpr unsigned strap_pc = 0xe04e, strap_retired = 6176, timer_pc = 0xe1ca;
constexpr unsigned final_retired = 6197, initial_hx = 0x12c, final_sp = 0x475;
constexpr unsigned trim_read_pc = 0xe1a5, trim_read_retired = 6184;
constexpr unsigned flags_known = 0xef, final_flags = 0x69, partial_a = 0xfe;
constexpr std::array pages{0xe000U, 0xe200U, 0xe400U, 0xe600U, 0xe800U, 0xea00U, 0xfe00U};

Hcs08FlashImage image(const std::span<const std::uint8_t> records, const bool supplied_trim) {
  Hcs08FlashImage result;
  for (unsigned page = 0; page < pages.size(); ++page) {
    const auto first = page * record;
    const unsigned address = (static_cast<unsigned>(records[first]) << 8U) | records[first + 1];
    require(address == pages[page], "Pinned records retain original physical page addresses");
    for (unsigned offset = 0; offset < hcs08_flash::page_bytes; ++offset) {
      const auto target = address + offset - hcs08_flash::first;
      result.data[target] = records[first + 2 + offset];
      result.known.set(target);
    }
  }
  const auto target = calibration - hcs08_flash::first;
  if (supplied_trim) {
    result.data[target] = declared_trim; // Explicit per-unit reference input,never a board default.
  } else {
    result.known.reset(target);
  }
  return result;
}

void run(const std::span<const std::uint8_t> records,
         const bool supplied_pads,
         const bool supplied_trim) {
  const auto supplied = image(records, supplied_trim);
  Hcs08BoardMemory memory(supplied, Hcs08BoardMemoryContract::candidate_gt8a_untimed);
  if (supplied_pads) {
    memory.attach_pad_source(Hcs08GpioPort::e, [](const unsigned mask) {
      return GpioPadSample{.known = mask, .levels = 8U & mask};
    });
  }
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.reset(Hcs08ResetEntry::normal, Hcs08ResetCause::power_on);
  require(cpu.state().pc == pages[0], "Only actual native FFFE/FFFF reads establish reset PC");
  unsigned retired = 0;
  while (retired < strap_retired) {
    cpu.step();
    ++retired;
  }
  require(cpu.state().pc == strap_pc, "Unmodified RAM initialization reaches the real strap read");
  if (!supplied_pads) {
    const auto failure = rejected([&] { cpu.step(); });
    require(failure.contains("GPIO") && failure.contains("pc=0xe04e") && cpu.faulted() &&
                memory.faulted() && memory.flash_image() == supplied,
            "Default board cannot invent input pads to enter a chosen original boot path");
    return;
  }
  while (retired < final_retired) {
    const auto before = cpu.state();
    try {
      cpu.step();
    } catch (const Fault &error) {
      if (supplied_trim || before.pc != trim_read_pc ||
          !std::string(error.what()).contains("unknown")) {
        throw;
      }
      require(retired == trim_read_retired && cpu.faulted() && memory.faulted() &&
                  memory.flash_image() == supplied,
              "Missing factory calibration stays unknown and cannot be silently replaced");
      std::cout << "MISSING trim pc=" << before.pc << " retired=" << retired << '\n';
      return;
    }
    ++retired;
  }
  require(supplied_trim && cpu.state() == Hcs08State{.pc = timer_pc,
                                                     .hx = initial_hx,
                                                     .sp = final_sp,
                                                     .a = 0,
                                                     .ccr_known = flags_known,
                                                     .ccr = final_flags,
                                                     .a_known = partial_a},
          "Original native ICG/ADC setup retains independent exact CPU state and unknown bits");
  const auto failure = rejected([&] { cpu.step(); });
  require(failure.contains("live TPM requires elapsed bus periods") &&
              failure.contains("pc=0xe1ca") && cpu.faulted() && memory.faulted() &&
              memory.flash_image() == supplied && memory.adc_state().conversions == 0,
          "Production memory owner reaches the actual strict timer frontier without any NVM patch");
  std::cout << "REFERENCE pc=" << cpu.state().pc << " retired=" << retired << '\n';
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Expected pinned original seven-page records");
    std::ifstream input(arguments[1], std::ios::binary);
    std::vector<std::uint8_t> records(extent);
    input.read(reinterpret_cast<char *>(records.data()),
               static_cast<std::streamsize>(records.size()));
    require(input.gcount() == static_cast<std::streamsize>(records.size()) &&
                input.peek() == std::char_traits<char>::eof(),
            "Exact original record extent");
    run(records, false, false);
    run(records, true, false);
    run(records, true, true);
    std::cout << "Verified original POR through independent production board memory.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
