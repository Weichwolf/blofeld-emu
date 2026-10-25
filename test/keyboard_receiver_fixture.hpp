// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace blofeld::test::keyboard_receiver {
inline void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

inline constexpr unsigned helper_base = 0x1200, helper_bytes = 1018;
inline constexpr unsigned caller = 0x2000, stack = 0x2200, callback_table = 0xf81b;
inline constexpr unsigned cache = 0x21f6, initialized = 0x121a, halfway = 0x1511;
inline constexpr unsigned second_call = 0x1213, third_call = 0x1217;
inline constexpr unsigned high_mask = 255, data_mask = 63;
inline constexpr unsigned first_chunk_shift = 18, first_byte_shift = 16;
inline constexpr unsigned oscillator_hz = 12000000, qram_mapping = 0x21;
inline constexpr unsigned initial_b_direction = 0xf7, e_direction = 0x90, e_inputs = 0x6f;
inline constexpr unsigned q_inputs = 0x7f, upper_ccr = 0xf0, caller_ccr = 0xd0;
inline constexpr unsigned saved_callback = 0x21f1, saved_return = 0x21fe;
inline constexpr std::array<std::uint8_t, 8> own_caller{
    0xcc, 0xf8, 0x1b, 0x16, 0x12, 0, 0x20, 0xfe};
inline constexpr std::array<unsigned, 3> returns{0x1211, 0x1216, initialized};
inline constexpr std::array<unsigned, 4> samples{0x14fb, 0x1507, 0x1520, 0x1537};
inline constexpr std::array<unsigned, 3> return_stack{0x21ef, 0x21ef, 0x21f1};
inline constexpr std::array<unsigned, 3> return_cursor{cache, cache, cache + 1};

inline auto registers(const CpuState &cpu) {
  return std::tuple(cpu.pc, cpu.d, cpu.x, cpu.y, cpu.sp, cpu.ccr);
}

inline std::vector<std::uint8_t> bytes(const char *path, const unsigned expected) {
  std::ifstream input(path, std::ios::binary);
  std::vector<char> storage(expected);
  input.read(storage.data(), static_cast<std::streamsize>(storage.size()));
  require(input.gcount() == static_cast<std::streamsize>(expected) &&
              input.peek() == std::char_traits<char>::eof(),
          "Private reference file must retain its exact bounded size");
  std::vector<std::uint8_t> result;
  result.reserve(expected);
  for (const auto value : storage) { result.push_back(static_cast<std::uint8_t>(value)); }
  return result;
}

struct Source {
  unsigned word = 0;
  std::optional<unsigned> missing;
  unsigned requests = 0, reads = 0, returned = 0;
  bool operator==(const Source &) const = default;
};

struct Saved {
  McuCheckpoint machine;
  Source source;
  unsigned steps;
};

struct Fixture {
  const Firmware &firmware;
  const std::vector<std::uint8_t> &helper;
  McuMachine machine;
  std::optional<McuCheckpoint> baseline;
  Source source;
  std::ostream *trace = nullptr;
  unsigned steps = 0;
  bool detailed = false;
  bool sample_pending = false;

  Fixture(const Firmware &image, const std::vector<std::uint8_t> &code)
      : firmware(image),
        helper(code),
        machine(
            McuFlashImage::from_known(image.mcu),
            BLOFELD_DEFAULT_SLA,
            McuRunContract{.clock = {.oscillator_hz = oscillator_hz,
                                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                           .memory_timing = McuRunMemory::internal_flash_reads}) {
    require(helper.size() == helper_bytes, "Complete original programming helper required");
    auto &bus = machine.bus();
    bus.write(hc12::initee, qram_mapping);
    bus.retire_instruction();
    for (unsigned n = 0; n < helper.size(); ++n) {
      bus.write(static_cast<std::uint16_t>(helper_base + n), helper[n]);
    }
    // Own LDD IMM16; JSR EXT; BRA self, using the HCS12COREUG encodings.
    for (unsigned n = 0; n < own_caller.size(); ++n) {
      bus.write(static_cast<std::uint16_t>(caller + n), own_caller[n]);
    }
    // Explicit helper-caller lab state, not default board initialization.
    bus.write(hc12::ddrb, initial_b_direction);
    bus.write(hc12::portb, 0x80);
    bus.write(hc12::porte, 0);
    bus.write(hc12::ddre, e_direction);
    bus.set_pin_drive(McuPort::e, {.mask = e_inputs, .levels = 3});
    bus.write(hc12::ptq, 0);
    bus.write(hc12::ddrq, 0x80);
    bus.set_pin_drive(McuPort::q, {.mask = q_inputs});
    machine.cpu().set_state({.pc = caller, .sp = stack, .ccr = caller_ccr});
    baseline = machine.checkpoint();
  }

  void bind(std::ostream &output) {
    trace = &output;
    machine.bus().set_observer([this](const BusEvent &event) { observe(event); });
    machine.set_cycle_observer([this](const Hc12BusCycle &cycle) {
      if (cycle.address != hc12::ptq || cycle.kind != 'r') { return; }
      require(cycle.size == 1 && !sample_pending && source.reads < samples.size() &&
                  source.requests == source.reads + 1 &&
                  cycle.context.pc == samples.at(source.reads),
              "Each actual Q read consumes one held declared source chunk");
      sample_pending = true;
    });
  }

  void reset(const unsigned word,
             std::ostream &output,
             const std::optional<unsigned> missing = std::nullopt) {
    if (!baseline.has_value()) { throw Fault("Receiver baseline checkpoint missing"); }
    machine.restore(baseline.value());
    source = {.word = word, .missing = missing};
    steps = 0;
    detailed = false;
    sample_pending = false;
    bind(output);
  }

  void observe(const BusEvent &event) {
    if (!event.write) { return; }
    if (event.address != hc12::ptq) {
      if (trace != nullptr) {
        *trace << "WRITE address=" << event.address
               << " value=" << static_cast<unsigned>(event.value) << '\n';
      }
      return;
    }
    const auto index = source.requests++;
    require(event.value == (index % 2 == 0 ? 128U : 0U),
            "Native receiver requests alternate actual Q7 levels");
    PinDrive drive{.mask = 0x40, .unknown = data_mask};
    if (index < samples.size() && source.missing != index) {
      const auto value = (source.word >> (first_chunk_shift - 6U * index)) & data_mask;
      drive = {.mask = q_inputs, .levels = static_cast<std::uint8_t>(value)};
    }
    machine.bus().set_pin_drive(McuPort::q, drive);
    *trace << "REQUEST index=" << index << " value=" << static_cast<unsigned>(event.value) << '\n';
  }

  void sampled() {
    const auto index = source.reads;
    const auto state = machine.cpu().state();
    const auto actual = index == 0 || index == 3 ? state.d >> 8U : state.d & high_mask;
    const auto value = (source.word >> (first_chunk_shift - 6U * index)) & data_mask;
    require(actual == (value | (index % 2 == 0 ? 128U : 0U)),
            "Actual sampled Q byte retains independent data and request polarity");
    ++source.reads;
    *trace << "SAMPLE index=" << index << " value=" << actual << '\n';
  }

  void returned() {
    const auto state = machine.cpu().state();
    if (source.returned >= returns.size() || state.pc != returns[source.returned]) { return; }
    const auto n = source.returned;
    const auto value = (source.word >> (first_byte_shift - 8U * n)) & high_mask;
    require((state.d & high_mask) == value && state.sp == return_stack[n] &&
                state.x == return_cursor[n] && state.y == 0 &&
                (state.ccr & upper_ccr) == caller_ccr,
            "Original return byte, argument frame, cursor and upper CCR differ");
    require(machine.bus().read(cache + 2) == n &&
                machine.bus().read(cache) == ((source.word >> 8U) & high_mask) &&
                machine.bus().read(cache + 1) == (source.word & high_mask),
            "Original cache contains the independent remaining two bytes and cursor");
    *trace << "BYTE index=" << n << " value=" << value << " pc=" << state.pc << '\n';
    ++source.returned;
  }

  void step() {
    const auto state = machine.cpu().state();
    if (state.pc == second_call || state.pc == third_call) {
      require(state.d == cache,
              "Original reload discards the prior byte and restores its argument");
    }
    if (detailed) {
      *trace << "STEP pc=" << state.pc << " d=" << state.d << " x=" << state.x << " y=" << state.y
             << " sp=" << state.sp << " ccr=" << static_cast<unsigned>(state.ccr)
             << " ticks=" << machine.run_clock().elapsed_ticks()
             << " cycles=" << machine.run_clock().bus_cycles() << '\n';
    }
    sample_pending = false;
    machine.cpu().step();
    if (sample_pending) { sampled(); }
    sample_pending = false;
    ++steps;
    returned();
  }

  void guards() {
    const auto image = machine.bus().power_off_flash_image();
    require(std::ranges::equal(image.bytes, firmware.mcu) && image.known.all() &&
                image.fresh.none(),
            "Every original MCU NVM byte, knowledge bit and erase history remains exact");
    for (unsigned n = 0; n < helper.size(); ++n) {
      require(machine.bus().read(static_cast<std::uint16_t>(helper_base + n)) == helper[n],
              "Original programming helper remains unchanged");
    }
    for (unsigned n = 0; n < own_caller.size(); ++n) {
      require(machine.bus().read(static_cast<std::uint16_t>(caller + n)) == own_caller[n],
              "Own native caller remains exact");
    }
  }

  [[nodiscard]] Saved save() const {
    return {.machine = machine.checkpoint(), .source = source, .steps = steps};
  }

  void restore(const Saved &saved, std::ostream &output) {
    machine.restore(saved.machine);
    source = saved.source;
    steps = saved.steps;
    detailed = true;
    sample_pending = false;
    bind(output);
  }
};
} // namespace blofeld::test::keyboard_receiver
