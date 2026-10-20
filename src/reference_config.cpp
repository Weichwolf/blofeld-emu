// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "blofeld/private_file.hpp"
#include "reference_board.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace blofeld::reference {
namespace {
constexpr std::size_t maximum_config_bytes = 65536;
constexpr std::size_t maximum_rom_words = 65536;
constexpr unsigned byte_mask = 0xff;

std::string token(std::istringstream &line) {
  std::string result;
  if (!(line >> result)) { throw Fault("Missing reference-board field"); }
  return result;
}

enum class NumberBase : std::uint8_t { decimal = 10, hexadecimal = 16 };

std::uint64_t number(std::istringstream &line,
                     const std::uint64_t maximum,
                     const NumberBase base = NumberBase::decimal) {
  const auto value = token(line);
  std::uint64_t result = 0;
  const auto parsed =
      std::from_chars(value.data(), value.data() + value.size(), result, static_cast<int>(base));
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || result > maximum) {
    throw Fault("Invalid reference-board number: " + value);
  }
  return result;
}

std::uint8_t byte(std::istringstream &line) {
  return static_cast<std::uint8_t>(number(line, byte_mask, NumberBase::hexadecimal));
}

std::uint8_t pin(std::istringstream &line) {
  const auto result = byte(line);
  if (!std::has_single_bit(result)) { throw Fault("Reference-board pin requires one bit"); }
  return result;
}

McuPort port(std::istringstream &line) {
  constexpr std::array ports{McuPort::a,
                             McuPort::b,
                             McuPort::e,
                             McuPort::t,
                             McuPort::m,
                             McuPort::j,
                             McuPort::p,
                             McuPort::q,
                             McuPort::r,
                             McuPort::s,
                             McuPort::u};
  constexpr std::string_view names = "ABETMJPQRSU";
  const auto name = token(line);
  const auto index = names.find(name);
  if (name.size() != 1 || index == std::string_view::npos) {
    throw Fault("Unknown reference-board port: " + name);
  }
  return ports.at(index);
}

void literal(std::istringstream &line, const std::string_view expected) {
  if (token(line) != expected) {
    throw Fault("Reference-board contract requires " + std::string(expected));
  }
}

Input input(std::istringstream &line) {
  Input result;
  const auto kind = token(line);
  if (kind == "drive") {
    result.port = port(line);
    result.drive = {.mask = byte(line), .levels = byte(line), .floating = byte(line)};
    if ((result.drive.mask & result.drive.floating) != 0 ||
        (result.drive.levels & ~static_cast<unsigned>(result.drive.mask)) != 0) {
      throw Fault("Reference-board drive has overlapping floating bits or unmasked levels");
    }
  } else if (kind == "phy-window") {
    result.kind = InputKind::phy_window;
    result.value = static_cast<unsigned>(number(line, std::numeric_limits<unsigned>::max()));
  } else if (kind == "usb-decoder") {
    result.kind = InputKind::usb_decoder;
    result.value = byte(line);
    constexpr unsigned decoder_mask = 15;
    if (result.value > decoder_mask) { throw Fault("USB decoder flags exceed four bits"); }
  } else if (kind == "timer-inputs") {
    result.kind = InputKind::timer_inputs;
    result.value = byte(line);
  } else {
    throw Fault("Unknown reference-board input: " + kind);
  }
  return result;
}

void clocks(Config &config, std::istringstream &line) {
  const auto oscillator = static_cast<std::uint32_t>(number(line, UINT32_MAX));
  const auto phase = number(line, UINT64_MAX);
  const auto extal = static_cast<std::uint32_t>(number(line, UINT32_MAX));
  const auto timebase = number(line, UINT64_MAX);
  literal(line, "reference-boundaries");
  config.mcu = {.clock = {.oscillator_hz = oscillator,
                          .initial_oscillator_phase_ticks = phase,
                          .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                .memory_timing = McuRunMemory::internal_flash_reads};
  config.dsp = {.clock = {.extal_hz = extal,
                          .timebase_hz = timebase,
                          .switch_timing = DspClockSwitch::reference_instruction_boundary},
                .timing = DspRunTiming::native_internal_estimates};
  const McuRunClock mcu(config.mcu.clock);
  const DspRunClock dsp(config.dsp.clock);
  if (dsp.timebase_hz() < mcu.timebase_hz() || timebase % mcu.timebase_hz() != 0) {
    throw Fault("Reference-board common timebase must be an integer MCU multiple");
  }
}

void ordering(Config &config, std::istringstream &line) {
  const auto tie = token(line);
  if (tie == "dsp-before-mcu") {
    config.schedule.shared_write_order = McuDspTieOrder::dsp_before_mcu;
  } else if (tie == "mcu-before-dsp") {
    config.schedule.shared_write_order = McuDspTieOrder::mcu_before_dsp;
  } else if (tie == "reject") {
    config.schedule.shared_write_order = McuDspTieOrder::unqualified;
  } else {
    throw Fault("Unknown reference-board shared-write order");
  }
  literal(line, "dsp-before-mcu");
  config.schedule.wait_order = McuDspWaitOrder::dsp_before_mcu;
  config.schedule.maximum_dsp_steps_per_cycle =
      static_cast<unsigned>(number(line, std::numeric_limits<unsigned>::max()));
  if (config.schedule.maximum_dsp_steps_per_cycle == 0) {
    throw Fault("Reference-board DSP step budget must be positive");
  }
}

void serial(Config &config, std::istringstream &line) {
  config.select_n = pin(line);
  config.clock = pin(line);
  config.mosi = pin(line);
  if (config.select_n == config.clock || config.select_n == config.mosi ||
      config.clock == config.mosi) {
    throw Fault("Reference-board SPI pins must be distinct");
  }
  const auto order = token(line);
  if (order == "clock-before-select") {
    config.pad_order = PadOrder::clock_before_select;
  } else if (order == "select-before-clock") {
    config.pad_order = PadOrder::select_before_clock;
  } else if (order != "reject") {
    throw Fault("Unknown reference-board SPI pad order");
  }
}

void setting(Config &config, const std::string_view key, std::istringstream &line) {
  if (key == "clock") {
    clocks(config, line);
  } else if (key == "ordering") {
    ordering(config, line);
  } else if (key == "spi-b") {
    serial(config, line);
  } else if (key == "dsp-release") {
    config.release_ticks = number(line, UINT64_MAX);
    const auto pinit = token(line);
    if (pinit != "low" && pinit != "high") { throw Fault("DSP PINIT must be low or high"); }
    config.pinit = pinit == "high";
    config.schedule.dsp.observation = DspStepObservation::reference_completion_boundary;
  } else if (key == "operations") {
    config.operations = number(line, UINT64_MAX);
    if (config.operations == 0) {
      throw Fault("Reference-board operation budget must be positive");
    }
  } else if (key == "loopback") {
    const Loopback route{.source = port(line),
                         .source_mask = pin(line),
                         .destination = port(line),
                         .destination_mask = pin(line)};
    if (route.source == route.destination && route.source_mask == route.destination_mask) {
      throw Fault("Reference-board loopback requires distinct pins");
    }
    config.loopback = route;
  } else {
    throw Fault("Unknown reference-board setting: " + std::string(key));
  }
}

struct Parser {
  Config result;
  std::vector<std::string> settings;
  bool header = false;

  void row(const std::string &key, std::istringstream &line) {
    if (!header) {
      if (key != "blofeld-reference-board") { throw Fault("Missing reference-board header"); }
      literal(line, "1");
      header = true;
    } else if (key == "initial") {
      result.initial.push_back(input(line));
    } else if (key == "at") {
      const auto when = number(line, UINT64_MAX);
      auto event = input(line);
      event.ticks = when;
      if (!result.events.empty() && when < result.events.back().ticks) {
        throw Fault("Reference-board events must be ordered by requested tick");
      }
      result.events.push_back(event);
    } else {
      if (std::ranges::find(settings, key) != settings.end()) {
        throw Fault("Duplicate reference-board setting: " + key);
      }
      setting(result, key, line);
      settings.push_back(key);
    }
    std::string extra;
    if (line >> extra) { throw Fault("Unexpected reference-board field: " + extra); }
  }

  void complete() const {
    for (const auto *required : {"clock", "ordering", "spi-b", "dsp-release", "operations"}) {
      if (std::ranges::find(settings, required) == settings.end()) {
        throw Fault("Missing reference-board setting: " + std::string(required));
      }
    }
  }
};

Config parse(const std::string &text) {
  Parser parser;
  std::istringstream source(text);
  std::string row;
  unsigned row_number = 0;
  while (std::getline(source, row)) {
    ++row_number;
    row = row.substr(0, row.find('#'));
    std::istringstream line(row);
    std::string key;
    if (!(line >> key)) { continue; }
    try {
      parser.row(key, line);
    } catch (const Fault &fault) {
      throw Fault("Reference-board line " + std::to_string(row_number) + ": " + fault.what());
    }
  }
  parser.complete();
  return parser.result;
}
}

Config load_config(const std::filesystem::path &path) {
  const auto bytes = read_private_file(path, maximum_config_bytes);
  if (bytes.empty()) { throw Fault("Empty reference-board input"); }
  const std::string text(bytes.begin(), bytes.end());
  if (text.find('\0') != std::string::npos) { throw Fault("NUL byte in reference-board input"); }
  return parse(text);
}

std::vector<std::uint32_t> load_rom(const std::filesystem::path &path) {
  const auto bytes = read_private_file(path, maximum_rom_words * 3);
  if (bytes.empty() || bytes.size() % 3 != 0) {
    throw Fault("DSP ROM requires nonempty packed big-endian 24-bit words");
  }
  std::vector<std::uint32_t> words;
  words.reserve(bytes.size() / 3);
  for (std::size_t offset = 0; offset < bytes.size(); offset += 3) {
    constexpr unsigned high_shift = 16;
    words.push_back((static_cast<std::uint32_t>(bytes[offset]) << high_shift) |
                    (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U) | bytes[offset + 2]);
  }
  return words;
}
}
