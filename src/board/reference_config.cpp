// SPDX-License-Identifier: GPL-3.0-only
#include "audio/serial_pcm.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_midi.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
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
  config.dsp.clock = {.extal_hz = extal,
                      .timebase_hz = timebase,
                      .switch_timing = DspClockSwitch::reference_instruction_boundary};
  config.dsp.timing = DspRunTiming::native_internal_estimates;
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

ReturnRoute return_route(std::istringstream &line) {
  ReturnRoute result{.destination = port(line), .mask = pin(line)};
  const auto polarity = token(line);
  if (polarity != "direct" && polarity != "inverted") {
    throw Fault("SHI return polarity must be direct or inverted");
  }
  result.inverted = polarity == "inverted";
  return result;
}

std::optional<bool> dsp_level(std::istringstream &line) {
  const auto level = token(line);
  if (level == "high" || level == "low") { return level == "high"; }
  if (level != "unknown") { throw Fault("DSP input level must be high, low or unknown"); }
  return std::nullopt;
}

DspInput dsp_input(std::istringstream &line) {
  const auto kind = token(line);
  if (kind != "tio" && kind != "esai") { throw Fault("Unknown DSP input kind: " + kind); }
  DspInput result{.index = static_cast<unsigned>(number(line, 1))};
  if (kind == "esai") {
    result.kind = DspInputKind::esai;
    result.esai.clock = dsp_level(line);
    result.esai.frame_sync = dsp_level(line);
    result.esai.high_clock = dsp_level(line);
    const auto timing = token(line);
    if (timing == "reference-reset-edges") {
      result.esai.timing = DspEsaiClockTiming::reference_reset_edges;
    } else if (timing != "raw") {
      throw Fault("DSP ESAI timing must be raw or reference-reset-edges");
    }
    return result;
  }
  result.level = dsp_level(line);
  const auto observation = token(line);
  if (observation != "raw" && observation != "synchronized") {
    throw Fault("DSP TIO observation must be raw or synchronized");
  }
  result.synchronized = observation == "synchronized";
  if (result.synchronized && !result.level.has_value()) {
    throw Fault("An unknown DSP TIO level cannot be synchronized");
  }
  return result;
}

DspEsaiClock dsp_clock(std::istringstream &line) {
  literal(line, "esai");
  const auto index = static_cast<unsigned>(number(line, 1));
  literal(line, "hckt");
  const auto start = number(line, UINT64_MAX);
  const auto half = number(line, UINT64_MAX);
  const auto edges = number(line, UINT64_MAX);
  literal(line, "reference-reset-edges");
  if (half == 0 || edges == 0 || (edges - 1) > (UINT64_MAX - start) / half) {
    throw Fault("Invalid DSP ESAI periodic-clock span");
  }
  return {.start_ticks = start, .half_period_ticks = half, .edges = edges, .index = index};
}

AudioClockRoute audio_clock(std::istringstream &line) {
  const auto frequency = number(line, UINT64_MAX);
  const auto epoch = number(line, UINT64_MAX);
  const auto level = dsp_level(line);
  if (!level) { throw Fault("Reference audio clock initial level must be known"); }
  const auto destinations = static_cast<unsigned>(number(line, 7));
  literal(line, "reference-reset-edges");
  if (frequency == 0 || destinations == 0) {
    throw Fault("Reference audio clock frequency/destinations must be nonzero");
  }
  return {.frequency_hz = frequency,
          .epoch_ticks = epoch,
          .destinations = destinations,
          .initial_level = *level};
}

void dsp_interrupts(Config &config, std::istringstream &line) {
  literal(line, "native-vector-estimates");
  config.dsp.interrupts = DspInterruptTiming::native_vector_estimates;
  const auto admission = token(line);
  if (admission == "prepare-boundary") {
    config.schedule.dsp.irq_admission = DspIrqAdmission::reference_prepare_boundary;
  } else if (admission == "reject-changes") {
    config.schedule.dsp.irq_admission = DspIrqAdmission::reject_changes;
  } else {
    throw Fault("Unknown reference-board DSP interrupt admission");
  }
}

void dsp_order(Config &config, std::istringstream &line) {
  const auto order = token(line);
  if (order == "dsp-before-input") {
    config.dsp_input_order = DspExternalOrder::dsp_before_external;
  } else if (order == "input-before-dsp") {
    config.dsp_input_order = DspExternalOrder::external_before_dsp;
  } else if (order == "reject") {
    config.dsp_input_order = DspExternalOrder::reject_tie;
  } else {
    throw Fault("Unknown reference-board DSP input order");
  }
  config.maximum_dsp_inputs_per_advance =
      static_cast<unsigned>(number(line, std::numeric_limits<unsigned>::max()));
  if (config.maximum_dsp_inputs_per_advance == 0) {
    throw Fault("Reference-board DSP input budget must be positive");
  }
}

PcmRoute pcm_route(std::istringstream &line) {
  literal(line, "esai");
  const auto instance = static_cast<unsigned>(number(line, 1));
  literal(line, "tx");
  const auto lane = static_cast<unsigned>(number(line, 5));
  literal(line, "after");
  const auto after = number(line, UINT64_MAX);
  const auto edge = token(line);
  if (edge != "rising" && edge != "falling") { throw Fault("Unknown PCM sampling edge"); }
  constexpr unsigned maximum_bits = 32;
  PcmRoute result{
      .instance = instance,
      .lane = lane,
      .after_ticks = after,
      .format = {.slot_bits = static_cast<unsigned>(number(line, maximum_bits)),
                 .word_bits = static_cast<unsigned>(number(line, maximum_bits)),
                 .channels = static_cast<unsigned>(number(line, SerialPcmFrame::maximum_channels)),
                 .sample_edge = edge == "rising" ? SerialPcmEdge::rising : SerialPcmEdge::falling}};
  const auto polarity = token(line);
  if (polarity != "high" && polarity != "low") { throw Fault("Unknown PCM frame polarity"); }
  result.format.frame_sync_high = polarity == "high";
  const SerialPcmReceiver validated(result.format);
  return result;
}

MidiRoute midi_route(std::istringstream &line) {
  const auto input_pin = token(line);
  if (input_pin != "PR2" && input_pin != "PS4") {
    throw Fault("Reference MIDI pin must be PR2 or PS4");
  }
  const auto hz = static_cast<std::uint32_t>(number(line, UINT32_MAX));
  literal(line, "after");
  return {.bus_hz = hz,
          .pin = input_pin == "PR2" ? SciReceivePin::pr2 : SciReceivePin::ps4,
          .after_ticks = number(line, UINT64_MAX)};
}

DspMidiRoute dsp_midi_route(std::istringstream &line) {
  const auto index = static_cast<unsigned>(number(line, 1));
  literal(line, "after");
  const auto when = number(line, UINT64_MAX);
  const auto timing = token(line);
  if (timing != "raw" && timing != "synchronized") {
    throw Fault("DSP MIDI input timing must be raw or synchronized");
  }
  return {.after_ticks = when, .index = index, .synchronized = timing == "synchronized"};
}

bool midi_setting(Config &config, const std::string_view key, std::istringstream &line) {
  if (key == "midi-sci") {
    config.midi = midi_route(line);
  } else if (key == "midi-transfer-reference") {
    const auto divisor = static_cast<unsigned>(number(line, UINT32_MAX));
    config.midi_reference = {.baud_divisor = divisor,
                             .transfer_delay_bus_cycles =
                                 static_cast<unsigned>(number(line, UINT32_MAX))};
  } else if (key == "midi-dsp-tio") {
    config.midi_dsp = dsp_midi_route(line);
  } else if (key == "midi-dsp-reference") {
    const auto period = number(line, UINT64_MAX);
    config.midi_dsp_reference = {.bit_ticks = period,
                                 .idle_bits = static_cast<unsigned>(number(line, 16))};

  } else {
    return false;
  }
  return true;
}

void setting(Config &config, const std::string_view key, std::istringstream &line) {
  if (midi_setting(config, key, line)) { return; }
  if (key == "clock") {
    clocks(config, line);
  } else if (key == "ordering") {
    ordering(config, line);
  } else if (key == "dsp-interrupts") {
    dsp_interrupts(config, line);
  } else if (key == "dsp-input-order") {
    dsp_order(config, line);
  } else if (key == "dsp-audio-clock") {
    config.audio_clock = audio_clock(line);
  } else if (key == "pcm-receive") {
    config.pcm = pcm_route(line);
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
  } else if (key == "checkpoint") {
    const auto operation = number(line, UINT64_MAX);
    const auto replays = static_cast<unsigned>(number(line, std::numeric_limits<unsigned>::max()));
    if (replays == 0) {
      throw Fault("Reference-board checkpoint requires a positive replay count");
    }
    config.checkpoint = {.operation = operation, .replays = replays};
  } else if (key == "shi-hreq") {
    config.hreq = return_route(line);
  } else if (key == "shi-miso") {
    config.miso = return_route(line);
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
    } else if (key == "dsp-initial") {
      result.dsp_initial.push_back(dsp_input(line));
    } else if (key == "dsp-clock") {
      const auto clock = dsp_clock(line);
      if (std::ranges::any_of(result.dsp_clocks,
                              [&](const auto &value) { return value.index == clock.index; })) {
        throw Fault("Duplicate DSP ESAI periodic clock");
      }
      result.dsp_clocks.push_back(clock);
    } else if (key == "dsp-at") {
      const auto when = number(line, UINT64_MAX);
      auto event = dsp_input(line);
      event.ticks = when;
      if (!result.dsp_events.empty() && when <= result.dsp_events.back().ticks) {
        throw Fault("DSP input event timestamps must increase strictly");
      }
      result.dsp_events.push_back(event);
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
    if (!result.dsp_events.empty() || !result.dsp_clocks.empty() || result.audio_clock) {
      if (result.dsp_input_order == DspExternalOrder::unqualified) {
        throw Fault("Timed DSP inputs require an explicit dsp-input-order");
      }
      if (!result.dsp_events.empty() && result.dsp_events.front().ticks < result.release_ticks) {
        throw Fault("Timed DSP inputs precede the requested DSP release");
      }
    }
    for (const auto &clock : result.dsp_clocks) {
      if (clock.start_ticks < result.release_ticks) {
        throw Fault("DSP ESAI periodic clock precedes the requested DSP release");
      }
    }
    if (result.pcm && result.pcm->after_ticks < result.release_ticks) {
      throw Fault("PCM observation threshold precedes the requested DSP release");
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
  if (parser.result.checkpoint && parser.result.checkpoint->operation > parser.result.operations) {
    throw Fault("Reference-board checkpoint exceeds the operation budget");
  }
  validate_returns(parser.result);
  validate_midi(parser.result);
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
