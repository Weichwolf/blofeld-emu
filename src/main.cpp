// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_file.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/host_midi.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/pcm_recording.hpp"
#include "host_midi_runner.hpp"
#include "reference_board.hpp"

#include <charconv>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

struct Options {
  std::filesystem::path firmware;
  std::filesystem::path specification;
  std::filesystem::path trace;
  std::filesystem::path flash_in;
  std::filesystem::path flash_out;
  std::filesystem::path reference_board;
  std::filesystem::path dsp_rom;
  std::filesystem::path audio_out;
  std::filesystem::path midi_file;
  std::filesystem::path midi_device;
  std::optional<std::uint32_t> audio_rate;
  std::optional<std::size_t> audio_max_frames;
  std::uint64_t instructions = 1000000;
  bool inspect = false;
  std::optional<blofeld::PinDrive> port_e_input;
};

std::uint8_t hex_byte(const std::string_view value) {
  unsigned result = 0;
  const auto conversion = std::from_chars(value.data(), value.data() + value.size(), result, 16);
  constexpr unsigned byte_mask = 0xff;
  if (conversion.ec != std::errc{} || conversion.ptr != value.data() + value.size() ||
      result > byte_mask) {
    throw blofeld::Fault("Invalid GPIO byte value");
  }
  return static_cast<std::uint8_t>(result);
}

blofeld::PinDrive pin_drive(const std::string_view value) {
  if (value.size() != 5 || value[2] != ':') {
    throw blofeld::Fault("GPIO input fixture requires two hexadecimal bytes: MASK:LEVELS");
  }
  return {.mask = hex_byte(value.substr(0, 2)), .levels = hex_byte(value.substr(3, 2))};
}

std::uint64_t positive(const std::string_view value, const std::uint64_t maximum) {
  std::uint64_t result = 0;
  const auto converted = std::from_chars(value.data(), value.data() + value.size(), result);
  if (converted.ec != std::errc{} || converted.ptr != value.data() + value.size() || result == 0 ||
      result > maximum) {
    throw blofeld::Fault("Invalid audio limit/rate");
  }
  return result;
}

void validate_options(const Options &config) {
  if (config.firmware.empty()) {
    throw blofeld::Fault("Usage: blofeld-headless --firmware update.zip [--inspect] "
                         "[--instructions N] [--trace file] [--port-e-input MASK:LEVELS] "
                         "[--flash-in state.bfnvm] [--flash-out state.bfnvm] "
                         "[--reference-board scenario.bfboard --dsp-rom packed24.bin] "
                         "[--midi-file bytes.bin | --midi-device /dev/snd/midiCxDy] "
                         "[--audio-out capture.wav --audio-rate HZ --audio-max-frames N]");
  }
  if (config.inspect && (!config.flash_in.empty() || !config.flash_out.empty())) {
    throw blofeld::Fault("--inspect cannot be combined with MCU NVM execution inputs/outputs");
  }
  if (config.reference_board.empty() != config.dsp_rom.empty()) {
    throw blofeld::Fault("--reference-board and --dsp-rom must be supplied together");
  }
  if (!config.reference_board.empty() && (config.inspect || config.port_e_input)) {
    throw blofeld::Fault("--reference-board conflicts with --inspect or --port-e-input");
  }
  if ((!config.midi_file.empty() && !config.midi_device.empty()) ||
      ((!config.midi_file.empty() || !config.midi_device.empty()) &&
       (config.reference_board.empty() || config.inspect))) {
    throw blofeld::Fault("MIDI input requires a reference board and exactly one of --midi-file "
                         "or --midi-device");
  }
  const bool audio = !config.audio_out.empty();
  if (audio != config.audio_rate.has_value() || audio != config.audio_max_frames.has_value() ||
      (audio && (config.reference_board.empty() || config.inspect))) {
    throw blofeld::Fault("Audio output requires a reference board, --audio-out, --audio-rate and "
                         "--audio-max-frames together");
  }
}

Options options(const int count, char **values) {
  Options result;
  for (int index = 1; index < count; ++index) {
    const std::string_view argument(values[index]);
    if (argument == "--inspect") {
      result.inspect = true;
      continue;
    }
    if (index + 1 >= count) { throw blofeld::Fault("Missing value for " + std::string(argument)); }
    const std::string_view value(values[++index]);
    if (argument == "--firmware") {
      result.firmware = value;
    } else if (argument == "--sla") {
      result.specification = value;
    } else if (argument == "--trace") {
      result.trace = value;
    } else if (argument == "--flash-in") {
      result.flash_in = value;
    } else if (argument == "--flash-out") {
      result.flash_out = value;
    } else if (argument == "--reference-board") {
      result.reference_board = value;
    } else if (argument == "--dsp-rom") {
      result.dsp_rom = value;
    } else if (argument == "--audio-out") {
      result.audio_out = value;
    } else if (argument == "--midi-file") {
      result.midi_file = value;
    } else if (argument == "--midi-device") {
      result.midi_device = value;
    } else if (argument == "--audio-rate") {
      result.audio_rate =
          static_cast<std::uint32_t>(positive(value, std::numeric_limits<std::uint32_t>::max()));
    } else if (argument == "--audio-max-frames") {
      result.audio_max_frames =
          static_cast<std::size_t>(positive(value, blofeld::PcmRecording::maximum_capacity));
    } else if (argument == "--port-e-input") {
      result.port_e_input = pin_drive(value);
    } else if (argument == "--instructions") {
      const auto conversion =
          std::from_chars(value.data(), value.data() + value.size(), result.instructions);
      if (conversion.ec != std::errc{} || conversion.ptr != value.data() + value.size()) {
        throw blofeld::Fault("Invalid instruction limit");
      }
    } else {
      throw blofeld::Fault("Unknown argument: " + std::string(argument));
    }
  }
  validate_options(result);
  return result;
}

struct FilePair {
  std::filesystem::path first;
  std::filesystem::path second;
};

void distinct_files(const FilePair &files) {
  if (files.first.empty() || files.second.empty()) { return; }
  const auto first = std::filesystem::weakly_canonical(files.first);
  const auto second = std::filesystem::weakly_canonical(files.second);
  bool same = first == second;
  if (!same && std::filesystem::exists(first) && std::filesystem::exists(second)) {
    same = std::filesystem::equivalent(first, second);
  }
  if (same) { throw blofeld::Fault("Input/output file collision: " + files.first.string()); }
}

void validate_files(const Options &config, const std::filesystem::path &specification) {
  if (!config.audio_out.empty() && config.audio_out.extension() != ".wav") {
    throw blofeld::Fault("Audio output requires the .wav extension");
  }
  for (const auto &path : {config.flash_in, config.flash_out}) {
    if (!path.empty() && path.extension() != ".bfnvm") {
      throw blofeld::Fault("MCU NVM files require the private .bfnvm extension");
    }
  }
  for (const auto &input : {config.firmware,
                            specification,
                            config.flash_in,
                            config.flash_out,
                            config.reference_board,
                            config.dsp_rom,
                            config.midi_file,
                            config.midi_device}) {
    distinct_files({.first = config.trace, .second = input});
    distinct_files({.first = config.audio_out, .second = input});
  }
  distinct_files({.first = config.audio_out, .second = config.trace});
  for (const auto &input : {config.firmware,
                            specification,
                            config.reference_board,
                            config.dsp_rom,
                            config.midi_file,
                            config.midi_device}) {
    distinct_files({.first = config.flash_out, .second = input});
  }
}

void persist_flash(const Options &config, const blofeld::McuBus &bus) {
  if (config.flash_out.empty()) { return; }
  blofeld::save_flash_image(config.flash_out, bus.power_off_flash_image());
  std::cout << "Saved private MCU NVM image: " << config.flash_out << '\n';
}

std::filesystem::path default_specification() {
  std::error_code error;
  const auto executable = std::filesystem::read_symlink("/proc/self/exe", error);
  if (!error) {
    const auto installed = executable.parent_path().parent_path() / "share/blofeld-emu/HC12.sla";
    if (std::filesystem::is_regular_file(installed)) { return installed; }
  }
  return BLOFELD_DEFAULT_SLA;
}

void print_state(std::ostream &output, const blofeld::CpuState &state) {
  output << "pc=" << blofeld::hex(state.pc) << " d=" << blofeld::hex(state.d)
         << " x=" << blofeld::hex(state.x) << " y=" << blofeld::hex(state.y)
         << " sp=" << blofeld::hex(state.sp) << " ccr=" << blofeld::hex(state.ccr, 2);
}

void trace_instruction(std::ofstream &trace,
                       const blofeld::Hc12 &cpu,
                       const std::uint64_t executed) {
  if (!trace.is_open()) { return; }
  trace << executed << ' ';
  print_state(trace, cpu.state());
  if (const auto request = cpu.pending_interrupt()) {
    trace << " interrupt-entry vector=" << blofeld::hex(request->vector) << '\n';
  } else {
    trace << ' ' << cpu.disassemble() << '\n';
  }
}

void trace_interrupt(std::ofstream &trace, const blofeld::CpuStep result) {
  if (!trace.is_open() || !result.interrupt_vector) { return; }
  trace << "  interrupt-vector " << blofeld::hex(*result.interrupt_vector)
        << " stack-reused=" << result.stack_reused << '\n';
}

int run_mcu(const Options &config,
            const blofeld::McuFlashImage &image,
            const std::filesystem::path &specification,
            std::ofstream &trace) {
  blofeld::McuBus bus(image);
  if (config.port_e_input) {
    bus.set_pin_drive(blofeld::McuPort::e, *config.port_e_input);
    std::cout << "Explicit simulated port E input fixture: mask="
              << blofeld::hex(config.port_e_input->mask, 2)
              << " levels=" << blofeld::hex(config.port_e_input->levels, 2)
              << "; board wiring is unverified.\n";
  }
  bus.set_observer([&trace](const blofeld::BusEvent &event) {
    if (trace.is_open()) {
      trace << "  io-write " << blofeld::hex(event.address) << '=' << blofeld::hex(event.value, 2)
            << ' ' << event.name << '\n';
    }
  });
  blofeld::Hc12 cpu(bus, specification);
  std::uint64_t executed = 0;
  std::uint64_t interrupts = 0;
  bool reset_completed = false;
  bool stopped_waiting = false;
  try {
    cpu.reset();
    reset_completed = true;
    while (executed < config.instructions) {
      trace_instruction(trace, cpu, executed);
      const auto result = cpu.step();
      if (result.instruction_executed) { ++executed; }
      if (result.interrupt_vector) { ++interrupts; }
      trace_interrupt(trace, result);
      if (cpu.waiting() && !cpu.pending_interrupt()) {
        stopped_waiting = true;
        break;
      }
    }
  } catch (const blofeld::Fault &fault) {
    if (trace.is_open()) { trace.flush(); }
    std::cerr << "Stopped after " << executed << " completed MCU instructions: " << fault.what()
              << "\n";
    if (reset_completed) {
      print_state(std::cerr, cpu.state());
      std::cerr << '\n';
    } else {
      std::cerr << "MCU reset did not complete; CPU state unavailable.\n";
    }
    persist_flash(config, bus);
    return 2;
  }
  if (trace.is_open()) { trace.flush(); }
  std::cout << (stopped_waiting
                    ? "MCU waiting for an external interrupt; no scheduler supplied a wakeup.\n"
                    : "Instruction budget reached; boot completion is not established.\n");
  std::cout << "Delivered interrupt entries: " << interrupts << '\n';
  print_state(std::cout, cpu.state());
  std::cout << '\n';
  persist_flash(config, bus);
  return 0;
}

void validate_board_options(const Options &config,
                            const std::optional<blofeld::reference::Config> &board) {
  if (!config.audio_out.empty() && (!board || !board->pcm)) {
    throw blofeld::Fault("Audio output requires pcm-receive in the reference board");
  }
  const bool midi = !config.midi_file.empty() || !config.midi_device.empty();
  if (midi && (!board || (!board->midi && !board->midi_dsp) || board->checkpoint)) {
    throw blofeld::Fault("Host MIDI requires midi-sci or midi-dsp-tio and rejects automatic "
                         "checkpoint replay");
  }
}

std::optional<blofeld::reference::AudioOutput> audio_output(const Options &config) {
  if (config.audio_out.empty() || !config.audio_rate || !config.audio_max_frames) { return {}; }
  return blofeld::reference::AudioOutput{.path = config.audio_out,
                                         .sample_rate = *config.audio_rate,
                                         .maximum_frames = *config.audio_max_frames};
}

int run_reference(const Options &config,
                  const blofeld::reference::Config &board,
                  const blofeld::reference::Execution &execution) {
  blofeld::reference::Board machine(board, execution);
  if (config.midi_file.empty() && config.midi_device.empty()) { return machine.run(); }
  const bool raw = config.midi_file.empty();
  blofeld::reference::HostMidiPump input(raw ? config.midi_device : config.midi_file,
                                         raw ? blofeld::MidiInputKind::linux_rawmidi
                                             : blofeld::MidiInputKind::bytes,
                                         board.dsp.clock.timebase_hz);
  return machine.run([&input](auto &target) { input(target); });
}

int run(const Options &config) {
  const auto specification =
      config.specification.empty() ? default_specification() : config.specification;
  validate_files(config, specification);
  const auto board = config.reference_board.empty()
                         ? std::optional<blofeld::reference::Config>{}
                         : blofeld::reference::load_config(config.reference_board);
  const auto rom =
      board ? blofeld::reference::load_rom(config.dsp_rom) : std::vector<std::uint32_t>{};
  const auto firmware = blofeld::Firmware::load(config.firmware);
  validate_board_options(config, board);
  std::cout << "Firmware: " << firmware.mcu.size() << " MCU bytes, " << firmware.dsp.size()
            << " DSP records, " << firmware.sysex_packets << " checked data packets\n";
  if (config.inspect) { return 0; }
  std::ofstream trace;
  if (!config.trace.empty()) {
    trace.open(config.trace);
    if (!trace) { throw blofeld::Fault("Cannot create trace file"); }
    trace.exceptions(std::ios::failbit | std::ios::badbit);
  }
  const auto image = config.flash_in.empty() ? blofeld::McuFlashImage::from_known(firmware.mcu)
                                             : blofeld::load_flash_image(config.flash_in);
  if (board) {
    const blofeld::reference::Execution execution{.image = image,
                                                  .rom = rom,
                                                  .specification = specification,
                                                  .flash_out = config.flash_out,
                                                  .trace = trace.is_open() ? &trace : nullptr,
                                                  .instructions = config.instructions,
                                                  .audio = audio_output(config)};
    return run_reference(config, *board, execution);
  }
  return run_mcu(config, image, specification, trace);
}

} // namespace

int main(const int argc, char **argv) {
  try {
    return run(options(argc, argv));
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
