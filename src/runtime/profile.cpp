// SPDX-License-Identifier: GPL-3.0-only
#include "runtime/profile.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "runtime/instrument.hpp"
#include "runtime/mcu_dsp_runtime.hpp"
#include "storage/nand.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/private_file.hpp"

#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <istream>
#include <set>
#include <sstream>
#include <string>

namespace blofeld::runtime {
Profile load_profile(const std::filesystem::path &path) {
  constexpr unsigned maximum_bytes = 16384;
  const auto bytes = read_private_file(path, maximum_bytes);
  const std::string text(bytes.begin(), bytes.end());
  if (text.contains('\0')) { throw Fault("NUL in instrument profile"); }
  std::istringstream input(text);
  Profile profile;
  std::set<std::string> seen;
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream fields(line);
    fields >> std::ws;
    if (fields.eof() || fields.peek() == '#') { continue; }
    std::string key;
    std::string value;
    std::string extra;
    if (!(fields >> key >> std::quoted(value)) || value.empty() || (fields >> extra) ||
        !seen.insert(key).second) {
      throw Fault("Invalid or duplicate instrument profile field: " + line);
    }
    const auto file = std::filesystem::path(value).is_absolute() ? std::filesystem::path(value)
                                                                 : path.parent_path() / value;
    if (key == "firmware") {
      profile.firmware = file;
    } else if (key == "boot-rom") {
      profile.boot_rom = file;
    } else if (key == "device-state") {
      profile.device_state = file;
    } else if (key == "nand") {
      profile.nand = file;
    } else if (key == "display") {
      profile.display = file;
    } else {
      throw Fault("Unknown instrument profile field: " + key);
    }
  }
  if (profile.firmware.empty() || profile.boot_rom.empty() || profile.device_state.empty() ||
      profile.nand.empty()) {
    throw Fault("Profile requires firmware, boot-rom, device-state and nand paths");
  }
  return profile;
}

namespace {
constexpr unsigned mcu_oscillator_hz = 12000000;
constexpr unsigned audio_oscillator_hz = 11289600;
constexpr unsigned reset_mode_mask = 15;
constexpr unsigned spi_boot_mode = 5;
constexpr unsigned data_byte_mask = 0xff;
constexpr unsigned lcd_reset_intervals = 10000;
constexpr unsigned serial_idle_high = 0x82;
constexpr unsigned serial_idle_low = 0x7d;
constexpr unsigned startup_input_mask = 0xfa;
constexpr std::uint64_t phy_ready_at = 17516520;
constexpr std::uint64_t usb_ready_at = 4964626296;
constexpr unsigned usb_input_mask = 0xea;
constexpr std::uint64_t timer_ready_at = 4965854040;
constexpr unsigned audio_destinations = 6;
constexpr std::uint64_t pcm_start_at = 167650560000;
}

Config instrument_config(const Profile &profile) {
  Config result;
  result.mcu.clock = {.oscillator_hz = mcu_oscillator_hz,
                      .switch_timing = McuRunClockSwitch::reference_cycle_boundary};
  result.mcu.memory_timing = McuRunMemory::internal_flash_reads;
  result.dsp_clock = {.extal_hz = audio_oscillator_hz,
                      .timebase_hz = timebase_hz,
                      .switch_timing = DspClockSwitch::functional_region_boundary};
  result.dsp.initial_reset = {.mode_known = reset_mode_mask,
                              .mode_levels = spi_boot_mode,
                              .pinit = false,
                              .recovery_cycles = 2};
  result.dsp.reset_dsp_first = true;
  result.dma_grant = DspDmaGrant::available;
  result.power_on = {.inputs = {.synchronization_cycles = 3}, .reset_pad = McuResetPad::high};
  result.nand = {
      .continuation = NandReadContinuation::k9f1208_j,
      .source_path = profile.nand,
      .write_protect_n = true,
      .cell_profile = {.contract = NandCellContract::reference_pulse_inhibit, .cells = {}}};
  result.gpio_peer = {.data_pull_down = data_byte_mask};
  result.mcu_u0_release = true;
  result.lcd_parallel = {.reset_release_ticks = timebase_hz / lcd_reset_intervals};
  result.port_b_pulls = {.up = serial_idle_high, .down = serial_idle_low};
  result.select_n = 2;
  result.clock = 4;
  result.mosi = 16;
  result.loopback = {
      .source = McuPort::e, .source_mask = 4, .destination = McuPort::e, .destination_mask = 8};
  // Retain the existing functional startup inputs; their physical timing is unqualified.
  result.initial = {{.port = McuPort::e, .drive = {.mask = startup_input_mask, .levels = 2}},
                    {.port = McuPort::r, .drive = {.mask = 4, .levels = 4}}};
  result.events = {{.ticks = phy_ready_at, .kind = InputKind::phy_window, .value = reset_mode_mask},
                   {.ticks = usb_ready_at, .kind = InputKind::usb_decoder, .value = 8},
                   {.ticks = usb_ready_at,
                    .port = McuPort::e,
                    .drive = {.mask = usb_input_mask, .levels = 2, .floating = 0x10}},
                   {.ticks = timer_ready_at, .port = McuPort::t, .drive = {.mask = 1, .levels = 1}},
                   {.ticks = timer_ready_at, .kind = InputKind::timer_inputs, .value = 1}};
  result.audio_clock = {.frequency_hz = audio_oscillator_hz,
                        .destinations = audio_destinations,
                        .scheduling = AudioClockScheduling::transmitter_events};
  result.pcm = {.instance = 1, .after_ticks = pcm_start_at, .format = {.frame_sync_high = false}};
  return result;
}
}
