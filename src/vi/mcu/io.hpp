// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/port.hpp"
#include "display/state.hpp"
#include "io/controls.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/mcu_flash.hpp"
#include "io/peer_port.hpp"
#include "io/sci.hpp"
#include "io/shi.hpp"
#include "io/usb.hpp"
#include "mcu/channels.hpp"
#include "mcu/clock.hpp"
#include "mcu/devices.hpp"
#include "mcu/memory.hpp"
#include "mcu/ports.hpp"
#include "mcu/timer.hpp"
#include "storage/flash_image.hpp"

#include <cpu12/core.hpp>
#include <cstdint>
#include <optional>
#include <stop_token>

namespace blofeld::vi::mcu {
class Io {
  static constexpr std::uint16_t peer_q_data = 0x260;
  static constexpr std::uint16_t peer_q_input = 0x261;
  static constexpr std::uint16_t peer_q_direction = 0x262;
  static constexpr std::uint16_t peer_j_data = 0x250;
  static constexpr std::uint16_t peer_j_direction = 0x252;
  static constexpr std::uint16_t mapping_first = 0x10;
  static constexpr std::uint16_t mapping_last = 0x13;
  static constexpr std::uint8_t all_bits = 0xff;

public:
  Io(Memory &memory,
     const FlashImage &image,
     Devices &devices,
     Shi &shi,
     DspReset &reset,
     const std::stop_token &stop,
     const Controls *controls,
     DisplayState *display,
     PeerPort *peer)
      : memory_(memory),
        flash_(image),
        ports_(shi),
        devices_(devices),
        reset_(reset),
        stop_(stop),
        controls_(controls),
        display_state_(display),
        peer_(peer) {
    // B3/B5..B7 are passive controls, with no input producer in the firmware
    // contract. Use an inactive sample while they are inputs, not a device reply.
    constexpr std::uint8_t passive_controls = 0xe8;
    // DSP56371UM table 2-7: SS, SCK and MOSI have internal pull-ups. The
    // application releases B1/B2/B4 after boot; this is bias, not HREQ/data.
    constexpr std::uint8_t spi_pullups = 0x16;
    constexpr std::uint8_t attached = passive_controls | spi_pullups;
    ports_.input(Port::b, {.mask = attached, .known = attached, .value = spi_pullups});
    // DSP56371 RESET has an internal pull-up. U0 drives it low or releases it.
    ports_.input(Port::u, {.mask = 1, .known = 1, .value = 1});
  }

  void bind(cpu12::Core &core) noexcept { core_ = &core; }

  cpu12::Io callbacks() noexcept {
    return {.owner = this,
            .read =
                [](void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
                  return static_cast<Io *>(owner)->read(address, value);
                },
            .write =
                [](void *owner, const cpu12::ByteWrite access) noexcept {
                  return static_cast<Io *>(owner)->write(access);
                },
            .read_word =
                [](void *owner, const std::uint16_t address, std::uint16_t &value) noexcept {
                  return static_cast<Io *>(owner)->read_word(address, value);
                },
            .write_word =
                [](void *owner, const cpu12::WordWrite access) noexcept {
                  return static_cast<Io *>(owner)->write_word(access);
                },
            .fetch =
                [](void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
                  return static_cast<Io *>(owner)->read(address, value);
                },
            .read_masked =
                [](void *owner, const cpu12::MaskedByteRead access, std::uint8_t &value) noexcept {
                  return static_cast<Io *>(owner)->read(access.address, value, access.mask);
                }};
  }

  void capture_display() const {
    if (display_state_ != nullptr) { display_state_->capture(display_.controller()); }
  }

  bool complete_events() {
    panel_sampled_ = false;
    capture_display();
    if (!devices_.advance(*core_) ||
        (devices_.time != nullptr &&
         !blofeld_mcu_time_publish(devices_.time, devices_.clock.ticks()))) {
      return false;
    }
    if (release_reset_) {
      if (peer_ != nullptr && !peer_->release_mcu_data(devices_.clock.ticks(), stop_)) {
        return false;
      }
      if (devices_.time != nullptr) { devices_.time->release_dsp(devices_.clock.ticks()); }
      if (!reset_.release(stop_)) { return false; }
      release_reset_ = false;
    }
    if (!ports_.boot_pending()) { return true; }
    return shared_access([&] { return ports_.complete_boot_transfer(stop_); });
  }

  [[nodiscard]] const char *error() const noexcept {
    if (peer_ != nullptr && peer_->error() != nullptr) { return peer_->error(); }
    return display_.error() != nullptr ? display_.error() : devices_.error();
  }

private:
  [[nodiscard]] bool peer_interrupt() const noexcept {
    std::uint64_t generation = 0;
    return blofeld_mcu_peer_interrupt(peer_, generation);
  }

  [[nodiscard]] PeerPort::Sample peer_data() const noexcept {
    PeerPort::Sample sample;
    blofeld_mcu_peer_data(peer_, sample);
    return sample;
  }

  [[nodiscard]] std::optional<PeerPort::Sample> read_peer(const std::uint8_t mask,
                                                          const std::uint64_t ticks) noexcept {
    PeerPort::Sample sample;
    if (blofeld_mcu_peer_read(peer_, mask, ticks, sample)) { return sample; }
    return {};
  }

  template <typename Action> bool shared_access(const Action &action) noexcept {
    panel_sampled_ = false;
    auto *time = devices_.time;
    if (time == nullptr) { return action(); }
    if (!devices_.advance(*core_) || !blofeld_mcu_time_enter(time, devices_.clock.ticks(), stop_)) {
      return false;
    }
    const bool accepted = action();
    blofeld_mcu_time_leave(time);
    return accepted;
  }

  [[nodiscard]] bool shared_write(const Ports::Write access) const noexcept {
    constexpr std::uint16_t data_u = 0x278;
    constexpr std::uint16_t direction_u = 0x27a;
    if (access.offset != peer_j_data && access.offset != peer_j_direction) {
      return access.offset == data_u || access.offset == direction_u;
    }
    const auto output = ports_.output(Port::j);
    if (!output || (output->known & 1) == 0) { return true; }
    const bool before = (output->direction & 1) != 0 && (output->value & 1) == 0;
    const auto direction = access.offset == peer_j_direction ? access.value : output->direction;
    const auto value = access.offset == peer_j_data ? access.value : output->value;
    const bool after = (direction & 1) != 0 && (value & 1) == 0;
    return before != after;
  }

  bool
  read_port(const std::uint16_t offset, std::uint8_t &value, const std::uint8_t mask) noexcept {
    const auto action = [&] {
      return sample_inputs(offset, mask) && ports_.read(offset, value, mask);
    };
    bool input = offset == peer_q_input;
    if (offset == peer_q_data) {
      const auto output = ports_.output(Port::q);
      input = !output || (output->direction & mask) != mask;
    }
    if (!input) { return action(); }
    if (panel_sampled_ && (peer_data().mask & mask) == 0) { return action(); }
    const bool accepted = shared_access(action);
    panel_sampled_ = accepted && peer_ != nullptr && local_input_bank() && peer_data().mask == 0 &&
                     !peer_interrupt();
    return accepted;
  }

  [[nodiscard]] bool local_input_bank() const noexcept {
    constexpr std::uint8_t banks = 0xc0;
    constexpr std::uint8_t bank_a = 0x40;
    constexpr std::uint8_t bank_b = 0x80;
    const auto select = ports_.output(Port::s);
    const auto request = ports_.output(Port::j);
    const auto data = ports_.output(Port::q);
    if (!select || !request || !data || data->direction != 0 ||
        (select->direction & select->known & banks) != banks ||
        (request->direction & request->known & request->value & 1) == 0) {
      return false;
    }
    const auto selected = select->value & banks;
    return selected == bank_a || selected == bank_b;
  }

  template <typename Action> bool access_devices(const Action &action) noexcept {
    panel_sampled_ = false;
    const auto pending = devices_.pending();
    const auto deadline = devices_.deadline();
    const auto enabled = devices_.interrupts_enabled();
    if (!devices_.advance(*core_) || !action()) { return false; }
    if (devices_.time != nullptr &&
        !blofeld_mcu_time_publish(devices_.time, devices_.clock.ticks())) {
      return false;
    }
    if (pending != devices_.pending() || deadline != devices_.deadline() ||
        enabled != devices_.interrupts_enabled()) {
      core_->request_yield();
    }
    return true;
  }

  bool interrupted() noexcept {
    if (peer_ != nullptr && peer_interrupt()) {
      panel_sampled_ = false;
      core_->request_yield();
    }
    if (!stop_.stop_requested()) { return false; }
    core_->request_yield();
    return true;
  }

  bool read(const std::uint16_t address,
            std::uint8_t &value,
            const std::uint8_t mask = all_bits) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(address);
    if (at.space == Space::registers && McuFlash::contains(at.offset)) {
      return flash_.read(at.offset, value);
    }
    if (at.space == Space::registers && Usb::contains(at.offset)) {
      std::uint16_t word = 0;
      const auto accepted =
          access_devices([&] { return devices_.usb.read({.offset = at.offset}, word); });
      value = static_cast<std::uint8_t>(word);
      return accepted;
    }
    if (at.space == Space::registers && Sci::contains(at.offset)) {
      std::uint16_t word = 0;
      const auto accepted =
          access_devices([&] { return devices_.sci.read({.offset = at.offset}, word); });
      value = static_cast<std::uint8_t>(word);
      return accepted;
    }
    if (at.space == Space::registers && Clock::contains(at.offset)) {
      return access_devices([&] { return devices_.clock.read(at.offset, value); });
    }
    if (at.space == Space::registers && Timer::contains(at.offset)) {
      std::uint16_t word = 0;
      const auto accepted =
          access_devices([&] { return devices_.timer.read({.offset = at.offset}, word); });
      value = static_cast<std::uint8_t>(word);
      return accepted;
    }
    if (at.space == Space::registers && Ports::contains(at.offset)) {
      if (at.offset == 0 && display_.reading()) { return display_.read(value, mask); }
      return read_port(at.offset, value, mask);
    }
    if (at.space == Space::registers && at.offset >= mapping_first && at.offset <= mapping_last) {
      value = memory_.read_mapping(at.offset);
      return true;
    }
    return memory_.read(address, value);
  }

  bool write_usb(const Usb::Write access) noexcept {
    return access_devices([&] {
      if (!devices_.usb.write(access, devices_.clock.usb_module_control())) { return false; }
      if (devices_.usb.mapping_pending()) { core_->request_yield(); }
      return true;
    });
  }

  bool sample_inputs(const std::uint16_t offset, const std::uint8_t mask) {
    if (peer_ != nullptr && offset == 8) { package_input(); }
    if (peer_ != nullptr && (offset == peer_q_data || offset == peer_q_input)) {
      const auto ticks = devices_.timestamp(*core_);
      if (!ticks) { return false; }
      const auto sample =
          offset == peer_q_data ? read_peer(mask, *ticks) : std::optional{peer_data()};
      if (!sample) { return false; }
      // DSP56371UM Table 2-8 documents internal pull-downs on PC0..7,
      // including reset-disconnected GPIO. They are not active reply data.
      ports_.input(Port::q,
                   {.mask = sample->mask,
                    .known = sample->known,
                    .value = sample->value,
                    .fallback_low = all_bits});
    }
    constexpr std::uint16_t port_m = 0x248;
    constexpr std::uint16_t input_m = 0x249;
    if (offset == port_m || offset == input_m) {
      constexpr std::uint8_t pedal_bit = 8;
      ports_.input(
          Port::m,
          {.mask = pedal_bit,
           .known = pedal_bit,
           .value = controls_ != nullptr && controls_->pedal() ? pedal_bit : std::uint8_t{0}});
    }
    return true;
  }

  void package_input() {
    constexpr std::uint8_t probe_output = 4;
    constexpr std::uint8_t package_sense = 8;
    const auto output = ports_.output(Port::e);
    const bool driven = output && (output->direction & probe_output) != 0;
    // The runtime Q interface requires the resident probe's 100-pin route.
    // Sample the driven E2 level at E3; this is a profile input, not a peer reply.
    ports_.input(
        Port::e,
        {.mask = driven ? package_sense : std::uint8_t{0},
         .known = driven && (output->known & probe_output) != 0 ? package_sense : std::uint8_t{0},
         .value = driven && (output->value & probe_output) != 0 ? package_sense : std::uint8_t{0}});
  }

  bool publish_peer(const std::uint16_t offset) noexcept {
    if (peer_ == nullptr || (offset != peer_q_data && offset != peer_q_direction &&
                             offset != peer_j_data && offset != peer_j_direction)) {
      return true;
    }
    const auto data = ports_.output(Port::q);
    const auto request = ports_.output(Port::j);
    if (!data || !request || (request->direction & ~request->known & 1) != 0) { return false; }
    if (!blofeld_mcu_peer_publish(
            peer_,
            {.mask = data->direction, .known = data->known, .value = data->value},
            devices_.clock.ticks(),
            stop_)) {
      return false;
    }
    const bool active = (request->direction & 1) != 0 && (request->value & 1) == 0;
    if (!blofeld_mcu_peer_strobe(peer_, active)) { return false; }
    if (active != peer_strobe_) {
      peer_strobe_ = active;
      core_->request_yield();
    }
    return true;
  }

  bool write_port(const Ports::Write access) noexcept {
    panel_sampled_ = false;
    constexpr std::uint16_t data_u = 0x278;
    constexpr std::uint16_t direction_u = 0x27a;
    if (!ports_.write(access) || !display_.write(ports_, access) || !publish_peer(access.offset)) {
      return false;
    }
    if (ports_.boot_pending()) { core_->request_yield(); }
    if (access.offset != data_u && access.offset != direction_u) { return true; }
    const auto output = ports_.output(Port::u);
    if (!output || ((output->direction & 1) != 0 && (output->known & 1) == 0)) { return false; }
    const bool held = (output->direction & 1) != 0 && (output->value & 1) == 0;
    DspReset::State reset;
    blofeld_mcu_reset_state(&reset_, reset);
    const bool previous = reset.held;
    if (held && !previous) {
      ports_.reset_boot_transfer();
      if (peer_ != nullptr) { blofeld_mcu_peer_hold(peer_); }
      if (!blofeld_mcu_reset_assert(&reset_)) { return false; }
      if (devices_.time != nullptr) { blofeld_mcu_time_hold(devices_.time); }
    } else if (!held && previous) {
      release_reset_ = true;
      core_->request_yield();
    }
    ports_.input(Port::u,
                 {.mask = 1, .known = 1, .value = held ? std::uint8_t{0} : std::uint8_t{1}});
    return true;
  }

  bool write(const cpu12::ByteWrite access) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(access.address);
    if (at.space == Space::flash) {
      return flash_.write_array({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && McuFlash::contains(at.offset)) {
      return flash_.write({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && Usb::contains(at.offset)) {
      return write_usb({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && Sci::contains(at.offset)) {
      return access_devices(
          [&] { return devices_.sci.write({.offset = at.offset, .value = access.value}); });
    }
    if (at.space == Space::registers && Clock::contains(at.offset)) {
      return access_devices(
          [&] { return devices_.clock.write({.offset = at.offset, .value = access.value}); });
    }
    if (at.space == Space::registers && Timer::contains(at.offset)) {
      return access_devices(
          [&] { return devices_.timer.write({.offset = at.offset, .value = access.value}); });
    }
    if (at.space == Space::registers && Ports::contains(at.offset)) {
      const auto action = [&] { return write_port({.offset = at.offset, .value = access.value}); };
      if (at.offset == peer_q_data || at.offset == peer_q_direction) {
        return access_devices(action);
      }
      return shared_write({.offset = at.offset, .value = access.value}) ? shared_access(action)
                                                                        : action();
    }
    if (at.space == Space::registers && at.offset >= mapping_first && at.offset <= mapping_last) {
      if (memory_.write_mapping({.address = at.offset, .value = access.value})) {
        core_->request_yield();
      }
      return true;
    }
    const bool accepted = memory_.write(access);
    if (memory_.program_changed()) { core_->request_yield(); }
    return accepted;
  }

  bool read_word(const std::uint16_t address, std::uint16_t &value) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(address);
    if (at.space == Space::registers && Usb::contains(at.offset) &&
        Usb::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices(
          [&] { return devices_.usb.read({.offset = at.offset, .bytes = 2}, value); });
    }
    if (at.space == Space::registers && Sci::contains(at.offset) &&
        Sci::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices(
          [&] { return devices_.sci.read({.offset = at.offset, .bytes = 2}, value); });
    }
    if (at.space == Space::registers && Timer::contains(at.offset) &&
        Timer::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices(
          [&] { return devices_.timer.read({.offset = at.offset, .bytes = 2}, value); });
    }
    std::uint8_t high = 0;
    std::uint8_t low = 0;
    if (!read(address, high) || !read(static_cast<std::uint16_t>(address + 1), low)) {
      return false;
    }
    value = static_cast<std::uint16_t>((unsigned{high} << 8U) | low);
    return true;
  }

  bool write_word(const cpu12::WordWrite access) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(access.address);
    const auto next = memory_.locate(static_cast<std::uint16_t>(access.address + 1));
    if (at.space == Space::flash && next.space == Space::flash && next.offset == at.offset + 1) {
      return flash_.write_array({.offset = at.offset, .value = access.value, .bytes = 2});
    }
    if (at.space == Space::registers && Usb::contains(at.offset) &&
        Usb::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return write_usb({.offset = at.offset, .value = access.value, .bytes = 2});
    }
    if (at.space == Space::registers && Sci::contains(at.offset) &&
        Sci::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices([&] {
        return devices_.sci.write({.offset = at.offset, .value = access.value, .bytes = 2});
      });
    }
    if (at.space == Space::registers && Clock::contains(at.offset) &&
        Clock::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices([&] {
        return devices_.clock.write({.offset = at.offset, .value = access.value, .bytes = 2});
      });
    }
    if (at.space == Space::registers && Timer::contains(at.offset) &&
        Timer::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices([&] {
        return devices_.timer.write({.offset = at.offset, .value = access.value, .bytes = 2});
      });
    }
    return write({.address = access.address,
                  .value = static_cast<std::uint8_t>(access.value >> 8U)}) &&
           write({.address = static_cast<std::uint16_t>(access.address + 1),
                  .value = static_cast<std::uint8_t>(access.value & all_bits)});
  }

  Memory &memory_;
  McuFlash flash_;
  Ports ports_;
  DisplayPort display_;
  Devices &devices_;
  DspReset &reset_;
  bool release_reset_ = false;
  const std::stop_token &stop_;
  const Controls *controls_;
  DisplayState *display_state_;
  PeerPort *peer_;
  bool peer_strobe_ = false;
  bool panel_sampled_ = false;
  cpu12::Core *core_ = nullptr;
};
}
