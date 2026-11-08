// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "runtime/instrument.hpp"
#include "usb/usb_midi_host.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <optional>

namespace blofeld::test {
// A real USB host, not an injected firmware queue or synthetic note handler.
// This stimulus is diagnostic; ACKed packets alone do not prove instrument sound.
class UsbMidiStimulus {
public:
  explicit UsbMidiStimulus(const std::uint64_t timebase)
      : host_(timebase / frames_per_second, timebase), interval_(timebase / 4U) {}

  void poll(runtime::Instrument &instrument, const bool audio_started) {
    for (const auto &completion : instrument.poll_usb(host_)) {
      if (!completion.input_direction) {
        output_bytes_ += completion.transferred;
        if (completion.sequence == first_note_) { note_acknowledged_ = true; }
      } else {
        input_bytes_ += completion.transferred;
      }
    }
    if (!host_.ready() || !audio_started) { return; }
    if (!prepared_) {
      // Select A001 through ordinary bank/program messages and set audible
      // channel controls. Neither program data nor guest state is injected.
      constexpr std::array<std::uint8_t, 24> setup{11, 0xb0, 0,  0,   11, 0xb0, 32, 0,
                                                   12, 0xc0, 0,  0,   11, 0xb0, 7,  100,
                                                   11, 0xb0, 11, 127, 11, 0xb0, 64, 0};
      static_cast<void>(host_.send_packets(setup));
      prepared_ = true;
      return;
    }
    const auto ticks = instrument.common_ticks();
    if (!last_event_ || ticks - *last_event_ >= interval_) {
      constexpr std::uint8_t note_on = 0x90;
      constexpr std::uint8_t note_off = 0x80;
      constexpr std::uint8_t pitch = 60;
      constexpr std::uint8_t velocity = 100;
      const std::array<std::uint8_t, 4> packet{static_cast<std::uint8_t>(gate_ ? 8 : 9),
                                               gate_ ? note_off : note_on,
                                               pitch,
                                               static_cast<std::uint8_t>(gate_ ? 0 : velocity)};
      const auto sequence = host_.send_packets(packet);
      if (!first_note_) { first_note_ = sequence; }
      gate_ = !gate_;
      last_event_ = ticks;
    }
  }

  void report() const {
    std::cout << "usb-midi-configured=" << host_.ready() << " note-on-acked=" << note_acknowledged_
              << " usb-midi-out-bytes=" << output_bytes_ << " usb-midi-in-bytes=" << input_bytes_
              << '\n';
  }

private:
  static constexpr unsigned frames_per_second = 1000;
  UsbMidiHost host_;
  std::uint64_t interval_;
  std::optional<std::uint64_t> last_event_;
  std::optional<std::uint32_t> first_note_;
  std::uint64_t output_bytes_ = 0;
  std::uint64_t input_bytes_ = 0;
  bool prepared_ = false;
  bool gate_ = false;
  bool note_acknowledged_ = false;
};
}
