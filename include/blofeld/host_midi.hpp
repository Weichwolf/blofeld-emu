// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>

namespace blofeld {
enum class MidiInputKind : std::uint8_t { bytes, linux_rawmidi };

// Host transport only. Opens no output stream and performs no emulation or MIDI parsing.
class HostMidiInput {
public:
  HostMidiInput(const std::filesystem::path &path, MidiInputKind kind);
  ~HostMidiInput();
  HostMidiInput(const HostMidiInput &) = delete;
  HostMidiInput &operator=(const HostMidiInput &) = delete;
  HostMidiInput(HostMidiInput &&) = delete;
  HostMidiInput &operator=(HostMidiInput &&) = delete;
  [[nodiscard]] std::size_t read(std::span<std::uint8_t> destination);

  [[nodiscard]] bool exhausted() const { return exhausted_; }

private:
  void configure_rawmidi();
  void check_status() const;
  [[noreturn]] void fail(const char *operation, int error) const;
  std::filesystem::path path_;
  MidiInputKind kind_;
  int descriptor_ = -1;
  bool exhausted_ = false;
  bool faulted_ = false;
};
}
