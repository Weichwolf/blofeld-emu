// SPDX-License-Identifier: GPL-3.0-only
#include "midi/host_midi.hpp"

#include "common/bus.hpp"

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <sound/asound.h>
#include <span>
#include <string>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>

namespace blofeld {
namespace {
constexpr std::size_t maximum_read = 4096;
constexpr unsigned protocol_major_shift = 16;
}

HostMidiInput::HostMidiInput(const std::filesystem::path &path, const MidiInputKind kind)
    : path_(path), kind_(kind) {
  if (kind != MidiInputKind::bytes && kind != MidiInputKind::linux_rawmidi) {
    throw Fault("Unknown host MIDI input kind");
  }
  descriptor_ = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOCTTY);
  if (descriptor_ < 0) { fail("open", errno); }
  try {
    struct stat info{};
    if (::fstat(descriptor_, &info) != 0) { fail("fstat", errno); }
    if (kind == MidiInputKind::bytes) {
      if (!S_ISREG(info.st_mode)) { throw Fault("MIDI byte input requires a regular binary file"); }
    } else {
      if (!S_ISCHR(info.st_mode)) { throw Fault("RawMIDI input requires a character device"); }
      configure_rawmidi();
    }
  } catch (...) {
    ::close(descriptor_);
    descriptor_ = -1;
    throw;
  }
}

HostMidiInput::~HostMidiInput() {
  if (descriptor_ >= 0) { ::close(descriptor_); }
}

void HostMidiInput::fail(const char *operation, const int error) const {
  throw Fault("Host MIDI " + std::string(operation) + " path=" + path_.string() + ": " +
              std::error_code(error, std::generic_category()).message());
}

void HostMidiInput::configure_rawmidi() {
  int protocol = 0;
  if (::ioctl(descriptor_, SNDRV_RAWMIDI_IOCTL_PVERSION, &protocol) < 0) { fail("version", errno); }
  if ((static_cast<unsigned>(protocol) >> protocol_major_shift) != 2) {
    throw Fault("Unsupported Linux RawMIDI protocol version");
  }
  snd_rawmidi_info info{};
  info.stream = SNDRV_RAWMIDI_STREAM_INPUT;
  if (::ioctl(descriptor_, SNDRV_RAWMIDI_IOCTL_INFO, &info) < 0) { fail("info", errno); }
  if ((info.flags & SNDRV_RAWMIDI_INFO_INPUT) == 0 ||
      (info.flags & (SNDRV_RAWMIDI_INFO_UMP | SNDRV_RAWMIDI_INFO_STREAM_INACTIVE)) != 0) {
    throw Fault("Host MIDI requires an active legacy MIDI-byte input, not a UMP endpoint");
  }
  if (info.subdevice != 0) {
    throw Fault("RawMIDI input subdevice=" + std::to_string(info.subdevice) +
                " is unsupported; refuse automatic fallback from subdevice 0");
  }
  snd_rawmidi_params parameters{};
  parameters.stream = SNDRV_RAWMIDI_STREAM_INPUT;
  parameters.buffer_size = maximum_read;
  parameters.avail_min = 1;
  parameters.no_active_sensing = 1;
  parameters.mode = SNDRV_RAWMIDI_MODE_FRAMING_NONE;
  if (::ioctl(descriptor_, SNDRV_RAWMIDI_IOCTL_PARAMS, &parameters) < 0) { fail("params", errno); }
  check_status();
}

void HostMidiInput::check_status() const {
  if (kind_ != MidiInputKind::linux_rawmidi) { return; }
  snd_rawmidi_status status{};
  status.stream = SNDRV_RAWMIDI_STREAM_INPUT;
  if (::ioctl(descriptor_, SNDRV_RAWMIDI_IOCTL_STATUS, &status) < 0) { fail("status", errno); }
  if (status.xruns != 0) {
    throw Fault("Host MIDI input path=" + path_.string() + " lost " + std::to_string(status.xruns) +
                " bytes in the kernel");
  }
}

std::size_t HostMidiInput::read(const std::span<std::uint8_t> destination) {
  if (faulted_) { throw Fault("Host MIDI failure requires a new input session"); }
  if (destination.size() > maximum_read) { throw Fault("Host MIDI read exceeds the bounded span"); }
  try {
    check_status(); // Check loss even when downstream backpressure admits no bytes.
    if (exhausted_ || destination.empty()) { return 0; }
    const auto count = ::read(descriptor_, destination.data(), destination.size());
    if (count < 0 && errno != EAGAIN && errno != EINTR) { fail("read", errno); }
    check_status(); // Do not deliver a prefix from a read that overlapped observed data loss.
    if (count < 0) { return 0; }
    if (count == 0) {
      if (kind_ == MidiInputKind::linux_rawmidi) { throw Fault("Host MIDI device reached EOF"); }
      exhausted_ = true;
    }
    return static_cast<std::size_t>(count);
  } catch (...) {
    faulted_ = true;
    throw;
  }
}
}
