// SPDX-License-Identifier: GPL-3.0-only
#include "midi/smf.hpp"

#include "midi/time.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#if defined(__APPLE__)
#include <sys/fcntl.h>
#else
#include <fcntl.h>
#endif
#include <system_error>
#include <unistd.h>
#include <vector>

namespace blofeld::host::midi {
namespace {
constexpr std::uint32_t header_signature = 0x4d546864, track_signature = 0x4d54726b;
constexpr std::uint8_t sysex_start = 0xf0, sysex_end = 0xf7, meta_status = 0xff;
constexpr std::uint8_t end_track = 0x2f, set_tempo = 0x51, data_mask = 0x7f;
constexpr std::uint8_t program_change = 0xc0, channel_pressure = 0xd0;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned header_length = 6;
constexpr std::uint32_t default_tempo = 500000, microseconds_per_second = 1000000;
constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();

struct Descriptor {
  explicit Descriptor(const std::filesystem::path &path)
      : value(::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW)) {
    if (value < 0) { throw std::system_error(errno, std::generic_category(), "Open SMF input"); }
  }

  ~Descriptor() { ::close(value); }

  Descriptor(const Descriptor &) = delete;
  Descriptor &operator=(const Descriptor &) = delete;
  int value;
};

struct stat information(const int file) {
  struct stat value{};
  if (::fstat(file, &value) != 0) {
    throw std::system_error(errno, std::generic_category(), "Inspect SMF input");
  }
  return value;
}

class File {
public:
  explicit File(const std::filesystem::path &path)
      : file_(path), before_(information(file_.value)) {
    if (!S_ISREG(before_.st_mode) || before_.st_size < 0) {
      throw std::runtime_error("SMF input must be a regular file");
    }
  }

  [[nodiscard]] std::uint64_t size() const noexcept {
    return static_cast<std::uint64_t>(before_.st_size);
  }

  void read(std::uint64_t offset, std::span<std::uint8_t> bytes) const {
    assert(offset <= size() && bytes.size() <= size() - offset);
    while (!bytes.empty()) {
      const auto count = ::pread(
          file_.value, bytes.data(), bytes.size(), static_cast<decltype(before_.st_size)>(offset));
      if (count < 0) {
        if (errno == EINTR) { continue; }
        throw std::system_error(errno, std::generic_category(), "Read SMF input");
      }
      if (count == 0) { throw std::runtime_error("SMF input was truncated while reading"); }
      const auto received = static_cast<std::size_t>(count);
      offset += received;
      bytes = bytes.subspan(received);
    }
  }

  void unchanged() const {
    const auto after = information(file_.value);
#if defined(__APPLE__)
    const auto &modified = before_.st_mtimespec;
    const auto &modified_after = after.st_mtimespec;
    const auto &changed = before_.st_ctimespec;
    const auto &changed_after = after.st_ctimespec;
#else
    const auto &modified = before_.st_mtim;
    const auto &modified_after = after.st_mtim;
    const auto &changed = before_.st_ctim;
    const auto &changed_after = after.st_ctim;
#endif
    if (before_.st_size != after.st_size || modified.tv_sec != modified_after.tv_sec ||
        modified.tv_nsec != modified_after.tv_nsec || changed.tv_sec != changed_after.tv_sec ||
        changed.tv_nsec != changed_after.tv_nsec) {
      throw std::runtime_error("SMF input changed while streaming");
    }
  }

private:
  Descriptor file_;
  struct stat before_;
};

struct Range {
  std::uint64_t first, end;
};

class Cursor {
public:
  Cursor(const File &file, const Range range)
      : file_(&file), position_(range.first), end_(range.end) {}

  [[nodiscard]] std::uint64_t position() const noexcept { return position_; }

  [[nodiscard]] bool empty() const noexcept { return position_ == end_; }

  void skip(const std::uint64_t bytes) {
    if (bytes > end_ - position_) { fail(); }
    position_ += bytes;
  }

  void read(std::span<std::uint8_t> bytes) {
    if (bytes.size() > end_ - position_) { fail(); }
    while (!bytes.empty()) {
      const auto available = cached();
      const auto count = std::min(available.size(), bytes.size());
      std::ranges::copy(available.first(count), bytes.begin());
      position_ += count;
      bytes = bytes.subspan(count);
    }
  }

  std::uint8_t byte() {
    const auto result = cached()[0];
    ++position_;
    return result;
  }

  std::uint32_t big(const unsigned bytes) {
    std::uint32_t result = 0;
    for (unsigned index = 0; index < bytes; ++index) { result = (result << 8U) | byte(); }
    return result;
  }

  std::uint32_t vlq() {
    std::uint32_t result = 0;
    for (unsigned index = 0; index < 4; ++index) {
      const auto value = byte();
      result = (result << 7U) | (value & data_mask);
      if (value < 0x80) { return result; }
    }
    throw std::runtime_error("SMF variable-length quantity exceeds four bytes");
  }

private:
  [[noreturn]] void fail() const {
    throw std::runtime_error("SMF chunk is truncated at byte " + std::to_string(position_));
  }

  std::span<const std::uint8_t> cached() {
    if (empty()) { fail(); }
    if (position_ < cached_first_ || position_ >= cached_end_) {
      const auto count =
          static_cast<std::size_t>(std::min<std::uint64_t>(buffer_.size(), end_ - position_));
      file_->read(position_, std::span(buffer_).first(count));
      cached_first_ = position_;
      cached_end_ = position_ + count;
    }
    return std::span(buffer_).subspan(static_cast<std::size_t>(position_ - cached_first_),
                                      static_cast<std::size_t>(cached_end_ - position_));
  }

  const File *file_;
  std::uint64_t position_, end_, cached_first_ = 0, cached_end_ = 0;
  std::array<std::uint8_t, 512> buffer_{};
};

struct Clock {
  Time time;
  std::uint32_t unit = default_tempo;
  bool metrical = true;

  explicit Clock(const unsigned division) {
    if ((division & 0x8000U) == 0) {
      if (division == 0) { throw std::runtime_error("SMF quarter-note division is zero"); }
      time.divisor = division;
      return;
    }
    metrical = false;
    const auto subdivision = division & byte_mask;
    if (subdivision == 0) { throw std::runtime_error("SMF time-code subdivision is zero"); }
    constexpr unsigned code_24 = 0xe8;
    constexpr unsigned code_25 = 0xe7;
    constexpr unsigned code_drop = 0xe3;
    constexpr unsigned code_30 = 0xe2;
    constexpr unsigned fps_24 = 24;
    constexpr unsigned fps_25 = 25;
    constexpr unsigned fps_30 = 30;
    constexpr unsigned ntsc_numerator = 30000;
    constexpr unsigned ntsc_denominator = 1001;
    unsigned frames = 0;
    unit = microseconds_per_second;
    switch (division >> 8U) {
      case code_24: frames = fps_24; break;
      case code_25: frames = fps_25; break;
      case code_30: frames = fps_30; break;
      case code_drop:
        frames = ntsc_numerator;
        unit *= ntsc_denominator;
        break;
      default: throw std::runtime_error("SMF time-code frame rate is not defined");
    }
    time.divisor = frames * subdivision;
  }

  void advance(const std::uint64_t pulses) {
    if (pulses > (maximum - time.fraction) / unit) {
      throw std::runtime_error("SMF elapsed interval is not representable");
    }
    const auto numerator = (pulses * unit) + time.fraction;
    const auto whole = numerator / time.divisor;
    if (whole > maximum - time.microseconds) {
      throw std::runtime_error("SMF elapsed time is not representable");
    }
    time.microseconds += whole;
    time.fraction = static_cast<std::uint32_t>(numerator % time.divisor);
  }
};

struct Pending {
  Smf::Kind kind = Smf::Kind::channel;
  std::uint8_t meta = 0;
  std::array<std::uint8_t, 3> message{};
  std::uint32_t remaining = 0;
  bool first = true;
};

struct Track {
  enum class Phase : std::uint8_t { events, sysex, ended };
  Cursor cursor;
  std::uint16_t index;
  std::uint64_t pulse = 0;
  std::uint8_t running = 0;
  Phase phase = Phase::events;
  Pending head{};

  void channel(const std::optional<std::uint8_t> first_data) {
    if (phase == Phase::sysex) {
      throw std::runtime_error("SMF channel event interrupts a split SysEx");
    }
    const auto status = head.message[0];
    running = status;
    const auto kind = status & sysex_start;
    head.remaining = (kind == program_change || kind == channel_pressure) ? 2 : 3;
    head.message[1] = first_data ? *first_data : cursor.byte();
    if (head.remaining == 3) { head.message[2] = cursor.byte(); }
    for (const auto byte : std::span(head.message).subspan(1, head.remaining - 1)) {
      if (byte >= 0x80) { throw std::runtime_error("SMF channel data has a status bit"); }
    }
  }

  void prime() {
    const auto delta = cursor.vlq();
    if (delta > maximum - pulse) {
      throw std::runtime_error("SMF pulse position exceeds its range");
    }
    pulse += delta;
    auto status = cursor.byte();
    const bool data = status < 0x80;
    const auto first_data = status;
    if (data) {
      if (running == 0) {
        throw std::runtime_error("SMF running status has no channel predecessor");
      }
      status = running;
    }
    head = {};
    head.message[0] = status;
    if (status < sysex_start) {
      channel(data ? std::optional{first_data} : std::nullopt);
      return;
    }
    running = 0;
    if (status == meta_status) {
      head.kind = Smf::Kind::meta;
      head.meta = cursor.byte();
    } else if (status == sysex_start || status == sysex_end) {
      if (status == sysex_start && phase == Phase::sysex) {
        throw std::runtime_error("SMF SysEx starts before its previous message ends");
      }
      head.kind =
          status == sysex_start || phase == Phase::sysex ? Smf::Kind::sysex : Smf::Kind::escape;
    } else {
      throw std::runtime_error("SMF event is neither a channel, SysEx nor meta event");
    }
    head.remaining = cursor.vlq();
  }
};

struct Header {
  unsigned format, tracks, division;
  std::uint64_t end = 0;
};

Header read_header(const File &file) {
  Cursor cursor(file, {.first = 0, .end = file.size()});
  if (cursor.big(4) != header_signature) { throw std::runtime_error("SMF header is missing"); }
  const auto length = cursor.big(4);
  if (length < header_length) { throw std::runtime_error("SMF header is too short"); }
  Header value{.format = cursor.big(2), .tracks = cursor.big(2), .division = cursor.big(2)};
  if (value.format > 2 || value.tracks == 0 || (value.format == 0 && value.tracks != 1)) {
    throw std::runtime_error("SMF format or track count is invalid");
  }
  cursor.skip(length - header_length);
  value.end = cursor.position();
  return value;
}
}

struct Smf::State {
  enum class Phase : std::uint8_t { active, failed, ended };
  File file;
  Header header;
  Clock clock;
  std::vector<Track> tracks;
  std::vector<std::size_t> heap;
  std::array<std::uint8_t, packet_bytes> buffer{};
  std::uint64_t pulse = 0;
  Phase phase = Phase::active;

  [[nodiscard]] auto compare() const {
    return [this](const std::size_t left, const std::size_t right) {
      if (tracks[left].pulse == tracks[right].pulse) { return left > right; }
      return tracks[left].pulse > tracks[right].pulse;
    };
  }

  State(const std::filesystem::path &path, const std::optional<std::uint16_t> pattern)
      : file(path), header(read_header(file)), clock(header.division) {
    if (header.format == 2) {
      if (!pattern || *pattern >= header.tracks) {
        throw std::invalid_argument("SMF format 2 requires an explicit valid pattern index");
      }
    } else if (pattern) {
      throw std::invalid_argument("SMF pattern selection requires format 2");
    }
    tracks.reserve(header.format == 2 ? 1 : header.tracks);
    Cursor cursor(file, {.first = header.end, .end = file.size()});
    unsigned count = 0;
    while (!cursor.empty()) {
      const auto kind = cursor.big(4);
      const auto length = cursor.big(4);
      const auto first = cursor.position();
      cursor.skip(length);
      if (kind == header_signature) { throw std::runtime_error("SMF contains a second header"); }
      if (kind != track_signature) { continue; }
      if (count >= header.tracks) { throw std::runtime_error("SMF contains too many tracks"); }
      if (!pattern || count == *pattern) {
        tracks.push_back({.cursor = Cursor(file, {.first = first, .end = first + length}),
                          .index = static_cast<std::uint16_t>(count)});
      }
      ++count;
    }
    if (count != header.tracks) { throw std::runtime_error("SMF is missing declared tracks"); }
    heap.reserve(tracks.size());
    for (std::size_t index = 0; index < tracks.size(); ++index) {
      tracks[index].prime();
      heap.push_back(index);
    }
    std::ranges::make_heap(heap, compare());
  }

  void metadata(Track &track, const std::span<const std::uint8_t> bytes) {
    const auto &head = track.head;
    if (head.meta != set_tempo || !head.first) { return; }
    if (bytes.size() < 3 || (header.format == 1 && track.index != 0)) {
      throw std::runtime_error("SMF tempo has an invalid length or track");
    }
    const auto tempo =
        (std::uint32_t{bytes[0]} << 16U) | (std::uint32_t{bytes[1]} << 8U) | bytes[2];
    if (tempo == 0) { throw std::runtime_error("SMF tempo is zero"); }
    if (clock.metrical) { clock.unit = tempo; }
  }

  std::span<const std::uint8_t> payload(Track &track) {
    auto &head = track.head;
    if (head.kind == Kind::channel) {
      const auto bytes = std::span(buffer).first(head.remaining);
      std::ranges::copy(std::span(head.message).first(head.remaining), bytes.begin());
      head.remaining = 0;
      return bytes;
    }
    std::size_t prefix = 0;
    if (head.kind == Kind::sysex && head.first && head.message[0] == sysex_start) {
      buffer[0] = sysex_start;
      track.phase = Track::Phase::sysex;
      prefix = 1;
    }
    const auto count = std::min<std::size_t>(head.remaining, buffer.size() - prefix);
    const auto bytes = std::span(buffer).subspan(prefix, count);
    track.cursor.read(bytes);
    if (head.kind == Kind::sysex) {
      for (std::size_t index = 0; index < bytes.size(); ++index) {
        constexpr std::uint8_t realtime = 0xf8;
        if (bytes[index] < 0x80 || bytes[index] >= realtime) { continue; }
        if (bytes[index] != sysex_end || index + 1 != head.remaining) {
          throw std::runtime_error("SMF SysEx has a status byte before its terminator");
        }
        track.phase = Track::Phase::events;
      }
    }
    if (head.kind == Kind::meta) { metadata(track, bytes); }
    head.remaining -= static_cast<std::uint32_t>(count);
    return std::span(buffer).first(prefix + count);
  }

  std::optional<Event> next() {
    if (heap.empty()) {
      file.unchanged();
      return {};
    }
    std::ranges::pop_heap(heap, compare());
    const auto index = heap.back();
    heap.pop_back();
    auto &track = tracks[index];
    assert(track.pulse >= pulse);
    clock.advance(track.pulse - pulse);
    pulse = track.pulse;
    auto &head = track.head;
    const auto bytes = payload(track);
    const Event event{.time = clock.time,
                      .track = track.index,
                      .kind = head.kind,
                      .meta = head.meta,
                      .first = head.first,
                      .last = head.remaining == 0,
                      .bytes = bytes};
    head.first = false;
    if (event.last) {
      if (head.kind == Kind::meta && head.meta == end_track) {
        if (track.phase == Track::Phase::sysex || !track.cursor.empty()) {
          throw std::runtime_error("SMF track ends inside SysEx or before its chunk boundary");
        }
        track.phase = Track::Phase::ended;
      } else {
        track.prime();
      }
    }
    if (track.phase != Track::Phase::ended) {
      heap.push_back(index);
      std::ranges::push_heap(heap, compare());
    }
    return event;
  }
};

Smf::Smf(const std::filesystem::path &path, const std::optional<std::uint16_t> pattern)
    : state_(std::make_unique<State>(path, pattern)) {}

Smf::~Smf() = default;

std::optional<Smf::Event> Smf::next() {
  if (state_->phase == State::Phase::failed) { throw std::logic_error("SMF stream has failed"); }
  if (state_->phase == State::Phase::ended) { return {}; }
  state_->phase = State::Phase::failed;
  const auto event = state_->next();
  state_->phase = event ? State::Phase::active : State::Phase::ended;
  return event;
}

Time Smf::time() const noexcept {
  return state_->clock.time;
}
}
