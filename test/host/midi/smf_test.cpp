// SPDX-License-Identifier: GPL-3.0-only
#include "midi/smf.hpp"
#include "midi/time.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>
#include <vector>

namespace {
using blofeld::host::midi::Smf;
using blofeld::host::midi::Time;
using Bytes = std::vector<std::uint8_t>;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned default_division = 96, merged_division = 480;
constexpr std::uint8_t sysex_start = 0xf0, realtime = 0xf8;
constexpr std::uint8_t set_tempo = 0x51;
constexpr std::array<std::uint8_t, 4> ending{0, 0xff, 0x2f, 0};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

template <typename Error> void rejects(auto &&action, const char *message) {
  bool rejected = false;
  try {
    action();
  } catch (const Error &) { rejected = true; }
  require(rejected, message);
}

void big_word(Bytes &bytes, const unsigned value) {
  bytes.push_back(static_cast<std::uint8_t>(value >> 8U));
  bytes.push_back(static_cast<std::uint8_t>(value & byte_mask));
}

void big_size(Bytes &bytes, const std::size_t value) {
  for (const auto shift : {24U, 16U, 8U, 0U}) {
    bytes.push_back(static_cast<std::uint8_t>((value >> shift) & byte_mask));
  }
}

void append(Bytes &bytes, const std::span<const std::uint8_t> data) {
  bytes.insert(bytes.end(), data.begin(), data.end());
}

struct Settings {
  unsigned format = 0, division = default_division;
  Bytes extension;
};

Bytes file_bytes(const std::vector<Bytes> &tracks, const Settings &settings = {}) {
  Bytes result{'M', 'T', 'h', 'd'};
  big_size(result, 6 + settings.extension.size());
  big_word(result, settings.format);
  big_word(result, static_cast<unsigned>(tracks.size()));
  big_word(result, settings.division);
  append(result, settings.extension);
  for (const auto &track : tracks) {
    append(result, std::array<std::uint8_t, 4>{'M', 'T', 'r', 'k'});
    big_size(result, track.size());
    append(result, track);
  }
  return result;
}

std::filesystem::path save(const std::filesystem::path &path, const Bytes &bytes) {
  std::ofstream output(path, std::ios::binary);
  output.write(reinterpret_cast<const char *>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  output.close();
  require(output.good(), "Write a complete authored SMF fixture");
  return path;
}

struct Expected {
  Time time;
  std::uint16_t track = 0;
  Smf::Kind kind = Smf::Kind::channel;
  std::uint8_t meta = 0;
  Bytes bytes;
};

void expect(Smf &stream, const Expected &wanted) {
  const auto event = stream.next();
  require(event && event->time == wanted.time && event->track == wanted.track &&
              event->kind == wanted.kind && event->meta == wanted.meta && event->first &&
              event->last && std::ranges::equal(event->bytes, wanted.bytes),
          "Merged event retains exact time, source, kind and bytes");
}

void default_time_and_status(const std::filesystem::path &root) {
  const Bytes track{0, 0xc0, 7, 0, 0x90, 60, 100, 1, 61, 0, 2, 0x80, 60, 64, 0, 0xff, 0x2f, 0};
  Smf stream(save(root / "default.mid", file_bytes({track}, {.division = 3, .extension = {}})));
  constexpr Time zero{.divisor = 3};
  constexpr Time third{.microseconds = 166666, .fraction = 2, .divisor = 3};
  constexpr Time quarter{.microseconds = 500000, .divisor = 3};
  require(stream.time() == zero, "A missing tempo means 500000 microseconds per quarter note");
  const std::array expected{
      Expected{.time = zero, .bytes = {0xc0, 7}},
      Expected{.time = zero, .bytes = {0x90, 60, 100}},
      Expected{.time = third, .bytes = {0x90, 61, 0}},
      Expected{.time = quarter, .bytes = {0x80, 60, 64}},
      Expected{.time = quarter, .kind = Smf::Kind::meta, .meta = 0x2f, .bytes = {}}};
  for (const auto &wanted : expected) { expect(stream, wanted); }
  require(!stream.next() && !stream.next() && stream.time() == quarter,
          "EOF is stable and time includes the actual end-of-track event");
}

void merged_tempo(const std::filesystem::path &root) {
  const Bytes tempo{0,    0xff, 0x51, 3,    7,    0xa1, 0x20, 0x81, 0x70, 0xff,
                    0x51, 3,    0x0f, 0x42, 0x40, 0x81, 0x70, 0xff, 0x2f, 0};
  const Bytes first{0,  0xc1, 17,   0,    0x91, 64,   80, 120, 65,   81,   120,
                    64, 0,    0x81, 0x70, 0xb1, 0x7b, 0,  120, 0xff, 0x2f, 0};
  const Bytes second{0x81, 0x70, 0x92, 67, 99, 0, 0x82, 67, 64, 0, 0xff, 0x2f, 0};
  Smf stream(save(root / "merged.mid",
                  file_bytes({tempo, first, second},
                             {.format = 1, .division = merged_division, .extension = {}})));
  constexpr Time zero{.divisor = 480};
  constexpr Time eighth{.microseconds = 125000, .divisor = 480};
  constexpr Time change{.microseconds = 250000, .divisor = 480};
  constexpr Time after{.microseconds = 750000, .divisor = 480};
  constexpr Time end{.microseconds = 1000000, .divisor = 480};
  const std::array expected{
      Expected{.time = zero, .kind = Smf::Kind::meta, .meta = 0x51, .bytes = {7, 0xa1, 0x20}},
      Expected{.time = zero, .track = 1, .bytes = {0xc1, 17}},
      Expected{.time = zero, .track = 1, .bytes = {0x91, 64, 80}},
      Expected{.time = eighth, .track = 1, .bytes = {0x91, 65, 81}},
      Expected{.time = change, .kind = Smf::Kind::meta, .meta = 0x51, .bytes = {0x0f, 0x42, 0x40}},
      Expected{.time = change, .track = 1, .bytes = {0x91, 64, 0}},
      Expected{.time = change, .track = 2, .bytes = {0x92, 67, 99}},
      Expected{.time = change, .track = 2, .bytes = {0x82, 67, 64}},
      Expected{.time = change, .track = 2, .kind = Smf::Kind::meta, .meta = 0x2f, .bytes = {}},
      Expected{.time = after, .kind = Smf::Kind::meta, .meta = 0x2f, .bytes = {}},
      Expected{.time = after, .track = 1, .bytes = {0xb1, 0x7b, 0}},
      Expected{.time = end, .track = 1, .kind = Smf::Kind::meta, .meta = 0x2f, .bytes = {}}};
  for (const auto &wanted : expected) { expect(stream, wanted); }
  require(!stream.next() && stream.time() == end, "Trailing track duration survives the merge");
}

void rational_tempo(const std::filesystem::path &root) {
  const Bytes track{0,    0xff, 0x51, 3,    0,    0,    1,    1,    0x90, 60, 1,    1,
                    61,   1,    0,    0xff, 0x51, 3,    0,    0,    2,    1,  0x90, 62,
                    1,    1,    63,   1,    0,    0xff, 0x51, 3,    0,    0,  3,    1,
                    0x90, 64,   1,    2,    65,   1,    0,    0xff, 0x2f, 0};
  Smf stream(save(root / "fractions.mid", file_bytes({track}, {.division = 7, .extension = {}})));
  constexpr std::array expected{Time{.fraction = 1, .divisor = 7},
                                Time{.fraction = 2, .divisor = 7},
                                Time{.fraction = 4, .divisor = 7},
                                Time{.fraction = 6, .divisor = 7},
                                Time{.microseconds = 1, .fraction = 2, .divisor = 7},
                                Time{.microseconds = 2, .fraction = 1, .divisor = 7}};
  std::size_t seen = 0;
  while (const auto event = stream.next()) {
    if (event->kind != Smf::Kind::channel) { continue; }
    require(seen < expected.size() && event->time == expected[seen],
            "Remainders survive tempo changes without per-event rounding");
    ++seen;
  }
  require(seen == expected.size() && stream.time() == expected.back(),
          "Fifteen accumulated microsecond numerators equal two and one seventh microseconds");
}

void time_code(const std::filesystem::path &root) {
  struct Case {
    unsigned division;
    std::array<std::uint8_t, 2> delta;
    Time note, end;
  };

  constexpr std::array cases{
      Case{.division = 0xe850,
           .delta = {0x8f, 0},
           .note = {.microseconds = 1000000, .divisor = 1920},
           .end = {.microseconds = 1000520, .fraction = 1600, .divisor = 1920}},
      Case{.division = 0xe728,
           .delta = {0x87, 0x68},
           .note = {.microseconds = 1000000, .divisor = 1000},
           .end = {.microseconds = 1001000, .divisor = 1000}},
      Case{.division = 0xe350,
           .delta = {0x92, 0x60},
           .note = {.microseconds = 1001000, .divisor = 2400000},
           .end = {.microseconds = 1001417, .fraction = 200000, .divisor = 2400000}},
      Case{.division = 0xe250,
           .delta = {0x92, 0x60},
           .note = {.microseconds = 1000000, .divisor = 2400},
           .end = {.microseconds = 1000416, .fraction = 1600, .divisor = 2400}}};
  for (const auto &test : cases) {
    constexpr std::array<std::uint8_t, 7> tempo_bytes{0, 0xff, 0x51, 3, 3, 0xd0, 0x90};
    constexpr std::array<std::uint8_t, 7> note_bytes{0x90, 60, 1, 1, 0xff, 0x2f, 0};
    Bytes track(tempo_bytes.begin(), tempo_bytes.end());
    append(track, test.delta);
    append(track, note_bytes);
    Smf stream(save(root / (std::to_string(test.division) + ".mid"),
                    file_bytes({track}, {.division = test.division, .extension = {}})));
    const auto tempo = stream.next();
    require(tempo && tempo->kind == Smf::Kind::meta,
            "A tempo annotation is metadata in time-code files");
    const std::array expected{
        Expected{.time = test.note, .bytes = {0x90, 60, 1}},
        Expected{.time = test.end, .kind = Smf::Kind::meta, .meta = 0x2f, .bytes = {}}};
    for (const auto &wanted : expected) { expect(stream, wanted); }
    require(!stream.next(), "Time-code duration does not follow metrical tempo changes");
  }
}

void streamed_sysex(const std::filesystem::path &root) {
  Bytes body;
  constexpr std::size_t count = 1025;
  for (std::size_t index = 0; index < count; ++index) {
    constexpr unsigned data_mask = 0x7f;
    body.push_back(static_cast<std::uint8_t>(index & data_mask));
  }
  body[512] = realtime;
  constexpr std::array<std::uint8_t, 4> prefix{0, 0xf0, 0x88, 1};
  Bytes track(prefix.begin(), prefix.end());
  append(track, body);
  const Bytes packets{24, 0xff, 1, 3,    'x', 'y',  'z', 24, 0xf7, 5,    1,    2, 3,
                      4,  0xf7, 0, 0xf7, 7,   0xf2, 1,   2,  0xf8, 0xff, 0xc0, 5};
  append(track, packets);
  append(track, ending);
  Smf stream(save(root / "packets.mid", file_bytes({track})));
  Bytes transmitted{sysex_start};
  append(transmitted, body);
  std::size_t offset = 0;
  while (offset < transmitted.size()) {
    const auto event = stream.next();
    if (!event) { throw std::runtime_error("SysEx stream ended before its authored payload"); }
    require(event->kind == Smf::Kind::sysex && event->time == Time{.divisor = default_division} &&
                !event->bytes.empty() && event->bytes.size() <= Smf::packet_bytes &&
                event->first == (offset == 0) &&
                event->last == (offset + event->bytes.size() == transmitted.size()) &&
                std::ranges::equal(event->bytes,
                                   std::span(transmitted).subspan(offset, event->bytes.size())),
            "SysEx streams exact bounded chunks, including interspersed realtime bytes");
    offset += event->bytes.size();
  }
  constexpr Time text{.microseconds = 125000, .divisor = 96};
  constexpr Time continuation{.microseconds = 250000, .divisor = 96};
  const std::array expected{
      Expected{.time = text, .kind = Smf::Kind::meta, .meta = 1, .bytes = {'x', 'y', 'z'}},
      Expected{.time = continuation, .kind = Smf::Kind::sysex, .bytes = {1, 2, 3, 4, 0xf7}},
      Expected{.time = continuation,
               .kind = Smf::Kind::escape,
               .bytes = {0xf2, 1, 2, 0xf8, 0xff, 0xc0, 5}},
      Expected{.time = continuation, .kind = Smf::Kind::meta, .meta = 0x2f, .bytes = {}}};
  for (const auto &wanted : expected) { expect(stream, wanted); }
  require(!stream.next(), "SMF continuation marker F7 is not itself transmitted");
}

void patterns(const std::filesystem::path &root) {
  const Bytes first{0, 0x90, 60, 1, 0x60, 0x80, 60, 0, 0, 0xff, 0x2f, 0};
  const Bytes second{
      0, 0xff, 0x51, 3, 0x0f, 0x42, 0x40, 0, 0x91, 61, 1, 0x60, 0x81, 61, 0, 0, 0xff, 0x2f, 0};
  const auto path =
      save(root / "patterns.mid", file_bytes({first, second}, {.format = 2, .extension = {}}));
  rejects<std::invalid_argument>([&] { const Smf stream(path); },
                                 "Independent patterns need an explicit selection");
  rejects<std::invalid_argument>([&] { const Smf stream(path, 2); },
                                 "Reject an out-of-range pattern");
  Smf stream(path, 1);
  constexpr Time zero{.divisor = 96};
  constexpr Time end{.microseconds = 1000000, .divisor = 96};
  const std::array expected{
      Expected{.time = zero,
               .track = 1,
               .kind = Smf::Kind::meta,
               .meta = 0x51,
               .bytes = {0x0f, 0x42, 0x40}},
      Expected{.time = zero, .track = 1, .bytes = {0x91, 61, 1}},
      Expected{.time = end, .track = 1, .bytes = {0x81, 61, 0}},
      Expected{.time = end, .track = 1, .kind = Smf::Kind::meta, .meta = 0x2f, .bytes = {}}};
  for (const auto &wanted : expected) { expect(stream, wanted); }
  require(!stream.next(), "Other independent patterns neither play nor contribute tempo");
  const auto ordinary = save(root / "ordinary.mid", file_bytes({first}));
  rejects<std::invalid_argument>([&] { const Smf selected(ordinary, 0); },
                                 "Do not interpret normal tracks as patterns");
}

void consume(const std::filesystem::path &path) {
  Smf stream(path);
  while (stream.next()) {}
}

void extensions(const std::filesystem::path &root) {
  constexpr std::array<std::uint8_t, 5> prefix{0, 0xff, 0x51, 0x82, 0x2c};
  Bytes track(prefix.begin(), prefix.end());
  constexpr std::size_t payload_bytes = 300;
  const Bytes tempo = [] {
    constexpr std::array<std::uint8_t, 3> value{7, 0xa1, 0x20};
    Bytes result(payload_bytes, 1);
    std::ranges::copy(value, result.begin());
    return result;
  }();
  append(track, tempo);
  const Bytes suffix{
      0, 0xff, 0x7f, 4, 0xff, 0xf0, 0x80, 0, 0, 0xc0, 5, 0x60, 0xff, 0x2f, 2, 42, 43};
  append(track, suffix);
  const Settings settings{.extension = {0xaa, 0xbb}};
  auto file = file_bytes({track}, settings);
  constexpr std::array<std::uint8_t, 11> alien{'J', 'U', 'N', 'K', 0, 0, 0, 3, 9, 8, 7};
  file.insert(file.begin() + 16, alien.begin(), alien.end());
  append(file, alien);
  Smf stream(save(root / "extensions.mid", file));
  std::size_t offset = 0;
  while (offset < tempo.size()) {
    const auto event = stream.next();
    if (!event) { throw std::runtime_error("Extended tempo was lost"); }
    require(
        event->kind == Smf::Kind::meta && event->meta == set_tempo &&
            event->first == (offset == 0) &&
            event->last == (offset + event->bytes.size() == tempo.size()) &&
            std::ranges::equal(event->bytes, std::span(tempo).subspan(offset, event->bytes.size())),
        "Recognized meta extensions are preserved and only their known prefix is interpreted");
    offset += event->bytes.size();
  }
  constexpr Time zero{.divisor = 96};
  constexpr Time end{.microseconds = 500000, .divisor = 96};
  const std::array expected{
      Expected{.time = zero, .kind = Smf::Kind::meta, .meta = 0x7f, .bytes = {0xff, 0xf0, 0x80, 0}},
      Expected{.time = zero, .bytes = {0xc0, 5}},
      Expected{.time = end, .kind = Smf::Kind::meta, .meta = 0x2f, .bytes = {42, 43}}};
  for (const auto &wanted : expected) { expect(stream, wanted); }
  require(!stream.next(), "Header extensions and alien chunks do not become musical events");
}

void channel_families(const std::filesystem::path &root) {
  const Bytes track{0,   0x8f, 60,   64,   0, 0x9f, 61,  0, 0,    0xaf, 62, 127,  0,    0xbf, 1,
                    127, 0,    0xcf, 45,   0, 0xdf, 126, 0, 0xef, 0,    64, 0,    0xff, 0x20, 1,
                    15,  0,    0xff, 0x54, 5, 1,    2,   3, 4,    5,    0,  0xff, 0x2f, 0};
  Smf stream(save(root / "channels.mid", file_bytes({track})));
  constexpr Time zero{.divisor = 96};
  const std::array expected{
      Expected{.time = zero, .bytes = {0x8f, 60, 64}},
      Expected{.time = zero, .bytes = {0x9f, 61, 0}},
      Expected{.time = zero, .bytes = {0xaf, 62, 127}},
      Expected{.time = zero, .bytes = {0xbf, 1, 127}},
      Expected{.time = zero, .bytes = {0xcf, 45}},
      Expected{.time = zero, .bytes = {0xdf, 126}},
      Expected{.time = zero, .bytes = {0xef, 0, 64}},
      Expected{.time = zero, .kind = Smf::Kind::meta, .meta = 0x20, .bytes = {15}},
      Expected{.time = zero, .kind = Smf::Kind::meta, .meta = 0x54, .bytes = {1, 2, 3, 4, 5}},
      Expected{.time = zero, .kind = Smf::Kind::meta, .meta = 0x2f, .bytes = {}}};
  for (const auto &wanted : expected) { expect(stream, wanted); }
  require(!stream.next(), "All channel families and non-transmittable channel/time labels survive");
  const Bytes empty{0, 0xf7, 0, 0, 0xf0, 0, 0, 0xf7, 1, 0xf7, 0, 0xff, 0x2f, 0};
  Smf packets(save(root / "empty-packets.mid", file_bytes({empty})));
  const std::array boundary{
      Expected{.time = zero, .kind = Smf::Kind::escape, .bytes = {}},
      Expected{.time = zero, .kind = Smf::Kind::sysex, .bytes = {0xf0}},
      Expected{.time = zero, .kind = Smf::Kind::sysex, .bytes = {0xf7}},
      Expected{.time = zero, .kind = Smf::Kind::meta, .meta = 0x2f, .bytes = {}}};
  for (const auto &wanted : boundary) { expect(packets, wanted); }
  require(!packets.next(), "Zero-length packets still advance without inventing bytes");
}

void framing(const std::filesystem::path &root) {
  const Bytes track(ending.begin(), ending.end());
  const auto ordinary = file_bytes({track});
  constexpr unsigned zero_subdivision = 0xe700;
  constexpr unsigned undefined_frame_rate = 0xe650;
  std::vector<Bytes> invalid{
      {},
      file_bytes({}),
      file_bytes({track, track}),
      file_bytes({track}, {.format = 3, .extension = {}}),
      file_bytes({track}, {.division = 0, .extension = {}}),
      file_bytes({track}, {.division = zero_subdivision, .extension = {}}),
      file_bytes({track}, {.division = undefined_frame_rate, .extension = {}}),
      file_bytes({Bytes{}})};
  auto short_header = ordinary;
  short_header[7] = 5;
  invalid.push_back(short_header);
  auto huge_header = ordinary;
  huge_header[4] = byte_mask;
  invalid.push_back(huge_header);
  auto missing = file_bytes({track}, {.format = 1, .extension = {}});
  missing[11] = 2;
  invalid.push_back(missing);
  auto extra = file_bytes({track, track}, {.format = 1, .extension = {}});
  extra[11] = 1;
  invalid.push_back(extra);
  auto truncated = ordinary;
  truncated.pop_back();
  invalid.push_back(truncated);
  auto tail = ordinary;
  tail.push_back(0);
  invalid.push_back(tail);
  auto duplicate = ordinary;
  append(duplicate, std::span(ordinary).first(14));
  invalid.push_back(duplicate);
  for (std::size_t index = 0; index < invalid.size(); ++index) {
    const auto path = save(root / ("framing-" + std::to_string(index) + ".mid"), invalid[index]);
    rejects<std::runtime_error>([&] { consume(path); },
                                "Invalid SMF framing cannot supply partial headers or tracks");
  }
  const auto valid = save(root / "regular.mid", ordinary);
  const auto link = root / "link.mid";
  std::filesystem::create_symlink(valid.filename(), link);
  rejects<std::system_error>([&] { const Smf stream(link); }, "SMF input does not follow symlinks");
  const auto fifo = root / "fifo";
  constexpr unsigned private_mode = 0600;
  require(::mkfifo(fifo.c_str(), private_mode) == 0, "Create owned FIFO");
  rejects<std::runtime_error>([&] { const Smf stream(fifo); },
                              "A non-regular source cannot block waiting for data");
  rejects<std::runtime_error>([&] { const Smf stream(root); }, "Directories are not MIDI sources");
}

void malformed_events(const std::filesystem::path &root) {
  const std::vector<Bytes> invalid{{0x81},
                                   {0x81, 0x80, 0x80, 0x80, 0},
                                   {0, 60, 100},
                                   {0, 0x90, 0x80, 1},
                                   {0, 0x90, 60},
                                   {0, 0xf1, 1},
                                   {0, 0x90, 60, 1},
                                   {0, 0xff, 0x2f, 0, 0},
                                   {0, 0xff, 1, 0x7f, 1},
                                   {0, 0xff, 0x51, 2, 1, 2},
                                   {0, 0xff, 0x51, 3, 0, 0, 0},
                                   {0, 0xf0, 1, 1, 0, 0x90, 60, 1},
                                   {0, 0xf0, 3, 1, 0xf7, 2},
                                   {0, 0xf0, 3, 1, 0x90, 0xf7},
                                   {0, 0xf0, 1, 1, 0, 0xff, 0x2f, 0},
                                   {0, 0xf0, 1, 1, 0, 0xf0, 1, 0xf7},
                                   {0, 0x90, 60, 1, 0, 0xff, 1, 0, 0, 61, 1},
                                   {0, 0x90, 60, 1, 0, 0xf7, 0, 0, 61, 1}};
  for (std::size_t index = 0; index < invalid.size(); ++index) {
    const auto path =
        save(root / ("invalid-" + std::to_string(index) + ".mid"), file_bytes({invalid[index]}));
    rejects<std::runtime_error>([&] { consume(path); },
                                "Reject malformed SMF events instead of guessing bytes or timing");
  }
  const Bytes misplaced{0, 0xff, 0x51, 3, 1, 2, 3, 0, 0xff, 0x2f, 0};
  const auto tempo = save(
      root / "wrong-tempo-track.mid",
      file_bytes({Bytes(ending.begin(), ending.end()), misplaced}, {.format = 1, .extension = {}}));
  rejects<std::runtime_error>([&] { consume(tempo); }, "Format 1 tempo belongs in track zero");
}

void input_changes(const std::filesystem::path &root) {
  const Bytes original{0, 0x90, 60, 1, 0, 0xff, 0x2f, 0};
  const auto path = save(root / "changing.mid", file_bytes({original}));
  Smf stream(path);
  require(stream.next().has_value(), "Begin a valid source before mutation");
  {
    std::ofstream changed(path, std::ios::app | std::ios::binary);
    changed.put('\0');
  }
  rejects<std::runtime_error>(
      [&] {
        while (stream.next()) {}
      },
      "Changed cached input cannot report a clean EOF");
  rejects<std::logic_error>([&] { static_cast<void>(stream.next()); },
                            "Failed input cannot resume from a partial state");
  constexpr std::array<std::uint8_t, 5> prefix{0, 0xff, 1, 0x8f, 0x50};
  Bytes body(prefix.begin(), prefix.end());
  constexpr std::size_t text_bytes = 2000;
  body.resize(prefix.size() + text_bytes, 1);
  append(body, ending);
  const auto truncated = save(root / "truncated.mid", file_bytes({body}));
  Smf shortened(truncated);
  std::filesystem::resize_file(truncated, 32);
  rejects<std::runtime_error>(
      [&] {
        while (shortened.next()) {}
      },
      "Truncation beyond a cached block fails without memory mapping faults");
}

void overflow(const std::filesystem::path &root) {
  constexpr std::array<std::uint8_t, 7> tempo{0, 0xff, 0x51, 3, 0xff, 0xff, 0xff};
  constexpr std::array<std::uint8_t, 6> note{0xff, 0xff, 0xff, 0x7f, 0xc0, 1};
  Bytes track(tempo.begin(), tempo.end());
  constexpr unsigned events = 4097;
  for (unsigned index = 0; index < events; ++index) { append(track, note); }
  append(track, ending);
  const auto path =
      save(root / "overflow.mid", file_bytes({track}, {.division = 1, .extension = {}}));
  rejects<std::runtime_error>([&] { consume(path); }, "Elapsed microseconds cannot silently wrap");
}
}

int main(const int argc, char **argv) try {
  require(argc == 2, "Supply a build artifact directory");
  const auto root = std::filesystem::path(argv[1]) / std::to_string(::getpid());
  require(std::filesystem::create_directories(root), "Create owned unit directory");
  default_time_and_status(root);
  merged_tempo(root);
  rational_tempo(root);
  time_code(root);
  streamed_sysex(root);
  patterns(root);
  extensions(root);
  channel_families(root);
  framing(root);
  malformed_events(root);
  input_changes(root);
  overflow(root);
  std::filesystem::remove_all(root);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
