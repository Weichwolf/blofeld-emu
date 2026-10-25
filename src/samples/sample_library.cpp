// SPDX-License-Identifier: GPL-3.0-only
#include "samples/sample_library.hpp"

#include "common/bus.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <string_view>

namespace blofeld::samples {
namespace {
bool xml_character(const std::uint32_t value) {
  constexpr unsigned last_bmp = 0xfffd;
  constexpr unsigned last_scalar = 0x10ffff;
  constexpr unsigned surrogate_begin = 0xd800;
  constexpr unsigned surrogate_end = 0xe000;
  return value == 9 || value == 10 || value == 13 || (value >= 0x20 && value < surrogate_begin) ||
         (value >= surrogate_end && value <= last_bmp) ||
         (value >= 0x10000 && value <= last_scalar);
}
}

void validate_text(const std::string_view text) {
  if (text.size() > maximum_text_bytes) {
    throw Fault("Sample-library text exceeds its resource limit");
  }
  std::size_t i = 0;
  while (i < text.size()) {
    const auto first = static_cast<unsigned char>(text[i++]);
    unsigned trailing = 0;
    std::uint32_t value = first;
    unsigned minimum = 0;
    constexpr unsigned payload_two = 0x1f;
    constexpr unsigned payload_six = 0x3f;
    constexpr unsigned ascii_end = 0x80;
    constexpr unsigned two_byte = 0xc0;
    constexpr unsigned three_byte = 0xe0;
    constexpr unsigned four_byte = 0xf0;
    constexpr unsigned lead_end = 0xf8;
    if (first >= two_byte && first < three_byte) {
      trailing = 1;
      value = first & payload_two;
      minimum = ascii_end;
    } else if (first >= three_byte && first < four_byte) {
      trailing = 2;
      value = first & 0x0fU;
      minimum = 0x800;
    } else if (first >= four_byte && first < lead_end) {
      trailing = 3;
      value = first & 7U;
      minimum = 0x10000;
    } else if (first >= ascii_end) {
      throw Fault("Invalid UTF-8 sample-library text");
    }
    for (unsigned j = 0; j < trailing; ++j) {
      if (i >= text.size()) { throw Fault("Truncated UTF-8 sample-library text"); }
      const auto next = static_cast<unsigned char>(text[i++]);
      if ((next & two_byte) != ascii_end) { throw Fault("Invalid UTF-8 continuation"); }
      value = (value << 6U) | (next & payload_six);
    }
    if (value < minimum || !xml_character(value)) {
      throw Fault("Invalid XML character in sample library");
    }
  }
}

void validate(const Library &library) {
  if (library.programs.size() > maximum_programs) { throw Fault("Too many sample programs"); }
  std::size_t total = 0;
  constexpr unsigned last_key = 127;
  for (const auto &program : library.programs) {
    validate_text(program.name);
    if (program.name.empty()) { throw Fault("A sample program needs a name"); }
    if (program.samples.size() > maximum_samples - total) {
      throw Fault("Too many sample references");
    }
    total += program.samples.size();
    unsigned previous = 0;
    for (const auto &sample : program.samples) {
      validate_text(sample.filename);
      validate_text(sample.source.generic_string());
      if (sample.filename.empty() || sample.source.empty() || !sample.source.is_absolute()) {
        throw Fault("A sample needs a name and an absolute resolved source path");
      }
      if (!std::isfinite(sample.pitch) || !std::isfinite(sample.level_db) ||
          sample.highest > last_key || sample.highest < previous) {
        throw Fault("Invalid sample pitch, gain or highest-key ordering");
      }
      previous = sample.highest;
    }
  }
}

void order_samples(Program &program) {
  std::ranges::stable_sort(program.samples, {}, &Sample::highest);
}

void move_program(Library &library, const std::size_t from, const std::size_t to) {
  if (from >= library.programs.size() || to >= library.programs.size()) {
    throw Fault("Sample program index is outside the collection");
  }
  auto from_iterator = std::next(library.programs.begin(), static_cast<std::ptrdiff_t>(from));
  auto to_iterator = std::next(library.programs.begin(), static_cast<std::ptrdiff_t>(to));
  if (from < to) { std::rotate(from_iterator, std::next(from_iterator), std::next(to_iterator)); }
  if (from > to) { std::rotate(to_iterator, from_iterator, std::next(from_iterator)); }
}

void transpose_roots(Program &program, const double semitones) {
  if (!std::isfinite(semitones)) { throw Fault("Root-key transposition must be finite"); }
  for (const auto &sample : program.samples) {
    if (!std::isfinite(sample.pitch + semitones)) {
      throw Fault("Root-key transposition overflow");
    }
  }
  for (auto &sample : program.samples) { sample.pitch += semitones; }
}
}
