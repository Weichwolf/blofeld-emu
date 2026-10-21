// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::samples {
// Resource limits for offline editing, not Blofeld hardware capacities.
inline constexpr std::size_t maximum_xml_bytes = std::size_t{16} * 1024 * 1024;
inline constexpr std::size_t maximum_programs = 1024;
inline constexpr std::size_t maximum_samples = 65536;
inline constexpr std::size_t maximum_text_bytes = 4096;

inline constexpr unsigned last_midi_key = 127;

struct Sample {
  std::string filename;
  std::filesystem::path source;
  double pitch = 0;
  unsigned highest = last_midi_key;
  bool normalize = false;
  bool fixed_pitch = false;
  double level_db = 0;
  bool operator==(const Sample &) const = default;
};

struct Program {
  std::string name;
  bool normalize = false;
  std::vector<Sample> samples;
  bool operator==(const Program &) const = default;
};

struct Library {
  std::vector<Program> programs;
  bool operator==(const Library &) const = default;
};

void validate_text(std::string_view text);
void validate(const Library &library);
void order_samples(Program &program);
void move_program(Library &library, std::size_t from, std::size_t to);
void transpose_roots(Program &program, double semitones);
[[nodiscard]] Library parse_xml(std::string_view text, const std::filesystem::path &origin);
[[nodiscard]] std::string encode_xml(const Library &library, const std::filesystem::path &output);
[[nodiscard]] Library load_xml(const std::filesystem::path &path);
void save_xml(const Library &library, const std::filesystem::path &path);
}
