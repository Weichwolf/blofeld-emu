// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "samples/sample_library.hpp"
#include "storage/private_file.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace blofeld::samples {
Library load_xml(const std::filesystem::path &path) {
  const auto bytes = read_private_file(path, maximum_xml_bytes);
  std::string text;
  text.reserve(bytes.size());
  for (const auto byte : bytes) { text.push_back(static_cast<char>(byte)); }
  return parse_xml(text, path);
}

void save_xml(const Library &library, const std::filesystem::path &path) {
  const auto out = std::filesystem::weakly_canonical(path);
  for (const auto &program : library.programs) {
    for (const auto &sample : program.samples) {
      const auto source = std::filesystem::weakly_canonical(sample.source);
      if (source == out || (std::filesystem::exists(out) && std::filesystem::exists(source) &&
                            std::filesystem::equivalent(out, source))) {
        throw Fault("Sample-set output collides with a referenced sample file");
      }
    }
  }
  const auto text = encode_xml(library, path);
  std::vector<std::uint8_t> data;
  data.reserve(text.size());
  for (const auto c : text) {
    data.push_back(static_cast<std::uint8_t>(static_cast<unsigned char>(c)));
  }
  write_private_file(path, data);
}
}
