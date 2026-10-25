// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace blofeld::samples::xml {
struct Tag {
  std::string name;
  std::vector<std::pair<std::string, std::string>> attributes;
  bool closing = false;
  bool empty = false;
};

// Nonrecursive reader for the element/attribute-only Spectre 0.5 vocabulary.
// No DTDs, external entities, processing instructions or implicit file access.
class Reader {
public:
  explicit Reader(std::string_view text);
  [[nodiscard]] bool done();
  [[nodiscard]] Tag next();

private:
  void trivia();
  void xml_declaration();
  void spaces();
  [[nodiscard]] std::string name();
  [[nodiscard]] std::string value();
  void expect(char character);
  void fields(Tag &tag);
  [[nodiscard]] bool starts(std::string_view text) const;
  std::string_view text_;
  std::size_t offset_ = 0;
  bool declaration_ = false;
  bool element_ = false;
};

[[nodiscard]] std::string escape(std::string_view text);
[[nodiscard]] std::string attribute(const Tag &tag, std::string_view name);
void attributes(const Tag &tag, const std::vector<std::string_view> &allowed);
[[nodiscard]] double number(std::string_view text);
[[nodiscard]] unsigned key(std::string_view text);
[[nodiscard]] bool boolean(std::string_view text);
[[nodiscard]] std::string decimal(double value);
}
