// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "samples/sample_library.hpp"
#include "samples/sample_xml.hpp"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace blofeld::samples::xml {
namespace {
bool whitespace(const char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

void scalar(std::string &out, const std::uint32_t value) {
  constexpr unsigned continuation = 0x80;
  constexpr unsigned last_ascii = 0x7f;
  constexpr unsigned last_two = 0x7ff;
  constexpr unsigned last_three = 0xffff;
  constexpr unsigned last_scalar = 0x10ffff;
  constexpr unsigned payload = 0x3f;
  constexpr unsigned lead_two = 0xc0;
  constexpr unsigned lead_three = 0xe0;
  constexpr unsigned lead_four = 0xf0;
  constexpr unsigned four_byte_shift = 18;
  const auto tail = [&out](const unsigned part) {
    out.push_back(static_cast<char>(continuation | (part & payload)));
  };
  if (value <= last_ascii) {
    out.push_back(static_cast<char>(value));
  } else if (value <= last_two) {
    out.push_back(static_cast<char>(lead_two | (value >> 6U)));
    tail(value);
  } else if (value <= last_three) {
    out.push_back(static_cast<char>(lead_three | (value >> 12U)));
    tail(value >> 6U);
    tail(value);
  } else if (value <= last_scalar) {
    out.push_back(static_cast<char>(lead_four | (value >> four_byte_shift)));
    tail(value >> 12U);
    tail(value >> 6U);
    tail(value);
  } else {
    throw Fault("XML character reference is outside Unicode");
  }
}

void entity(std::string &out, const std::string_view name) {
  if (name == "amp") {
    out.push_back('&');
  } else if (name == "lt") {
    out.push_back('<');
  } else if (name == "gt") {
    out.push_back('>');
  } else if (name == "apos") {
    out.push_back('\'');
  } else if (name == "quot") {
    out.push_back('"');
  } else if (name.starts_with('#')) {
    auto digits = name.substr(1);
    const bool hexadecimal = digits.starts_with('x');
    if (hexadecimal) { digits.remove_prefix(1); }
    std::uint32_t value = 0;
    const auto result =
        std::from_chars(digits.data(), digits.data() + digits.size(), value, hexadecimal ? 16 : 10);
    if (digits.empty() || result.ec != std::errc{} || result.ptr != digits.data() + digits.size()) {
      throw Fault("Invalid XML character reference");
    }
    scalar(out, value);
  } else {
    throw Fault("Unsupported XML entity");
  }
}
}

Reader::Reader(const std::string_view text) : text_(text) {
  if (text.size() > maximum_xml_bytes) { throw Fault("Spectre XML exceeds its resource limit"); }
  if (text_.starts_with("\xef\xbb\xbf")) { offset_ = 3; }
}

bool Reader::starts(const std::string_view text) const {
  return text_.substr(offset_).starts_with(text);
}

void Reader::spaces() {
  while (offset_ < text_.size() && whitespace(text_[offset_])) { ++offset_; }
}

void Reader::expect(const char character) {
  if (offset_ == text_.size() || text_[offset_] != character) {
    throw Fault("Malformed Spectre XML near byte " + std::to_string(offset_));
  }
  ++offset_;
}

std::string Reader::name() {
  const auto start = offset_;
  while (offset_ < text_.size()) {
    const auto c = text_[offset_];
    const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
    const bool legal = letter || (offset_ > start && ((c >= '0' && c <= '9') || c == '-'));
    if (!legal) { break; }
    ++offset_;
  }
  constexpr unsigned maximum_name = 64;
  if (offset_ == start || offset_ - start > maximum_name) {
    throw Fault("Invalid Spectre XML name");
  }
  return std::string(text_.substr(start, offset_ - start));
}

std::string Reader::value() {
  if (offset_ == text_.size() || (text_[offset_] != '\'' && text_[offset_] != '"')) {
    throw Fault("XML attribute requires quotes");
  }
  const auto quote = text_[offset_++];
  std::string result;
  while (offset_ < text_.size() && text_[offset_] != quote) {
    const auto c = text_[offset_++];
    if (c == '<') { throw Fault("Unescaped XML attribute character"); }
    if (c == '&') {
      const auto end = text_.find(';', offset_);
      constexpr unsigned maximum_entity = 12;
      if (end == std::string_view::npos || end - offset_ > maximum_entity) {
        throw Fault("Invalid or unbounded XML entity");
      }
      entity(result, text_.substr(offset_, end - offset_));
      offset_ = end + 1;
    } else if (c == '\r') {
      if (offset_ < text_.size() && text_[offset_] == '\n') { ++offset_; }
      result.push_back(' ');
    } else {
      result.push_back(c == '\n' || c == '\t' ? ' ' : c);
    }
    if (result.size() > maximum_text_bytes) {
      throw Fault("XML attribute exceeds its resource limit");
    }
  }
  expect(quote);
  validate_text(result);
  return result;
}

void Reader::fields(Tag &tag) {
  while (offset_ < text_.size()) {
    const auto before = offset_;
    spaces();
    if (starts(">") || starts("/") || starts("?")) { return; }
    if (offset_ == before) { throw Fault("XML attributes require whitespace"); }
    auto key = name();
    spaces();
    expect('=');
    spaces();
    auto content = value();
    if (std::ranges::any_of(tag.attributes,
                            [&key](const auto &item) { return item.first == key; })) {
      throw Fault("Duplicate XML attribute: " + key);
    }
    constexpr unsigned maximum_attributes = 12;
    if (tag.attributes.size() == maximum_attributes) { throw Fault("Too many XML attributes"); }
    tag.attributes.emplace_back(std::move(key), std::move(content));
  }
  throw Fault("Truncated XML element");
}

void Reader::xml_declaration() {
  if (declaration_ || element_) { throw Fault("Unexpected XML processing instruction"); }
  offset_ += 2;
  Tag tag{.name = name(), .attributes = {}, .closing = false, .empty = false};
  if (tag.name != "xml") { throw Fault("Unsupported XML processing instruction"); }
  fields(tag);
  expect('?');
  expect('>');
  attributes(tag, {"version", "encoding", "standalone"});
  if (attribute(tag, "version") != "1.0") { throw Fault("Unsupported XML declaration version"); }
  for (const auto &[key, content] : tag.attributes) {
    if ((key == "encoding" && content != "UTF-8" && content != "utf-8") ||
        (key == "standalone" && content != "yes" && content != "no")) {
      throw Fault("Unsupported XML declaration");
    }
  }
  declaration_ = true;
}

void Reader::trivia() {
  while (true) {
    spaces();
    if (starts("<!--")) {
      offset_ += 4;
      const auto end = text_.find("--", offset_);
      if (end == std::string_view::npos || text_.substr(end, 3) != "-->") {
        throw Fault("Malformed XML comment");
      }
      validate_text(text_.substr(offset_, end - offset_));
      offset_ = end + 3;
    } else if (starts("<?")) {
      xml_declaration();
    } else {
      return;
    }
  }
}

bool Reader::done() {
  trivia();
  return offset_ == text_.size();
}

Tag Reader::next() {
  trivia();
  expect('<');
  Tag tag;
  if (starts("/")) {
    tag.closing = true;
    ++offset_;
  }
  tag.name = name();
  if (!tag.closing) {
    fields(tag);
  } else {
    spaces();
  }
  if (starts("/") && !tag.closing) {
    tag.empty = true;
    ++offset_;
  }
  expect('>');
  element_ = true;
  return tag;
}
}
