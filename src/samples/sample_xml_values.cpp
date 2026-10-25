// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "samples/sample_library.hpp"
#include "samples/sample_xml.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace blofeld::samples::xml {
std::string attribute(const Tag &tag, const std::string_view name) {
  for (const auto &[key, value] : tag.attributes) {
    if (key == name) { return value; }
  }
  throw Fault("Missing " + tag.name + " attribute: " + std::string(name));
}

void attributes(const Tag &tag, const std::vector<std::string_view> &allowed) {
  for (const auto &[key, value] : tag.attributes) {
    static_cast<void>(value);
    if (std::ranges::find(allowed, key) == allowed.end()) {
      throw Fault("Unsupported " + tag.name + " attribute: " + key);
    }
  }
}

double number(std::string_view text) {
  if (text.starts_with('+')) {
    text.remove_prefix(1);
    if (text.starts_with('-') || text.starts_with('+')) { throw Fault("Invalid numeric sign"); }
  }
  if (text.empty()) { throw Fault("Missing sample-library number"); }
  double result = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
      !std::isfinite(result)) {
    throw Fault("Sample-library number must be finite and complete");
  }
  return result;
}

unsigned key(const std::string_view text) {
  unsigned result = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  constexpr unsigned last_key = 127;
  if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
      result > last_key) {
    throw Fault("Highest key must be an integer from 0 to 127");
  }
  return result;
}

bool boolean(const std::string_view text) {
  if (text == "0") { return false; }
  if (text == "1") { return true; }
  throw Fault("Sample-library boolean must be 0 or 1");
}

std::string decimal(const double value) {
  if (!std::isfinite(value)) { throw Fault("Cannot encode a nonfinite sample-library number"); }
  std::array<char, 512> buffer{};
  const auto result = std::to_chars(
      buffer.data(), buffer.data() + buffer.size(), value, std::chars_format::fixed, 2);
  if (result.ec != std::errc{}) { throw Fault("Cannot encode sample-library number"); }
  return {buffer.data(), result.ptr};
}

std::string escape(const std::string_view text) {
  validate_text(text);
  std::string result;
  for (const auto c : text) {
    switch (c) {
      case '&': result += "&amp;"; break;
      case '<': result += "&lt;"; break;
      case '>': result += "&gt;"; break;
      case '"': result += "&quot;"; break;
      case '\'': result += "&apos;"; break;
      case '\r': result += "&#13;"; break;
      case '\n': result += "&#10;"; break;
      case '\t': result += "&#9;"; break;
      default: result.push_back(c); break;
    }
  }
  return result;
}
}
