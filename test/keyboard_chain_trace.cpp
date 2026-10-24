// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "keyboard_chain_fixture.hpp"

#include <algorithm>
#include <cstddef>
#include <ios>
#include <streambuf>
#include <string>
#include <string_view>

namespace blofeld::test::keyboard_chain {
std::streamsize TraceFilter::xsputn(const char *data, const std::streamsize size) {
  if (size < 0) { throw Fault("Negative controller trace write extent"); }
  const auto *end = data + static_cast<std::size_t>(size);
  while (data != end) {
    const auto *newline = std::find(data, end, '\n');
    line_.append(data, static_cast<std::size_t>(newline - data));
    if (newline == end) { break; }
    line_ += '\n';
    emit();
    data = newline + 1;
  }
  return size;
}

TraceFilter::int_type TraceFilter::overflow(const int_type value) {
  if (traits_type::eq_int_type(value, traits_type::eof())) { return traits_type::not_eof(value); }
  const auto byte = traits_type::to_char_type(value);
  line_ += byte;
  if (byte == '\n') { emit(); }
  return value;
}

int TraceFilter::sync() {
  if (!line_.empty()) { emit(); }
  return 0;
}

void TraceFilter::restore_counts(const TraceCounts saved) {
  if (!line_.empty()) { throw Fault("Controller trace restore requires a complete row boundary"); }
  counts_ = saved;
}

void TraceFilter::emit() {
  bool omit = false;
  if (line_.starts_with("D ")) {
    ++counts_.dsp;
    omit = true;
  } else if (line_.starts_with("M ")) {
    ++counts_.mcu;
    omit = true;
  } else if (line_.starts_with("PAD_QUERY ")) {
    const std::string_view row(line_);
    const auto token = [row](const std::string_view key) {
      const auto begin = row.find(key);
      if (begin == std::string_view::npos) { throw Fault("Malformed owned pad trace"); }
      const auto field = row.substr(begin + key.size());
      return field.substr(0, field.find_first_of(" \n"));
    };
    const auto conflicts = token("conflicts=");
    if (token("requested=") == token("known=") && conflicts.starts_with("0x") &&
        conflicts.substr(2).find_first_not_of('0') == std::string_view::npos) {
      ++counts_.pads;
      omit = true;
    }
  }
  if (!omit) { trace_ << line_; }
  line_.clear();
}
}
