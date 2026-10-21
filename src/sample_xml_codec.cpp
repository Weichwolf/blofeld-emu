// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/sample_library.hpp"
#include "sample_xml.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace blofeld::samples {
namespace {
std::filesystem::path source_path(const xml::Tag &tag, const std::filesystem::path &origin) {
  auto relative = xml::attribute(tag, "RelativePath");
  std::ranges::replace(relative, '\\', '/');
  if (!relative.empty()) {
    const std::filesystem::path path(relative);
    if (path.is_absolute() || (relative.size() > 1 && relative[1] == ':')) {
      throw Fault("RelativePath must be relative to the sample-set file");
    }
    return (std::filesystem::absolute(origin).parent_path() / path).lexically_normal();
  }
  const auto absolute = xml::attribute(tag, "Path");
  const std::filesystem::path path(absolute);
  if (!path.is_absolute()) { throw Fault("Unresolved foreign sample path; supply a RelativePath"); }
  return path.lexically_normal();
}

Sample decode_sample(const xml::Tag &tag, const std::filesystem::path &origin) {
  xml::attributes(
      tag,
      {"Filename", "Path", "RelativePath", "Pitch", "HighKey", "Normalize", "FixedPitch", "Level"});
  // Require the original Path even when a portable RelativePath takes precedence.
  static_cast<void>(xml::attribute(tag, "Path"));
  return {.filename = xml::attribute(tag, "Filename"),
          .source = source_path(tag, origin),
          .pitch = xml::number(xml::attribute(tag, "Pitch")),
          .highest = xml::key(xml::attribute(tag, "HighKey")),
          .normalize = xml::boolean(xml::attribute(tag, "Normalize")),
          .fixed_pitch = xml::boolean(xml::attribute(tag, "FixedPitch")),
          .level_db = xml::number(xml::attribute(tag, "Level"))};
}

struct DecodeState {
  Library library;
  std::optional<std::size_t> current;
  std::size_t samples = 0;
  bool ended = false;
  std::filesystem::path origin;

  void program(const xml::Tag &tag) {
    if (tag.closing) {
      if (!current) { throw Fault("Unexpected SampleProgram end"); }
      order_samples(library.programs.at(*current));
      current.reset();
      return;
    }
    if (current || library.programs.size() == maximum_programs) {
      throw Fault("Nested or excessive SampleProgram elements");
    }
    xml::attributes(tag, {"Name", "Normalize"});
    library.programs.push_back({.name = xml::attribute(tag, "Name"),
                                .normalize = xml::boolean(xml::attribute(tag, "Normalize")),
                                .samples = {}});
    if (!tag.empty) { current = library.programs.size() - 1; }
  }

  void consume(const xml::Tag &tag) {
    if (tag.name == "SampleProgram") {
      program(tag);
      return;
    }
    if (tag.name == "Sample" && current && !tag.closing && tag.empty) {
      if (samples == maximum_samples) { throw Fault("Too many sample references"); }
      library.programs.at(*current).samples.push_back(decode_sample(tag, origin));
      ++samples;
      return;
    }
    if (tag.name == "WaldorfSampleSet" && tag.closing && !current) {
      ended = true;
      return;
    }
    throw Fault("Unsupported or misplaced Spectre XML element: " + tag.name);
  }
};

void put(std::string &out, const std::string_view key, const std::string_view value) {
  out += " " + std::string(key) + "=\"" + xml::escape(value) + "\"";
  if (out.size() > maximum_xml_bytes) {
    throw Fault("Encoded sample library exceeds its resource limit");
  }
}

void encode_sample(std::string &out, const Sample &sample, const std::filesystem::path &base) {
  const auto relative = sample.source.lexically_relative(base);
  if (relative.empty()) { throw Fault("Cannot express sample path relative to output"); }
  out += "    <Sample";
  put(out, "Filename", sample.filename);
  put(out, "Path", sample.source.generic_string());
  put(out, "RelativePath", relative.generic_string());
  put(out, "Pitch", xml::decimal(sample.pitch));
  put(out, "HighKey", std::to_string(sample.highest));
  put(out, "Normalize", sample.normalize ? "1" : "0");
  put(out, "FixedPitch", sample.fixed_pitch ? "1" : "0");
  put(out, "Level", xml::decimal(sample.level_db));
  out += "/>\n";
}
}

Library parse_xml(const std::string_view text, const std::filesystem::path &origin) {
  xml::Reader reader(text);
  auto root = reader.next();
  xml::attributes(root, {"Path", "Version"});
  if (root.name != "WaldorfSampleSet" || root.closing || xml::attribute(root, "Version") != "0.5") {
    throw Fault("Unsupported Spectre XML root/version");
  }
  DecodeState state{
      .library = {}, .current = {}, .samples = 0, .ended = root.empty, .origin = origin};
  while (!reader.done()) {
    if (state.ended) { throw Fault("Unexpected XML after sample-set root"); }
    state.consume(reader.next());
  }
  if (!state.ended || state.current) { throw Fault("Truncated Spectre XML sample set"); }
  auto result = std::move(state.library);
  validate(result);
  return result;
}

std::string encode_xml(const Library &library, const std::filesystem::path &output) {
  validate(library);
  const auto absolute = std::filesystem::absolute(output).lexically_normal();
  std::string out = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\n<WaldorfSampleSet";
  put(out, "Path", absolute.generic_string());
  put(out, "Version", "0.5");
  out += ">\n";
  for (const auto &program : library.programs) {
    out += "  <SampleProgram";
    put(out, "Name", program.name);
    put(out, "Normalize", program.normalize ? "1" : "0");
    out += ">\n";
    for (const auto &sample : program.samples) {
      encode_sample(out, sample, absolute.parent_path());
    }
    out += "  </SampleProgram>\n";
  }
  out += "</WaldorfSampleSet>\n";
  if (out.size() > maximum_xml_bytes) {
    throw Fault("Encoded sample library exceeds its resource limit");
  }
  return out;
}
}
