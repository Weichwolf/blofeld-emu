// SPDX-License-Identifier: GPL-3.0-only
#include "sample_client.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/sample_library.hpp"
#include "sample_xml.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstddef>
#include <filesystem>
#include <iterator>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace blofeld::samples {
namespace {
constexpr std::string_view usage =
    "Usage: blofeld-headless samples COMMAND ...\n"
    "  new OUTPUT.xml\n"
    "  list INPUT.xml\n"
    "  add-program INPUT.xml OUTPUT.xml NAME\n"
    "  rename-program INPUT.xml OUTPUT.xml PROGRAM NAME\n"
    "  move-program INPUT.xml OUTPUT.xml PROGRAM DESTINATION\n"
    "  remove-program INPUT.xml OUTPUT.xml PROGRAM\n"
    "  export-program INPUT.xml OUTPUT.xml PROGRAM\n"
    "  append INPUT.xml OUTPUT.xml OTHER.xml\n"
    "  remove-sample INPUT.xml OUTPUT.xml PROGRAM SAMPLE\n"
    "  set-sample INPUT.xml OUTPUT.xml PROGRAM SAMPLE FIELD VALUE\n"
    "  set-program-normalize INPUT.xml OUTPUT.xml PROGRAM 0|1\n"
    "  transpose-roots INPUT.xml OUTPUT.xml PROGRAM SEMITONES\n"
    "Indices start at 1. Fields: pitch, highest, level, normalize, fixed-pitch, filename.\n";

class Arguments {
public:
  explicit Arguments(const std::span<char *> values) : values_(values) {}

  [[nodiscard]] std::string_view get(const std::size_t i) const {
    if (i >= values_.size()) { throw Fault(std::string(usage)); }
    return values_[i];
  }

  void count(const std::size_t size) const {
    if (values_.size() != size) { throw Fault(std::string(usage)); }
  }

  [[nodiscard]] std::size_t index(const std::size_t at, const auto &collection) const {
    const auto text = get(at);
    std::size_t value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        value == 0 || value > collection.size()) {
      throw Fault("Sample-library index is outside the collection");
    }
    return value - 1;
  }

private:
  std::span<char *> values_;
};

std::string json_string(const std::string_view text) {
  constexpr std::string_view hex = "0123456789abcdef";
  std::string out = "\"";
  for (const auto c : text) {
    const auto byte = static_cast<unsigned char>(c);
    if (c == '"' || c == '\\') {
      out.push_back('\\');
      out.push_back(c);
    } else if (byte < 0x20) {
      out += "\\u00";
      out.push_back(hex[byte >> 4U]);
      out.push_back(hex[byte & 15U]);
    } else {
      out.push_back(c);
    }
  }
  return out + '"';
}

void listing(const Library &library, std::ostream &output) {
  output << "{\"programs\":[";
  std::size_t index = 0;
  for (const auto &program : library.programs) {
    if (index != 0) { output << ','; }
    output << "{\"index\":" << ++index << ",\"name\":" << json_string(program.name)
           << ",\"normalize\":" << (program.normalize ? "true" : "false") << ",\"samples\":[";
    std::size_t sample_index = 0;
    for (const auto &sample : program.samples) {
      if (sample_index != 0) { output << ','; }
      output << "{\"index\":" << ++sample_index << ",\"filename\":" << json_string(sample.filename)
             << ",\"path\":" << json_string(sample.source.generic_string())
             << ",\"pitch\":" << xml::decimal(sample.pitch) << ",\"highest\":" << sample.highest
             << ",\"level_db\":" << xml::decimal(sample.level_db)
             << ",\"normalize\":" << (sample.normalize ? "true" : "false")
             << ",\"fixed_pitch\":" << (sample.fixed_pitch ? "true" : "false") << '}';
    }
    output << "]}";
  }
  output << "]}\n";
}

void set_sample(Program &program, const Arguments &args) {
  args.count(7);
  auto &sample = program.samples.at(args.index(4, program.samples));
  const auto field = args.get(5);
  const auto value = args.get(6);
  if (field == "pitch") {
    sample.pitch = xml::number(value);
  } else if (field == "highest") {
    sample.highest = xml::key(value);
  } else if (field == "level") {
    sample.level_db = xml::number(value);
  } else if (field == "normalize") {
    sample.normalize = xml::boolean(value);
  } else if (field == "fixed-pitch") {
    sample.fixed_pitch = xml::boolean(value);
  } else if (field == "filename") {
    sample.filename = value;
  } else {
    throw Fault("Unknown sample metadata field: " + std::string(field));
  }
  order_samples(program);
}

void modify(Library &library, const Arguments &args) {
  const auto command = args.get(0);
  if (command == "add-program") {
    args.count(4);
    library.programs.push_back(
        {.name = std::string(args.get(3)), .normalize = false, .samples = {}});
    return;
  }
  if (command == "append") {
    args.count(4);
    auto other = load_xml(args.get(3));
    if (other.programs.size() > maximum_programs - library.programs.size()) {
      throw Fault("Too many sample programs after append");
    }
    for (auto &program : other.programs) { library.programs.push_back(std::move(program)); }
    return;
  }
  const auto index = args.index(3, library.programs);
  auto &program = library.programs.at(index);
  if (command == "rename-program") {
    args.count(5);
    program.name = args.get(4);
  } else if (command == "move-program") {
    args.count(5);
    move_program(library, index, args.index(4, library.programs));
  } else if (command == "remove-program") {
    args.count(4);
    library.programs.erase(std::next(library.programs.begin(), static_cast<std::ptrdiff_t>(index)));
  } else if (command == "export-program") {
    args.count(4);
    library = {.programs = {program}};
  } else if (command == "remove-sample") {
    args.count(5);
    const auto sample = args.index(4, program.samples);
    program.samples.erase(std::next(program.samples.begin(), static_cast<std::ptrdiff_t>(sample)));
  } else if (command == "set-sample") {
    set_sample(program, args);
  } else if (command == "set-program-normalize") {
    args.count(5);
    program.normalize = xml::boolean(args.get(4));
  } else if (command == "transpose-roots") {
    args.count(5);
    transpose_roots(program, xml::number(args.get(4)));
  } else {
    throw Fault("Unknown samples command: " + std::string(command));
  }
}
}

int run_client(const std::span<char *> arguments, std::ostream &output) {
  if (arguments.empty() || std::string_view(arguments[0]) == "--help") {
    output << usage;
    return 0;
  }
  const Arguments args(arguments);
  const auto command = args.get(0);
  constexpr std::array<std::string_view, 12> commands{"new",
                                                      "list",
                                                      "add-program",
                                                      "rename-program",
                                                      "move-program",
                                                      "remove-program",
                                                      "export-program",
                                                      "append",
                                                      "remove-sample",
                                                      "set-sample",
                                                      "set-program-normalize",
                                                      "transpose-roots"};
  if (std::ranges::find(commands, command) == commands.end()) {
    throw Fault("Unsupported samples command: " + std::string(command));
  }
  if (command == "new") {
    args.count(2);
    save_xml({}, args.get(1));
    return 0;
  }
  auto library = load_xml(args.get(1));
  if (command == "list") {
    args.count(2);
    listing(library, output);
    return 0;
  }
  modify(library, args);
  save_xml(library, args.get(2));
  return 0;
}
}
