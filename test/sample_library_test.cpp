// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/sample_library.hpp"

#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld::samples;
constexpr std::string_view own_xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<WaldorfSampleSet Version="0.5" Path="Z:\old\set.xml">
  <SampleProgram Name="Own &amp; &#x3a9;" Normalize="1">
    <Sample Filename="high.wav" Path="Z:\old\high.wav" RelativePath="samples\high.wav"
            Pitch="69.05" HighKey="127" Normalize="0" FixedPitch="1" Level="-6.25"/>
    <Sample Filename="low.wav" Path="Z:\old\low.wav" RelativePath="samples\low.wav"
            Pitch="48.00" HighKey="60" Normalize="1" FixedPitch="0" Level="0.00"/>
  </SampleProgram>
</WaldorfSampleSet>)";

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw blofeld::Fault(std::string(message)); }
}

void reject(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault("An invalid sample-library input reported success");
}

std::string replaced(const std::string_view old, const std::string_view value) {
  auto text = std::string(own_xml);
  const auto at = text.find(old);
  require(at != std::string::npos, "The negative control changes a real fixture field");
  text.replace(at, old.size(), value);
  return text;
}

void test_independent_xml() {
  const auto library = parse_xml(own_xml, "/tmp/project/set.xml");
  require(library.programs.size() == 1 && library.programs[0].name == "Own & Ω" &&
              library.programs[0].normalize && library.programs[0].samples.size() == 2,
          "Own XML uses the element/attribute vocabulary observed in original Spectre");
  const auto &low = library.programs[0].samples[0];
  const auto &high = library.programs[0].samples[1];
  constexpr double root = 69.05;
  constexpr double gain = -6.25;
  constexpr unsigned low_highest = 60;
  require(low.filename == "low.wav" && low.source == "/tmp/project/samples/low.wav" &&
              low.highest == low_highest && low.normalize && !low.fixed_pitch &&
              high.source == "/tmp/project/samples/high.wav" && high.pitch == root &&
              high.level_db == gain && high.fixed_pitch,
          "Independent Unicode/entity, key-order, gain and portable path expectations match");
  const auto encoded = encode_xml(library, "/tmp/project/export/moved.xml");
  require(encoded.contains("RelativePath=\"../samples/low.wav\"") &&
              encoded.contains("Name=\"Own &amp; Ω\"") && encoded.contains("Pitch=\"69.05\"") &&
              encoded.contains("Version=\"0.5\""),
          "Output escapes names and rebases actual sample references without moving audio files");
  require(parse_xml(encoded, "/tmp/project/export/moved.xml") == library,
          "A relocated save additionally retains the already independently checked model");
}

void test_invalid_xml() {
  for (const auto &text : std::vector<std::string>{
           "",
           "<!DOCTYPE x [<!ENTITY e SYSTEM 'file:///etc/passwd'>]>" + std::string(own_xml),
           replaced("Version=\"0.5\"", "Version=\"9\""),
           replaced("Normalize=\"1\"", "Normalize=\"2\""),
           replaced("Pitch=\"69.05\"", "Pitch=\"nan\""),
           replaced("Pitch=\"69.05\"", "Pitch=\"+-1\""),
           replaced("Pitch=\"69.05\"", "Pitch=\"1e9999\""),
           replaced("HighKey=\"127\"", "HighKey=\"128\""),
           replaced("Name=\"Own &amp; &#x3a9;\"", "Name=\"&#0;\""),
           replaced("Name=\"Own &amp; &#x3a9;\"", "Name=\"&#xD800;\""),
           replaced("Name=\"Own &amp; &#x3a9;\"", "Name=\"&unknown;\""),
           replaced(R"(Normalize="1")", R"(Normalize="1" Normalize="0")"),
           replaced(R"(RelativePath="samples\high.wav")", R"(RelativePath="C:\foreign.wav")"),
           replaced("<Sample Filename", "<Unexpected Filename"),
           replaced("</SampleProgram>", ""),
           replaced("</WaldorfSampleSet>", "</WaldorfSampleSet><Extra/>"),
           replaced("encoding=\"UTF-8\"", "encoding=\"UTF-16\""),
           replaced("Own &amp;", "Own <"),
           replaced("Own &amp;", std::string("Own ") + static_cast<char>(0xc0)),
           replaced("<SampleProgram", "<SampleProgram><SampleProgram"),
           replaced("Name=\"Own", "Name=\"Own&#10;") + "<?bad?>"}) {
    reject([&] { static_cast<void>(parse_xml(text, "/tmp/set.xml")); });
  }
  const std::string oversized(maximum_xml_bytes + 1, ' ');
  reject([&] { static_cast<void>(parse_xml(oversized, "/tmp/set.xml")); });
}

void test_edit_operations() {
  Library library{.programs = {{.name = "A", .normalize = false, .samples = {}},
                               {.name = "B", .normalize = false, .samples = {}},
                               {.name = "C", .normalize = false, .samples = {}}}};
  move_program(library, 0, 2);
  require(library.programs[0].name == "B" && library.programs[2].name == "A",
          "Forward movement preserves intervening program order");
  move_program(library, 2, 0);
  require(library.programs[0].name == "A" && library.programs[1].name == "B",
          "Backward movement has an independent expected order");
  const auto before = library;
  reject([&] { move_program(library, 3, 0); });
  require(library == before, "Out-of-range movement cannot partially reorder a collection");
  auto model = parse_xml(own_xml, "/tmp/project/set.xml");
  auto &program = model.programs[0];
  constexpr double octave = 12;
  constexpr double original = 69.05;
  constexpr unsigned transposed_low = 60;
  transpose_roots(program, octave);
  require(program.samples[0].pitch == transposed_low &&
              program.samples[1].pitch == original + octave,
          "Root transposition preserves individual tuning and highest-key ordering");
  const auto unchanged = program;
  reject([&] { transpose_roots(program, std::numeric_limits<double>::infinity()); });
  require(program == unchanged, "Invalid transposition is rejected before changing sample roots");
}
}

int main() {
  try {
    test_independent_xml();
    test_invalid_xml();
    test_edit_operations();
    std::cout << "Verified Spectre 0.5 XML vocabulary, independent metadata/path expectations, "
                 "bounded rejection and collection edits. No WAV decoding or transmission claim.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
