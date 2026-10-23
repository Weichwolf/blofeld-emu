// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "dsp_voice_fixture.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <string_view>

namespace {
class Diagnostics {
public:
  Diagnostics() : previous_(std::cerr.rdbuf(output_.rdbuf())) {}

  ~Diagnostics() { std::cerr.rdbuf(previous_); }

  Diagnostics(const Diagnostics &) = delete;
  Diagnostics &operator=(const Diagnostics &) = delete;

  [[nodiscard]] std::string text() const { return output_.str(); }

private:
  std::ostringstream output_;
  std::streambuf *previous_;
};

void run(const int count, char **arguments) {
  if (count != 4) { throw blofeld::Fault("Expected firmware, private trace and guard profile"); }
  const std::string_view profile(arguments[3]);
  if (profile != "foreground" && profile != "stream") {
    throw blofeld::Fault("Expected foreground or stream program-guard profile");
  }
  const auto mode = profile == "stream" ? blofeld::test::VoiceInput::stream
                                        : blofeld::test::VoiceInput::foreground;
  const Diagnostics diagnostics;
  const auto code = blofeld::test::run_voice_test(
      3, arguments, false, true, false, {.program_guard_negative = true}, mode);
  const auto error = diagnostics.text();
  if (code != 1 || !error.contains("Original foreground executable word changed at P:0x000100") ||
      !error.contains("pc=0x000198") || !error.contains("omr=")) {
    throw blofeld::Fault("The exact original program-guard failure is absent: " + error);
  }
  std::ifstream trace(arguments[2]);
  trace.exceptions(std::ios::badbit);
  unsigned blocks = 0;
  unsigned mutations = 0;
  for (std::string line; std::getline(trace, line);) {
    if (line.starts_with("BLOCK ")) { ++blocks; }
    if (line == "PROGRAM_GUARD_CORRUPTED 100") { ++mutations; }
  }
  if (blocks != 1 || mutations != 1) {
    throw blofeld::Fault("The program guard must reject before a second original block");
  }
  std::cout << "program-guard-rejected profile=" << profile << " blocks=1\n";
}
}

int main(const int count, char **arguments) {
  try {
    run(count, arguments);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
