// SPDX-License-Identifier: GPL-3.0-only
#include "common/telemetry.hpp"

#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void disabled() {
  const blofeld::Telemetry output;
  unsigned formats = 0;
  output.emit([&formats](std::ostream &) {
    ++formats;
    throw std::runtime_error("A disabled event evaluated its formatter");
  });
  require(formats == 0, "Disabled telemetry never evaluates formatting or its side effects");
}

void enabled() {
  std::ostringstream stream;
  const blofeld::Telemetry output(&stream);
  unsigned formats = 0;
  output.emit([&formats](std::ostream &destination) {
    ++formats;
    destination << "event=write address=17 value=23\n";
  });
  require(formats == 1 && stream.str() == "event=write address=17 value=23\n",
          "Selected output receives the exact event once");
  const auto copy = output;
  copy.emit([](std::ostream &destination) { destination << "second\n"; });
  require(stream.str() == "event=write address=17 value=23\nsecond\n",
          "Copied host policy retains its non-owning stream");
}

void failure() {
  std::ostringstream stream;
  const blofeld::Telemetry output(&stream);
  bool observed = false;
  try {
    output.emit([](std::ostream &) { throw std::runtime_error("formatter failure"); });
  } catch (const std::runtime_error &error) {
    observed = std::string(error.what()) == "formatter failure";
  }
  require(observed, "An enabled formatter failure remains visible");
}
}

int main() {
  try {
    disabled();
    enabled();
    failure();
    std::cout << "Lazy telemetry off/output/copy/failure contracts passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
