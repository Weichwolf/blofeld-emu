// SPDX-License-Identifier: GPL-3.0-only
#include "audio/playback.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>

int main() try {
  bool rejected = false;
  try {
    const blofeld::host::audio::Playback invalid(0);
  } catch (const std::invalid_argument &) { rejected = true; }
  if (!rejected) {
    throw std::runtime_error("Invalid audio rates must fail before opening a device");
  }
  rejected = false;
  constexpr unsigned rate = 44100;
  try {
    const blofeld::host::audio::Playback unavailable(rate);
  } catch (const std::runtime_error &) { rejected = true; }
  if (!rejected) {
    throw std::runtime_error("An absent test server cannot produce invented audio grants");
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
