// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/host_midi.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <span>
#include <string>

namespace {
constexpr std::size_t source_bytes = 8192;
constexpr std::size_t maximum_read = 4096;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected a rejected host MIDI operation");
}

std::size_t descriptor_count() {
  return static_cast<std::size_t>(std::distance(
      std::filesystem::directory_iterator("/proc/self/fd"), std::filesystem::directory_iterator{}));
}

void write_source(const std::filesystem::path &path) {
  std::ofstream output(path, std::ios::binary);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  for (std::size_t index = 0; index < source_bytes; ++index) {
    output.put(static_cast<char>(index % 256));
  }
}

void test_fragmented_reads(const std::filesystem::path &path) {
  const auto before = descriptor_count();
  {
    blofeld::HostMidiInput input(path, blofeld::MidiInputKind::bytes);
    require(input.read({}) == 0 && !input.exhausted(), "An empty destination must not consume EOF");
    std::array<std::uint8_t, maximum_read + 1> bytes{};
    require(rejects([&] { static_cast<void>(input.read(bytes)); }).contains("bounded"),
            "Caller spans cannot exceed the fixed read bound");
    std::size_t cursor = 0;
    constexpr std::array<std::size_t, 4> fragments{1, 7, 13, maximum_read};
    unsigned chunk = 0;
    while (!input.exhausted()) {
      const auto offered = fragments.at(chunk++ % fragments.size());
      const auto count = input.read(std::span{bytes}.first(offered));
      require(count <= offered, "The host read never exceeds downstream capacity");
      for (std::size_t index = 0; index < count; ++index) {
        require(bytes.at(index) == (cursor + index) % 256,
                "Every status/data/realtime/SysEx byte survives fragmented reads unchanged");
      }
      cursor += count;
    }
    require(cursor == source_bytes && input.read(std::span{bytes}.first(1)) == 0,
            "A complete finite source ends exactly once without padding or repeated bytes");
  }
  require(descriptor_count() == before, "Host input RAII closes the read-only descriptor");
}

void test_rejected_sources(const std::filesystem::path &path) {
  const auto before = descriptor_count();
  for (unsigned attempt = 0; attempt < 16; ++attempt) {
    require(rejects([&] {
              const blofeld::HostMidiInput input(path, blofeld::MidiInputKind::linux_rawmidi);
            }).contains("character device"),
            "A regular file cannot masquerade as a successfully opened RawMIDI device");
    require(rejects([&] {
              const blofeld::HostMidiInput input(path.parent_path(), blofeld::MidiInputKind::bytes);
            }).contains("regular binary file"),
            "Directories are rejected after fstat without leaking descriptors");
  }
  require(rejects([&] {
            const blofeld::HostMidiInput input(path / "absent", blofeld::MidiInputKind::bytes);
          }).contains("Host MIDI open path="),
          "Open errors retain host operation and source path");
  require(descriptor_count() == before, "All rejected constructor paths retain descriptor count");
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 2) { throw blofeld::Fault("Usage: host-midi-test private-output-directory"); }
    const std::filesystem::path directory(arguments[1]);
    std::filesystem::create_directories(directory);
    const auto path = directory / "all-bytes.bin";
    write_source(path);
    test_fragmented_reads(path);
    test_rejected_sources(path);
    require(std::filesystem::file_size(path) == source_bytes, "The input file remains intact");
    std::cout << "Bounded read-only MIDI file input and rejection/descriptor guards passed. "
                 "No physical RawMIDI device or kernel overrun path was exercised.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
