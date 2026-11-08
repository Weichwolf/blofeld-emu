// SPDX-License-Identifier: GPL-3.0-only
#include "execution.hpp"
#include "files/read.hpp"
#include "profile.hpp"
#include "storage/flash_image.hpp"
#include "vm_context.hpp"

#include <cstddef>
#include <dsp56300/memory.hpp>
#include <exception>
#include <filesystem>
#include <ios>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace {
std::filesystem::path selected_profile(const int argc, char **argv) {
  if (argc == 1) { return blofeld::app::default_profile(); }
  if (argc == 3 && std::string_view(argv[1]) == "--profile") { return argv[2]; }
  throw std::invalid_argument("Usage: blofeld-headless [--profile FILE]");
}

const char *operation(const blofeld::vi::Access access) {
  switch (access) {
    case blofeld::vi::Access::read: return "read";
    case blofeld::vi::Access::write: return "write";
    case blofeld::vi::Access::fetch: return "fetch";
    case blofeld::vi::Access::bit: return "bit";
    case blofeld::vi::Access::execute: return "execute";
    case blofeld::vi::Access::reset: return "reset";
    default: return "unknown";
  }
}

std::vector<dsp56300::Word> boot_rom(const std::filesystem::path &path) {
  constexpr std::size_t bytes = 576;
  const auto input = blofeld::host::read_file(path, bytes);
  if (input.size() != bytes) { throw std::runtime_error("DSP SPI boot ROM requires 576 bytes"); }
  std::vector<dsp56300::Word> words;
  words.reserve(bytes / 3);
  for (std::size_t at = 0; at < input.size(); at += 3) {
    words.push_back((static_cast<dsp56300::Word>(input[at]) << 16U) |
                    (static_cast<dsp56300::Word>(input[at + 1]) << 8U) | input[at + 2]);
  }
  return words;
}
}

int main(const int argc, char **argv) try {
  const auto profile = blofeld::app::load_profile(selected_profile(argc, argv));
  auto image = std::make_unique<blofeld::vi::FlashImage>(blofeld::vi::FlashImage::decode(
      blofeld::host::read_file(profile.device_state, blofeld::vi::FlashImage::file_bytes)));
  blofeld::vi::VmContext vm(std::move(image), boot_rom(profile.boot_rom));
  const auto reports = vm.run();
  for (std::size_t index = 0; index < reports.size(); ++index) {
    const auto &report = reports[index];
    if (report.exception) { std::rethrow_exception(report.exception); }
    const auto *name = index == 0 ? "MCU" : "DSP";
    std::cerr << name << " retired=" << report.instructions;
    if (report.failure) {
      const auto &fault = *report.failure;
      std::cerr << " " << operation(fault.operation) << " space=" << fault.space << " pc=0x"
                << std::hex << fault.pc << " address=0x" << fault.address << " value=0x"
                << fault.value << std::dec << " code=" << fault.code << ": " << fault.reason;
    }
    std::cerr << '\n';
  }
  std::cerr << "Startup failed: the instrument is not playable; audio/MIDI are not connected.\n";
  return 2;
} catch (const std::exception &error) {

  std::cerr << "blofeld-headless: " << error.what() << '\n';
  return 1;
}
