// SPDX-License-Identifier: GPL-3.0-only
#include "files/read.hpp"
#include "io/memory_controller.hpp"
#include "storage/media_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::MemoryController;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

constexpr unsigned address_bits = 24;
constexpr std::uint32_t last_block_page = 31, unknown_column = 17;
constexpr std::uint8_t spare = 0x50;
constexpr std::uint32_t idle = 0x300, command = 0x400, address = 0x800, read_active = 0x200;

void pulse(MemoryController &controller, const MemoryController::Write access) {
  require(controller.write(
              {.controls = access.controls | idle, .value = access.value, .known = true}) &&
              controller.write(
                  {.controls = access.controls | 0x100, .value = access.value, .known = true}) &&
              controller.write(
                  {.controls = access.controls | idle, .value = access.value, .known = true}),
          "Publish exactly one decoded command/address strobe");
}

struct Read {
  std::uint32_t page = 0;
  std::uint8_t column = 0, plane = 0;
};

void begin(MemoryController &controller, const Read request) {
  pulse(controller, {.controls = command, .value = request.plane});
  pulse(controller, {.controls = address, .value = request.column});
  for (unsigned shift = 0; shift < address_bits; shift += 8) {
    pulse(controller,
          {.controls = address, .value = static_cast<std::uint8_t>(request.page >> shift)});
  }
  bool ready = true;
  require(controller.pending() && controller.ready(ready) && !ready,
          "A completed address requests a real operation rather than claiming ready");
  require(controller.complete() && !controller.pending() && controller.ready(ready) && ready,
          "The execution owner completes the image operation outside native callbacks");
  require(controller.write({.controls = idle}), "Leave the address phase");
}

std::uint8_t read(MemoryController &controller) {
  require(controller.write({.controls = read_active}), "Select the current image byte");
  const auto value = controller.output();
  if (!value) { throw std::runtime_error("The selected byte must exist"); }
  require(controller.output() == value, "Repeated register reads retain a held byte");
  require(controller.write({.controls = idle}) && !controller.output(),
          "Advance only on read release");
  return *value;
}

void reads(MemoryController &controller) {
  begin(controller, {});
  const auto first = read(controller);
  const auto second = read(controller);
  const auto third = read(controller);
  require(first == 0 && second == 1 && third == 2,
          "Read consecutive bytes from actual backing storage");
  begin(controller, {.page = last_block_page, .column = 15, .plane = spare});
  constexpr std::uint8_t expected_end = 46;
  require(read(controller) == expected_end, "Read the final spare byte of a block");
  begin(controller, {.page = 32});
  constexpr std::uint8_t expected_first = 32;
  require(read(controller) == expected_first, "An explicit new address crosses a block boundary");
}

void errors(MemoryController &controller) {
  controller.reset();
  begin(controller, {.page = 32, .column = unknown_column});
  require(!controller.write({.controls = read_active}) && controller.error() != nullptr &&
              !controller.output(),
          "Missing image bits fail without a fabricated byte");
  controller.reset();
  require(controller.write({.controls = command | idle, .known = true}) &&
              controller.write({.controls = command | 0x100, .known = true}) &&
              !controller.write({.controls = command | idle, .value = 0x80, .known = true}),
          "An unimplemented program command cannot claim success");
  controller.reset();
  require(!controller.output() && !controller.pending(),
          "Reset discards incomplete transfer state");
  pulse(controller, {.controls = command});
  for (const auto value : std::array<std::uint8_t, 4>{0, 1, 0, 0}) {
    pulse(controller, {.controls = address, .value = value});
  }
  bool ready = true;
  require(controller.pending() && controller.ready(ready) && !ready && !controller.complete() &&
              controller.error() != nullptr && !controller.ready(ready),
          "Missing image pages cannot complete or publish readiness");
}
}

int main(const int argc, char **argv) try {
  require(argc == 3, "Supply artifact and fixture directories");
  const blofeld::vi::MediaImage image(blofeld::host::read_file(
      std::filesystem::path(argv[2]) / "media.bfnand", blofeld::vi::MediaImage::max_bytes));
  MemoryController controller(&image);
  reads(controller);
  errors(controller);
  const MemoryController missing;
  bool ready = false;
  require(!missing.ready(ready), "No storage image means no invented ready source");
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
