// SPDX-License-Identifier: GPL-3.0-only
#include "files/read.hpp"
#include "io/memory_controller.hpp"
#include "storage/media_image.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
using blofeld::vi::MemoryController;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

constexpr unsigned address_bits = 24;
constexpr std::uint32_t last_block_page = 31, unknown_column = 17;
constexpr std::uint8_t spare = 0x50, erased = 0xff;
constexpr std::uint8_t reset_command = 0xff;
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

void cancel_and_reset(MemoryController &controller) {
  controller.reset();
  pulse(controller, {.controls = command});
  for (const auto value : std::array<std::uint8_t, 4>{0, 0, 0, 0}) {
    pulse(controller, {.controls = address, .value = value});
  }
  const auto read_epoch = controller.epoch();
  require(controller.pending() && controller.reference_ticks() == 64,
          "A complete read address publishes one nominal timed operation");
  pulse(controller, {.controls = command, .value = reset_command});
  bool ready = true;
  require(controller.pending() && controller.epoch() == read_epoch + 1 &&
              controller.reference_ticks() == 32 && controller.ready(ready) && !ready &&
              !controller.output(),
          "A real RESET supersedes the old completion epoch and keeps readiness low");
  require(controller.complete() && controller.ready(ready) && ready && !controller.output(),
          "Reset completion cannot expose a page retained from the canceled read");
}

void begin_erase(MemoryController &controller, const std::uint32_t page) {
  constexpr std::uint8_t setup = 0x60;
  constexpr std::uint8_t confirm = 0xd0;
  pulse(controller, {.controls = command, .value = setup});
  for (unsigned shift = 0; shift < address_bits; shift += 8) {
    pulse(controller, {.controls = address, .value = static_cast<std::uint8_t>(page >> shift)});
  }
  bool ready = false;
  require(!controller.pending() && controller.ready(ready) && ready,
          "Three row bytes alone cannot erase data or claim a pending operation");
  pulse(controller, {.controls = command, .value = confirm});
  constexpr unsigned erase_ticks = 22580;
  require(controller.pending() && controller.reference_ticks() == erase_ticks &&
              controller.ready(ready) && !ready,
          "Only erase confirm starts the real timed block operation");
}

void erases(const std::filesystem::path &root) {
  using blofeld::vi::MediaImage;
  MediaImage image(blofeld::host::read_file(root / "media-writable.bfnand", MediaImage::max_bytes));
  MemoryController controller(&image);
  begin_erase(controller, last_block_page);
  std::uint8_t value = 0;
  require(image.read({}, value) && value == 0, "Busy erase has not committed an erased block");
  constexpr std::uint8_t status_command = 0x70;
  constexpr std::uint8_t busy = 0x80;
  constexpr std::uint8_t pass = 0xc0;
  constexpr std::uint8_t failed = 0xc1;
  pulse(controller, {.controls = command, .value = status_command});
  require(controller.write({.controls = read_active}) && controller.output() == busy,
          "Status may be read during erase without prematurely claiming completion");
  require(controller.complete() && controller.output() == pass && image.read({}, value) &&
              value == erased,
          "Actual erase completion updates held status without a second read edge");
  require(controller.write({.controls = idle}), "Release the held status byte");
  const auto encoded = image.encoded();
  const auto before = std::vector<std::uint8_t>(encoded.begin(), encoded.end());
  begin_erase(controller, MediaImage::block_pages * 2);
  pulse(controller, {.controls = command, .value = status_command});
  require(controller.complete() && read(controller) == failed &&
              std::ranges::equal(image.encoded(), before),
          "A protected erase completes with real failure status and leaves the image unchanged");
  begin_erase(controller, MediaImage::block_pages * 4);
  require(!controller.complete() && controller.error() != nullptr,
          "Unknown block metadata cannot become a successful operation");
}

void interrupted_erase(const std::filesystem::path &root) {
  using blofeld::vi::MediaImage;
  MediaImage image(blofeld::host::read_file(root / "media-writable.bfnand", MediaImage::max_bytes));
  MemoryController controller(&image);
  begin_erase(controller, 0);
  const auto old_epoch = controller.epoch();
  pulse(controller, {.controls = command, .value = reset_command});
  require(controller.pending() && controller.epoch() == old_epoch + 1 &&
              controller.reference_ticks() == 32,
          "RESET supersedes the pending erase deadline instead of completing it");
  std::uint8_t value = 0;
  require(!image.read({}, value) && controller.complete() && !controller.pending(),
          "Aborted erase preserves uncertainty instead of restoring the preimage or inventing FF");
  begin_erase(controller, 0);
  require(controller.complete() && image.read({}, value) && value == erased,
          "A complete subsequent erase restores a known recoverable block");
  begin_erase(controller, 0);
  controller.reset();
  require(!image.read({}, value) && !controller.pending(),
          "Device teardown or power reset also invalidates an interrupted erase");
}

void invalid_confirm(MemoryController &controller) {
  controller.reset();
  constexpr std::uint8_t confirm = 0xd0;
  require(controller.write({.controls = command | idle, .value = confirm, .known = true}) &&
              controller.write({.controls = command | 0x100, .value = confirm, .known = true}) &&
              !controller.write({.controls = command | idle, .value = confirm, .known = true}),
          "Erase confirm without a complete setup cannot mutate the image");
}

}

int main(const int argc, char **argv) try {
  require(argc == 3, "Supply artifact and fixture directories");
  erases(argv[2]);
  interrupted_erase(argv[2]);
  blofeld::vi::MediaImage image(blofeld::host::read_file(
      std::filesystem::path(argv[2]) / "media.bfnand", blofeld::vi::MediaImage::max_bytes));
  MemoryController controller(&image);
  reads(controller);
  cancel_and_reset(controller);
  errors(controller);
  invalid_confirm(controller);
  const MemoryController missing;
  bool ready = false;
  require(!missing.ready(ready), "No storage image means no invented ready source");
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
