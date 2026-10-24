// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/device_identity.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/private_file.hpp"
#include "blofeld/virtual_device_state.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
#include <system_error>
#include <unistd.h>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned flash_first = 0x8000, erased = 0xff, canary = 0xa5;
constexpr unsigned serial_offset = device_identity::serial_address - flash_first;
constexpr unsigned own_code_offset = 0x300, updated_code = 0x5a;
constexpr std::string_view own_serial = "EMU-SAVED-01";

void require(const bool condition, const std::string_view text) {
  if (!condition) { throw Fault(std::string(text)); }
}

void rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &) { return; }
  throw Fault("Expected persistent device state refusal");
}

class TemporaryDirectory {
public:
  TemporaryDirectory() {
    constexpr unsigned attempts = 64;
    for (unsigned n = 0; n < attempts; ++n) {
      const auto candidate =
          std::filesystem::temp_directory_path() /
          ("blofeld-identity-" + std::to_string(getpid()) + "-" + std::to_string(n));
      if (!std::filesystem::create_directory(candidate)) { continue; }
      path_ = candidate;
      std::filesystem::permissions(path_, std::filesystem::perms::owner_all);
      return;
    }
    throw Fault("Cannot create own device-state test directory");
  }

  ~TemporaryDirectory() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }

  TemporaryDirectory(const TemporaryDirectory &) = delete;
  TemporaryDirectory &operator=(const TemporaryDirectory &) = delete;

  [[nodiscard]] std::filesystem::path file() const { return path_ / "unit.bfnvm"; }

private:
  std::filesystem::path path_;
};

McuFlashImage initial_image() {
  McuFlashImage image;
  image.bytes.fill(canary);
  image.known.set();
  for (unsigned n = 0; n < device_identity::serial_bytes; ++n) {
    image.bytes[serial_offset + n] = erased;
  }
  return image;
}

void persisted_unit() {
  TemporaryDirectory directory;
  const auto original = initial_image();
  unsigned generated = 0;
  const auto factory = [&] {
    ++generated;
    return std::string(own_serial);
  };
  {
    VirtualDeviceState device(directory.file(), original, factory);
    require(generated == 1 && device.image().bytes[serial_offset] == 'E',
            "A new virtual unit receives its own actual factory NVM serial exactly once");
    require(std::filesystem::status(directory.file()).permissions() ==
                (std::filesystem::perms::owner_read | std::filesystem::perms::owner_write),
            "Manufacturer-containing runtime NVM remains private0600");
    rejected([&] { const VirtualDeviceState competing(directory.file(), original, factory); });
    require(generated == 1, "A competing writer cannot allocate another identity for this unit");
    auto changed = device.image();
    changed.bytes[own_code_offset] = updated_code;
    device.save(changed);
  }
  const McuFlashImage
      unrelated_template; // No known data: an existing device must never merge this template.
  {
    VirtualDeviceState device(directory.file(), unrelated_template, factory);
    require(generated == 1 && device.image().bytes[own_code_offset] == updated_code &&
                device.image().bytes[serial_offset] == 'E',
            "Cold reopen restores persistent program and identity without regenerating or ZIP "
            "fallback");
    auto interrupted = device.image();
    interrupted.invalidate({.offset = serial_offset, .size = 1});
    device.save(interrupted);
  }
  {
    const VirtualDeviceState device(directory.file(), original, factory);
    require(generated == 1 && !device.image().known[serial_offset] &&
                device.image().bytes[serial_offset] == 0,
            "Persisted unknown factory bytes survive cold reopen; no automatic identity repair");
  }
}

void failed_input() {
  TemporaryDirectory directory;
  const auto original = initial_image();
  unsigned generated = 0;
  const auto factory = [&] {
    ++generated;
    return std::string(own_serial);
  };
  constexpr std::array<std::uint8_t, 3> corrupt{1, 2, 3};
  write_private_file(directory.file(), corrupt);
  rejected([&] { const VirtualDeviceState bad(directory.file(), original, factory); });
  require(generated == 0 && read_private_file(directory.file(), corrupt.size()) ==
                                std::vector<std::uint8_t>(corrupt.begin(), corrupt.end()),
          "Corrupt existing state fails without generating a new unit or replacing its file");
}

void actual_entropy_format() {
  const auto serial = generate_virtual_serial();
  require(serial.size() == 12 && serial.starts_with("EMU"),
          "Kernel entropy supplies the documented bounded virtual serial format");
  for (const char character : std::string_view(serial).substr(3)) {
    require(std::string_view("0123456789ABCDEFGHJKMNPQRSTVWXYZ").contains(character),
            "The generated suffix fits the original ASCII/USB string representation");
  }
}

void failed_publication() {
  const TemporaryDirectory directory;
  const auto original = initial_image();
  const auto preserved = directory.file().parent_path() / "preserved.bfnvm";
  std::vector<std::uint8_t> committed;
  {
    VirtualDeviceState device(directory.file(), original, [] { return std::string(own_serial); });
    committed = read_private_file(directory.file(), flash_image_file_bytes);
    auto invalid = device.image();
    invalid.known.reset(0); // The nonzero byte is deliberately noncanonical own input.
    rejected([&] { device.save(invalid); });
    require(device.image().known[0] &&
                read_private_file(directory.file(), flash_image_file_bytes) == committed,
            "Pre-publication input validation leaves the current unit healthy and unchanged");
    std::filesystem::rename(directory.file(), preserved);
    std::filesystem::create_symlink(preserved, directory.file());
    rejected([&] { device.save(device.image()); });
    rejected([&] { (void)device.image(); });
    require(read_private_file(preserved, flash_image_file_bytes) == committed,
            "A failed publication preserves the committed image and quarantines the owner");
    std::filesystem::remove(directory.file());
    std::filesystem::rename(preserved, directory.file());
  }
  const VirtualDeviceState reopened(directory.file(), original, {});
  require(encode_flash_image(reopened.image()) == committed,
          "Reopen recovers the exact last publication without allocating another identity");
}
}

int main() {
  try {
    persisted_unit();
    failed_input();
    actual_entropy_format();
    failed_publication();
    std::cout
        << "Virtual MCU identity survives restart, code changes and failed-state retention.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
