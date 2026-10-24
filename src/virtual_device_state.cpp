// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/virtual_device_state.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/device_identity.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/private_file.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <sys/random.h>

namespace blofeld {
std::string generate_virtual_serial() {
  constexpr unsigned entropy_bytes = 8;
  constexpr unsigned digit_bits = 5;
  constexpr unsigned digits = 9;
  constexpr std::string_view alphabet = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";
  std::array<std::uint8_t, entropy_bytes> entropy{};
  std::size_t filled = 0;
  while (filled < entropy.size()) {
    const auto count = getrandom(entropy.data() + filled, entropy.size() - filled, 0);
    if (count < 0 && errno == EINTR) { continue; }
    if (count <= 0) {
      throw Fault("Cannot generate virtual device identity errno=" + std::to_string(errno));
    }
    filled += static_cast<std::size_t>(count);
  }
  std::uint64_t word = 0;
  for (const auto byte : entropy) { word = (word << 8U) | byte; }
  std::string serial = "EMU";
  for (unsigned n = 0; n < digits; ++n) {
    serial += alphabet[word & (alphabet.size() - 1)];
    word >>= digit_bits;
  }
  return serial;
}

class VirtualDeviceState::Impl {
public:
  Impl(const std::filesystem::path &path,
       const McuFlashImage &initial,
       const VirtualSerialFactory &serial_factory)
      : directory(path.has_parent_path() ? path.parent_path() : "."),
        filename(path.filename()),
        lease(directory, filename.string() + ".lock") {
    if (directory.contains(filename)) {
      image = decode_flash_image(directory.read(filename, flash_image_file_bytes));
    } else {
      if (!serial_factory) { throw Fault("New virtual device requires an identity source"); }
      image = provision_virtual_identity(initial, serial_factory());
      directory.write(filename, encode_flash_image(image));
    }
  }

  void healthy() const {
    if (!valid) { throw Fault("Virtual device NVM publication failed; reopen the device state"); }
  }

  PrivateDirectory directory;
  std::filesystem::path filename;
  PrivateFileLease lease;
  McuFlashImage image;
  bool valid = true;
};

VirtualDeviceState::VirtualDeviceState(const std::filesystem::path &path,
                                       const McuFlashImage &initial_update,
                                       const VirtualSerialFactory &serial_factory)
    : impl_(std::make_unique<Impl>(path, initial_update, serial_factory)) {}

VirtualDeviceState::~VirtualDeviceState() = default;

const McuFlashImage &VirtualDeviceState::image() const {
  impl_->healthy();
  return impl_->image;
}

void VirtualDeviceState::save(const McuFlashImage &image) {
  impl_->healthy();
  const auto bytes = encode_flash_image(image);
  impl_->valid = false;
  impl_->directory.write(impl_->filename, bytes);
  impl_->image = image;
  impl_->valid = true;
}
}
