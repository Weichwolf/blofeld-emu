// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/flash_image.hpp"

#include <filesystem>
#include <functional>
#include <memory>
#include <string>

namespace blofeld {
[[nodiscard]] std::string generate_virtual_serial();
using VirtualSerialFactory = std::function<std::string()>;

// Owns the private MCU NVM file and writer lease for one emulated device.
// Existing NVM is loaded exactly, including damaged/unknown identity data.
// NAND, DSP ROM and a running board snapshot remain separate resources.
class VirtualDeviceState {
public:
  VirtualDeviceState(const std::filesystem::path &path,
                     const McuFlashImage &initial_update,
                     const VirtualSerialFactory &serial_factory = generate_virtual_serial);
  ~VirtualDeviceState();
  VirtualDeviceState(const VirtualDeviceState &) = delete;
  VirtualDeviceState &operator=(const VirtualDeviceState &) = delete;
  [[nodiscard]] const McuFlashImage &image() const;
  void save(const McuFlashImage &image);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
