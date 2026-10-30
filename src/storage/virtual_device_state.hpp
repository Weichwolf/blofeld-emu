// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/flash_image.hpp"
#include "storage/private_file.hpp"

#include <filesystem>
#include <functional>
#include <memory>
#include <string>

namespace blofeld {
[[nodiscard]] std::string generate_virtual_serial();
using VirtualSerialFactory = std::function<std::string()>;

// Single-threaded owner of the private MCU NVM file and writer lease for one device.
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
  // A live MCU claims one opaque capability; its private reset/transaction copies
  // share that capability. Ordinary saves cannot bypass an active binding.
  [[nodiscard]] std::shared_ptr<const void> claim_writer();
  void save(const McuFlashImage &image, const std::shared_ptr<const void> &writer);
  // Offline file transaction observations; not emulated electrical phases.
  void set_observer(PrivateFileObserver observer);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
