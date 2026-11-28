// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "aot/artifact.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/foreground.hpp"
#include "dsp/memory.hpp"

#include <dsp56300/core.hpp>
#include <filesystem>
#include <memory>
#include <sched.h>

namespace llvm::json {
class Object;
}

namespace blofeld::app {
class DspWorkers {
public:
  explicit DspWorkers(vi::dsp::Memory::Storage storage);
  ~DspWorkers();
  DspWorkers(const DspWorkers &) = delete;
  DspWorkers &operator=(const DspWorkers &) = delete;

  // Synchronous preparation for these original program variants.
  void prepare(dsp56300::CodeImage image,
               vi::dsp::BootRom boot,
               const llvm::json::Object &plan,
               const std::filesystem::path &cache,
               host::aot::Preparation preparation);
  [[nodiscard]] vi::dsp::Foreground foreground() noexcept;
  static void playback_ready(void *context) noexcept;
  void exclude_dsp_core(const cpu_set_t &physical) noexcept;

private:
  class Impl;
  vi::dsp::Memory::Storage storage_;
  std::unique_ptr<Impl> impl_;
};
}
