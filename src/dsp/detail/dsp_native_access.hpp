// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/detail/dsp_reference_execution.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_native_access.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
class DspNativeBus {
public:
  explicit DspNativeBus(DspBus &bus) : bus_(bus), reference_(bus) {}

  void begin(DspNativeAccessObserver observer, DspNativeAccess context);
  void end() noexcept;
  [[nodiscard]] std::uint32_t read(DspAddress target,
                                   DspNativeAccessKind kind = DspNativeAccessKind::read);
  void write(DspStore target);
  [[nodiscard]] std::optional<bool> bit(DspBitAccess target);
  [[nodiscard]] std::uint32_t peek_program(std::uint32_t address);
  void reset_peripherals();

  [[nodiscard]] DspReferenceTape &reference() { return reference_; }

  [[nodiscard]] const DspReferenceTape &reference() const { return reference_; }

private:
  [[nodiscard]] DspNativeAccess before(DspNativeAccess access);
  void after(DspNativeAccess access, std::uint32_t value) const;
  void notify(const DspNativeAccess &access) const;
  DspBus &bus_;
  DspReferenceTape reference_;
  DspNativeAccessObserver observer_;
  DspNativeAccess context_;
};

class DspNativeAccessScope {
public:
  DspNativeAccessScope(DspNativeBus &bus,
                       DspNativeAccessObserver observer,
                       DspNativeAccess context);
  ~DspNativeAccessScope();
  DspNativeAccessScope(const DspNativeAccessScope &) = delete;
  DspNativeAccessScope &operator=(const DspNativeAccessScope &) = delete;

private:
  DspNativeBus &bus_;
};
}
