// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <concepts>
#include <memory>
#include <ostream>
#include <type_traits>

namespace blofeld {
// Select the sink once. Disabled events never invoke their lazy formatter.
class Telemetry {
public:
  explicit Telemetry(std::ostream *output = nullptr) noexcept
      : output_(output), emit_(output != nullptr ? enabled : disabled) {}

  template <typename Formatter>
    requires std::is_object_v<std::remove_reference_t<Formatter>> &&
             std::invocable<const std::remove_reference_t<Formatter> &, std::ostream &>
  void emit(Formatter &&formatter) const {
    emit_(output_, std::addressof(formatter), [](std::ostream &stream, const void *payload) {
      const auto &write = *static_cast<const std::remove_reference_t<Formatter> *>(payload);
      write(stream);
    });
  }

private:
  using Write = void (*)(std::ostream &, const void *);
  using Emit = void (*)(std::ostream *, const void *, Write);

  static void
  disabled(std::ostream * /*output*/, const void * /*payload*/, Write /*write*/) noexcept {}

  static void enabled(std::ostream *output, const void *payload, const Write write) {
    write(*output, payload);
  }

  std::ostream *output_;
  Emit emit_;
};
}
