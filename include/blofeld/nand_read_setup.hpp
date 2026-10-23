// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
struct NandReadSetupPins {
  std::optional<bool> ce_n;
  std::optional<bool> re_n;
  std::optional<bool> we_n;
  std::optional<bool> ale;
  std::optional<bool> cle;
  std::optional<bool> ready;
};

// K9F1208U0C Rev1.1, 3.3V array-read admission: pp.13,20,22,25.
// Does not generate Ready, infer protocol mode, consume bytes or drive outputs.
// Observe every control/Ready change. Unknown Ready is allowed until a selected
// falling RE edge, which requires its complete known-high setup interval.
class NandReadSetup {
public:
  explicit NandReadSetup(std::uint32_t timebase_hz);
  // True only for a selected falling RE edge meeting all setup requirements.
  // Compose before protocol consumption; failures retain the preceding state.
  [[nodiscard]] bool observe(std::uint64_t ticks, NandReadSetupPins pins);

private:
  struct KnownPins {
    bool ce_n;
    bool re_n;
    bool we_n;
    bool ale;
    bool cle;
    std::optional<bool> ready;
  };

  [[noreturn]] void fail(const std::string &reason) const;
  void check_read(std::uint64_t ticks, const KnownPins &pins) const;
  std::uint32_t timebase_hz_;
  std::uint64_t ticks_ = 0;
  std::array<std::uint64_t, 4> stable_since_{};
  KnownPins pins_{
      .ce_n = true, .re_n = true, .we_n = true, .ale = false, .cle = false, .ready = std::nullopt};
  bool initialized_ = false;
};
}
