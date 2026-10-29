// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
enum class NandReadKind : std::uint8_t { unqualified, array, status70 };

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
  bool operator==(const NandReadSetup &) const = default;
  // True only for a selected falling RE edge meeting all setup requirements.
  // Compose before protocol consumption; failures retain the preceding state.
  [[nodiscard]] bool observe(std::uint64_t ticks, NandReadSetupPins pins);
  // Actual 70h register reads do not require Ready or the array-only tRR.
  [[nodiscard]] bool observe(std::uint64_t ticks, NandReadSetupPins pins, NandReadKind kind);

  // Requires initialized RE/WE/Ready high and expired setup constraints.
  // The caller proves unchanged non-CE controls/Ready throughout [start,end].
  // Resume records a conservative CE bound, never a manufactured pin edge.
  void qualify_ce_window(std::uint64_t start) const;
  void resume_ce_window(std::uint64_t start, std::uint64_t end, bool ce_n);

private:
  struct KnownPins {
    bool ce_n;
    bool re_n;
    bool we_n;
    bool ale;
    bool cle;
    std::optional<bool> ready;
    bool operator==(const KnownPins &) const = default;
  };

  [[noreturn]] void fail(const std::string &reason) const;
  void check_read(std::uint64_t ticks, const KnownPins &pins, NandReadKind kind) const;
  std::uint32_t timebase_hz_;
  std::uint64_t ticks_ = 0;
  // nullopt means the minimum is permanently met until the next actual change.
  std::array<std::optional<std::uint64_t>, 4> stable_since_{};
  KnownPins pins_{
      .ce_n = true, .re_n = true, .we_n = true, .ale = false, .cle = false, .ready = std::nullopt};
  bool initialized_ = false;
};
}
