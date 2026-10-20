// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
enum class DspDaxTiming : std::uint8_t { unqualified, reference_rising_edges };

struct DspDaxRates {
  std::uint64_t core_hz = 0;
  std::uint64_t aci_hz = 0;
  bool operator==(const DspDaxRates &) const = default;
};

struct DspDaxInputs {
  std::optional<bool> core_clock = std::nullopt;
  std::optional<bool> aci = std::nullopt;
  DspDaxRates rates{};
  DspDaxTiming timing = DspDaxTiming::unqualified;
};

struct DspDaxOutput {
  bool driven = false;
  bool level = false;
  bool operator==(const DspDaxOutput &) const = default;
};

// DSP56371UM ch10. No implicit core/board clock, IRQ delivery or host audio sink.
class DspDax {
public:
  static constexpr std::uint32_t word_mask = 0xffffff;
  void reset();
  [[nodiscard]] static bool handles(unsigned address);
  [[nodiscard]] std::uint32_t read(unsigned address, std::uint32_t mask = word_mask);
  void write(unsigned address, std::uint32_t value);
  void set_active(bool active);
  void observe(DspDaxInputs inputs);
  [[nodiscard]] DspDaxOutput output() const;
  [[nodiscard]] bool request() const;
  [[nodiscard]] bool external_clock() const;

  [[nodiscard]] std::uint64_t half_bits() const { return half_bits_; }

private:
  enum class Register : std::uint8_t { control, non_audio, audio_a, audio_b, status };

  struct Pending {
    unsigned channel = 0;
    std::uint32_t value = 0;
  };

  void reset_paths();
  [[nodiscard]] static Register reg(unsigned address);
  void write_checked(Register target, std::uint32_t value);
  [[nodiscard]] std::uint32_t read_checked(Register target, std::uint32_t mask);
  void write_audio(std::uint32_t value);
  void promote();
  void require_clock(const DspDaxInputs &inputs) const;
  [[nodiscard]] std::optional<bool> clock(const DspDaxInputs &inputs) const;
  [[nodiscard]] unsigned divider() const;
  void observe_checked(DspDaxInputs inputs);
  void emit();
  void begin_frame();
  void begin_subframe();

  std::array<std::optional<std::uint32_t>, 2> buffers_;
  std::array<std::optional<std::uint32_t>, 2> last_words_;
  std::optional<Pending> pending_;
  std::optional<std::uint32_t> non_audio_;
  DspDaxInputs observed_;
  std::optional<bool> previous_;
  unsigned control_ = 0;
  unsigned status_ = 0;
  unsigned next_write_ = 0;
  unsigned clear_writes_ = 0;
  unsigned phase_ = 0;
  unsigned position_ = 0;
  unsigned block_frame_ = 0;
  unsigned preamble_ = 0;
  unsigned payload_ = 0;
  unsigned frame_non_audio_ = 0;
  std::uint64_t half_bits_ = 0;
  bool active_ = false;
  bool level_ = true;
  bool repeating_ = false;
  bool clear_armed_ = false;
  bool rates_frozen_ = false;
};
}
