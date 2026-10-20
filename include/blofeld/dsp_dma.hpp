// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>

namespace blofeld {
enum class DspDmaSpace : std::uint8_t { x, y, p };
enum class DspDmaTiming : std::uint8_t { unqualified, reference_sampled_cycles };
enum class DspDmaGrant : std::uint8_t { unknown, available, wait };
enum class DspDmaPhaseKind : std::uint8_t { source_read, destination_write };

struct DspDmaAddress {
  DspDmaSpace space = DspDmaSpace::x;
  unsigned address = 0;
  bool operator==(const DspDmaAddress &) const = default;
};

struct DspDmaInputs {
  std::uint64_t cycle = 0;
  std::uint32_t known_requests = 0;
  std::uint32_t requests = 0;
  DspDmaTiming timing = DspDmaTiming::unqualified;
  DspDmaGrant grant = DspDmaGrant::unknown;
};

struct DspDmaPhase {
  DspDmaPhaseKind kind = DspDmaPhaseKind::source_read;
  unsigned channel = 0;
  DspDmaAddress source{};
  DspDmaAddress destination{};
  std::optional<std::uint32_t> value = std::nullopt;
};

// Called after real requests are sampled and the non-preemptible phase selected.
// It must not mutate the controller or perform I/O. A copy is invoked synchronously;
// no callback is retained or checkpointed. Global wait still samples/adjudicates
// the phase, but prevents its actual I/O even when admission returns available.
using DspDmaAdmission = std::function<DspDmaGrant(const DspDmaPhase &)>;

// Callbacks are invoked synchronously and never retained; no real-time guarantee.
struct DspDmaIo {
  std::function<std::uint32_t(DspDmaAddress)> read;
  std::function<void(DspDmaAddress, std::uint32_t)> write;
  DspDmaAdmission admit = nullptr;
};

struct DspDmaWord {
  unsigned channel = 0;
  DspDmaAddress source;
  DspDmaAddress destination;
  std::uint32_t value = 0;
  bool operator==(const DspDmaWord &) const = default;
};

// DSP56300FM ch10: explicit reference-cycle grants, not implicit core bus timing.
class DspDma {
public:
  static constexpr std::uint32_t full_word = 0xffffff;
  void reset();
  [[nodiscard]] static bool handles(unsigned address);
  [[nodiscard]] std::uint32_t read(unsigned address, std::uint32_t mask = full_word) const;
  void write(unsigned address, std::uint32_t value);
  [[nodiscard]] std::uint32_t requested_sources() const;
  [[nodiscard]] bool active() const;
  [[nodiscard]] std::optional<DspDmaWord> advance(DspDmaInputs inputs, const DspDmaIo &io);

private:
  struct Channel {
    unsigned source = 0;
    unsigned destination = 0;
    unsigned counter = 0;
    unsigned reload = 0;
    unsigned control = 0;
    unsigned known = 0;
    unsigned settling = 0;
    bool pending = false;
    bool done = true;
    bool done_known = true;
    bool first_sample = false;
  };

  struct Layout {
    unsigned low;
    unsigned middle;
  };

  [[nodiscard]] static unsigned mode(const Channel &channel);
  [[nodiscard]] static unsigned request(const Channel &channel);
  [[nodiscard]] static unsigned addressing(const Channel &channel, bool destination);
  [[nodiscard]] static Layout layout(const Channel &channel);
  [[nodiscard]] static DspDmaAddress endpoint(const Channel &channel, bool destination);
  [[nodiscard]] static bool complete(const Channel &channel);
  [[nodiscard]] static unsigned offset_mask(const Channel &channel);
  [[nodiscard]] unsigned next_address(const Channel &channel, bool destination) const;
  void update_addresses(Channel &channel) const;
  void validate_enable(const Channel &channel) const;
  [[nodiscard]] bool busy(unsigned channel) const;
  [[nodiscard]] unsigned read_status(unsigned mask) const;
  [[nodiscard]] std::uint32_t read_checked(unsigned address, std::uint32_t mask) const;
  void write_control(unsigned index, unsigned value);
  void write_checked(unsigned address, unsigned value);
  void sample(DspDmaInputs inputs);
  void sample_channel(unsigned index, DspDmaInputs inputs);
  [[nodiscard]] std::optional<unsigned> select() const;
  [[nodiscard]] static bool
  admitted(DspDmaGrant grant, const DspDmaPhase &phase, const DspDmaAdmission &admission);
  [[nodiscard]] std::optional<DspDmaWord> advance_checked(DspDmaInputs inputs, const DspDmaIo &io);

  std::array<Channel, 6> channels_;
  std::array<unsigned, 4> offsets_{};
  unsigned known_offsets_ = 0;
  std::array<unsigned, 4> next_channel_{};
  std::optional<DspDmaWord> active_;
  std::optional<std::uint64_t> cycle_;
  std::uint32_t known_requests_ = 0;
  std::uint32_t requests_ = 0;
};
}
