// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_usb.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test::gpio_midi {
inline constexpr unsigned ring = 0x2d00;
inline constexpr unsigned ring_words = 128;
inline constexpr unsigned guard = 0x2468ac;
inline constexpr unsigned field_seed = 0x13579b;
inline constexpr unsigned clock_count = 16;
using Bytes = std::vector<std::uint8_t>;

enum class Transport : std::uint8_t { frame, usb, endpoint };

struct Scenario {
  Transport transport;
  bool inverted;
  unsigned ratio;
  unsigned channel;
};

struct Source {
  Bytes bytes;
  Bytes storage;
  Bytes counts;
  std::vector<unsigned> pitches;
};

void require(bool condition, const std::string &message);
[[nodiscard]] Source make_source(Scenario scenario);

class Peer {
public:
  struct Saved {
    McuCheckpoint machine;
    unsigned steps;
    unsigned calls;
    unsigned transitions;
    std::optional<bool> irq;
    unsigned cursor = 0;
    bool data1 = false;
    unsigned usb_irqs = 0;
    unsigned naks = 0;
    unsigned frame_reuses = 0;
  };

  struct Result {
    std::array<unsigned, 6> cpu;
    Bytes ram;
    std::array<unsigned, 7> ports;
    std::vector<unsigned> buffers;
    std::array<unsigned, 6> usb{};
    unsigned steps;
    unsigned calls;
    unsigned transitions;
    bool operator==(const Result &) const = default;
  };

  Peer(const Firmware &firmware, Scenario scenario, const Source &source);
  void synchronize(DspBus &dsp);
  void step();
  [[nodiscard]] bool done() const;
  [[nodiscard]] bool irq_level() const;
  [[nodiscard]] unsigned byte(unsigned index) const;
  [[nodiscard]] Saved save() const;
  void restore(const Saved &saved);
  void verify();
  [[nodiscard]] Result result();

private:
  void prepare();
  void prepare_endpoint();
  void submit();
  void check_usb_frame();
  void step_endpoint();
  [[nodiscard]] bool endpoint_done() const;
  void verify_endpoint();
  [[nodiscard]] std::vector<unsigned> endpoint_buffers();
  void endpoint_result(Result &result);
  Scenario scenario_;
  const Source &source_;
  McuMachine machine_;
  McuFlashImage flash_;
  Bytes before_;
  unsigned finished_ = 0;
  unsigned steps_ = 0;
  unsigned calls_ = 0;
  unsigned transitions_ = 0;
  std::optional<bool> last_irq_;
  unsigned cursor_ = 0;
  bool data1_ = false;
  unsigned usb_irqs_ = 0;
  unsigned naks_ = 0;
  unsigned frame_reuses_ = 0;
  std::vector<unsigned> before_buffers_;
  UsbHostEvidence evidence_;
  UsbConfigurationInfo configuration_;
};

struct Stats {
  unsigned received = 0;
  unsigned stores = 0;
  unsigned irqs = 0;
  unsigned returns = 0;
  unsigned timestamps = 0;
  unsigned consumed = 0;
  unsigned callbacks = 0;
  unsigned pitches = 0;
  unsigned preemptions = 0;
  unsigned steps = 0;
  unsigned producer_steps = 0;
  unsigned phase = 0;
  std::optional<DspState> interrupted;
  std::array<unsigned, ring_words> expected{};
  bool operator==(const Stats &) const = default;
};

class Fixture {
public:
  struct Saved {
    Peer::Saved peer;
    DspCheckpoint dsp;
    Stats stats;
  };

  struct Result {
    Peer::Result peer;
    DspDebugState dsp;
    std::vector<unsigned> memory;
    Stats stats;
    bool operator==(const Result &) const = default;
  };

  Fixture(const Firmware &firmware, Scenario scenario);
  void step();
  void run();
  void verify();
  [[nodiscard]] bool complete();
  [[nodiscard]] bool checkpointable();
  [[nodiscard]] Saved save();
  void restore(const Saved &saved);
  [[nodiscard]] Result result();

  [[nodiscard]] const Stats &stats() const { return stats_; }

private:
  void install();
  void prepare();
  void dsp_step();
  void observe(const DspNativeAccess &access);
  void observe_producer(const DspNativeAccess &access);
  void observe_parser(const DspNativeAccess &access);
  [[nodiscard]] unsigned read(unsigned address);
  Scenario scenario_;
  const Firmware &firmware_;
  Source source_;
  Peer peer_;
  DspMachine dsp_;
  Stats stats_;
  unsigned loop_ = 0;
  unsigned complete_ = 0;
};
}
