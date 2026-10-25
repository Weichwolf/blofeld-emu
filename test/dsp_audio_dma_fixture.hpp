// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "audio/serial_pcm.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <ostream>
#include <string>

namespace blofeld::test::audio_dma {
inline constexpr unsigned ring_start = 0x660, ring_size = 64, frames = 66;
inline constexpr unsigned word_count = frames * 2;
inline constexpr unsigned enable_entry = 0x2ebb, stopped = 0x2ebf;
inline constexpr unsigned component_address = 0x2d73;
inline constexpr unsigned control0 = 0xaab604, control3 = 0xa8b600;
inline constexpr unsigned replacement = 0x5abcde;
inline constexpr unsigned first_launch = 7, first_sample = 11, slot_ticks = 256;
inline constexpr unsigned bit_ticks = 8, frame_ticks = 512;
inline constexpr unsigned frame_bits = 64;
inline constexpr unsigned final_tick = first_sample + (((frames * frame_bits) - 1) * bit_ticks);
inline constexpr unsigned captured_tick = first_launch;
inline constexpr unsigned word_mask = 0xffffff, guard = 0x123abc;
inline constexpr unsigned source0 = 0xffffef;
inline constexpr unsigned source3 = 0xffffe3;
inline constexpr unsigned esai_tx = 0xffff80;
inline constexpr unsigned esai_status = 0xffff93;
inline constexpr unsigned esai_underrun = 0x4000;
inline constexpr unsigned dax_tx = 0xffffd2;
inline constexpr unsigned dma_status = 0xfffff4;
inline constexpr unsigned reference_core_hz = 54000000;
inline constexpr unsigned reference_aci_hz = 12288000;

[[nodiscard]] unsigned sample(unsigned index);
[[nodiscard]] unsigned expected_word(unsigned channel, unsigned ordinal, bool changed);
[[nodiscard]] std::int32_t signed_word(unsigned word);

struct Driver {
  SerialPcmReceiver pcm{{.frame_sync_high = true}};
  std::array<unsigned, 2> words{};
  unsigned tick = 0, cycle = 0, decoded = 0;
  bool changed = false;
  std::string bmc;
  std::string trace;
  bool operator==(const Driver &) const = default;
};

struct Experiment {
  explicit Experiment(const Firmware &firmware);
  void step();
  void finish();
  void verify();
  void intact() const;
  void emit(std::ostream &output, const std::string &name) const;
  DspMachine machine;
  Driver driver;

  void configure();
  void grant();
  void pcm_frame(const SerialPcmFrame &frame);
  void dax_frames() const;
  const DspRecord &record_;
  DspDebugState paused_;
};
} // namespace blofeld::test::audio_dma
