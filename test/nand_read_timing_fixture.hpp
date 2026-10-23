// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/nand.hpp"
#include "blofeld/nand_array.hpp"
#include "blofeld/nand_read_timing.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_norm_fixture.hpp"
#include "nand_gpio_fixture.hpp"
#include "nand_program_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <span>
#include <string>
#include <utility>

namespace blofeld::test::nand_read_reference {
inline constexpr unsigned timebase_hz = 1080000000;

inline void install_stage(DspBus &bus, const Firmware &firmware) {
  constexpr std::size_t frame_offset = 0x7875;
  constexpr unsigned stage_base = 0x6a;
  constexpr std::size_t count = 94;
  const auto frame = std::span(firmware.mcu).subspan(frame_offset, (count + 2) * 3);
  require(dsp_boot_word(std::span<const std::uint8_t, 3>(frame.first<3>())) == count &&
              dsp_boot_word(std::span<const std::uint8_t, 3>(frame.subspan<3, 3>())) == stage_base,
          "Pinned original MCU supplies the complete unchanged first-stage object");
  for (std::size_t index = 0; index < count; ++index) {
    bus.write({.address = stage_base + static_cast<unsigned>(index),
               .value = dsp_boot_word(
                   std::span<const std::uint8_t, 3>(frame.subspan((index + 2) * 3, 3)))});
  }
}

class Link {
public:
  struct State {
    NandReadTiming timing;
    unsigned reads = 0;
  };

  Link(DspMachine &machine, Nand &nand, std::ostream &trace)
      : machine_(machine), nand_(nand), trace_(trace), timing_(timebase_hz) {
    timing_.observe(0, {.ce_n = true, .re_n = true});
    drive(0);
  }

  void step(DspNativeAccessObserver observer = {}) {
    const auto before = machine_.cpu().state();
    drive(machine_.run_clock().elapsed_ticks());
    machine_.cpu().step(std::move(observer));
    const auto ticks = machine_.run_clock().elapsed_ticks();
    try {
      const auto control = machine_.bus().gpio(DspPort::c).observe_pads(nand_controls);
      if (control.known == nand_controls) {
        timing_.observe(ticks, {.ce_n = false, .re_n = (control.levels & nand_re) != 0}, [&] {
          const auto value = nand_.read();
          ++reads_;
          return NandByte{.value = value, .known = Nand::byte_mask};
        });
      }
      drive(ticks);
      trace_ << "pc=" << hex(before.pc, 6) << " ticks=" << ticks << " column=" << nand_.column()
             << " reads=" << reads_ << '\n';
    } catch (const Fault &error) {
      throw Fault("NAND reference read-edge at X:" + hex(port_c_data, 6) + " dsp-pc=" +
                  hex(before.pc, 6) + " omr=" + hex(before.omr, 6) + ": " + error.what());
    }
  }

  Link(DspMachine &machine, Nand &nand, std::ostream &trace, const State &saved)
      : machine_(machine), nand_(nand), trace_(trace), timing_(saved.timing), reads_(saved.reads) {
    drive(machine_.run_clock().elapsed_ticks());
  }

  [[nodiscard]] State state() const { return {.timing = timing_, .reads = reads_}; }

  [[nodiscard]] unsigned reads() const { return reads_; }

private:
  void drive(const std::uint64_t ticks) {
    const auto output = timing_.output(ticks);
    DspGpioDrive drive{.mask = nand_other_f};
    if (!output.driven.has_value()) {
      drive.unknown = nand_data;
    } else if (*output.driven) {
      drive.mask |= output.data.known;
      drive.levels = output.data.value;
      drive.unknown = nand_data & ~static_cast<unsigned>(output.data.known);
    }
    machine_.bus().gpio(DspPort::f).set_drive(drive);
  }

  DspMachine &machine_;
  Nand &nand_;
  std::ostream &trace_;
  NandReadTiming timing_;
  unsigned reads_ = 0;
};

}
