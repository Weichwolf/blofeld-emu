// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "mcu/effects.hpp"

#include <cstdint>
#include <optional>
#include <stop_token>

namespace cpu12 {
class Core;
}

namespace blofeld::vi {
class MidiPort;
class PeerPort;
class Shi;
}

namespace blofeld::vi::mcu {
struct Devices;

class Output {
public:
  Output(Effects &effects, const Devices &devices, cpu12::Core &core);
  ~Output();
  Output(const Output &) = delete;
  Output &operator=(const Output &) = delete;

  [[nodiscard]] bool put(Effects::Kind kind,
                         std::uint64_t value,
                         std::optional<std::uint64_t> ticks = {}) const noexcept;
  [[nodiscard]] static const Output *current() noexcept;

private:
  Effects &effects_;
  const Devices &devices_;
  cpu12::Core &core_;
};

[[nodiscard]] bool deliver(const Effects::Entry &entry,
                           PeerPort &peer,
                           Shi &shi,
                           MidiPort &midi,
                           const std::stop_token &stop);
}
