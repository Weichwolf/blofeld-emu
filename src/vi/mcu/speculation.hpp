// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "display/state.hpp"
#include "execution.hpp"
#include "io/controls.hpp"
#include "io/coupling.hpp"
#include "io/dsp_reset.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "mcu/devices.hpp"
#include "mcu/effects.hpp"
#include "mcu/epoch.hpp"
#include "mcu/input_history.hpp"
#include "mcu/io.hpp"
#include "mcu/memory.hpp"
#include "mcu/program.hpp"

#include <array>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld::vi::mcu {
class Speculation {
public:
  Speculation(Coupling &coupling,
              Memory &memory,
              Devices &devices,
              Io &io,
              cpu12::Core &core,
              const Program &program,
              PeerPort &peer,
              MidiPort &midi,
              DspReset &reset,
              ExecutionReport &report,
              DisplayState *display,
              const Controls *controls);
  void run();
  [[nodiscard]] bool poll() noexcept;
  [[nodiscard]] bool emit(Effects::Kind kind, std::uint64_t value, std::uint64_t time) noexcept;
  [[nodiscard]] bool cancelled() noexcept;
  [[nodiscard]] bool data(PeerPort::Sample &sample) noexcept;
  [[nodiscard]] bool read(std::uint8_t mask, std::uint64_t time, PeerPort::Sample &sample) noexcept;
  [[nodiscard]] bool irq(std::uint64_t &generation) noexcept;
  [[nodiscard]] bool mode(bool selected, bool wait, Shi::Mode &mode) noexcept;
  [[nodiscard]] bool await_midi(std::uint64_t time) noexcept;
  [[nodiscard]] bool midi(MidiPort::Event &event) const noexcept;
  void pop_midi() noexcept;
  [[nodiscard]] DspReset::State reset_state() const noexcept;
  [[nodiscard]] bool assert_reset() noexcept;
  [[nodiscard]] bool release_reset() noexcept;

private:
  enum class Action : std::uint8_t { run, confirm };

  struct State {
    DspReset::State reset;
    std::uint64_t input_cursor = 0, accepted_announcement = 0, irq_window = 0;
    std::uint64_t instructions = 0;
    std::uint32_t data = 0;
    bool strobe = false, reply = false, ready = false;
  };

  struct Checkpoint {
    cpu12::Snapshot cpu;
    Memory::Snapshot memory;
    Devices::Snapshot devices;
    Io::Snapshot io;
    State state;
    std::uint64_t binding = 0;
  };

  [[nodiscard]] std::uint64_t ticks() const noexcept;
  void publish() noexcept;
  [[nodiscard]] bool observe(const Coupling::Input &update) noexcept;
  [[nodiscard]] bool settle(std::uint64_t time) noexcept;
  [[nodiscard]] std::optional<std::uint64_t>
  input(Epoch::Signal signal, std::uint64_t time, std::uint64_t mask = ~std::uint64_t{0}) noexcept;
  [[nodiscard]] std::optional<std::uint64_t> pending_irq() noexcept;
  void bind();
  void advance();
  void ready();
  void save();
  void restore();
  void confirm();
  [[nodiscard]] bool interrupt();
  [[nodiscard]] std::uint64_t deadline(std::uint64_t target) const;
  void step(std::uint64_t target);
#if defined(BLOFELD_CENSUS)
  void record_wait(const cpu12::RunResult &result);
#endif
  void recover();
  [[nodiscard]] bool check() const;

  Coupling &coupling_;
  Memory &memory_;
  Devices &devices_;
  Io &io_;
  cpu12::Core &core_;
  const Program &program_;
  PeerPort &peer_;
  MidiPort &midi_;
  ExecutionReport &report_;
  DisplayState *display_;
  const Controls *controls_;
  std::array<Epoch::Read, 4096> reads_{}, changes_{};
  Epoch epoch_{reads_, 64};
  InputHistory history_{changes_, {}, changes_.size() / 2};
  Effects effects_;
  State state_;
  std::optional<Checkpoint> checkpoint_;
  std::vector<cpu12::Ram> grants_;
  std::vector<std::uint64_t> mounted_;
  std::optional<Coupling::Input> pending_;
  std::uint64_t seen_ = 0, binding_ = 0;
  const char *refused_ = nullptr;
  Action action_ = Action::run;
  bool active_ = false, replaying_ = false, pedal_ = false;
};
}
