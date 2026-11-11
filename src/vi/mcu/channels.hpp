// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"

#include <cstdint>
#include <stop_token>

namespace blofeld::vi::mcu {
extern "C" {
[[nodiscard]] bool blofeld_mcu_shi_select(Shi *channel, bool active, Shi::Mode &mode) noexcept;
[[nodiscard]] bool blofeld_mcu_peer_interrupt(const PeerPort *channel,
                                              std::uint64_t &generation) noexcept;
void blofeld_mcu_peer_data(const PeerPort *channel, PeerPort::Sample &sample) noexcept;
[[nodiscard]] bool blofeld_mcu_peer_read(PeerPort *channel,
                                         std::uint8_t mask,
                                         std::uint64_t ticks,
                                         PeerPort::Sample &sample) noexcept;
void blofeld_mcu_peer_hold(PeerPort *channel) noexcept;
[[nodiscard]] bool blofeld_mcu_peer_publish(PeerPort *channel,
                                            const PeerPort::Sample &sample,
                                            std::uint64_t ticks,
                                            const std::stop_token &stop);
[[nodiscard]] bool blofeld_mcu_peer_strobe(PeerPort *channel, bool active) noexcept;
[[nodiscard]] bool blofeld_mcu_time_publish(ExecutionTime *channel, std::uint64_t ticks) noexcept;
[[nodiscard]] bool
blofeld_mcu_time_enter(ExecutionTime *channel, std::uint64_t ticks, const std::stop_token &stop);
void blofeld_mcu_time_leave(ExecutionTime *channel) noexcept;
void blofeld_mcu_time_hold(ExecutionTime *channel) noexcept;
[[nodiscard]] bool blofeld_mcu_midi_limit(const MidiPort *channel, std::uint64_t &ticks) noexcept;
[[nodiscard]] bool blofeld_mcu_midi_peek(const MidiPort *channel, MidiPort::Event &event) noexcept;
void blofeld_mcu_midi_pop(MidiPort *channel) noexcept;
[[nodiscard]] bool blofeld_mcu_midi_write(MidiPort *channel, const MidiPort::Event &event) noexcept;
[[nodiscard]] bool blofeld_mcu_midi_await(MidiPort *channel,
                                          std::uint64_t ticks,
                                          const std::stop_token &stop) noexcept;
void blofeld_mcu_reset_state(const DspReset *channel, DspReset::State &state) noexcept;
[[nodiscard]] bool blofeld_mcu_reset_assert(DspReset *channel) noexcept;
}
}
