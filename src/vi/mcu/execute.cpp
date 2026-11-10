// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/execute.hpp"

#include "display/port.hpp"
#include "display/state.hpp"
#include "execution.hpp"
#include "io/controls.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/mcu_flash.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/sci.hpp"
#include "io/shi.hpp"
#include "io/usb.hpp"
#include "mcu/clock.hpp"
#include "mcu/memory.hpp"
#include "mcu/ports.hpp"
#include "mcu/timer.hpp"
#include "preparation_clock.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <vector>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint16_t reset_vector = 0xfffe;
constexpr std::uint16_t xirq_vector = 0xfff4;
constexpr std::uint16_t peer_q_data = 0x260;
constexpr std::uint16_t peer_q_input = 0x261;
constexpr std::uint16_t peer_q_direction = 0x262;
constexpr std::uint16_t peer_j_data = 0x250;
constexpr std::uint16_t peer_j_direction = 0x252;
constexpr std::uint16_t mapping_first = 0x10;
constexpr std::uint16_t mapping_last = 0x13;
constexpr std::uint8_t all_bits = 0xff;
// An enabled but masked IRQ still needs native CCR/RTI effect boundaries.
// This unreachable horizon selects that Core path without an execution quantum.
constexpr auto no_deadline = std::numeric_limits<Timer::Cycle>::max() / 2;

struct PeerAdmission {
  PeerPort *peer;

  ~PeerAdmission() {
    if (peer != nullptr) { static_cast<void>(peer->mcu_interrupt_ready(false)); }
  }
};

struct Devices {
  explicit Devices(const std::stop_token &cancellation) : stop(cancellation) {}

  const std::stop_token &stop;
  PeerPort *peer = nullptr;
  ExecutionTime *time = nullptr;
  MidiPort *midi = nullptr;
  Timer timer;
  Clock clock;
  Sci sci;
  Usb usb;
  Timer::Cycle idle = 0;
  const char *midi_error = nullptr;

  [[nodiscard]] std::optional<std::uint64_t> input_tick() const noexcept {
    if (midi == nullptr) { return {}; }
    auto next = midi->input_limit();
    if (const auto input = midi->peek_input(); input && (!next || input->ticks < *next)) {
      next = input->ticks;
    }
    return next;
  }

  bool wait_input(const std::uint64_t ticks) noexcept {
    if (time != nullptr && !time->publish_mcu(clock.ticks())) { return false; }
    return midi->await_input(ticks, stop);
  }

  bool input_ready() noexcept {
    if (const auto limit = midi->input_limit(); limit && clock.ticks() >= *limit) {
      return wait_input(*limit);
    }
    return midi->await_input(clock.ticks(), stop);
  }

  bool point(const Timer::Cycle now) noexcept {
    const auto before = clock.cycles();
    const auto ticks = clock.ticks();
    const auto period = clock.bus_period();
    if (!timer.advance(now) || !clock.advance(now) || !sci.advance(now)) { return false; }
    if (midi == nullptr) { return true; }
    while (const auto output = sci.take_output()) {
      if (output->bits != 8 || output->word > all_bits || output->cycle < before ||
          output->cycle > now) {
        midi_error = "SCI MIDI output has an unsupported format or completion time";
        return false;
      }
      if (!midi->write_output({.ticks = ticks + ((output->cycle - before) * period),
                               .byte = static_cast<std::uint8_t>(output->word)})) {
        return false;
      }
    }
    while (const auto input = midi->peek_input()) {
      if (input->ticks > clock.ticks()) { break; }
      if (!sci.receive({.word = input->byte})) {
        midi_error = "SCI could not accept the scheduled MIDI character";
        return false;
      }
      midi->pop_input();
    }
    midi->publish_time(clock.ticks());
    return midi->error() == nullptr;
  }

  bool input_point(const Timer::Cycle now) noexcept {
    while (const auto limit = midi->input_limit()) {
      if (*limit <= clock.ticks() || (*limit - clock.ticks()) % clock.bus_period() == 0) { break; }
      const auto boundary = clock.at_ticks(*limit);
      if (!boundary) { return false; }
      if (now < *boundary) { break; }
      // An unaligned exclusive frontier cannot authorize the following bus edge.
      if (!point(*boundary - 1) || !wait_input(*limit)) { return false; }
    }
    return point(now);
  }

  bool advance(const cpu12::Core &core) noexcept {
    const auto cycles = core.progress().estimated_cycles;
    if (idle > std::numeric_limits<Timer::Cycle>::max() - cycles) { return false; }
    const auto now = cycles + idle;
    if (midi == nullptr) { return point(now); }
    for (;;) {
      if (!input_ready()) { return false; }
      auto next = now;
      if (const auto event = clock.deadline(); event && *event < next) { next = *event; }
      if (const auto tick = input_tick()) {
        const auto event = clock.at_ticks(*tick);
        if (!event) { return false; }
        next = std::min(next, *event);
      }
      if (!input_point(next)) { return false; }
      if (next == now) { return true; }
    }
  }

  [[nodiscard]] const char *error() const noexcept {
    if (midi_error != nullptr) { return midi_error; }
    if (midi != nullptr && midi->error() != nullptr) { return midi->error(); }
    return sci.error();
  }

  [[nodiscard]] std::optional<std::uint16_t> pending() const noexcept {
    return std::max({timer.pending(), clock.pending(), sci.pending(), usb.pending()});
  }

  [[nodiscard]] std::optional<Timer::Cycle> deadline() const noexcept {
    auto next = timer.deadline();
    const auto offer = [&next](const std::optional<Timer::Cycle> candidate) {
      if (candidate && (!next || *candidate < *next)) { next = candidate; }
    };
    offer(clock.deadline());
    offer(sci.deadline());
    if (const auto tick = input_tick()) { offer(clock.at_ticks(*tick)); }
    return next;
  }

  [[nodiscard]] bool interrupts_enabled() const noexcept {
    return peer != nullptr || timer.interrupts_enabled() || clock.interrupts_enabled() ||
           sci.interrupts_enabled() || usb.interrupts_enabled();
  }

  void activity(const cpu12::ExecutionState state) {
    switch (state) {
      case cpu12::ExecutionState::running:
        clock.activity(Clock::Activity::running);
        timer.activity(Timer::Activity::running);
        break;
      case cpu12::ExecutionState::waiting:
        if (time != nullptr) { time->publish_mcu_wait(clock.ticks()); }
        clock.activity(Clock::Activity::waiting);
        timer.activity(clock.bus_running() ? Timer::Activity::waiting : Timer::Activity::stopped);
        break;
      case cpu12::ExecutionState::stopped:
        clock.activity(Clock::Activity::stopped);
        timer.activity(Timer::Activity::stopped);
        break;
      default: throw std::logic_error("Unknown MCU execution state");
    }
    sci.activity(
        {.waiting = state == cpu12::ExecutionState::waiting, .clocked = clock.bus_running()});
  }
};

class Io {
public:
  Io(Memory &memory,
     const FlashImage &image,
     Devices &devices,
     Shi &shi,
     DspReset &reset,
     const std::stop_token &stop,
     const Controls *controls,
     DisplayState *display,
     PeerPort *peer)
      : memory_(memory),
        flash_(image),
        ports_(shi),
        devices_(devices),
        reset_(reset),
        stop_(stop),
        controls_(controls),
        display_state_(display),
        peer_(peer) {
    // B3/B5..B7 are passive controls, with no input producer in the firmware
    // contract. Use an inactive sample while they are inputs, not a device reply.
    constexpr std::uint8_t passive_controls = 0xe8;
    // DSP56371UM table 2-7: SS, SCK and MOSI have internal pull-ups. The
    // application releases B1/B2/B4 after boot; this is bias, not HREQ/data.
    constexpr std::uint8_t spi_pullups = 0x16;
    constexpr std::uint8_t attached = passive_controls | spi_pullups;
    ports_.input(Port::b, {.mask = attached, .known = attached, .value = spi_pullups});
    // DSP56371 RESET has an internal pull-up. U0 drives it low or releases it.
    ports_.input(Port::u, {.mask = 1, .known = 1, .value = 1});
  }

  void bind(cpu12::Core &core) noexcept { core_ = &core; }

  cpu12::Io callbacks() noexcept;

  void capture_display() const {
    if (display_state_ != nullptr) { display_state_->capture(display_.controller()); }
  }

  bool complete_events() {
    panel_sampled_ = false;
    capture_display();
    if (!devices_.advance(*core_) ||
        (devices_.time != nullptr && !devices_.time->publish_mcu(devices_.clock.ticks()))) {
      return false;
    }
    if (release_reset_) {
      if (peer_ != nullptr && !peer_->release_mcu_data(devices_.clock.ticks(), stop_)) {
        return false;
      }
      if (devices_.time != nullptr) { devices_.time->release_dsp(devices_.clock.ticks()); }
      if (!reset_.release(stop_)) { return false; }
      release_reset_ = false;
    }
    if (!ports_.boot_pending()) { return true; }
    return shared_access([&] { return ports_.complete_boot_transfer(stop_); });
  }

  [[nodiscard]] const char *error() const noexcept {
    if (peer_ != nullptr && peer_->error() != nullptr) { return peer_->error(); }
    return display_.error() != nullptr ? display_.error() : devices_.error();
  }

private:
  template <typename Action> bool shared_access(const Action &action) noexcept {
    panel_sampled_ = false;
    auto *time = devices_.time;
    if (time == nullptr) { return action(); }
    if (!devices_.advance(*core_) || !time->enter_mcu(devices_.clock.ticks(), stop_)) {
      return false;
    }
    const bool accepted = action();
    time->leave_mcu();
    return accepted;
  }

  [[nodiscard]] bool shared_write(const Ports::Write access) const noexcept {
    constexpr std::uint16_t data_u = 0x278;
    constexpr std::uint16_t direction_u = 0x27a;
    if (access.offset != peer_j_data && access.offset != peer_j_direction) {
      return access.offset == data_u || access.offset == direction_u;
    }
    const auto output = ports_.output(Port::j);
    if (!output || (output->known & 1) == 0) { return true; }
    const bool before = (output->direction & 1) != 0 && (output->value & 1) == 0;
    const auto direction = access.offset == peer_j_direction ? access.value : output->direction;
    const auto value = access.offset == peer_j_data ? access.value : output->value;
    const bool after = (direction & 1) != 0 && (value & 1) == 0;
    return before != after;
  }

  bool
  read_port(const std::uint16_t offset, std::uint8_t &value, const std::uint8_t mask) noexcept {
    const auto action = [&] {
      return sample_inputs(offset, mask) && ports_.read(offset, value, mask);
    };
    bool input = offset == peer_q_input;
    if (offset == peer_q_data) {
      const auto output = ports_.output(Port::q);
      input = !output || (output->direction & mask) != mask;
    }
    if (!input) { return action(); }
    if (panel_sampled_ && (peer_->dsp_data().mask & mask) == 0) { return action(); }
    const bool accepted = shared_access(action);
    panel_sampled_ = accepted && peer_ != nullptr && local_input_bank() &&
                     peer_->dsp_data().mask == 0 && !peer_->mcu_interrupt();
    return accepted;
  }

  [[nodiscard]] bool local_input_bank() const noexcept {
    constexpr std::uint8_t banks = 0xc0;
    constexpr std::uint8_t bank_a = 0x40;
    constexpr std::uint8_t bank_b = 0x80;
    const auto select = ports_.output(Port::s);
    const auto request = ports_.output(Port::j);
    const auto data = ports_.output(Port::q);
    if (!select || !request || !data || data->direction != 0 ||
        (select->direction & select->known & banks) != banks ||
        (request->direction & request->known & request->value & 1) == 0) {
      return false;
    }
    const auto selected = select->value & banks;
    return selected == bank_a || selected == bank_b;
  }

  template <typename Action> bool access_devices(const Action &action) noexcept {
    panel_sampled_ = false;
    const auto pending = devices_.pending();
    const auto deadline = devices_.deadline();
    const auto enabled = devices_.interrupts_enabled();
    if (!devices_.advance(*core_) || !action()) { return false; }
    if (devices_.time != nullptr && !devices_.time->publish_mcu(devices_.clock.ticks())) {
      return false;
    }
    if (pending != devices_.pending() || deadline != devices_.deadline() ||
        enabled != devices_.interrupts_enabled()) {
      core_->request_yield();
    }
    return true;
  }

  bool interrupted() noexcept {
    if (peer_ != nullptr && peer_->mcu_interrupt()) {
      panel_sampled_ = false;
      core_->request_yield();
    }
    if (!stop_.stop_requested()) { return false; }
    core_->request_yield();
    return true;
  }

  bool read(const std::uint16_t address,
            std::uint8_t &value,
            const std::uint8_t mask = all_bits) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(address);
    if (at.space == Space::registers && McuFlash::contains(at.offset)) {
      return flash_.read(at.offset, value);
    }
    if (at.space == Space::registers && Usb::contains(at.offset)) {
      std::uint16_t word = 0;
      const auto accepted =
          access_devices([&] { return devices_.usb.read({.offset = at.offset}, word); });
      value = static_cast<std::uint8_t>(word);
      return accepted;
    }
    if (at.space == Space::registers && Sci::contains(at.offset)) {
      std::uint16_t word = 0;
      const auto accepted =
          access_devices([&] { return devices_.sci.read({.offset = at.offset}, word); });
      value = static_cast<std::uint8_t>(word);
      return accepted;
    }
    if (at.space == Space::registers && Clock::contains(at.offset)) {
      return access_devices([&] { return devices_.clock.read(at.offset, value); });
    }
    if (at.space == Space::registers && Timer::contains(at.offset)) {
      std::uint16_t word = 0;
      const auto accepted =
          access_devices([&] { return devices_.timer.read({.offset = at.offset}, word); });
      value = static_cast<std::uint8_t>(word);
      return accepted;
    }
    if (at.space == Space::registers && Ports::contains(at.offset)) {
      if (at.offset == 0 && display_.reading()) { return display_.read(value, mask); }
      return read_port(at.offset, value, mask);
    }
    if (at.space == Space::registers && at.offset >= mapping_first && at.offset <= mapping_last) {
      value = memory_.read_mapping(at.offset);
      return true;
    }
    return memory_.read(address, value);
  }

  bool write_usb(const Usb::Write access) noexcept {
    return access_devices([&] {
      if (!devices_.usb.write(access, devices_.clock.usb_module_control())) { return false; }
      if (devices_.usb.mapping_pending()) { core_->request_yield(); }
      return true;
    });
  }

  bool sample_inputs(const std::uint16_t offset, const std::uint8_t mask) {
    if (peer_ != nullptr && offset == 8) { package_input(); }
    if (peer_ != nullptr && (offset == peer_q_data || offset == peer_q_input)) {
      const auto sample =
          offset == peer_q_data ? peer_->read_dsp_data(mask) : std::optional{peer_->dsp_data()};
      if (!sample) { return false; }
      // DSP56371UM Table 2-8 documents internal pull-downs on PC0..7,
      // including reset-disconnected GPIO. They are not active reply data.
      ports_.input(Port::q,
                   {.mask = sample->mask,
                    .known = sample->known,
                    .value = sample->value,
                    .fallback_low = all_bits});
    }
    constexpr std::uint16_t port_m = 0x248;
    constexpr std::uint16_t input_m = 0x249;
    if (offset == port_m || offset == input_m) {
      constexpr std::uint8_t pedal_bit = 8;
      ports_.input(
          Port::m,
          {.mask = pedal_bit,
           .known = pedal_bit,
           .value = controls_ != nullptr && controls_->pedal() ? pedal_bit : std::uint8_t{0}});
    }
    return true;
  }

  void package_input() {
    constexpr std::uint8_t probe_output = 4;
    constexpr std::uint8_t package_sense = 8;
    const auto output = ports_.output(Port::e);
    const bool driven = output && (output->direction & probe_output) != 0;
    // The runtime Q interface requires the resident probe's 100-pin route.
    // Sample the driven E2 level at E3; this is a profile input, not a peer reply.
    ports_.input(
        Port::e,
        {.mask = driven ? package_sense : std::uint8_t{0},
         .known = driven && (output->known & probe_output) != 0 ? package_sense : std::uint8_t{0},
         .value = driven && (output->value & probe_output) != 0 ? package_sense : std::uint8_t{0}});
  }

  bool publish_peer(const std::uint16_t offset) noexcept {
    if (peer_ == nullptr || (offset != peer_q_data && offset != peer_q_direction &&
                             offset != peer_j_data && offset != peer_j_direction)) {
      return true;
    }
    const auto data = ports_.output(Port::q);
    const auto request = ports_.output(Port::j);
    if (!data || !request || (request->direction & ~request->known & 1) != 0) { return false; }
    if (!peer_->mcu_data({.mask = data->direction, .known = data->known, .value = data->value},
                         devices_.clock.ticks(),
                         stop_)) {
      return false;
    }
    const bool active = (request->direction & 1) != 0 && (request->value & 1) == 0;
    if (!peer_->mcu_strobe(active)) { return false; }
    if (active != peer_strobe_) {
      peer_strobe_ = active;
      core_->request_yield();
    }
    return true;
  }

  bool write_port(const Ports::Write access) noexcept {
    panel_sampled_ = false;
    constexpr std::uint16_t data_u = 0x278;
    constexpr std::uint16_t direction_u = 0x27a;
    if (!ports_.write(access) || !display_.write(ports_, access) || !publish_peer(access.offset)) {
      return false;
    }
    if (ports_.boot_pending()) { core_->request_yield(); }
    if (access.offset != data_u && access.offset != direction_u) { return true; }
    const auto output = ports_.output(Port::u);
    if (!output || ((output->direction & 1) != 0 && (output->known & 1) == 0)) { return false; }
    const bool held = (output->direction & 1) != 0 && (output->value & 1) == 0;
    const bool previous = reset_.state().held;
    if (held && !previous) {
      ports_.reset_boot_transfer();
      if (peer_ != nullptr) { peer_->hold_mcu_data(); }
      if (!reset_.assert_reset()) { return false; }
      if (devices_.time != nullptr) { devices_.time->hold_dsp(); }
    } else if (!held && previous) {
      release_reset_ = true;
      core_->request_yield();
    }
    ports_.input(Port::u,
                 {.mask = 1, .known = 1, .value = held ? std::uint8_t{0} : std::uint8_t{1}});
    return true;
  }

  bool write(const cpu12::ByteWrite access) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(access.address);
    if (at.space == Space::flash) {
      return flash_.write_array({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && McuFlash::contains(at.offset)) {
      return flash_.write({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && Usb::contains(at.offset)) {
      return write_usb({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && Sci::contains(at.offset)) {
      return access_devices(
          [&] { return devices_.sci.write({.offset = at.offset, .value = access.value}); });
    }
    if (at.space == Space::registers && Clock::contains(at.offset)) {
      return access_devices(
          [&] { return devices_.clock.write({.offset = at.offset, .value = access.value}); });
    }
    if (at.space == Space::registers && Timer::contains(at.offset)) {
      return access_devices(
          [&] { return devices_.timer.write({.offset = at.offset, .value = access.value}); });
    }
    if (at.space == Space::registers && Ports::contains(at.offset)) {
      const auto action = [&] { return write_port({.offset = at.offset, .value = access.value}); };
      if (at.offset == peer_q_data || at.offset == peer_q_direction) {
        return access_devices(action);
      }
      return shared_write({.offset = at.offset, .value = access.value}) ? shared_access(action)
                                                                        : action();
    }
    if (at.space == Space::registers && at.offset >= mapping_first && at.offset <= mapping_last) {
      if (memory_.write_mapping({.address = at.offset, .value = access.value})) {
        core_->request_yield();
      }
      return true;
    }
    const bool accepted = memory_.write(access);
    if (memory_.program_changed()) { core_->request_yield(); }
    return accepted;
  }

  bool read_word(const std::uint16_t address, std::uint16_t &value) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(address);
    if (at.space == Space::registers && Usb::contains(at.offset) &&
        Usb::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices(
          [&] { return devices_.usb.read({.offset = at.offset, .bytes = 2}, value); });
    }
    if (at.space == Space::registers && Sci::contains(at.offset) &&
        Sci::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices(
          [&] { return devices_.sci.read({.offset = at.offset, .bytes = 2}, value); });
    }
    if (at.space == Space::registers && Timer::contains(at.offset) &&
        Timer::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices(
          [&] { return devices_.timer.read({.offset = at.offset, .bytes = 2}, value); });
    }
    std::uint8_t high = 0;
    std::uint8_t low = 0;
    if (!read(address, high) || !read(static_cast<std::uint16_t>(address + 1), low)) {
      return false;
    }
    value = static_cast<std::uint16_t>((unsigned{high} << 8U) | low);
    return true;
  }

  bool write_word(const cpu12::WordWrite access) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(access.address);
    const auto next = memory_.locate(static_cast<std::uint16_t>(access.address + 1));
    if (at.space == Space::flash && next.space == Space::flash && next.offset == at.offset + 1) {
      return flash_.write_array({.offset = at.offset, .value = access.value, .bytes = 2});
    }
    if (at.space == Space::registers && Usb::contains(at.offset) &&
        Usb::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return write_usb({.offset = at.offset, .value = access.value, .bytes = 2});
    }
    if (at.space == Space::registers && Sci::contains(at.offset) &&
        Sci::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices([&] {
        return devices_.sci.write({.offset = at.offset, .value = access.value, .bytes = 2});
      });
    }
    if (at.space == Space::registers && Clock::contains(at.offset) &&
        Clock::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices([&] {
        return devices_.clock.write({.offset = at.offset, .value = access.value, .bytes = 2});
      });
    }
    if (at.space == Space::registers && Timer::contains(at.offset) &&
        Timer::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices([&] {
        return devices_.timer.write({.offset = at.offset, .value = access.value, .bytes = 2});
      });
    }
    return write({.address = access.address,
                  .value = static_cast<std::uint8_t>(access.value >> 8U)}) &&
           write({.address = static_cast<std::uint16_t>(access.address + 1),
                  .value = static_cast<std::uint8_t>(access.value & all_bits)});
  }

  Memory &memory_;
  McuFlash flash_;
  Ports ports_;
  DisplayPort display_;
  Devices &devices_;
  DspReset &reset_;
  bool release_reset_ = false;
  const std::stop_token &stop_;
  const Controls *controls_;
  DisplayState *display_state_;
  PeerPort *peer_;
  bool peer_strobe_ = false;
  bool panel_sampled_ = false;
  cpu12::Core *core_ = nullptr;
};

cpu12::Io Io::callbacks() noexcept {
  return {.owner = this,
          .read =
              [](void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
                return static_cast<Io *>(owner)->read(address, value);
              },
          .write =
              [](void *owner, const cpu12::ByteWrite access) noexcept {
                return static_cast<Io *>(owner)->write(access);
              },
          .read_word =
              [](void *owner, const std::uint16_t address, std::uint16_t &value) noexcept {
                return static_cast<Io *>(owner)->read_word(address, value);
              },
          .write_word =
              [](void *owner, const cpu12::WordWrite access) noexcept {
                return static_cast<Io *>(owner)->write_word(access);
              },
          .fetch =
              [](void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
                return static_cast<Io *>(owner)->read(address, value);
              },
          .read_masked =
              [](void *owner, const cpu12::MaskedByteRead access, std::uint8_t &value) noexcept {
                return static_cast<Io *>(owner)->read(access.address, value, access.mask);
              }};
}

Access operation(const cpu12::Fault fault) {
  switch (fault) {
    case cpu12::Fault::memory_read: return Access::read;
    case cpu12::Fault::memory_write: return Access::write;
    case cpu12::Fault::instruction_fetch: return Access::fetch;
    case cpu12::Fault::none:
    case cpu12::Fault::invalid_input:
    case cpu12::Fault::unsupported_instruction:
    case cpu12::Fault::unsupported_host:
    case cpu12::Fault::reentrant:
    case cpu12::Fault::unsupported_state:
    default: return Access::execute;
  }
}

bool interrupt(cpu12::Core &core, const Devices &devices, ExecutionReport &report) {
  if (devices.peer != nullptr &&
      !devices.peer->mcu_interrupt_ready(core.interrupt_ready(cpu12::InterruptKind::xirq))) {
    throw std::overflow_error(devices.peer->error());
  }
  const auto external = devices.peer != nullptr ? devices.peer->mcu_interrupt() : std::nullopt;
  const bool xirq = external && core.interrupt_ready(cpu12::InterruptKind::xirq);
  const auto vector = xirq ? std::optional<std::uint16_t>{xirq_vector} : devices.pending();
  const auto kind = xirq ? cpu12::InterruptKind::xirq : cpu12::InterruptKind::maskable;
  if (!vector || !core.interrupt_ready(kind)) { return false; }
  const auto entered = core.interrupt({.vector = *vector, .kind = kind});
  if (entered.fault != cpu12::Fault::none) {
    report.failure = Failure{.operation = operation(entered.fault),
                             .pc = entered.pc,
                             .address = entered.address,
                             .value = entered.value,
                             .code = static_cast<std::uint32_t>(entered.fault),
                             .reason = "MCU device interrupt entry failed"};
    return false;
  }
  if (!entered.accepted) { throw std::logic_error("Eligible MCU device interrupt was refused"); }
  if (xirq &&
      (!devices.peer->mcu_interrupt_ready(false) || !devices.peer->acknowledge_mcu(*external))) {
    throw std::logic_error("Accepted MCU peer interrupt has an invalid generation");
  }
  return true;
}

void advance(const cpu12::Core &core, Devices &devices) {
  if (!devices.advance(core) ||
      (devices.time != nullptr && !devices.time->publish_mcu(devices.clock.ticks()))) {
    if (devices.stop.stop_requested()) { return; }
    if (devices.error() != nullptr) { throw std::runtime_error(devices.error()); }
    throw std::overflow_error("MCU device time is not representable or monotonic");
  }
}

std::vector<cpu12::Ram> regions(Memory &memory, Usb &usb) {
  auto result = memory.regions();
  constexpr std::uint16_t register_mapping = 0x11;
  const auto base =
      static_cast<std::uint16_t>(unsigned{memory.read_mapping(register_mapping)} << 8U);
  for (const auto window : usb.regions(base)) {
    if (!window.bytes.empty()) { result.push_back(window); }
  }
  return result;
}

enum class Dispatch : std::uint8_t { run, again, done };

void prepare_code(cpu12::Core &core, ExecutionReport &report, PreparationClock *preparation) {
  const std::array entries{core.registers().pc};
  const PreparationClock::Scope phase(preparation);
  const auto begun = std::chrono::steady_clock::now();
  const auto prepared = core.prepare(entries);
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begun)
          .count();
  report.preparation_nanoseconds += static_cast<std::uint64_t>(elapsed);
  report.prepared_native_bytes += prepared.native_bytes;
  if (prepared.failure.fault != cpu12::Fault::none) {
    report.preparation_diagnostic = Failure{
        .operation = operation(prepared.failure.fault),
        .pc = prepared.failure.pc,
        .address = prepared.failure.address,
        .value = prepared.failure.value,
        .code = static_cast<std::uint32_t>(prepared.failure.fault),
        .reason = "MCU preparation incomplete; remaining entries use retained native execution"};
  }
}

void bind_memory(cpu12::Core &core,
                 Memory &memory,
                 Usb &usb,
                 std::vector<cpu12::Ram> &grants,
                 ExecutionReport &report,
                 PreparationClock *preparation) {
  const bool mapping_changed = memory.commit_mapping();
  const bool buffers_changed = usb.commit_mapping();
  const bool program_changed = memory.bind_program(core.registers().pc);
  if (!mapping_changed && !buffers_changed && !program_changed) { return; }
  auto next_grants = regions(memory, usb);
  core.set_memory(cpu12::Memory::mapped(next_grants));
  grants.swap(next_grants);
  prepare_code(core, report, preparation);
}

Dispatch prepare(cpu12::Core &core, Devices &devices, ExecutionReport &report) {
  advance(core, devices);
  if (devices.stop.stop_requested()) { return Dispatch::done; }
  const auto state = core.state().execution;
  if (state == cpu12::ExecutionState::stopped) { return Dispatch::done; }
  if (interrupt(core, devices, report)) {
    devices.activity(cpu12::ExecutionState::running);
    return Dispatch::again;
  }
  if (report.failure) { return Dispatch::done; }
  if (core.state().returned_frame) { core.close_return_window(); }
  if (state == cpu12::ExecutionState::waiting) {
    const auto next = devices.deadline();
    if (!next) { return Dispatch::done; }
    devices.idle = *next - core.progress().estimated_cycles;
    return Dispatch::again;
  }
  return Dispatch::run;
}

}

ExecutionReport execute(FlashImage &image,
                        Shi &shi,
                        DspReset &reset,
                        const std::stop_token &stop,
                        const Controls *controls,
                        DisplayState *display,
                        PeerPort *peer,
                        ExecutionTime *time,
                        MidiPort *midi,
                        PreparationClock *preparation) {
  const PeerAdmission admission{peer};
  if (peer != nullptr && !peer->mcu_interrupt_ready(false)) {
    throw std::overflow_error(peer->error());
  }
  Memory memory(image);
  Devices devices(stop);
  devices.peer = peer;
  devices.time = time;
  devices.midi = midi;
  auto grants = regions(memory, devices.usb);
  Io io(memory, image, devices, shi, reset, stop, controls, display, peer);
  auto callbacks = io.callbacks();
  cpu12::Core core(cpu12::Memory::mapped(grants), &callbacks);
  io.bind(core);
  io.capture_display();
  ExecutionReport report;
  const auto initial = core.reset(reset_vector);
  if (!initial.accepted) {
    report.failure = Failure{.operation = Access::reset,
                             .pc = initial.pc,
                             .address = initial.address,
                             .value = initial.value,
                             .code = static_cast<std::uint32_t>(initial.fault),
                             .reason = "MCU reset vector access failed"};
    return report;
  }
  prepare_code(core, report, preparation);
  while (!stop.stop_requested()) {
    bind_memory(core, memory, devices.usb, grants, report, preparation);
    const auto dispatch = prepare(core, devices, report);
    if (dispatch == Dispatch::done) { break; }
    if (dispatch == Dispatch::again) { continue; }
    const auto deadline = devices.deadline();
    const auto result = deadline || devices.interrupts_enabled()
                            ? core.run_until(deadline ? *deadline - devices.idle : no_deadline)
                            : core.run();
    report.instructions += result.retired;
    if (stop.stop_requested()) { break; }
    if (result.fault != cpu12::Fault::none) {
      report.failure =
          Failure{.operation = operation(result.fault),
                  .pc = result.pc,
                  .address = result.address,
                  .value = result.value,
                  .code = static_cast<std::uint32_t>(result.fault),
                  .reason = io.error() != nullptr ? io.error()
                                                  : "MCU access or instruction is not implemented"};
      break;
    }
    if (!io.complete_events()) { break; }
    advance(core, devices);
    devices.activity(result.state);
  }

  if (display != nullptr) { display->request(); }
  io.capture_display();
  report.stopped_pc = core.registers().pc;
  return report;
}
}
