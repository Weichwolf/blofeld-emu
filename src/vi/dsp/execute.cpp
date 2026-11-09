// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/execute.hpp"

#include "dsp/audio.hpp"
#include "dsp/clock.hpp"
#include "dsp/dax.hpp"
#include "dsp/dma.hpp"
#include "dsp/esai.hpp"
#include "dsp/interrupts.hpp"
#include "dsp/memory.hpp"
#include "dsp/ports.hpp"
#include "dsp/timers.hpp"
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/memory_controller.hpp"
#include "io/pcm_output.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "storage/media_image.hpp"

#include <array>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>

namespace blofeld::vi::dsp {
namespace {
constexpr dsp56300::Word boot_first = 0xff0000;
constexpr dsp56300::Word reset_status = 0xc00300;
constexpr dsp56300::Word reset_mode = 0x305;
constexpr dsp56300::Word once_data_address = 0xfffffc;
constexpr dsp56300::Word reserved_x_address = 0xffff80;
constexpr dsp56300::Word audio_data_address = 0xffff80;
constexpr dsp56300::Word peer_data_address = 0xffffbd;
constexpr unsigned esai1_transmit_request = 22;
constexpr unsigned dax_request = 10;
constexpr unsigned timer_request_first = 18;
constexpr dsp56300::Word timer0_control = 0xffff8f;
constexpr dsp56300::Word timer0_compare_vector = 0x54;
constexpr dsp56300::Word timer2_overflow_vector = 0x5e;
constexpr dsp56300::Word dma0_vector = 0x18, dma5_vector = 0x22, vector_base_mask = 0xffff00;

struct BitAccess {
  dsp56300::Space space;
  dsp56300::Word address, pc, index;
  dsp56300::BitAction action;
};

class Io final : public dsp56300::Io {
public:
  Io(const std::span<const dsp56300::Word> boot,
     Shi &shi,
     DspReset &reset,
     const std::stop_token &stop,
     MediaImage *image,
     const PcmOutput output,
     PeerPort *peer,
     ExecutionTime *time)
      : boot_(boot),
        shi_(shi),
        reset_(reset),
        stop_(stop),
        audio_(esai_1_, dax_, output, stop),
        controller_(image),
        peer_(peer),
        time_(time) {}

  void bind(dsp56300::Core &core, const std::uint64_t epoch) noexcept {
    core_ = &core;
    epoch_ = epoch;
    reset_interrupted_ = false;
    program_changed_ = false;
    waiting_ = false;
    idle_ = 0;
    media_epoch_ = 0;
    media_deadline_.reset();
    media_error_ = nullptr;
    once_data_.reset();
    clock_.reset(true);
    origin_ = time_ != nullptr ? time_->dsp_origin() : 0;
    peer_deadline_.reset();
    interrupts_.reset();
    dma_.reset();
    esai_1_.reset();
    dax_.reset();
    audio_.reset(epoch);
    audio_dirty_ = true;
    timers_.reset();
    memory_.configure(0);
    ports_.reset();
    if (peer_ != nullptr) { peer_->reset_dsp(); }
    controller_.reset();
    bool ready = false;
    if (controller_.ready(ready)) { static_cast<void>(timers_.input(1, ready)); }
  }

  void unbind() noexcept { core_ = nullptr; }

  bool complete_events(ExecutionReport &report) {
    if (media_deadline_ && device_cycles() >= *media_deadline_) {
      if (!controller_.complete() || !update_ports()) {
        report.failure = completion_failure_;
        report.failure->reason = controller_.error();
        return false;
      }
      media_deadline_.reset();
    }
    if (!synchronize_timers()) {
      report.failure = Failure{.pc = core_->registers().pc,
                               .address = timer0_control,
                               .space = 'X',
                               .reason = timers_.error()};
      return false;
    }
    if (audio_dirty_) {
      if (!audio_.configure(device_cycles(), clock_.ratio())) { return audio_failure(report); }
      audio_dirty_ = false;
    }
    while (const auto next = audio_.deadline()) {
      if (*next > device_cycles()) { break; }
      if (!audio_.clock()) { return audio_failure(report); }
      if (!service_dma(report)) { return false; }
      if (!audio_.submit()) { return audio_failure(report); }
    }
    return service_dma(report);
  }

  [[nodiscard]] bool flush_audio(ExecutionReport &report) {
    return audio_.submit(true) || audio_failure(report);
  }

  bool service_dma(ExecutionReport &report) {
    trigger_audio_requests();
    const auto requests = timers_.take_requests();
    for (unsigned index = 0; index < 3; ++index) {
      if ((requests & (1U << index)) != 0) {
        static_cast<void>(dma_.trigger(timer_request_first + index));
      }
    }
    if (!dma_.service(
            dma_pc_,
            [this](const Dma::Address access, dsp56300::Word &value) {
              return dma_read(access, value);
            },
            [this](const Dma::Address access, const dsp56300::Word value) {
              return dma_write(access, value);
            })) {
      if (!stop_.stop_requested() && !reset_interrupted_) { report.failure = dma_.failure(); }
      return false;
    }
    if (program_changed_) {
      revision_ ^= 1U;
      program_changed_ = false;
    }
    return true;
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override { return revision_; }

  [[nodiscard]] bool interrupt_pending(const dsp56300::Word status) const noexcept override {
    return interrupts_.select(interrupt_sources(), status).has_value();
  }

  [[nodiscard]] const char *error() const noexcept {
    if (audio_.error() != nullptr) { return audio_.error(); }
    if (media_error_ != nullptr) { return media_error_; }
    if (timers_.error() != nullptr) { return timers_.error(); }
    return esai_1_.error() != nullptr ? esai_1_.error() : controller_.error();
  }

  [[nodiscard]] dsp56300::RunResult run() {
    if (!synchronize_peer()) {
      dsp56300::RunResult stopped;
      stopped.yielded = true;
      return stopped;
    }
    if (waiting_ && !interrupts_.select(interrupt_sources(), core_->registers().sr)) {
      if (const auto next = deadline()) {
        idle_ = *next - core_->progress().estimated_cycles;
        if (!synchronize_timers()) { throw std::runtime_error(timers_.error()); }
      }
    }
    const auto sources = interrupt_sources();
    if (interrupts_.select(sources, 0)) {
      const auto state = core_->registers();
      const auto request = interrupts_.select(sources, state.sr);
      if (request) {
        if (core_->interrupt_ready()) {
          const auto external = peer_ != nullptr && request->vector == 0x10
                                    ? peer_->dsp_interrupt(interrupts_.external_edge(0x10))
                                    : std::nullopt;
          core_->request_interrupt((state.vba & vector_base_mask) | request->vector,
                                   request->level);
          acknowledge(request->vector);
          if (external && !peer_->acknowledge_dsp(*external)) {
            throw std::logic_error("Accepted DSP peer interrupt has an invalid generation");
          }
        }
        // An eligible request still observes REP completion and the interrupt shadow.
        return completed(core_->step());
      }
      // Masked requests do not turn ordinary execution into single stepping.
      // The Core returns at a real mask change before another instruction runs.
    }
    const auto next = deadline();
    return completed(next ? core_->run_until(*next - idle_) : core_->run());
  }

  [[nodiscard]] bool waiting_event() const noexcept {
    return waiting_ && (deadline() || interrupts_.select(interrupt_sources(), 0));
  }

  [[nodiscard]] bool reset_interrupted() const noexcept { return reset_interrupted_; }

  [[nodiscard]] std::optional<dsp56300::Word>
  inspect_program(const dsp56300::Word address) const noexcept override {
    dsp56300::Word value = 0;
    if (memory_.read(dsp56300::Space::program, address, value)) { return value; }
    if (address < boot_first || address - boot_first >= boot_.size()) { return {}; }
    return boot_[address - boot_first];
  }

  std::span<const dsp56300::MemoryWindow>
  memory_windows(const dsp56300::Space space) noexcept override {
    if (core_ != nullptr) { memory_.configure(core_->registers().omr); }
    return memory_.windows(space);
  }

  bool read(const dsp56300::Space space,
            const dsp56300::Word address,
            dsp56300::Word /*pc*/,
            dsp56300::Word &value) noexcept override {
    if (interrupted()) { return false; }
    if (memory_.read(space, address, value)) { return true; }
    if (Ports::contains(space, address)) { return read_port(space, address, value); }
    if (space == dsp56300::Space::y && Esai::contains(address)) {
      return esai_1_.read(address, value);
    }
    if (space == dsp56300::Space::x) { return read_x(address, value); }
    const auto word = space == dsp56300::Space::program ? inspect_program(address) : std::nullopt;
    if (!word) { return false; }
    value = *word;
    return true;
  }

  bool write(const dsp56300::Space space,
             const dsp56300::Word address,
             const dsp56300::Word pc,
             const dsp56300::Word value) noexcept override {
    if (interrupted()) { return false; }
    if (memory_.write({.space = space, .address = address, .value = value})) { return true; }
    if (Ports::contains(space, address)) {
      return write_port({.space = space, .address = address, .value = value}, pc);
    }
    if (space == dsp56300::Space::y && Esai::contains(address)) {
      return write_esai({.address = address, .value = value});
    }
    if (space != dsp56300::Space::x) { return false; }
    // DSP56371UM table 3-7 reserves X:$FFFF80. The original audio
    // initializer writes it; discard the write without creating a readable latch.
    if (address == reserved_x_address) { return true; }
    if (Timers::contains(address)) { return write_timer({.address = address, .value = value}); }
    if (Dax::contains(address)) { return write_dax({.address = address, .value = value}); }
    if (Dma::contains(address)) { return write_dma({.address = address, .value = value}, pc); }
    if (Interrupts::contains(address)) {
      return write_priority({.address = address, .value = value});
    }
    if (address == Clock::address) { return write_clock(value); }
    if (address != once_data_address) { return shi_.write({.address = address, .value = value}); }
    // DSP56300FM Rev.5, 7.2.4: OGDBR is a 24-bit output latch, readable
    // only through JTAG. A core write also enables the ROM-secured OnCE port.
    once_data_ = value;
    return true;
  }

  bool bit(const dsp56300::Space space,
           const dsp56300::Word address,
           const dsp56300::Word pc,
           const dsp56300::Word bit,
           const dsp56300::BitAction action,
           bool &previous) noexcept override {
    const BitAccess request{
        .space = space, .address = address, .pc = pc, .index = bit, .action = action};
    constexpr unsigned word_bits = 24;
    if (interrupted()) { return false; }
    if (request.index >= word_bits) { return false; }
    dsp56300::Word value = 0;
    if (!memory_.read(request.space, request.address, value)) {
      bool accepted = false;
      if (Ports::contains(request.space, request.address)) {
        accepted = read_port(request.space, request.address, value);
      } else if (request.space == dsp56300::Space::x) {
        accepted = read_x(request.address, value);
      } else if (request.space == dsp56300::Space::y) {
        accepted = esai_1_.read(request.address, value);
      }
      if (!accepted) { return false; }
    }
    const auto mask = dsp56300::Word{1} << request.index;
    previous = (value & mask) != 0;
    switch (request.action) {
      case dsp56300::BitAction::test: return true;
      case dsp56300::BitAction::clear: value &= ~mask; break;
      case dsp56300::BitAction::set: value |= mask; break;
      case dsp56300::BitAction::change: value ^= mask; break;
      default: return false;
    }
    if (memory_.write({.space = request.space, .address = request.address, .value = value})) {
      return true;
    }
    if (Ports::contains(request.space, request.address)) {
      return write_port({.space = request.space, .address = request.address, .value = value},
                        request.pc);
    }
    if (request.space == dsp56300::Space::y && Esai::contains(request.address)) {
      return write_esai({.address = request.address, .value = value});
    }
    if (request.space != dsp56300::Space::x) { return false; }
    if (Timers::contains(request.address)) {
      return write_timer({.address = request.address, .value = value});
    }
    if (Dax::contains(request.address)) {
      return write_dax({.address = request.address, .value = value});
    }
    if (Dma::contains(request.address)) {
      return write_dma({.address = request.address, .value = value}, request.pc);
    }
    if (Interrupts::contains(request.address)) {
      return write_priority({.address = request.address, .value = value});
    }
    if (request.address == Clock::address) { return write_clock(value); }
    return shi_.write({.address = request.address, .value = value});
  }

private:
  bool audio_failure(ExecutionReport &report) const {
    if (!stop_.stop_requested() && !report.failure) {
      report.failure = Failure{.operation = Access::execute,
                               .pc = core_->registers().pc,
                               .address = audio_data_address,
                               .space = 'Y',
                               .reason = audio_.error()};
    }
    return false;
  }

  void audio_changed() noexcept {
    audio_dirty_ = true;
    core_->request_yield();
  }

  bool write_esai(const Esai::Write access) noexcept {
    if (!esai_1_.write(access)) { return false; }
    constexpr dsp56300::Word skip = 0xffff92;
    if (!Esai::data_register(access.address) && access.address != skip) { audio_changed(); }
    return true;
  }

  bool write_dax(const Dax::Write access) noexcept {
    if (!dax_.write(access)) { return false; }
    constexpr dsp56300::Word control = 0xffffd0;
    if (access.address == control) { audio_changed(); }
    return true;
  }

  bool write_clock(const dsp56300::Word value) noexcept {
    if (!clock_.advance(device_cycles()) || !clock_.write(value)) { return false; }
    audio_changed();
    return true;
  }

  bool synchronize_peer() {
    if (!clock_.advance(device_cycles())) {
      throw std::overflow_error("DSP instrument time is not representable or monotonic");
    }
    if (time_ == nullptr) { return true; }
    if (clock_.ticks() > std::numeric_limits<std::uint64_t>::max() - origin_) {
      throw std::overflow_error("DSP reset origin exceeds instrument time");
    }
    const auto limit = time_->dsp_limit(origin_ + clock_.ticks(), stop_);
    if (!limit) { return false; }
    peer_deadline_ = clock_.deadline(*limit - origin_);
    if (!peer_deadline_) { throw std::overflow_error("DSP peer deadline is not representable"); }
    return true;
  }

  void trigger_audio_requests() noexcept {
    if (esai_1_.request()) { static_cast<void>(dma_.trigger(esai1_transmit_request)); }
    if (dax_.request()) { static_cast<void>(dma_.trigger(dax_request)); }
  }

  [[nodiscard]] std::uint64_t device_cycles() const noexcept {
    return core_->progress().estimated_cycles + idle_;
  }

  [[nodiscard]] std::optional<std::uint64_t> deadline() const noexcept {
    auto result = timers_.deadline();
    for (const auto next : {media_deadline_, audio_.deadline()}) {
      if (next && (!result || *next < *result)) { result = next; }
    }
    if (peer_deadline_ && (!result || *peer_deadline_ < *result)) { result = peer_deadline_; }
    return result;
  }

  bool schedule_media() noexcept {
    const auto ratio = clock_.ratio();
    const auto ticks = std::uint64_t{controller_.reference_ticks()} * ratio.numerator;
    const auto delay = (ticks + ratio.denominator - 1) / ratio.denominator;
    const auto now = device_cycles();
    if (delay > std::numeric_limits<std::uint64_t>::max() - now) {
      media_error_ = "Media completion deadline is not representable";
      return false;
    }
    media_epoch_ = controller_.epoch();
    media_deadline_ = now + delay;
    return true;
  }

  [[nodiscard]] std::array<dsp56300::Word, 9> interrupt_sources() const noexcept {
    std::array<dsp56300::Word, 9> sources{};
    const auto pending = dma_.interrupts();
    for (unsigned channel = 0; channel < 6; ++channel) {
      if ((pending & (1U << channel)) != 0) { sources[channel] = dma0_vector + (channel * 2); }
    }
    sources[6] = timers_.interrupt().value_or(0);
    sources[7] = dax_.interrupt().value_or(0);
    if (peer_ != nullptr && peer_->dsp_interrupt(interrupts_.external_edge(0x10))) {
      sources[8] = 0x10;
    }
    return sources;
  }

  void acknowledge(const dsp56300::Word vector) {
    if (vector >= dma0_vector && vector <= dma5_vector &&
        !dma_.acknowledge((vector - dma0_vector) / 2)) {
      throw std::logic_error("Admitted DMA interrupt has no actual pending request");
    }
    if (vector >= timer0_compare_vector && vector <= timer2_overflow_vector &&
        !timers_.acknowledge(vector)) {
      throw std::logic_error("Admitted timer interrupt has no actual pending flag");
    }
    // DAX uses level requests: only firmware data writes clear its conditions.
  }

  dsp56300::RunResult completed(const dsp56300::RunResult result) noexcept {
    waiting_ = result.state == dsp56300::ExecutionState::waiting;
    return result;
  }

  bool write_priority(const Interrupts::Write access) noexcept {
    if (!interrupts_.write(access)) { return false; }
    core_->request_yield();
    return true;
  }

  bool synchronize_timers(const bool in_callback = false) noexcept {
    if (!timers_.advance(device_cycles())) { return false; }
    bool ready = false;
    if (controller_.ready(ready) && !timers_.input(1, ready)) { return false; }
    if (in_callback && (timers_.requests() != 0 || timers_.interrupt())) { core_->request_yield(); }
    return true;
  }

  bool write_timer(const Timers::Write access) noexcept {
    if (!synchronize_timers(true)) { return false; }
    const auto next = timers_.deadline();
    const auto interrupt = timers_.interrupt();
    if (!timers_.write(access)) { return false; }
    if (next != timers_.deadline() || interrupt != timers_.interrupt()) { core_->request_yield(); }
    return true;
  }

  bool write_dma(const Dma::Write access, const dsp56300::Word pc) noexcept {
    if (!dma_.write(access)) { return false; }
    trigger_audio_requests();
    if (dma_.pending() || dma_.interrupts() != 0) {
      dma_pc_ = pc;
      core_->request_yield();
    }
    return true;
  }

  bool dma_read(const Dma::Address access, dsp56300::Word &value) noexcept {
    if (interrupted()) { return false; }
    if (memory_.read(access.space, access.address, value)) { return true; }
    constexpr dsp56300::Word shi_receive = 0xffff94;
    if (access.space == dsp56300::Space::x && access.address == shi_receive) {
      return shi_.read(access.address, value);
    }
    if (access.space != dsp56300::Space::program) { return false; }
    const auto word = inspect_program(access.address);
    if (!word) { return false; }
    value = *word;
    return true;
  }

  bool dma_write(const Dma::Address access, const dsp56300::Word value) noexcept {
    if (interrupted()) { return false; }
    constexpr dsp56300::Word peripheral_first = 0xffff80;
    constexpr dsp56300::Word shi_transmit = 0xffff93;
    if (access.space != dsp56300::Space::program && access.address >= peripheral_first) {
      if (access.space == dsp56300::Space::y && Esai::data_register(access.address)) {
        return esai_1_.connected() && esai_1_.write({.address = access.address, .value = value});
      }
      if (access.space == dsp56300::Space::x && Dax::data_register(access.address)) {
        return dax_.connected() && dax_.write({.address = access.address, .value = value});
      }
      return access.space == dsp56300::Space::x && access.address == shi_transmit &&
             shi_.write({.address = access.address, .value = value});
    }
    if (!memory_.write({.space = access.space, .address = access.address, .value = value})) {
      return false;
    }
    if (access.space == dsp56300::Space::program) { program_changed_ = true; }
    return true;
  }

  bool write_port(const Ports::Write access, const dsp56300::Word pc) noexcept {
    if (!ports_.write(access) || !update_ports()) { return false; }
    if (controller_.pending() && controller_.epoch() != media_epoch_) {
      if (!synchronize_timers(true) || !schedule_media()) { return false; }
      completion_failure_ = {.operation = Access::write,
                             .pc = pc,
                             .address = access.address,
                             .value = access.value,
                             .space = access.space == dsp56300::Space::x ? 'X' : 'Y'};
      core_->request_yield();
    }
    return true;
  }

  bool read_port(const dsp56300::Space space, const dsp56300::Word address, dsp56300::Word &value) {
    if (peer_ != nullptr && space == dsp56300::Space::x && address == peer_data_address) {
      const auto input = peer_->mcu_data();
      ports_.input(Port::c, {.mask = input.mask, .known = input.known, .value = input.value});
    }
    return ports_.read(space, address, value);
  }

  [[nodiscard]] bool publish_peer() const {
    if (peer_ == nullptr) { return true; }
    const auto data = ports_.output(Port::c);
    constexpr dsp56300::Word byte_mask = 0xff;
    constexpr dsp56300::Word ready = 1U << 9U;
    peer_->dsp_data({.mask = static_cast<std::uint8_t>(data.mask & byte_mask),
                     .known = static_cast<std::uint8_t>(data.known & byte_mask),
                     .value = static_cast<std::uint8_t>(data.value & byte_mask)});
    const auto request = ports_.output(Port::e);
    if ((request.mask & ~request.known & ready) != 0) { return false; }
    return peer_->dsp_ready((request.mask & ready) != 0 && (request.value & ready) == 0);
  }

  bool update_ports() noexcept {
    const bool esai_connected = esai_1_.connected();
    const bool dax_connected = dax_.connected();
    esai_1_.connected(ports_.peripheral(Port::e) != 0);
    dax_.connected(ports_.peripheral(Port::d) != 0);
    if (esai_connected != esai_1_.connected() || dax_connected != dax_.connected()) {
      audio_changed();
    }
    constexpr dsp56300::Word control_mask = 0xf00;
    constexpr dsp56300::Word data_mask = 0xff;
    const auto controls = ports_.output(Port::c);
    const auto data = ports_.output(Port::f);
    if ((controls.mask & controls.known & control_mask) == control_mask &&
        !controller_.write({.controls = controls.value & control_mask,
                            .value = static_cast<std::uint8_t>(data.value & data_mask),
                            .known = (data.mask & data.known & data_mask) == data_mask})) {
      return false;
    }
    const auto output = controller_.output();
    ports_.input(Port::f,
                 {.mask = data_mask, .known = output ? data_mask : 0, .value = output.value_or(0)});
    return publish_peer();
  }

  bool read_x(const dsp56300::Word address, dsp56300::Word &value) noexcept {
    if (Timers::contains(address)) {
      return synchronize_timers(true) && timers_.read(address, value);
    }
    if (Dax::contains(address)) { return dax_.read(address, value); }
    if (Dma::contains(address)) { return dma_.read(address, value); }
    if (Interrupts::contains(address)) { return interrupts_.read(address, value); }
    if (address != Clock::address) { return shi_.read(address, value); }
    value = clock_.read();
    return true;
  }

  bool interrupted() noexcept {
    if (peer_ != nullptr && peer_->dsp_interrupt(interrupts_.external_edge(0x10))) {
      constexpr std::array<dsp56300::Word, 1> source{0x10};
      if (interrupts_.select(source, 0) && interrupts_.select(source, core_->registers().sr)) {
        core_->request_yield();
      }
    }
    if (stop_.stop_requested()) {
      core_->request_yield();
      return true;
    }
    if (!reset_.pending(epoch_)) { return false; }
    reset_interrupted_ = true;
    core_->request_yield();
    return true;
  }

  std::span<const dsp56300::Word> boot_;
  Shi &shi_;
  DspReset &reset_;
  std::uint64_t epoch_ = 0;
  bool reset_interrupted_ = false;
  bool program_changed_ = false;
  bool audio_dirty_ = true;
  bool waiting_ = false;
  std::uint64_t idle_ = 0;
  std::uint64_t media_epoch_ = 0;
  std::optional<std::uint64_t> media_deadline_;
  const char *media_error_ = nullptr;
  std::uint64_t revision_ = 0;
  dsp56300::Word dma_pc_ = 0;
  const std::stop_token &stop_;
  dsp56300::Core *core_ = nullptr;
  std::optional<dsp56300::Word> once_data_;
  Clock clock_;
  Interrupts interrupts_;
  Dma dma_;
  Esai esai_1_;
  Dax dax_;
  Audio audio_;
  Timers timers_;
  Memory memory_;
  Ports ports_;
  MemoryController controller_;
  PeerPort *peer_;
  ExecutionTime *time_;
  std::uint64_t origin_ = 0;
  std::optional<std::uint64_t> peer_deadline_;
  Failure completion_failure_;
};

Access operation(const dsp56300::FaultKind fault) {
  switch (fault) {
    case dsp56300::FaultKind::io_read:
    case dsp56300::FaultKind::memory_read: return Access::read;
    case dsp56300::FaultKind::io_write:
    case dsp56300::FaultKind::memory_write: return Access::write;
    case dsp56300::FaultKind::io_bit: return Access::bit;
    case dsp56300::FaultKind::none:
    case dsp56300::FaultKind::unsupported_instruction:
    case dsp56300::FaultKind::unsupported_mode:
    case dsp56300::FaultKind::stack:
    case dsp56300::FaultKind::invalid_instruction:
    case dsp56300::FaultKind::host_event:
    default: return Access::execute;
  }
}

bool run_instance(Io &io,
                  DspReset &reset,
                  const std::uint64_t epoch,
                  const std::stop_token &stop,
                  ExecutionReport &report) {
  dsp56300::Core core(dsp56300::Memory::external(), &io);
  io.bind(core, epoch);
  core.reset();
  auto registers = core.registers();
  registers.pc = boot_first;
  registers.sr = reset_status;
  registers.omr = reset_mode;
  core.set_registers(registers);
  bool restart = false;
  while (!stop.stop_requested()) {
    if (reset.pending(epoch)) {
      restart = true;
      break;
    }
    const auto result = io.run();
    report.instructions += result.retired;
    if (stop.stop_requested()) { break; }
    if (io.reset_interrupted()) {
      restart = true;
      break;
    }
    if (result.fault.kind != dsp56300::FaultKind::none) {
      const auto &fault = result.fault;
      char space = 'P';
      if (fault.space == dsp56300::Space::x) { space = 'X'; }
      if (fault.space == dsp56300::Space::y) { space = 'Y'; }
      report.failure =
          Failure{.operation = operation(fault.kind),
                  .pc = fault.pc,
                  .address = fault.address,
                  .value = fault.value,
                  .code = static_cast<std::uint32_t>(fault.kind),
                  .space = space,
                  .reason = io.error() != nullptr ? io.error()
                                                  : "DSP access or instruction is not implemented"};
      break;
    }
    if (reset.pending(epoch)) {
      restart = true;
      break;
    }
    if (!io.complete_events(report)) {
      restart = io.reset_interrupted();
      break;
    }
    if (result.state != dsp56300::ExecutionState::running && !io.waiting_event()) { break; }
  }
  static_cast<void>(io.flush_audio(report));
  io.unbind();
  return restart;
}
}

ExecutionReport execute(const std::span<const dsp56300::Word> boot,
                        Shi &shi,
                        DspReset &reset,
                        const std::stop_token &stop,
                        MediaImage *image,
                        const PcmOutput output,
                        PeerPort *peer,
                        ExecutionTime *time) {
  if (boot.empty() || boot.size() > dsp56300::address_words - boot_first) {
    throw std::invalid_argument("DSP boot ROM has invalid bounds");
  }
  Io io(boot, shi, reset, stop, image, output, peer, time);
  ExecutionReport report;
  std::uint64_t applied = 0;
  while (!stop.stop_requested()) {
    const auto state = reset.state();
    if (state.epoch != applied) {
      // run_instance has destroyed the previous Core before peripherals change.
      // Reset retains SRAM; its known bytes remain owned by this execution Io.
      shi.reset();
      applied = state.epoch;
      if (!reset.acknowledge(applied)) { throw std::logic_error("Invalid DSP reset epoch"); }
    }
    if (state.held) {
      if (!reset.wait_release(stop)) { break; }
      continue;
    }
    if (!run_instance(io, reset, applied, stop, report)) { break; }
  }
  return report;
}
}
