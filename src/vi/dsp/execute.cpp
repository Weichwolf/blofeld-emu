// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/execute.hpp"

#include "dsp/clock.hpp"
#include "dsp/memory.hpp"
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/shi.hpp"

#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
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

class Io final : public dsp56300::Io {
public:
  Io(const std::span<const dsp56300::Word> boot,
     Shi &shi,
     DspReset &reset,
     const std::stop_token &stop)
      : boot_(boot), shi_(shi), reset_(reset), stop_(stop) {}

  void bind(dsp56300::Core &core, const std::uint64_t epoch) noexcept {
    core_ = &core;
    epoch_ = epoch;
    reset_interrupted_ = false;
    once_data_.reset();
    clock_.reset(true);
    memory_.configure(0);
  }

  void unbind() noexcept { core_ = nullptr; }

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
    if (space == dsp56300::Space::x) { return read_x(address, value); }
    const auto word = space == dsp56300::Space::program ? inspect_program(address) : std::nullopt;
    if (!word) { return false; }
    value = *word;
    return true;
  }

  bool write(const dsp56300::Space space,
             const dsp56300::Word address,
             dsp56300::Word /*pc*/,
             const dsp56300::Word value) noexcept override {
    if (interrupted()) { return false; }
    if (memory_.write({.space = space, .address = address, .value = value})) { return true; }
    if (space != dsp56300::Space::x) { return false; }
    if (address == Clock::address) { return clock_.write(value); }
    if (address != once_data_address) { return shi_.write({.address = address, .value = value}); }
    // DSP56300FM Rev.5, 7.2.4: OGDBR is a 24-bit output latch, readable
    // only through JTAG. A core write also enables the ROM-secured OnCE port.
    once_data_ = value;
    return true;
  }

  bool bit(const dsp56300::Space space,
           const dsp56300::Word address,
           dsp56300::Word /*pc*/,
           const dsp56300::Word bit,
           const dsp56300::BitAction action,
           bool &previous) noexcept override {
    constexpr unsigned word_bits = 24;
    if (interrupted()) { return false; }
    dsp56300::Word value = 0;
    if (bit >= word_bits) { return false; }
    if (!memory_.read(space, address, value) &&
        (space != dsp56300::Space::x || !read_x(address, value))) {
      return false;
    }
    const auto mask = dsp56300::Word{1} << bit;
    previous = (value & mask) != 0;
    switch (action) {
      case dsp56300::BitAction::test: return true;
      case dsp56300::BitAction::clear: value &= ~mask; break;
      case dsp56300::BitAction::set: value |= mask; break;
      case dsp56300::BitAction::change: value ^= mask; break;
      default: return false;
    }
    if (memory_.write({.space = space, .address = address, .value = value})) { return true; }
    if (space != dsp56300::Space::x) { return false; }
    if (address == Clock::address) { return clock_.write(value); }
    return shi_.write({.address = address, .value = value});
  }

private:
  bool read_x(const dsp56300::Word address, dsp56300::Word &value) noexcept {
    if (address != Clock::address) { return shi_.read(address, value); }
    value = clock_.read();
    return true;
  }

  bool interrupted() noexcept {
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
  const std::stop_token &stop_;
  dsp56300::Core *core_ = nullptr;
  std::optional<dsp56300::Word> once_data_;
  Clock clock_;
  Memory memory_;
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
    const auto result = core.run();
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
      report.failure = Failure{.operation = operation(fault.kind),
                               .pc = fault.pc,
                               .address = fault.address,
                               .value = fault.value,
                               .code = static_cast<std::uint32_t>(fault.kind),
                               .space = space,
                               .reason = "DSP access or instruction is not implemented"};
      break;
    }
    if (reset.pending(epoch)) {
      restart = true;
      break;
    }
    if (result.state != dsp56300::ExecutionState::running) { break; }
  }
  io.unbind();
  return restart;
}
}

ExecutionReport execute(const std::span<const dsp56300::Word> boot,
                        Shi &shi,
                        DspReset &reset,
                        const std::stop_token &stop) {
  if (boot.empty() || boot.size() > dsp56300::address_words - boot_first) {
    throw std::invalid_argument("DSP boot ROM has invalid bounds");
  }
  Io io(boot, shi, reset, stop);
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
