// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hcs08_bdc.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_flash.hpp"

#include <cstdint>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned reset_active_control = 0x88, active_mask = 0x40, enable_mask = 0x80;
}

Hcs08Bdc::Hcs08Bdc(Hcs08 &cpu,
                   Hcs08BdcMemory &memory,
                   const Hcs08FlashContract contract,
                   const Hcs08BdcReset sampled_reset)
    : cpu_(cpu), memory_(memory) {
  if (contract != Hcs08FlashContract::candidate_gt8a) {
    fail("configure", "requires the explicit candidate GT8A contract");
  }
  if (sampled_reset != Hcs08BdcReset::normal && sampled_reset != Hcs08BdcReset::bkgd_held_low) {
    fail("configure", "invalid sampled reset mode");
  }
  static_cast<void>(cpu_.checkpoint());
  state_.active = sampled_reset == Hcs08BdcReset::bkgd_held_low;
  state_.control = state_.active ? reset_active_control : 0;
}

void Hcs08Bdc::healthy() const {
  if (!healthy_ || cpu_.faulted()) {
    fail("access", "incomplete execution requires coordinated recovery");
  }
}

[[noreturn]] void Hcs08Bdc::fail(const std::string &operation, const std::string &reason) const {
  const auto cpu = cpu_.state();
  throw Fault("HCS08 BDC " + operation + " command=" + hex(state_.frame[0]) + " pc=" +
              (cpu.pc_known ? hex(cpu.pc) : "unknown pc-known=0") + " ccr=" + hex(cpu.ccr) +
              " received=" + std::to_string(state_.received) + ": " + reason);
}

unsigned Hcs08Bdc::frame_size(const std::uint8_t command) const {
  switch (command) {
    case hcs08_bdc::go:
    case hcs08_bdc::read_a:
    case hcs08_bdc::read_ccr:
    case hcs08_bdc::read_pc:
    case hcs08_bdc::read_hx:
    case hcs08_bdc::read_sp:
    case hcs08_bdc::background:
    case hcs08_bdc::read_status: return 1;
    case hcs08_bdc::write_a:
    case hcs08_bdc::write_ccr:
    case hcs08_bdc::write_control: return 2;
    case hcs08_bdc::write_pc:
    case hcs08_bdc::write_hx:
    case hcs08_bdc::write_sp:
    case hcs08_bdc::read_byte: return 3;
    case hcs08_bdc::write_byte: return 4;
    default: fail("receive", "unsupported command");
  }
}

void Hcs08Bdc::receive(const std::uint8_t byte) {
  healthy();
  healthy_ = false;
  if (state_.reply_cursor != state_.reply_size ||
      (state_.expected != 0 && state_.received == state_.expected)) {
    fail("receive", "previous reply or completed command is still pending");
  }
  if (state_.received == 0) {
    state_.frame = {};
    state_.frame[0] = byte;
    state_.expected = frame_size(byte);
  }
  state_.frame.at(state_.received++) = byte;
  healthy_ = true;
}

void Hcs08Bdc::service(const Hcs08BdcGrant grant) {
  healthy();
  healthy_ = false;
  if (state_.received == 0 || state_.received != state_.expected) {
    fail("service", "no complete command frame");
  }
  if (grant != Hcs08BdcGrant::unqualified &&
      grant != Hcs08BdcGrant::reference_instruction_boundary) {
    fail("service", "invalid memory grant");
  }
  static_cast<void>(cpu_.checkpoint());
  execute(grant);
  ++state_.serviced;
  state_.received = 0;
  state_.expected = 0;
  healthy_ = true;
}

std::uint8_t Hcs08Bdc::read_reply() {
  healthy();
  healthy_ = false;
  if (state_.reply_cursor == state_.reply_size) { fail("reply", "no serviced response byte"); }
  const auto byte = state_.reply.at(state_.reply_cursor++);
  healthy_ = true;
  return byte;
}

void Hcs08Bdc::communication_timeout() {
  healthy();
  state_.received = 0;
  state_.expected = 0;
  state_.frame = {};
  state_.reply = {};
  state_.reply_size = 0;
  state_.reply_cursor = 0;
}

void Hcs08Bdc::step_cpu() {
  healthy();
  if (state_.active) { fail("CPU step", "active background mode owns the halted CPU"); }
  healthy_ = false;
  cpu_.step();
  if (cpu_.background_requested()) {
    if ((state_.control & enable_mask) == 0 ||
        memory_.secured({.pc = cpu_.state().pc, .ccr = cpu_.state().ccr})) {
      fail("BGND", "disabled or secured BGND admission is not qualified");
    }
    state_.active = true;
  }
  healthy_ = true;
}

std::uint8_t Hcs08Bdc::status() const {
  healthy();
  return static_cast<std::uint8_t>(state_.control | (state_.active ? active_mask : 0));
}

bool Hcs08Bdc::faulted() const {
  return !healthy_ || cpu_.faulted();
}

Hcs08BdcState Hcs08Bdc::state() const {
  return state_;
}

Hcs08State Hcs08Bdc::cpu_state() const {
  healthy();
  return cpu_.state();
}

Hcs08BdcCheckpoint Hcs08Bdc::checkpoint() const {
  healthy();
  static_cast<void>(cpu_.checkpoint());
  Hcs08BdcCheckpoint checkpoint;
  checkpoint.state_ = state_;
  return checkpoint;
}

void Hcs08Bdc::restore_checkpoint(const Hcs08BdcCheckpoint &checkpoint) {
  static_cast<void>(cpu_.checkpoint());
  state_ = checkpoint.state_;
  healthy_ = true;
}
} // namespace blofeld
