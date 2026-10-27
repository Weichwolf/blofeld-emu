// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_vector_read.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace blofeld::detail::hcs08 {
// Transient observation of actual SWI p-code transactions;never supplies PC/SP
// effects or retains an owner across an instruction/checkpoint boundary.
class SoftwareInterruptFrame {
public:
  SoftwareInterruptFrame(Bus &bus, const Hcs08State &before)
      : bus_(bus), known_(dynamic_cast<Hcs08KnownMemory *>(&bus)), before_(before) {
    constexpr unsigned all = 0xff;
    if (!before.pc_known ||
        (known_ == nullptr &&
         (before.a_known != all || (before.hx_known & all) != all || before.ccr_known != all))) {
      fail("admission", vector, "known PC and masked RAM capability required for partial frame");
    }
  }

  void store(const MemoryWrite access) {
    constexpr unsigned mask = 0xff;
    constexpr unsigned shift = 8;
    if (writes_ == 0) {
      const auto address =
          static_cast<std::uint16_t>(static_cast<unsigned>(before_.sp) + 65536U - 1U);
      const auto value = static_cast<std::uint16_t>(before_.pc + 1U);
      if (access.size != 2 || access.address != address || access.value != value) {
        fail("stack write",
             access.address,
             "native return-PC word differs from actual SWI continuation");
      }
      // Native STORE supplies the word;the architectural PCL transaction comes first.
      bus_.write_memory({.address = before_.sp, .value = access.value & mask});
      bus_.write_memory({.address = address, .value = access.value >> shift});
      writes_ = 2;
      return;
    }
    constexpr unsigned frame_bytes = 5;
    const std::array<Hcs08KnownByte, 3> bytes{
        {{.value = static_cast<std::uint8_t>(before_.hx & mask),
          .known = static_cast<std::uint8_t>(before_.hx_known & mask)},
         {.value = before_.a, .known = before_.a_known},
         {.value = before_.ccr, .known = before_.ccr_known}}};
    if (writes_ >= frame_bytes || access.size != 1 || access.value > mask ||
        access.address !=
            static_cast<std::uint16_t>(static_cast<unsigned>(before_.sp) + 65536U - writes_)) {
      fail("stack write", access.address, "unexpected native byte/frame order");
    }
    const auto expected = bytes.at(writes_ - 2);
    if ((access.value & expected.known) != expected.value) {
      fail("stack write", access.address, "native frame register differs in known bits");
    }
    const auto value = static_cast<std::uint8_t>(access.value & expected.known);
    if (expected.known == mask) {
      bus_.write(access.address, value);
    } else {
      if (known_ == nullptr) { fail("masked write", access.address, "lost masked RAM capability"); }
      known_->write_known_byte({.address = access.address, .value = value}, expected.known);
    }
    ++writes_;
  }

  [[nodiscard]] std::uint16_t read(const MemoryRead access) {
    constexpr unsigned frame_bytes = 5;
    constexpr unsigned word_bytes = 2;
    constexpr unsigned shift = 8;
    if (writes_ != frame_bytes || reads_ != 0 || access.address != vector ||
        access.size != word_bytes) {
      fail("vector read",
           access.address,
           "requires completed native frame and exactly one vector word load");
    }
    const auto high = vector_byte(bus_, vector);
    ++reads_;
    const auto low = vector_byte(bus_, vector + 1);
    ++reads_;
    target_ = static_cast<std::uint16_t>((static_cast<unsigned>(high) << shift) | low);
    return target_;
  }

  struct Completion {
    std::uint64_t pc, sp;
  };

  void require_decoder(const bool valid) const {
    if (!valid) {
      fail("decode", vector, "native stack/vector-load/indirect-call shape is unqualified");
    }
  }

  void finish(const Completion result) const {
    constexpr unsigned frame_bytes = 5;
    constexpr unsigned word_bytes = 2;
    if (writes_ != frame_bytes || reads_ != word_bytes || result.pc != target_ ||
        result.sp !=
            static_cast<std::uint16_t>(static_cast<unsigned>(before_.sp) + 65536U - frame_bytes)) {
      fail("completion",
           vector,
           "native frame/vector/branch/SP effects are incomplete or inconsistent");
    }
  }

  static constexpr unsigned vector = 0xfffc;

private:
  [[noreturn]] void
  fail(const std::string &operation, const unsigned address, const std::string &reason) const {
    throw Fault("HCS08 SWI " + operation + " address=" + hex(address) + " pc=" + hex(before_.pc) +
                " ccr=" + hex(before_.ccr) + " ccr-known=" + hex(before_.ccr_known) +
                " agent=CPU: " + reason);
  }

  Bus &bus_;
  Hcs08KnownMemory *known_;
  Hcs08State before_;
  unsigned writes_ = 0, reads_ = 0;
  std::uint16_t target_ = 0;
};
}
