// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "hc12_cycle_fixture.hpp"
#include "mcu/hc12.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace blofeld::test {
inline constexpr unsigned indexed_byte_mask = 0xff;
inline constexpr unsigned indexed_word_mask = 0xffff;
inline constexpr unsigned indexed_initial_d = 0x89ab;
inline constexpr unsigned indexed_entry = 0x8000;

enum class IndexedKind : std::uint8_t { short_offset, offset9, offset16, indirect_d, indirect16 };

struct IndexedInput {
  std::array<std::uint8_t, 3> operand{};
  unsigned bytes = 1;
  IndexedKind kind = IndexedKind::short_offset;
  unsigned base_register = 0;
  unsigned base = 0;
  int offset = 0;
  int update = 0;
  bool pre = false;
  unsigned d = indexed_initial_d;
};

struct IndexedOperation {
  std::uint8_t opcode;
  unsigned destination; // 0 = D/A, 1 = X, 2 = Y, 3 = SP.
  bool word;
  bool store;
  bool lea;
};

struct IndexedAssignment {
  unsigned index;
  unsigned value;
};

inline void set_indexed_register(CpuState &state, const IndexedAssignment assignment) {
  const auto word = static_cast<std::uint16_t>(assignment.value & indexed_word_mask);
  switch (assignment.index) {
    case 0: state.x = word; break;
    case 1: state.y = word; break;
    case 2: state.sp = word; break;
    case 3: state.pc = word; break;
    default: throw Fault("Invalid own indexed register");
  }
}

inline unsigned indexed_sum(const unsigned base, const int offset) {
  return static_cast<unsigned>(static_cast<int>(base) + offset +
                               static_cast<int>(indexed_word_mask + 1)) &
         indexed_word_mask;
}

inline unsigned indexed_memory(const CycleMemory &memory, const unsigned address, const bool word) {
  const auto first = memory.bytes.at(address);
  return word ? (static_cast<unsigned>(first) << 8U) |
                    memory.bytes.at((address + 1) & indexed_word_mask)
              : first;
}

inline bool indexed_indirect(const IndexedKind kind) {
  return kind == IndexedKind::indirect_d || kind == IndexedKind::indirect16;
}

inline std::string_view indexed_read_slots(const IndexedKind kind, const bool word) {
  switch (kind) {
    case IndexedKind::short_offset: return word ? "RPf" : "rPf";
    case IndexedKind::offset9: return word ? "RPO" : "rPO";
    case IndexedKind::offset16: return word ? "fRPP" : "frPP";
    case IndexedKind::indirect_d: return word ? "fIfRPf" : "fIfrPf";
    case IndexedKind::indirect16: return word ? "fIPRPf" : "fIPrPf";
    default: throw Fault("Invalid own indexed load mode");
  }
}

inline std::string_view indexed_write_slots(const IndexedKind kind, const bool word) {
  switch (kind) {
    case IndexedKind::short_offset: return word ? "PW" : "Pw";
    case IndexedKind::offset9: return word ? "PWO" : "PwO";
    case IndexedKind::offset16: return word ? "PWP" : "PwP";
    case IndexedKind::indirect_d: return word ? "PIfW" : "PIfw";
    case IndexedKind::indirect16: return word ? "PIPW" : "PIPw";
    default: throw Fault("Invalid own indexed store mode");
  }
}

inline std::string_view indexed_slots(const IndexedInput &input,
                                      const IndexedOperation &operation) {
  if (operation.lea) {
    switch (input.kind) {
      case IndexedKind::short_offset: return "Pf";
      case IndexedKind::offset9: return "PO";
      case IndexedKind::offset16: return "PP";
      case IndexedKind::indirect_d:
      case IndexedKind::indirect16: throw Fault("Indirect LEA is not an instruction");
      default: throw Fault("Invalid own indexed LEA mode");
    }
  }
  return operation.store ? indexed_write_slots(input.kind, operation.word)
                         : indexed_read_slots(input.kind, operation.word);
}

inline void check_indexed_clocks(const CycleFixture &fixture,
                                 const IndexedInput &input,
                                 const IndexedOperation &operation,
                                 const unsigned pointer,
                                 const unsigned address) {
  const auto slots = indexed_slots(input, operation);
  require_cycle(fixture.clocks.size() == slots.size(), "Indexed manual cycle count differs");
  for (unsigned index = 0; index < slots.size(); ++index) {
    const auto &cycle = fixture.clocks.at(index);
    const auto kind = slots[index];
    require_cycle(cycle.kind == kind, "Indexed manual bus slot kind differs");
    if (kind == 'I') {
      require_cycle(cycle.address == pointer && cycle.size == 2,
                    "Indirect pointer must be one word read at the calculated address");
    } else if (kind == 'R' || kind == 'W' || kind == 'r' || kind == 'w') {
      require_cycle(cycle.address == address && cycle.size == (operation.word ? 2U : 1U),
                    "Indexed operand must retain its exact effective address and width");
    } else {
      require_cycle(!cycle.address && cycle.size == 0,
                    "Prefetch/free cycle must not become an invented data access");
    }
  }
}
}
