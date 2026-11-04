// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_native_dependency_operands.hpp"

#include "dsp/dsp_interlocks.hpp"
#include "dsp/dsp_native_dependencies.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>

namespace {
enum class Kind : std::uint8_t {
  agu_writer,
  agu_reader,
  immediate_agu,
  memory_agu,
  multiply,
  partial_transfer,
  accumulator_read,
  status_read,
  long_store
};

struct OperandCase {
  unsigned opcode;
  unsigned base;
  Kind kind;
  unsigned index;
};

// Own operand encodings assembled independently; FM A.2.3/A.2.4 supply delays.
constexpr std::array operands{
    OperandCase{.opcode = 0x209000, .base = 1, .kind = Kind::agu_writer, .index = 0},
    OperandCase{.opcode = 0x209800, .base = 1, .kind = Kind::agu_writer, .index = 0},
    OperandCase{.opcode = 0x04c4a0, .base = 1, .kind = Kind::agu_writer, .index = 0},
    OperandCase{.opcode = 0x46e000, .base = 1, .kind = Kind::agu_reader, .index = 0},
    OperandCase{.opcode = 0x60f400, .base = 2, .kind = Kind::immediate_agu, .index = 0},
    OperandCase{.opcode = 0x60f000, .base = 2, .kind = Kind::memory_agu, .index = 0},
    OperandCase{.opcode = 0x209100, .base = 1, .kind = Kind::agu_writer, .index = 1},
    OperandCase{.opcode = 0x209900, .base = 1, .kind = Kind::agu_writer, .index = 1},
    OperandCase{.opcode = 0x04c4a1, .base = 1, .kind = Kind::agu_writer, .index = 1},
    OperandCase{.opcode = 0x46e100, .base = 1, .kind = Kind::agu_reader, .index = 1},
    OperandCase{.opcode = 0x61f400, .base = 2, .kind = Kind::immediate_agu, .index = 1},
    OperandCase{.opcode = 0x61f000, .base = 2, .kind = Kind::memory_agu, .index = 1},
    OperandCase{.opcode = 0x209200, .base = 1, .kind = Kind::agu_writer, .index = 2},
    OperandCase{.opcode = 0x209a00, .base = 1, .kind = Kind::agu_writer, .index = 2},
    OperandCase{.opcode = 0x04c4a2, .base = 1, .kind = Kind::agu_writer, .index = 2},
    OperandCase{.opcode = 0x46e200, .base = 1, .kind = Kind::agu_reader, .index = 2},
    OperandCase{.opcode = 0x62f400, .base = 2, .kind = Kind::immediate_agu, .index = 2},
    OperandCase{.opcode = 0x62f000, .base = 2, .kind = Kind::memory_agu, .index = 2},
    OperandCase{.opcode = 0x209300, .base = 1, .kind = Kind::agu_writer, .index = 3},
    OperandCase{.opcode = 0x209b00, .base = 1, .kind = Kind::agu_writer, .index = 3},
    OperandCase{.opcode = 0x04c4a3, .base = 1, .kind = Kind::agu_writer, .index = 3},
    OperandCase{.opcode = 0x46e300, .base = 1, .kind = Kind::agu_reader, .index = 3},
    OperandCase{.opcode = 0x63f400, .base = 2, .kind = Kind::immediate_agu, .index = 3},
    OperandCase{.opcode = 0x63f000, .base = 2, .kind = Kind::memory_agu, .index = 3},
    OperandCase{.opcode = 0x209400, .base = 1, .kind = Kind::agu_writer, .index = 4},
    OperandCase{.opcode = 0x209c00, .base = 1, .kind = Kind::agu_writer, .index = 4},
    OperandCase{.opcode = 0x04c4a4, .base = 1, .kind = Kind::agu_writer, .index = 4},
    OperandCase{.opcode = 0x46e400, .base = 1, .kind = Kind::agu_reader, .index = 4},
    OperandCase{.opcode = 0x64f400, .base = 2, .kind = Kind::immediate_agu, .index = 4},
    OperandCase{.opcode = 0x64f000, .base = 2, .kind = Kind::memory_agu, .index = 4},
    OperandCase{.opcode = 0x209500, .base = 1, .kind = Kind::agu_writer, .index = 5},
    OperandCase{.opcode = 0x209d00, .base = 1, .kind = Kind::agu_writer, .index = 5},
    OperandCase{.opcode = 0x04c4a5, .base = 1, .kind = Kind::agu_writer, .index = 5},
    OperandCase{.opcode = 0x46e500, .base = 1, .kind = Kind::agu_reader, .index = 5},
    OperandCase{.opcode = 0x65f400, .base = 2, .kind = Kind::immediate_agu, .index = 5},
    OperandCase{.opcode = 0x65f000, .base = 2, .kind = Kind::memory_agu, .index = 5},
    OperandCase{.opcode = 0x209600, .base = 1, .kind = Kind::agu_writer, .index = 6},
    OperandCase{.opcode = 0x209e00, .base = 1, .kind = Kind::agu_writer, .index = 6},
    OperandCase{.opcode = 0x04c4a6, .base = 1, .kind = Kind::agu_writer, .index = 6},
    OperandCase{.opcode = 0x46e600, .base = 1, .kind = Kind::agu_reader, .index = 6},
    OperandCase{.opcode = 0x66f400, .base = 2, .kind = Kind::immediate_agu, .index = 6},
    OperandCase{.opcode = 0x66f000, .base = 2, .kind = Kind::memory_agu, .index = 6},
    OperandCase{.opcode = 0x209700, .base = 1, .kind = Kind::agu_writer, .index = 7},
    OperandCase{.opcode = 0x209f00, .base = 1, .kind = Kind::agu_writer, .index = 7},
    OperandCase{.opcode = 0x04c4a7, .base = 1, .kind = Kind::agu_writer, .index = 7},
    OperandCase{.opcode = 0x46e700, .base = 1, .kind = Kind::agu_reader, .index = 7},
    OperandCase{.opcode = 0x67f400, .base = 2, .kind = Kind::immediate_agu, .index = 7},
    OperandCase{.opcode = 0x67f000, .base = 2, .kind = Kind::memory_agu, .index = 7},
    OperandCase{.opcode = 0x2000d0, .base = 1, .kind = Kind::multiply, .index = 0},
    OperandCase{.opcode = 0x208c00, .base = 1, .kind = Kind::partial_transfer, .index = 0},
    OperandCase{.opcode = 0x218500, .base = 1, .kind = Kind::accumulator_read, .index = 0},
    OperandCase{.opcode = 0x0445b9, .base = 1, .kind = Kind::status_read, .index = 0},
    OperandCase{.opcode = 0x547000, .base = 2, .kind = Kind::long_store, .index = 0},
};
unsigned checks = 0;

void check(const bool condition) {
  ++checks;
  if (!condition) { throw std::runtime_error("Independent FM dependency operand expectation"); }
}

blofeld::DspInterlockInstruction decoded(const OperandCase item) {
  const auto result = blofeld::inspect_native_dependencies(item.opcode, item.base);
  if (!result) { throw std::runtime_error("Missing authored operand description"); }
  ++checks;
  return *result;
}

void retire(blofeld::DspInterlocks &history, const blofeld::DspInterlockInstruction &instruction) {
  history.prepare(instruction);
  history.observe_opcode(instruction.opcode);
  static_cast<void>(history.complete(instruction.native_units));
}

void agu_cases() {
  constexpr unsigned groups = 8;
  constexpr unsigned group_size = 6;
  constexpr unsigned register_writers = 3;
  constexpr unsigned maximum_gap = 4;
  const auto nop = decoded({.opcode = 0, .base = 1, .kind = Kind::agu_reader, .index = 0});
  for (unsigned group = 0; group < groups; ++group) {
    const auto begin = group * group_size;
    const auto reader = decoded(operands.at(begin + register_writers));
    check(reader.agu_reads == (1U << group) && reader.agu_writes == 0);
    for (unsigned offset = 0; offset < group_size; ++offset) {
      if (offset == register_writers) { continue; }
      const auto writer = decoded(operands.at(begin + offset));
      const unsigned remaining = operands.at(begin + offset).kind == Kind::immediate_agu ? 2U : 3U;
      check(writer.agu_writes == (1U << group) && writer.agu_write_delay == remaining);
      for (unsigned gap = 0; gap < maximum_gap; ++gap) {
        blofeld::DspInterlocks history;
        retire(history, writer);
        for (unsigned pad = 0; pad < gap; ++pad) { retire(history, nop); }
        check(history.preview(reader) == remaining - std::min(remaining, gap));
        auto restored = history;
        check(restored.preview(reader) == history.preview(reader));
        restored.reset();
        check(restored.preview(reader) == 0);
      }
    }
  }
}

void accumulator_cases() {
  constexpr unsigned data_begin = 48;
  const auto multiply = decoded(operands.at(data_begin));
  const auto transfer = decoded(operands.at(data_begin + 1));
  const auto read = decoded(operands.at(data_begin + 2));
  const auto status = decoded(operands.at(data_begin + 3));
  const auto store = decoded(operands.at(data_begin + 4));
  const auto nop = decoded({.opcode = 0, .base = 1, .kind = Kind::agu_reader, .index = 0});
  check(multiply.arithmetic_writes == 1 && multiply.status_write);
  check(transfer.move_writes == 1 && transfer.transfer_write_delay == 1);
  check(read.move_reads == 1 && read.partial_status_source && status.status_read);
  check(store.move_reads == 1 && store.move_read_advance == 1);
  for (const auto producer : {multiply, transfer}) {
    blofeld::DspInterlocks history;
    retire(history, producer);
    check(history.preview(read) == 1 && history.preview(store) == 0);
    retire(history, nop);
    check(history.preview(read) == 0);
  }
  blofeld::DspInterlocks history;
  retire(history, multiply);
  check(history.preview(status) == 2);
  retire(history, nop);
  check(history.preview(status) == 1);
  retire(history, nop);
  check(history.preview(status) == 0);
}
}

unsigned blofeld::test::native_dependency_operand_checks() {
  agu_cases();
  accumulator_cases();
  return checks;
}
