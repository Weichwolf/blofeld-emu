// SPDX-License-Identifier: GPL-3.0-only
#include "clear_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_prefetch_move_fixture.hpp"
#include "hcs08_prefetch_word_fixture.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace blofeld::test::clear {
namespace {
namespace word = blofeld::test::word;
namespace move = blofeld::test::move;
constexpr unsigned word_value = 0x4567, move_value = 0x80, source_poison = 0x44;

void word_snapshots() {
  word::Machine parent;
  word::Machine fresh;
  const word::Input input{.form = word::Form::dir, .value = word_value};
  for (unsigned cut = 0; cut <= word::periods(input); ++cut) {
    parent.prepare(input, contract);
    for (unsigned n = 0; n < cut; ++n) { parent.tick(); }
    const auto saved = parent.cpu.checkpoint();
    fresh.memory = parent.memory;
    fresh.cpu.restore_checkpoint(saved);
    word::finish(parent, input);
    word::finish(fresh, input);
    require(parent.cpu.state() == fresh.cpu.state() &&
                parent.cpu.prefetch_state() == fresh.cpu.prefetch_state() &&
                parent.memory.events == fresh.memory.events &&
                fresh.memory.bytes[operand] == (word_value >> 8U) &&
                fresh.memory.bytes[operand + 1] == (word_value & byte_mask) &&
                fresh.memory.write_widths == std::vector<unsigned>{1, 1} &&
                fresh.memory.native_starts == 1 && fresh.memory.retired == 1,
            "New clear contract retains suspended native STHX across both real writes");
  }
}

void move_snapshots() {
  move::Machine parent;
  move::Machine fresh;
  const move::Input input{.value = move_value};
  for (unsigned cut = 0; cut <= move::periods; ++cut) {
    parent.prepare(input, contract);
    for (unsigned n = 0; n < cut; ++n) { parent.tick(); }
    const auto saved = parent.cpu.checkpoint();
    fresh.memory = parent.memory;
    fresh.cpu.restore_checkpoint(saved);
    if (cut != 0) {
      parent.memory.install(input.source, std::array{static_cast<std::uint8_t>(source_poison)});
      fresh.memory.install(input.source, std::array{static_cast<std::uint8_t>(source_poison)});
    }
    move::finish(parent);
    move::finish(fresh);
    require(parent.cpu.state() == fresh.cpu.state() &&
                parent.cpu.prefetch_state() == fresh.cpu.prefetch_state() &&
                parent.memory.events == fresh.memory.events &&
                fresh.memory.bytes[input.destination] == move_value &&
                fresh.memory.write_widths == std::vector<unsigned>{1} &&
                fresh.memory.native_starts == 1 && fresh.memory.retired == 1,
            "New clear contract resumes the actual native latched MOV without rereading source");
  }
}
}

void inherited_stores() {
  word_snapshots();
  move_snapshots();
}
}
