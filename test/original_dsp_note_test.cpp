// SPDX-License-Identifier: GPL-3.0-only
// Execute original MIDI note dispatch up to its strict reserved-read boundary.
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <ostream>
#include <span>
#include <string>
#include <vector>

namespace {
using enum blofeld::DspSpace;
using blofeld::test::require;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned sound = 0x936;
constexpr unsigned entry = 0xc500;
constexpr unsigned entry_stride = 0x106;
constexpr unsigned entries = 25;
constexpr unsigned first_state = 0x3e;
constexpr unsigned idle_callback = 0x160e;
constexpr unsigned list = 0x6d2;
constexpr unsigned list_end = 0x6eb;
constexpr unsigned ring = 0x2f80;
constexpr unsigned ring_size = 64;
constexpr unsigned producer = 0x3b;
constexpr unsigned consumer = 0x3c;
constexpr unsigned parser = 0xc53;
constexpr unsigned note_handler = 0xd60;
constexpr unsigned note_begin = 0x116e;
constexpr unsigned velocity_transform = 0x126b;
constexpr unsigned allocator = 0x4c7;
constexpr unsigned reserved_read = 0x4ce;
constexpr unsigned entry_setup = 0xe58;
constexpr unsigned voice = 0x4df;
constexpr unsigned register_prefix = 0x10b;
constexpr unsigned register_end = 0x11d;
constexpr unsigned pointer_prefix = 0x17e;
constexpr unsigned pointer_end = 0x185;
constexpr unsigned controller_reset = 0xd1e;
constexpr unsigned list_initialize = 0x44a;
constexpr unsigned event_initialize = 0x1bf4;
constexpr unsigned block_initialize = 0x1acd;
constexpr unsigned factory_sound = 0x1e25;
constexpr unsigned raw_sound_words = 383;
constexpr unsigned sound_words = 427;
constexpr unsigned history_offset = 406;
constexpr unsigned history_words = 16;
constexpr unsigned event_base = 0x175a;
constexpr unsigned event_stride = 83;
constexpr unsigned queued_events = 68;
constexpr unsigned selector = 0x800;
constexpr unsigned caller_scratch = 0x700;
constexpr unsigned channel_setting = 0x744;
constexpr unsigned clock_setting = 0x750;
constexpr unsigned velocity_setting = 0x752;
constexpr unsigned staged_status = 0x24;
constexpr unsigned staged_note = 0x25;
constexpr unsigned staged_velocity = 0x26;
constexpr unsigned byte_shift = 16;
constexpr unsigned velocity_shift = 8;
constexpr unsigned midi_max = 127;
constexpr unsigned byte_mask = 255;
constexpr unsigned channel_count = 16;
constexpr unsigned note_on = 0x90;
constexpr unsigned note_off = 0x80;
constexpr unsigned type_mask = 0xf0;
constexpr unsigned guard = 0x13579b;
constexpr unsigned original_records = 127;
constexpr unsigned instruction_limit = 4096;
constexpr unsigned entry_adjust = 0x105;

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected strict original note rejection");
}

struct Counts {
  unsigned handlers = 0;
  unsigned begins = 0;
  unsigned transforms = 0;
  unsigned allocations = 0;
  unsigned setups = 0;
  unsigned voices = 0;
};

struct NoteInput {
  unsigned note;
  unsigned velocity;
  unsigned channel;
};

struct Message {
  unsigned status;
  unsigned note;
  unsigned velocity;
};

enum class Delivery : std::uint8_t { complete, fragmented, running };

struct Fixture {
  Fixture(const blofeld::Firmware &firmware, std::ostream &trace) : trace_(trace) {
    require(firmware.dsp.size() == original_records, "All ordered original records are supplied");
    auto &bus = machine.bus();
    bus.observe_omr(memory_mode);
    bus.advance_instruction_cycles(3);
    for (const auto &record : firmware.dsp) {
      for (unsigned i = 0; i < record.words.size(); ++i) {
        write(static_cast<blofeld::DspSpace>(record.space), record.address + i, record.words[i]);
        if (record.space == 0) { original_program_[record.address + i] = record.words[i]; }
      }
    }
    harness();
    machine.cpu().reset_spi_boot();
    run(register_prefix, register_end);
    machine.cpu().reset_spi_boot();
    for (unsigned i = 0; i < sound_words; ++i) {
      write(x, sound + i, i < raw_sound_words ? read(y, factory_sound + i) & byte_mask : 0U);
    }
    run(controller_reset);
    for (unsigned slot = 0; slot < entries; ++slot) {
      for (unsigned i = 8; i < entry_stride; ++i) {
        write(x, entry + (slot * entry_stride) + i, 0);
      }
    }
    for (unsigned at = list; at <= list_end; ++at) { write(x, at, 0); }
    for (unsigned block = 0; block < channel_count; ++block) {
      write(y, event_base + (block * event_stride) + queued_events, 0);
    }
    run(list_initialize);
    run(event_initialize);
    run(block_initialize);
    run(pointer_prefix, pointer_end);
    machine.cpu().reset_spi_boot();
    // Only declared caller settings and transport readability are host inputs.
    write(x, 6, 0);
    write(x, channel_setting, 0);
    write(x, clock_setting, 1);
    write(x, velocity_setting, 0);
    for (unsigned i = 0; i < ring_size; ++i) { write(x, ring + i, guard ^ i); }
    require(read(x, producer) == ring && read(x, consumer) == ring,
            "Original primary ring starts empty");
    run(parser); // Execute the own memory switch and genuine empty-ring return after reset.
    verify_unallocated();
    inputs_only_ = true;
  }

  void write(const blofeld::DspSpace space, const unsigned at, const unsigned value) {
    require(!inputs_only_ ||
                (space == x && (at == selector || at == channel_setting || at == producer ||
                                (at >= ring && at < ring + ring_size))),
            "Host may supply only declared channel/transport/caller inputs after setup");
    machine.bus().write({.space = space, .address = at, .value = value});
  }

  unsigned read(const blofeld::DspSpace space, const unsigned at) {
    return machine.bus().read({.space = space, .address = at});
  }

  void send(const std::span<const unsigned> bytes) {
    auto cursor = read(x, producer);
    require(cursor == read(x, consumer) && bytes.size() < ring_size,
            "Own transport starts with an empty bounded ring");
    for (const auto byte : bytes) {
      require(byte <= byte_mask, "Own transport supplies MIDI bytes");
      write(x, cursor, byte << byte_shift);
      cursor = ring + ((cursor - ring + 1) % ring_size);
    }
    write(x, producer, cursor);
    run(parser);
    require(read(x, consumer) == cursor && machine.cpu().state().sp == 0,
            "Original parser drains the ring and returns through its actual frame");
  }

  void clear_observations() {
    counts = {};
    failed_state_.reset();
    accesses_.clear();
  }

  void verify_unallocated() {
    for (unsigned at = list; at <= list_end; ++at) {
      require(read(x, at) == 0, "No original active entry was inserted");
    }
    for (unsigned slot = 0; slot < entries; ++slot) {
      const auto base = entry + (slot * entry_stride);
      for (unsigned i = 8; i < entry_stride; ++i) {
        const auto expected = i == first_state || i == first_state + 1 ? idle_callback : 0U;
        require(read(x, base + i) == expected,
                "Original slot payload remains at its declared initialized state");
      }
    }
    for (const auto &[at, value] : original_program_) {
      require(machine.bus().peek_program(at) == value, "Every original executable word is intact");
    }
  }

  void verify_history(const unsigned note, const unsigned velocity) {
    for (unsigned i = 0; i < history_words; ++i) {
      const auto expected =
          i == 0 && velocity != 0 ? (note << byte_shift) | (velocity << velocity_shift) : 0U;
      require(read(x, sound + history_offset + i) == expected,
              "Original packed note/velocity history differs from independent MIDI fields");
    }
  }

  void verify_boundary(const std::string &error, const NoteInput input) {
    const auto [note, velocity, channel] = input;
    require(error.contains("pc=0x0004ce") && error.contains("omr=0x600389") &&
                error.contains("Unimplemented DSP peripheral read at Y:0xffffff"),
            "Actual original note allocation must expose the exact reserved-read context");
    if (!failed_state_.has_value()) {
      throw blofeld::Fault("The actual failed instruction has no pre-step observation");
    }
    const auto &state = failed_state_.value();
    require(state.pc == reserved_read && state.r[0] == sound && state.r[1] == word_mask &&
                state.r[7] == entry + first_state && state.m[1] == word_mask &&
                state.n[7] == entry_adjust && state.sp == 8 && state.lc == entries,
            "First original allocator iteration retains its genuine call/loop/address context");
    require(counts.handlers == 1 && counts.begins == 1 && counts.transforms == 1 &&
                counts.allocations == 1 && counts.setups == 0 && counts.voices == 0,
            "The real note and velocity handlers reach allocation before entry setup or rendering");
    require(read(x, 3) == ((note_on | channel) << byte_shift) &&
                read(x, staged_status) == (note_on << byte_shift) &&
                read(x, staged_note) == (note << byte_shift) &&
                read(x, staged_velocity) == (velocity << byte_shift) &&
                read(x, consumer) == read(x, producer),
            "Original single-block dispatch stages and consumes the independently supplied note");
    verify_accesses();
    verify_history(note, velocity);
    verify_unallocated();
    static_cast<void>(rejected([&] { machine.cpu().step(); }));
    static_cast<void>(rejected([&] { static_cast<void>(machine.checkpoint()); }));
    ++frontiers;
    trace_ << "FAULT note=" << note << " velocity=" << velocity << " channel=" << channel << " "
           << error << '\n';
  }

  blofeld::DspMachine machine;
  Counts counts;
  unsigned frontiers = 0;
  unsigned controls = 0;
  unsigned fragments = 0;

  void verify_completed(const Message input, const bool filtered) {
    require(counts.handlers == (filtered ? 0U : 1U) &&
                counts.begins == (!filtered && (input.status & type_mask) == note_on ? 1U : 0U) &&
                counts.transforms == 0 && counts.allocations == 0 && counts.setups == 0 &&
                counts.voices == 0,
            "Filtered or release-only note returns without allocating or rendering");
    verify_history(0, 0);
    verify_unallocated();
    ++controls;
    trace_ << "CONTROL status=" << input.status << " note=" << input.note
           << " velocity=" << input.velocity << " selector=" << read(x, channel_setting) << '\n';
  }

  void harness() {
    blofeld::test::DspProgram code;
    code.immediate("x1", memory_mode);
    code.emit("move x1,omr");
    for (unsigned i = 0; i < 3; ++i) { code.emit("nop"); }
    const auto begin = code.pc();
    for (unsigned i = 0; i < 8; ++i) { code.immediate("m" + std::to_string(i), word_mask); }
    code.immediate("sr", 0);
    code.immediate("b", 0); // The original per-block reset takes its block index in B1.
    code.immediate("r0", sound);
    code.immediate("r2", entry);
    code.immediate("r5", entry);
    code.immediate("r3", caller_scratch);
    code.immediate("r4", selector);
    code.immediate("r6", 0);
    code.immediate("r7", 0);
    code.emit("move x:>$800,r1");
    code.emit("jsr (r1)");
    stop_ = code.pc();
    code.emit("nop");
    code.word(absolute_jump);
    code.word(begin);
    code.install(machine.bus());
  }

  void observe(const blofeld::DspState &state) {
    counts.handlers += static_cast<unsigned>(state.pc == note_handler);
    counts.begins += static_cast<unsigned>(state.pc == note_begin);
    counts.transforms += static_cast<unsigned>(state.pc == velocity_transform);
    counts.allocations += static_cast<unsigned>(state.pc == allocator);
    counts.setups += static_cast<unsigned>(state.pc == entry_setup);
    counts.voices += static_cast<unsigned>(state.pc == voice);
  }

  void run(const unsigned target, const std::optional<unsigned> end = std::nullopt) {
    write(x, selector, target);
    unsigned steps = 0;
    const blofeld::DspNativeAccessObserver observer{
        .observe = [this](const blofeld::DspNativeAccess &access) {
          if (access.pc == reserved_read) { accesses_.push_back(access); }
        }};
    while (machine.cpu().state().pc != end.value_or(stop_) || steps == 0) {
      require(steps++ < instruction_limit, "Original note experiment instruction bound");
      const auto state = machine.cpu().state();
      if (target == parser) { observe(state); }
      try {
        machine.cpu().step(observer);
        machine.bus().advance_instruction_cycles(1);
      } catch (const blofeld::Fault &) {
        failed_state_ = state;
        throw;
      }
    }
    require(end.has_value() || machine.cpu().state().sp == 0,
            "Original complete initialization/parser call returns with its actual frame");
  }

  void verify_accesses() {
    using enum blofeld::DspNativeAccessKind;
    using enum blofeld::DspNativeAccessStage;
    unsigned completed_x = 0;
    unsigned pending_y = 0;
    for (const auto &access : accesses_) {
      if (access.kind == read && access.target.space == x) {
        require(pending_y == 0 && access.target.address == entry + first_state,
                "Original X state read precedes the reserved parallel Y operand");
        if (access.stage == after) {
          require(access.value == idle_callback, "Original slot state has its real reset value");
          ++completed_x;
        }
      } else if (access.kind == read && access.target.space == y) {
        require(completed_x == 1 && access.stage == before && access.target.address == word_mask &&
                    !access.value.has_value(),
                "Reserved Y read has no fabricated value or successful access completion");
        ++pending_y;
      } else {
        require(access.kind == opcode || access.kind == extension,
                "No write or unrelated operation occurs inside the rejected instruction");
      }
    }
    require(completed_x == 1 && pending_y == 1,
            "Actual access prefix contains one completed X and one rejected Y read");
  }

  std::ostream &trace_;
  std::map<unsigned, unsigned> original_program_;
  std::optional<blofeld::DspState> failed_state_;
  std::vector<blofeld::DspNativeAccess> accesses_;
  unsigned stop_ = 0;
  bool inputs_only_ = false;
};

void accepted(Fixture &fixture,
              const blofeld::DspCheckpoint &initial,
              const NoteInput input,
              const Delivery delivery = Delivery::complete) {
  const auto [note, velocity, channel] = input;
  fixture.machine.restore(initial);
  fixture.clear_observations();
  fixture.write(x, channel_setting, channel + 1);
  const std::array message{note_on | channel, note, velocity};
  unsigned offset = 0;
  if (delivery == Delivery::fragmented) {
    for (unsigned i = 0; i < 2; ++i) {
      fixture.send(std::span(message).subspan(i, 1));
      require(fixture.counts.handlers == 0 && fixture.counts.allocations == 0,
              "Incomplete original MIDI message cannot dispatch or allocate");
      fixture.verify_history(0, 0);
      fixture.verify_unallocated();
      ++fixture.fragments;
    }
    offset = 2;
  } else if (delivery == Delivery::running) {
    const std::array zero{note_on | channel, note, 0U};
    fixture.send(zero);
    fixture.verify_completed({.status = zero[0], .note = note, .velocity = 0}, false);
    fixture.clear_observations();
    offset = 1;
  }
  const auto error = rejected([&] { fixture.send(std::span(message).subspan(offset)); });
  fixture.verify_boundary(error, input);
}

void completed(Fixture &fixture,
               const blofeld::DspCheckpoint &initial,
               const Message input,
               const unsigned channel,
               const bool filtered) {
  fixture.machine.restore(initial);
  fixture.clear_observations();
  fixture.write(x, channel_setting, channel);
  const std::array message{input.status, input.note, input.velocity};
  fixture.send(message);
  fixture.verify_completed(input, filtered);
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Usage: original-dsp-note-test original-firmware trace");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    Fixture fixture(firmware, trace);
    const auto initial = fixture.machine.checkpoint();
    for (const unsigned note : {0U, 64U, midi_max}) {
      for (const unsigned velocity : {1U, 64U, midi_max}) {
        accepted(fixture, initial, {.note = note, .velocity = velocity, .channel = 0});
      }
      completed(fixture, initial, {.status = note_on, .note = note, .velocity = 0}, 0, false);
      completed(fixture, initial, {.status = note_off, .note = note, .velocity = 64}, 0, false);
    }
    for (unsigned channel = 0; channel < channel_count; ++channel) {
      accepted(fixture, initial, {.note = 64, .velocity = 64, .channel = channel});
      completed(fixture,
                initial,
                {.status = note_on | channel, .note = 64, .velocity = 64},
                ((channel + 1) % channel_count) + 1,
                true);
    }
    accepted(fixture, initial, {.note = 64, .velocity = 64, .channel = 0}, Delivery::fragmented);
    accepted(fixture, initial, {.note = 64, .velocity = 64, .channel = 0}, Delivery::running);
    trace << "SUMMARY allocation-frontiers=" << fixture.frontiers
          << " completed-controls=" << fixture.controls
          << " fragmented-prefixes=" << fixture.fragments << '\n';
    std::cout << "Original MIDI note dispatch reaches the strict reserved-read frontier.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
