// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_rom_program.hpp"
#include "esai_fixture.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {
using blofeld::DspSpace;
using blofeld::test::DspRomRegister;
using blofeld::test::esai::require;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned masked_status = 0xc00300;
constexpr unsigned stack_shift = 24;
// DSP56300FM Rev.5 MOVE ea/R6 and JSR direct R6, with a full X address extension.
constexpr unsigned load_selector = 0x66f000;
constexpr unsigned call_r6 = 0x0be680;
using NativeState = std::pair<dsp56300::Registers, dsp56300::Progress>;
constexpr unsigned parameter_program = 0x808;
constexpr unsigned message_program = 0x967;
constexpr unsigned primary_entry = 0xc53;
constexpr unsigned secondary_entry = 0xc5d;
constexpr unsigned idle_entry = 0xc5c;
constexpr unsigned first_byte = 0xbe4;
constexpr unsigned second_byte = 0xbe9;
constexpr unsigned paired_callback = 0xcbb;
constexpr unsigned callback_return = 0xc05;
constexpr unsigned primary_context = 0x22;
constexpr unsigned secondary_context = 0x14f6;
constexpr unsigned flags_address = 6;
constexpr unsigned channel_setting = 0x744;
constexpr unsigned clock_setting = 0x750;
constexpr unsigned harness_selector = 0x50;
constexpr unsigned field_address = 0xab6;
constexpr unsigned field_sentinel = 0x13579b;
constexpr unsigned guard_value = 0x2468ac;
constexpr unsigned byte_shift = 16;
constexpr unsigned midi_center = 8192;
constexpr unsigned midi_values = 16384;
constexpr std::int32_t midi_scale = 1024;
constexpr unsigned maximum_byte = 255;
constexpr unsigned data_mask = 127;
constexpr unsigned status_minimum = 0x80;
constexpr unsigned pitch_status = 0xe0;
constexpr unsigned type_mask = 0xf0;
constexpr unsigned realtime_minimum = 0xf8;
constexpr unsigned channel_count = 16;
constexpr unsigned step_limit = 2048;
constexpr unsigned progress_interval = 4096;
constexpr unsigned absolute_jump = 0x0af080; // FM long absolute JMP plus one extension word.
constexpr unsigned x_ram_end = 0x3000;
constexpr unsigned y_ram_end = 0x2000;
constexpr unsigned replay_primary_lsb = 0x13;
constexpr unsigned replay_secondary_lsb = 0x25;
constexpr unsigned replay_primary_msb = 0x61;
constexpr unsigned replay_secondary_msb = 0x41;
constexpr std::array missing_input_pc{0xc5aU, 0xc6aU};

struct Ring {
  unsigned base;
  unsigned size;
  unsigned producer;
  unsigned consumer;
  unsigned entry;
};

constexpr std::array rings{
    Ring{.base = 0x2f80, .size = 64, .producer = 0x3b, .consumer = 0x3c, .entry = primary_entry},
    Ring{
        .base = 0x2d00, .size = 128, .producer = 0x32, .consumer = 0x33, .entry = secondary_entry}};

struct Context {
  unsigned status = 0;
  unsigned data1 = 0;
  unsigned data2 = 0;
  bool first_received = false;
};

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected strict missing-input rejection");
}

unsigned bend_word(const unsigned lsb, const unsigned msb) {
  const auto centered =
      static_cast<std::int32_t>(lsb + (128 * msb)) - static_cast<std::int32_t>(midi_center);
  return static_cast<std::uint32_t>(centered * midi_scale) & word_mask;
}

void install(blofeld::DspBus &bus, const blofeld::Firmware &firmware) {
  unsigned programs = 0;
  for (const auto &record : firmware.dsp) {
    if (record.space == 0) {
      if (record.address != parameter_program && record.address != message_program) { continue; }
      ++programs;
    } else {
      const auto limit = record.space == 1 ? x_ram_end : y_ram_end;
      if (record.address + record.words.size() > limit) { continue; }
    }
    for (std::size_t index = 0; index < record.words.size(); ++index) {
      bus.write({.space = static_cast<DspSpace>(record.space),
                 .address = record.address + static_cast<unsigned>(index),
                 .value = record.words[index]});
    }
  }
  require(programs == 2, "Both complete original parser/parameter program records are supplied");
}

class Fixture {
public:
  explicit Fixture(const blofeld::Firmware &firmware,
                   const std::optional<unsigned> missing = std::nullopt) {
    install(machine_.bus(), firmware);
    for (const auto ring : rings) {
      require(read(ring.producer) == ring.base && read(ring.consumer) == ring.base,
              "Original initialized ring cursors agree with the reviewed source");
      for (unsigned offset = 0; offset < ring.size; ++offset) {
        if (missing != ring.base + offset) { write(ring.base + offset, guard_value ^ offset); }
      }
      write(ring.base - 1, guard_value);
      write(ring.base + ring.size, guard_value);
    }
    write(flags_address, 0); // Explicit single-block caller context, not a boot observation.
    write(channel_setting, 0);
    write(clock_setting, 1); // Declared setting ignores MIDI clock without replacing its code.
    write(field_address, field_sentinel);
    write(field_address - 1, guard_value);
    write(field_address + 1, guard_value);
    blofeld::test::DspRomProgram code;
    for (unsigned index = 0; index < 8; ++index) {
      code.immediate(static_cast<DspRomRegister>(static_cast<unsigned>(DspRomRegister::m0) + index),
                     word_mask);
    }
    code.immediate(DspRomRegister::sr, masked_status);
    loop_ = code.pc();
    code.word(load_selector);
    code.word(harness_selector);
    code.word(call_r6);
    complete_ = code.pc();
    code.word(0); // FM NOP.
    code.word(absolute_jump);
    code.word(loop_);
    blofeld::test::dma_test::NativeProgram program;
    for (const auto word : code.words()) { program.word(word); }
    blofeld::test::dma_test::reset_native(machine_, program);
    for (unsigned setup_steps = 0; machine_.cpu().registers().pc != loop_; ++setup_steps) {
      require(setup_steps < step_limit, "Own harness setup reaches its genuine call loop");
      step();
    }
    verify();
    retirement_base_ = machine_.cpu().progress().instructions;
  }

  void channel(const unsigned value) {
    require(value <= channel_count, "Fixture channel is omni or one of sixteen selectors");
    channel_ = value;
    write(channel_setting, value);
  }

  void send(const unsigned selected, const std::span<const unsigned> bytes) {
    require(selected < rings.size(), "Fixture selects one actual ring");
    expected_updates_.clear();
    update_index_ = 0;
    callbacks_ = 0;
    const auto ring = rings.at(selected);
    auto cursor = read(ring.producer);
    require(cursor == read(ring.consumer),
            "Every preceding complete call drained its selected ring");
    require(bytes.size() < ring.size, "Own producer does not create an ambiguous full ring");
    for (const auto byte : bytes) {
      require(byte <= maximum_byte, "Own producer supplies complete MIDI bytes");
      write(cursor, byte << byte_shift);
      cursor = ring.base + ((cursor - ring.base + 1) % ring.size);
      model(contexts_.at(selected), byte);
    }
    write(ring.producer, cursor);
    write(harness_selector, ring.entry);
    for (unsigned steps = 0;; ++steps) {
      require(steps < step_limit,
              "Original ring/parser/callback must return within the fixed bound");
      const auto state = machine_.cpu().registers();
      if (state.pc == paired_callback) {
        require(state.sp == 2 && (state.stack.at(2) >> stack_shift) == callback_return,
                "Original dispatcher enters the genuine callback with its actual nested JSR frame");
        ++callbacks_;
      }
      step(true);
      ++instructions_;
      if (machine_.cpu().registers().pc == complete_) { break; }
    }
    const auto state = machine_.cpu().registers();
    require(
        state.sp == 0 && callbacks_ == expected_updates_.size() &&
            update_index_ == expected_updates_.size() && read(ring.consumer) == cursor,
        "Complete original callbacks return and consume exactly the independently expected input");
    completed_ += callbacks_;
    completed_by_ring_.at(selected) += callbacks_;
    bytes_ += bytes.size();
    ++calls_;
    if (calls_ % progress_interval == 0) {
      std::clog << "Original MIDI matrix: calls=" << calls_ << " callbacks=" << completed_
                << " retired=" << instructions_ << '\n';
    }
    require(machine_.cpu().progress().instructions == retirement_base_ + instructions_,
            "Every observed fixture step corresponds to one actual native instruction retirement");
    verify();
  }

  void summary(std::ostream &trace) const {
    constexpr unsigned expected_calls = 34576;
    constexpr unsigned expected_completions = 32873;
    constexpr unsigned expected_bytes = 70430;
    constexpr std::uint64_t expected_retired = 3101868;
    require(calls_ == expected_calls && completed_ == expected_completions &&
                bytes_ == expected_bytes && instructions_ == expected_retired,
            "The complete fixed matrix executes with its independently pinned coverage counts");
    trace << "calls=" << calls_ << " completed-pitch-messages=" << completed_
          << " primary-completions=" << completed_by_ring_.at(0)
          << " secondary-completions=" << completed_by_ring_.at(1) << " bytes=" << bytes_
          << " retired-fixture-instructions=" << instructions_ << '\n';
  }

  void checkpoint_checks(std::ostream &trace) {
    const std::array primary{pitch_status | 4U, replay_primary_lsb};
    const std::array secondary{pitch_status | 9U, replay_secondary_lsb};
    send(0, primary);
    send(1, secondary);
    const auto saved = machine_.checkpoint();
    const auto before_cpu = native_state();
    const auto before_contexts = contexts_;
    const auto before_field = expected_field_;
    const auto before_instructions = instructions_;
    const auto before_memory = signature();
    std::optional<NativeState> first_cpu;
    std::vector<unsigned> first_memory;
    for (unsigned replay = 0; replay < 3; ++replay) {
      machine_.restore(saved);
      contexts_ = before_contexts;
      expected_field_ = before_field;
      instructions_ = before_instructions;
      require(native_state() == before_cpu && signature() == before_memory,
              "Trusted snapshot restores both partial messages, every mutable cell and whole CPU");
      verify();
      const std::array secondary_finish{replay_secondary_msb};
      const std::array primary_finish{replay_primary_msb + (replay == 2 ? 1U : 0U)};
      send(1, secondary_finish);
      send(0, primary_finish);
      if (replay == 0) {
        first_cpu = native_state();
        first_memory = signature();
      } else if (replay == 1) {
        require(
            first_cpu.has_value() && native_state() == first_cpu.value() &&
                signature() == first_memory,
            "Identical actual parser/callback suffix replays every CPU and permitted RAM effect");
      } else {
        require(
            read(field_address) == bend_word(replay_primary_lsb, replay_primary_msb + 1) &&
                read(field_address) != bend_word(replay_primary_lsb, replay_primary_msb),
            "Changed continuation uses the restored original first byte and new MIDI second byte");
      }
    }
    trace << "mid-message-identical-replays=2 changed-continuations=1 actual-callbacks=6\n";
  }

  void missing_input_check(const unsigned selected, std::ostream &trace) {
    const auto ring = rings.at(selected);
    const std::array<unsigned, 0> empty{};
    const auto error = rejected([&] { send(selected, empty); });
    require(
        error.contains("Unknown DSP memory read at X:" + blofeld::hex(ring.base, 6)) &&
            error.contains("pc=" + blofeld::hex(missing_input_pc.at(selected), 6)) &&
            error.contains("omr=0x000305") && read(ring.consumer) == ring.base &&
            read(ring.producer) == ring.base && read(field_address) == field_sentinel &&
            machine_.cpu().registers().sp == 1 &&
            machine_.cpu().progress().instructions == retirement_base_ + instructions_,
        "Actual speculative empty-ring read fails contextually without consumption or retirement");
    static_cast<void>(rejected([&] { step(); }));
    const auto diagnostic = machine_.checkpoint();
    const auto failed_cpu = native_state();
    const auto failed_fault = machine_.cpu().fault();
    machine_.restore(diagnostic);
    require(native_state() == failed_cpu && machine_.cpu().fault() == failed_fault &&
                read(ring.consumer) == ring.base && read(ring.producer) == ring.base &&
                read(field_address) == field_sentinel,
            "Diagnostic snapshots preserve the actual fault, CPU and unconsumed input");
    static_cast<void>(rejected([&] { step(); }));
    trace << "strict-empty-ring-prefetch-fault ring=" << selected << " " << error << '\n';
  }

private:
  [[nodiscard]] NativeState native_state() const {
    return {machine_.cpu().registers(), machine_.cpu().progress()};
  }

  void healthy(const dsp56300::RunResult &progress) const {
    if (progress.fault.kind == dsp56300::FaultKind::none) { return; }
    std::string cause = "native MIDI instruction fault";
    const auto &failure = machine_.io().failure();
    if (failure && failure->cause) {
      try {
        std::rethrow_exception(failure->cause);
      } catch (const std::exception &error) { cause = error.what(); }
    }
    throw blofeld::Fault(cause + " pc=" + blofeld::hex(progress.fault.pc, 6) +
                         " omr=" + blofeld::hex(machine_.cpu().registers().omr, 6) +
                         " address=" + blofeld::hex(progress.fault.address, 6) +
                         " kind=" + std::to_string(static_cast<unsigned>(progress.fault.kind)));
  }

  void step(const bool watch = false) {
    const auto before = machine_.cpu().progress().instructions;
    const auto ticks = machine_.clock().elapsed_ticks();
    blofeld::DspIoWriteObserver observer;
    if (watch) {
      observer = {
          .context = this,
          .completed = [](void *context, const blofeld::DspStore &store, const unsigned pc) {
            static_cast<Fixture *>(context)->observe(store, pc);
          }};
    }
    const auto progress = machine_.run_unclocked(observer);
    healthy(progress);
    require(progress.dispatches == 1 && machine_.cpu().progress().instructions == before + 1 &&
                machine_.clock().elapsed_ticks() == ticks,
            "One native MIDI instruction retires without implicit clock periods");
  }

  std::vector<unsigned> signature() {
    std::vector<unsigned> result;
    for (const auto address : {3U, flags_address, field_address}) {
      result.push_back(read(address));
    }
    for (unsigned index = 0; index < 5; ++index) {
      result.push_back(read(primary_context + index));
      result.push_back(
          machine_.bus().read({.space = DspSpace::y, .address = secondary_context + index}));
    }
    for (const auto ring : rings) {
      result.push_back(read(ring.producer));
      result.push_back(read(ring.consumer));
      for (unsigned offset = 0; offset < ring.size; ++offset) {
        result.push_back(read(ring.base + offset));
      }
    }
    return result;
  }

  [[nodiscard]] unsigned read(const unsigned address) {
    return machine_.bus().read({.space = DspSpace::x, .address = address});
  }

  void write(const unsigned address, const unsigned value) {
    machine_.bus().write({.space = DspSpace::x, .address = address, .value = value});
  }

  void model(Context &context, const unsigned byte) {
    if (byte >= realtime_minimum) { return; }
    if (byte >= status_minimum) {
      require((byte & type_mask) == pitch_status, "This finite model qualifies pitch bend only");
      context.status = byte;
      context.first_received = false;
    } else if (context.status != 0) {
      if (!context.first_received) {
        context.data1 = byte;
        context.first_received = true;
      } else {
        context.data2 = byte;
        context.first_received = false;
        if (channel_ == 0 || (context.status & 15U) + 1 == channel_) {
          expected_field_ = bend_word(context.data1, byte);
          expected_updates_.push_back(expected_field_);
        }
      }
    }
  }

  void observe(const blofeld::DspStore &access, const unsigned pc) {
    const auto address = access.address;
    const bool allowed = access.space == DspSpace::x
                             ? address == 3 || address == flags_address ||
                                   (address >= primary_context && address < primary_context + 5) ||
                                   address == rings[0].consumer || address == rings[1].consumer ||
                                   address == field_address
                             : access.space == DspSpace::y && address >= secondary_context &&
                                   address < secondary_context + 5;
    require(allowed,
            "Unexpected original parser RAM write at " + blofeld::hex(address) +
                " pc=" + blofeld::hex(pc));
    if (access.space == DspSpace::x && address == field_address) {
      require(update_index_ < expected_updates_.size() &&
                  access.value == expected_updates_.at(update_index_),
              "Every actual original field write matches the independent fourteen-bit MIDI value");
      ++update_index_;
    }
  }

  void verify() {
    require(read(flags_address) == 0 && read(field_address) == expected_field_ &&
                read(field_address - 1) == guard_value && read(field_address + 1) == guard_value,
            "Declared caller mode, independent pitch value and adjacent guards remain exact");
    for (unsigned selected = 0; selected < rings.size(); ++selected) {
      const auto &context = contexts_.at(selected);
      auto input = primary_entry;
      if (context.status != 0) { input = context.first_received ? second_byte : first_byte; }
      const auto complete = context.status == 0 ? idle_entry : paired_callback;
      const std::array expected{input,
                                complete,
                                context.status << byte_shift,
                                context.data1 << byte_shift,
                                context.data2 << byte_shift};
      for (unsigned index = 0; index < expected.size(); ++index) {
        const auto actual =
            selected == 0
                ? read(primary_context + index)
                : machine_.bus().read({.space = DspSpace::y, .address = secondary_context + index});
        require(actual == expected.at(index),
                "Both independent parser contexts match message state");
      }
      require(read(rings.at(selected).base - 1) == guard_value &&
                  read(rings.at(selected).base + rings.at(selected).size) == guard_value,
              "Original ring wrapping preserves both external guards");
    }
  }

  blofeld::OwnDspMachine machine_{blofeld::test::dma_test::native_clocks};
  std::array<Context, 2> contexts_{};
  std::vector<unsigned> expected_updates_;
  std::size_t update_index_ = 0;
  unsigned channel_ = 0;
  unsigned expected_field_ = field_sentinel;
  unsigned loop_ = 0;
  unsigned complete_ = 0;
  unsigned callbacks_ = 0;
  std::uint64_t calls_ = 0;
  std::uint64_t completed_ = 0;
  std::uint64_t instructions_ = 0;
  std::uint64_t retirement_base_ = 0;
  std::uint64_t bytes_ = 0;
  std::array<std::uint64_t, 2> completed_by_ring_{};
};

void smoke(Fixture &fixture) {
  for (const auto ring : {0U, 1U}) {
    const std::array<unsigned, 0> empty{};
    fixture.send(ring, empty);
    for (const auto value : {0U, midi_center, midi_values - 1}) {
      const std::array message{pitch_status, value & data_mask, value >> 7};
      fixture.send(ring, message);
    }
  }
}

void orphan_data(Fixture &fixture) {
  for (const auto ring : {0U, 1U}) {
    for (unsigned value = 0; value <= data_mask; ++value) {
      const std::array byte{value};
      fixture.send(ring, byte);
    }
  }
}

void interleaved(Fixture &fixture) {
  const std::array primary_start{pitch_status | 2U, 5U};
  const std::array secondary_start{pitch_status | 7U, 11U};
  fixture.send(0, primary_start);
  fixture.send(1, secondary_start);
  const std::array clock{realtime_minimum};
  fixture.send(0, clock);
  const std::array secondary_finish{realtime_minimum, 9U};
  fixture.send(1, secondary_finish);
  const std::array primary_finish{12U};
  fixture.send(0, primary_finish);
  const std::array incomplete{4U};
  fixture.send(0, incomplete);
  const std::array replacement{pitch_status | 5U};
  fixture.send(0, replacement);
  const std::array replacement_data{6U, 7U};
  fixture.send(0, replacement_data);
}

void running_values(Fixture &fixture) {
  for (const auto ring : {0U, 1U}) {
    for (unsigned value = 0; value < midi_values; ++value) {
      const std::array bytes{value & data_mask, value >> 7};
      fixture.send(ring, bytes);
    }
  }
}

void filtered(Fixture &fixture) {
  for (unsigned setting = 1; setting <= channel_count; ++setting) {
    fixture.channel(setting);
    for (const auto ring : {0U, 1U}) {
      for (unsigned channel = 0; channel < channel_count; ++channel) {
        for (const auto value : {0U, midi_center, midi_values - 1}) {
          const std::array bytes{pitch_status | channel, value & data_mask, value >> 7};
          fixture.send(ring, bytes);
        }
      }
    }
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Usage: original-dsp-midi-test original-firmware trace");
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    const auto firmware = blofeld::Firmware::load(std::filesystem::path(arguments[1]));
    {
      Fixture fixture(firmware);
      orphan_data(fixture);
      smoke(fixture);
      interleaved(fixture);
      running_values(fixture);
      filtered(fixture);
      fixture.summary(trace);
    }
    {
      Fixture fixture(firmware);
      fixture.checkpoint_checks(trace);
    }
    for (unsigned ring = 0; ring < rings.size(); ++ring) {
      Fixture fixture(firmware, rings.at(ring).base);
      fixture.missing_input_check(ring, trace);
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
