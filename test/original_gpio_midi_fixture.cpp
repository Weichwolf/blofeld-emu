// SPDX-License-Identifier: GPL-3.0-only
#include "original_gpio_midi_fixture.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <string>

namespace blofeld::test::gpio_midi {
namespace {
constexpr unsigned vector = 0x10;
constexpr unsigned producer_entry = 0x2662;
constexpr unsigned producer_end = 0x2677;
constexpr unsigned parameter_program = 0x808;
constexpr unsigned message_program = 0x967;
constexpr unsigned secondary_entry = 0xc5d;
constexpr unsigned producer = 0x32;
constexpr unsigned consumer = 0x33;
constexpr unsigned stamp = 0x1f;
constexpr unsigned flags = 6;
constexpr unsigned field = 0xab6;
constexpr unsigned primary_context = 0x22;
constexpr unsigned secondary_context = 0x14f6;
constexpr unsigned primary_first = 0xc53;
constexpr unsigned idle_return = 0xc5c;
constexpr unsigned first_data = 0xbe4;
constexpr unsigned second_data = 0xbe9;
constexpr unsigned callback = 0xcbb;
constexpr unsigned callback_return = 0xc05;
constexpr unsigned selector = 0x50;
constexpr unsigned channel_setting = 0x744;
constexpr unsigned clock_setting = 0x750;
constexpr unsigned scratch_a = 0x36;
constexpr unsigned scratch_r = 0x37;
constexpr unsigned scratch_x = 0x38;
constexpr unsigned clock_byte = 0xf8;
constexpr unsigned switched = 0x600085;
constexpr unsigned status = 0xc00000;
constexpr unsigned alias = 0xf000;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned native_rom = 0xff0000;
constexpr unsigned all_bits = 255;
constexpr unsigned step_limit = 100000;
constexpr unsigned producer_steps_per_byte = 19;

bool selected(const DspRecord &record) {
  return record.space == 0 &&
         (record.address == vector || record.address == producer_entry ||
          record.address == parameter_program || record.address == message_program);
}
}

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

Fixture::Fixture(const Firmware &firmware, const Scenario scenario)
    : scenario_(scenario),
      firmware_(firmware),
      source_(make_source(scenario)),
      peer_(firmware, scenario, source_) {
  install();
  prepare();
}

void Fixture::install() {
  unsigned programs = 0;
  for (const auto &record : firmware_.dsp) {
    if (record.space == 0) {
      if (!selected(record)) { continue; }
      ++programs;
    } else {
      const auto end = record.space == 1 ? 0x3000U : 0x2000U;
      if (record.address + record.words.size() > end) { continue; }
    }
    for (std::size_t index = 0; index < record.words.size(); ++index) {
      const auto address = record.address + static_cast<unsigned>(index);
      const bool mapped = record.space == 0 && address >= 0x1000;
      dsp_.bus().write({.space = mapped ? DspSpace::x : static_cast<DspSpace>(record.space),
                        .address = mapped ? address + alias : address,
                        .value = record.words[index]});
    }
  }
  require(programs == 4,
          "Four complete unchanged final original vector/GPIO/parser/parameter records load");
}

void Fixture::prepare() {
  auto &bus = dsp_.bus();
  require(read(producer) == ring && read(consumer) == ring,
          "Original secondary ring cursors start at their real base");
  for (unsigned index = 0; index < ring_words; ++index) {
    stats_.expected[index] = guard ^ index;
    bus.write({.space = DspSpace::x, .address = ring + index, .value = stats_.expected[index]});
  }
  for (const auto address : {ring - 1, ring + ring_words, field - 1, field + 1}) {
    bus.write({.space = DspSpace::x, .address = address, .value = guard});
  }
  bus.write({.space = DspSpace::x, .address = stamp, .value = field_seed});
  bus.write({.space = DspSpace::x, .address = flags, .value = 0});
  bus.write({.space = DspSpace::x, .address = field, .value = field_seed});
  bus.write({.space = DspSpace::x, .address = channel_setting, .value = scenario_.channel});
  bus.write({.space = DspSpace::x, .address = clock_setting, .value = 1});
  bus.write({.space = DspSpace::x, .address = selector, .value = secondary_entry});
  bus.observe_irqa(peer_.irq_level());
  bus.gpio(DspPort::c).set_drive({.floating = all_bits});
  DspProgram program;
  program.immediate("omr", switched);
  program.emit("nop");
  program.emit("nop");
  program.emit("nop");
  const auto mapped = program.pc();
  program.immediate("sr", status);
  for (unsigned index = 0; index < 8; ++index) {
    program.immediate("m" + std::to_string(index), word_mask);
  }
  program.emit("movep #>$f00,x:<<$ffffbe");
  program.emit("movep #>0,x:<<$ffffbd");
  program.emit("movep #>$ff,x:<<$ffffbf");
  program.emit("movep #>6,x:<<$ffffff");
  loop_ = program.pc();
  program.emit("move x:>$50,r6");
  program.emit("jsr (r6)");
  complete_ = program.pc();
  program.emit("nop");
  program.word(absolute_jump);
  program.word(loop_);
  program.install(bus);
  for (unsigned count = 0; dsp_.cpu().state().pc != loop_; ++count) {
    require(count < ring_words, "Own native map/priority/GPIO caller setup terminates");
    if (dsp_.cpu().state().pc == mapped) { bus.advance_instruction_cycles(3); }
    dsp_.cpu().step();
  }
  peer_.synchronize(bus);
}

unsigned Fixture::read(const unsigned address) {
  return dsp_.bus().read({.space = DspSpace::x, .address = address});
}

void Fixture::observe_producer(const DspNativeAccess &access) {
  require(access.target.space == DspSpace::x && access.value.has_value(),
          "Native producer writes require known X values");
  const auto address = access.target.address;
  if (address >= ring && address < ring + ring_words) {
    require(address == ring + (stats_.received % ring_words) &&
                access.value == peer_.byte(stats_.received) << 16U,
            "Actual GPIO ring store must equal the independent next native-sent byte and cursor");
    ++stats_.stores;
  } else if (address == stamp) {
    require(peer_.byte(stats_.received) == clock_byte && access.value == 0,
            "Actual F8 timestamp samples the real reset Timer2 counter");
    ++stats_.timestamps;
  } else {
    require(address == producer || address == scratch_a || address == scratch_r ||
                address == scratch_x,
            "Unexpected original producer write pc=" + hex(access.pc) + " address=" + hex(address));
  }
}

void Fixture::observe_parser(const DspNativeAccess &access) {
  const auto address = access.target.address;
  const bool allowed = access.target.space == DspSpace::x
                           ? address == 3 || address == flags || address == consumer ||
                                 address == field ||
                                 (address >= primary_context && address < primary_context + 5)
                           : access.target.space == DspSpace::y && address >= secondary_context &&
                                 address < secondary_context + 5;
  require(allowed,
          "Unexpected original parser write pc=" + hex(access.pc) + " address=" + hex(address));
  if (access.target.space == DspSpace::x && address == consumer) {
    require(access.value == ring + ((stats_.consumed + 1) % ring_words),
            "Native consumer advances exactly one actual queue word");
    ++stats_.consumed;
  }
  if (access.target.space == DspSpace::x && address == field) {
    require(stats_.pitches < source_.pitches.size() &&
                access.value == source_.pitches[stats_.pitches],
            "Each actual pitch-field write matches independent channel-filtered fourteen-bit MIDI "
            "math");
    ++stats_.pitches;
  }
}

void Fixture::observe(const DspNativeAccess &access) {
  if (access.stage != DspNativeAccessStage::after) { return; }
  if (access.kind == DspNativeAccessKind::opcode && access.target.space == DspSpace::p &&
      access.target.address == callback) {
    const auto state = dsp_.cpu().state();
    require(state.sp == 2 && state.stack_pc == callback_return,
            "Original dispatcher uses its real nested callback frame");
    ++stats_.callbacks;
  }
  if (access.kind != DspNativeAccessKind::write) { return; }
  if (access.pc >= producer_entry && access.pc < producer_end) {
    observe_producer(access);
  } else {
    observe_parser(access);
  }
}

void Fixture::dsp_step() {
  const auto before = dsp_.cpu().state();
  if (before.pc >= producer_entry && before.pc < producer_end) { ++stats_.producer_steps; }
  dsp_.cpu().step({.observe = [this](const DspNativeAccess &access) { observe(access); }});
  ++stats_.steps;
  const auto after = dsp_.cpu().state();
  if (after.pc == producer_entry && after.sp == before.sp + 1) {
    require(!stats_.interrupted && after.stack_pc == before.pc && after.stack_sr == before.sr,
            "Real IRQA vector pushes the actual interrupted native foreground state");
    stats_.interrupted = before;
    ++stats_.irqs;
    ++stats_.producer_steps;
    if (before.pc < native_rom) { ++stats_.preemptions; }
  }
  if (!stats_.interrupted || after.sp != stats_.interrupted->sp ||
      after.pc != stats_.interrupted->pc) {
    return;
  }
  require(after == *stats_.interrupted && stats_.stores == stats_.received + 1,
          "Real original RTI restores every interrupted visible CPU register after one store");
  stats_.interrupted.reset();
  stats_.expected[stats_.received % ring_words] = peer_.byte(stats_.received) << 16U;
  ++stats_.received;
  ++stats_.returns;
  require(read(producer) == ring + (stats_.received % ring_words),
          "Actual producer cursor wraps independently modulo128");
  for (unsigned index = 0; index < ring_words; ++index) {
    require(read(ring + index) == stats_.expected[index],
            "Every ring cell retains the independent input or untouched sentinel");
  }
}

void Fixture::step() {
  require(stats_.steps < step_limit, "Native concurrent producer/parser loop remains bounded");
  if (stats_.phase == 0 && !peer_.done()) {
    peer_.step();
    peer_.synchronize(dsp_.bus());
  }
  dsp_step();
  stats_.phase = (stats_.phase + 1) % scenario_.ratio;
}

bool Fixture::complete() {
  return peer_.done() && stats_.received == source_.bytes.size() &&
         stats_.consumed == source_.bytes.size() && dsp_.cpu().state().pc == complete_ &&
         dsp_.cpu().state().sp == 0 && !stats_.interrupted;
}

void Fixture::run() {
  while (!complete()) { step(); }
  verify();
}

bool Fixture::checkpointable() {
  return stats_.received >= source_.bytes.size() / 2 && stats_.interrupted &&
         stats_.interrupted->pc < native_rom && stats_.interrupted->sp > 0 &&
         (read(primary_context) == second_data ||
          dsp_.bus().read({.space = DspSpace::y, .address = secondary_context}) == second_data);
}

Fixture::Saved Fixture::save() {
  require(checkpointable(),
          "Joint snapshot requires a real in-flight IRQ and partial parser message");
  return {.peer = peer_.save(), .dsp = dsp_.checkpoint(), .stats = stats_};
}

void Fixture::restore(const Saved &saved) {
  peer_.restore(saved.peer);
  dsp_.restore(saved.dsp);
  stats_ = saved.stats;
}

void Fixture::verify() {
  peer_.verify();
  require(complete() && stats_.irqs == source_.bytes.size() && stats_.returns == stats_.irqs &&
              stats_.stores == stats_.irqs && stats_.timestamps == clock_count &&
              stats_.producer_steps ==
                  (producer_steps_per_byte * stats_.irqs) + (2 * clock_count) &&
              stats_.preemptions > 0,
          "Native IRQ/store/RTI/timestamp totals and independently counted original ISR "
          "instructions match");
  require(
      stats_.callbacks == source_.pitches.size() && stats_.pitches == source_.pitches.size() &&
          read(field) == source_.pitches.back() && read(flags) == 0 && read(stamp) == 0 &&
          read(consumer) == read(producer) && !dsp_.bus().interrupt_request(0),
      "Original consumer/callbacks drain and return with the independently expected final field");
  for (const auto address : {ring - 1, ring + ring_words, field - 1, field + 1}) {
    require(read(address) == guard, "Actual native processing preserves ring and field boundaries");
  }
  const std::array primary{primary_first, idle_return, 0U, 0U, 0U};
  const std::array secondary{first_data, callback, 0xef0000U, 0x7f0000U, 0x7f0000U};
  for (unsigned index = 0; index < primary.size(); ++index) {
    require(read(primary_context + index) == primary[index] &&
                dsp_.bus().read({.space = DspSpace::y, .address = secondary_context + index}) ==
                    secondary[index],
            "Primary context stays idle; real secondary context retains final running status/data");
  }
  for (const auto &record : firmware_.dsp) {
    if (!selected(record)) { continue; }
    for (unsigned index = 0; index < record.words.size(); ++index) {
      require(dsp_.bus().peek_program(record.address + index) == record.words[index],
              "All four original program images remain unchanged");
    }
  }
}

Fixture::Result Fixture::result() {
  Result result{
      .peer = peer_.result(), .dsp = dsp_.cpu().debug_state(), .memory = {}, .stats = stats_};
  for (unsigned index = 0; index < ring_words; ++index) {
    result.memory.push_back(read(ring + index));
  }
  for (const auto address : {producer,
                             consumer,
                             stamp,
                             flags,
                             field,
                             field - 1,
                             field + 1,
                             ring - 1,
                             ring + ring_words}) {
    result.memory.push_back(read(address));
  }
  for (unsigned index = 0; index < 5; ++index) {
    result.memory.push_back(read(primary_context + index));
    result.memory.push_back(
        dsp_.bus().read({.space = DspSpace::y, .address = secondary_context + index}));
  }
  return result;
}
}
