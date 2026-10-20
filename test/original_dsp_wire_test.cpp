// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using blofeld::DspSpace;
using blofeld::test::require;
constexpr unsigned original_program = 0x2cd9;
constexpr unsigned physical_alias_offset = 0xf000;
constexpr unsigned ordinary_status = 0xc00000;
constexpr unsigned control_without_di = 0x3fefff;
constexpr unsigned status_handler = 0x2cf9;
constexpr unsigned original_vector = 0x54;
constexpr unsigned control = 0xffff8f;
constexpr unsigned compare = control - 2;
constexpr unsigned counter = control - 3;
constexpr unsigned priority = 0xfffffe;
constexpr unsigned capture_control = 0x165;
constexpr unsigned producer = 0x3b;
constexpr unsigned ring_base = 0x2f80;
constexpr unsigned step_limit = 128;
constexpr unsigned jump = 0x0af080;
constexpr unsigned switch_omr = 0x600085;
constexpr unsigned input_handler = 0x2ce2;
constexpr unsigned first_compare = 0xfff350;
constexpr unsigned first_load = 0xffe048;
constexpr unsigned bit_period = 3249;
constexpr unsigned first_delay = first_compare - first_load + 1;
constexpr unsigned first_overflow_delay = 0x1000000 - first_compare;
constexpr unsigned ring_words = 64;
constexpr unsigned ring_guard = 0x2468ac;
constexpr unsigned state_word = 0x3f;
constexpr unsigned saved_accumulator = 0x3d;
constexpr unsigned sample_seed = 0x80;
constexpr unsigned timestamp = 0x1f;
constexpr unsigned timestamp_seed = 0x13579b;
constexpr unsigned clock_byte = 0xf8;
constexpr unsigned data_shift = 16;
constexpr unsigned byte_bits = 8;
constexpr unsigned ordinary_control = 5;
constexpr unsigned sampling_control = 0x203;
constexpr unsigned source_match = 0x200000;
constexpr unsigned peripheral_base = 0xffff00;
constexpr unsigned expected_irq_count = 27;
constexpr unsigned capture_return = 0x2d03;
constexpr unsigned sample_return = 0x2d10;
constexpr unsigned input_return = 0x2cf8;
constexpr unsigned parameters = 0x808;
constexpr unsigned x_low_end = 0x3000;
constexpr unsigned y_low_end = 0x2000;
constexpr unsigned parser_steps = 2048;
constexpr unsigned consumer = 0x3c;
constexpr unsigned messages = 0x967;
constexpr unsigned parser_entry = 0xc53;
constexpr unsigned callback_entry = 0xcbb;
constexpr unsigned callback_return = 0xc05;
constexpr unsigned harness_selector = 0x50;
constexpr unsigned field = 0xab6;
constexpr unsigned field_seed = 0x246813;
constexpr unsigned mode_flags = 6;
constexpr unsigned midi_channel = 0x744;
constexpr unsigned midi_clock = 0x750;
constexpr unsigned midi_center = 8192;
constexpr unsigned midi_pitch_status = 0xe0;
constexpr unsigned midi_values = 16384;
constexpr unsigned midi_data_mask = 127;
constexpr std::int32_t midi_scale = 1024;

void install(blofeld::DspBus &bus, const blofeld::Firmware &firmware) {
  unsigned programs = 0;
  for (const auto &record : firmware.dsp) {
    if (record.space == 0) {
      if (record.address != original_program && record.address != original_vector &&
          record.address != parameters && record.address != messages) {
        continue;
      }
      ++programs;
    } else {
      const auto limit = record.space == 1 ? x_low_end : y_low_end;
      if (record.address + record.words.size() > limit) { continue; }
    }
    for (std::size_t index = 0; index < record.words.size(); ++index) {
      const auto address = record.address + static_cast<unsigned>(index);
      const bool aliased = record.space == 0 && record.address == original_program;
      bus.write({.space = aliased ? DspSpace::x : static_cast<DspSpace>(record.space),
                 .address = aliased ? address + physical_alias_offset : address,
                 .value = record.words[index]});
    }
  }
  require(programs == 4,
          "All four complete unchanged original producer/vector/parser records load");
}

[[nodiscard]] unsigned bend_word(const unsigned lsb, const unsigned msb) {
  const auto centered =
      static_cast<std::int32_t>(lsb | (msb << 7)) - static_cast<std::int32_t>(midi_center);
  return static_cast<unsigned>(centered * midi_scale) & blofeld::test::word_mask;
}

struct SampleExpectation {
  unsigned byte;
  unsigned bit;
  unsigned words_before;
};

struct WireCompletion {
  unsigned byte;
  unsigned elapsed;
  unsigned words_before;
};

struct ExpectedMessage {
  unsigned word;
  bool completed;
};

class Fixture {
public:
  explicit Fixture(const blofeld::Firmware &firmware) {
    install(machine_.bus(), firmware);
    auto &bus = machine_.bus();
    bus.write({.space = DspSpace::x, .address = producer, .value = ring_base});
    bus.write({.space = DspSpace::x, .address = timestamp, .value = timestamp_seed});
    for (unsigned index = 0; index < ring_words; ++index) {
      expected_ring_.at(index) = ring_guard ^ index;
      bus.write(
          {.space = DspSpace::x, .address = ring_base + index, .value = expected_ring_.at(index)});
    }
    bus.write({.space = DspSpace::x, .address = ring_base - 1, .value = ring_guard});
    bus.write({.space = DspSpace::x, .address = ring_base + ring_words, .value = ring_guard});
    bus.write({.space = DspSpace::x, .address = mode_flags, .value = 0});
    bus.write({.space = DspSpace::x, .address = midi_channel, .value = 0});
    bus.write({.space = DspSpace::x, .address = midi_clock, .value = 1});
    bus.write({.space = DspSpace::x, .address = field, .value = field_seed});
    bus.write({.space = DspSpace::x, .address = field - 1, .value = ring_guard});
    bus.write({.space = DspSpace::x, .address = field + 1, .value = ring_guard});
    bus.timers().set_tio_drive(0, true);
    bus.timers().observe_synchronized_tio(0, true);
    blofeld::test::DspProgram code;
    code.immediate("omr", switch_omr);
    code.emit("nop");
    code.emit("nop");
    code.emit("nop");
    const auto mapped = code.pc();
    code.immediate("sr", ordinary_status);
    code.immediate("a1", timestamp_seed);
    for (unsigned index = 0; index < 8; ++index) {
      code.immediate("m" + std::to_string(index), blofeld::test::word_mask);
    }
    code.emit("movep #>$200,x:<<$fffffe");
    code.immediate("r6", original_program);
    code.emit("jsr (r6)");
    foreground_ = code.pc();
    code.emit("move x:>$50,r6");
    code.emit("jsr (r6)");
    complete_ = code.pc();
    code.emit("nop");
    code.word(jump);
    code.word(foreground_);
    code.install(bus);
    for (unsigned step = 0; machine_.cpu().state().pc != foreground_; ++step) {
      require(step < step_limit, "Own setup and original configuration must return");
      if (machine_.cpu().state().pc == mapped) { bus.advance_instruction_cycles(3); }
      machine_.cpu().step();
    }
    require(
        machine_.cpu().state().omr == switch_omr &&
            bus.timers().read(control, control_without_di) == capture_control &&
            bus.timers().read(compare) == 1 && bus.timers().read(counter) == 0 &&
            bus.read({.space = DspSpace::x, .address = priority}) == 0x200,
        "Original configuration establishes the declared native map, priority and capture mode");
  }

  void run_bytes(std::ostream &trace) {
    constexpr unsigned byte_values = 256;
    for (unsigned byte = 0; byte < byte_values; ++byte) { send(byte); }
    constexpr unsigned expected_irqs = byte_values * 9;
    require(words_ == byte_values && irqs_ == expected_irqs && returns_ == expected_irqs &&
                timestamp_writes_ == 1,
            "Every byte produces one real queue word, nine original IRQs/RTIs and a clock stamp");
    trace << "bytes=" << words_ << " original-irq=" << irqs_ << " original-rti=" << returns_
          << " ring-wraps=" << words_ / ring_words << " timestamps=" << timestamp_writes_
          << " producer-instructions=" << instructions_ << '\n';
  }

  void run_pitch(std::ostream &trace) {
    deliver(midi_pitch_status, {.word = field_seed, .completed = false});
    unsigned clocks = 0;
    for (unsigned value = 0; value < midi_values; ++value) {
      const auto centered =
          static_cast<std::int32_t>(value) - static_cast<std::int32_t>(midi_center);
      const auto expected =
          static_cast<std::uint32_t>(centered * midi_scale) & blofeld::test::word_mask;
      deliver(value & midi_data_mask, {.word = expected_field_, .completed = false});
      if (value == 0 || value == midi_center || value == midi_values - 1) {
        deliver(clock_byte, {.word = expected_field_, .completed = false});
        ++clocks;
      }
      deliver(value >> 7, {.word = expected, .completed = true});
    }
    const auto expected_words = 1 + (2 * midi_values) + clocks;
    require(
        words_ == expected_words && irqs_ == expected_words * 9 && returns_ == irqs_ &&
            timestamp_writes_ == clocks && callbacks_ == midi_values,
        "Every literal wire byte traverses original IRQs, real queue, parser and pitch callback");
    trace << "wire-bytes=" << words_ << " original-irq=" << irqs_ << " original-rti=" << returns_
          << " ring-wraps=" << words_ / ring_words << " clocks=" << timestamp_writes_
          << " pitch-values=" << callbacks_ << " producer-instructions=" << instructions_ << '\n';
  }

  void run_checkpoint(std::ostream &trace) {
    constexpr unsigned lsb = 0x35;
    constexpr unsigned first_msb = 0x23;
    constexpr unsigned changed_msb = 0x63;
    deliver(midi_pitch_status, {.word = field_seed, .completed = false});
    deliver(lsb, {.word = field_seed, .completed = false});
    start_wire();
    unsigned elapsed = 0;
    for (unsigned bit = 0; bit < 2; ++bit) {
      prepare_sample(first_msb, bit, elapsed);
      interrupt(bit == 0 ? status_handler : input_handler);
      verify_sample({.byte = first_msb, .bit = bit, .words_before = words_});
    }
    prepare_sample(first_msb, 2, elapsed);
    expected_byte_ = first_msb;
    const auto context = begin_interrupt(input_handler);
    // One real handler instruction retires before the trusted checkpoint.
    machine_.cpu().step(producer_observer());
    require(machine_.cpu().state().sp == 1 && words_ == 2 && read(field) == field_seed,
            "Snapshot is inside the genuine third-bit ISR with an incomplete MIDI message");
    const auto saved = machine_.checkpoint();
    const auto before_cpu = machine_.cpu().debug_state();
    const auto before_host = host_state();
    const auto before_memory = signature();
    std::optional<blofeld::DspDebugState> first_cpu;
    std::vector<unsigned> first_memory;
    for (unsigned replay = 0; replay < 3; ++replay) {
      machine_.restore(saved);
      restore_host(before_host);
      require(machine_.cpu().debug_state() == before_cpu && signature() == before_memory,
              "Restore preserves whole CPU, active IRQ frame and every watched RAM cell");
      const auto byte = replay == 2 ? changed_msb : first_msb;
      expected_byte_ = byte;
      finish_interrupt(context);
      verify_sample({.byte = byte, .bit = 2, .words_before = 2});
      auto remaining_elapsed = elapsed;
      for (unsigned bit = 3; bit < byte_bits; ++bit) {
        prepare_sample(byte, bit, remaining_elapsed);
        interrupt(input_handler);
        verify_sample({.byte = byte, .bit = bit, .words_before = 2});
      }
      finish_wire({.byte = byte, .elapsed = remaining_elapsed, .words_before = 2});
      const auto expected = bend_word(lsb, byte);
      consume({.word = expected, .completed = true});
      require(callbacks_ == 1 && words_ == 3 && irqs_ == expected_irq_count &&
                  returns_ == expected_irq_count,
              "Each restored suffix completes exactly one real byte and one original callback");
      if (replay == 0) {
        first_cpu = machine_.cpu().debug_state();
        first_memory = signature();
      } else if (replay == 1) {
        require(first_cpu.has_value() && machine_.cpu().debug_state() == first_cpu.value() &&
                    signature() == first_memory,
                "Identical remaining wire bits replay every CPU and watched memory effect");
      } else {
        require(read(field) != bend_word(lsb, first_msb),
                "Changed remaining wire bit changes the genuine callback result");
      }
    }
    trace << "inside-original-isr-snapshots=1 identical-suffixes=2 changed-suffixes=1 "
             "original-callbacks=3 data-sample-irqs-entered=3\n";
  }

private:
  unsigned read(const unsigned address) {
    return machine_.bus().read({.space = DspSpace::x, .address = address});
  }

  void drive(const bool level) {
    machine_.bus().timers().set_tio_drive(0, level);
    machine_.bus().timers().observe_synchronized_tio(0, level);
  }

  struct InterruptContext {
    blofeld::DspDebugState before;
    unsigned returns;
  };

  [[nodiscard]] blofeld::DspNativeAccessObserver producer_observer() {
    return {.observe = [this](const blofeld::DspNativeAccess &access) {
      watch(access);
      observe(access);
    }};
  }

  [[nodiscard]] InterruptContext begin_interrupt(const unsigned entry) {
    const InterruptContext context{.before = machine_.cpu().debug_state(), .returns = returns_};
    machine_.cpu().step(producer_observer());
    require(machine_.cpu().state().pc == entry && machine_.cpu().state().sp == 1,
            "Actual native original vector enters its real handler and pushes one frame");
    return context;
  }

  void finish_interrupt(const InterruptContext &context) {
    for (unsigned steps = 0; machine_.cpu().state().pc != foreground_; ++steps) {
      require(steps < step_limit, "Original handler must execute its actual finite return path");
      const auto pc = machine_.cpu().state().pc;
      machine_.cpu().step(producer_observer());
      if (pc == capture_return || pc == sample_return || pc == input_return) { ++returns_; }
    }
    const auto after = machine_.cpu().debug_state();
    require(after.cpu == context.before.cpu && returns_ == context.returns + 1 &&
                machine_.bus().timers().interrupt_vectors().empty(),
            "Original RTI restores all foreground registers and acknowledges its real source");
    instructions_ += after.native_instructions - context.before.native_instructions;
    ++irqs_;
  }

  void interrupt(const unsigned entry) { finish_interrupt(begin_interrupt(entry)); }

  void start_wire() {
    drive(true);
    machine_.bus().timers().advance_clk2(bit_period);
    drive(false);
    interrupt(status_handler);
    require(machine_.bus().timers().read(control, control_without_di) == ordinary_control &&
                machine_.bus().timers().read(compare) == first_compare &&
                machine_.bus().timers().read(counter) == 0,
            "Actual original captured-start ISR disables/reconfigures and returns");
  }

  void prepare_sample(const unsigned byte, const unsigned bit, unsigned &elapsed) {
    const auto boundary = (bit + 1) * bit_period;
    machine_.bus().timers().advance_clk2(boundary - elapsed);
    elapsed = boundary;
    drive(((byte >> bit) & 1U) != 0);
    const auto sample =
        bit == 0 ? first_delay : first_delay + first_overflow_delay + ((bit - 1) * bit_period);
    machine_.bus().timers().advance_clk2(sample - elapsed);
    elapsed = sample;
    expected_byte_ = byte;
  }

  void verify_sample(const SampleExpectation expectation) {
    const auto byte = expectation.byte;
    const auto bit = expectation.bit;
    const auto words_before = expectation.words_before;
    const auto received = bit + 1;
    const auto prefix = ((byte & ((1U << received) - 1)) << (blofeld::test::word_bits - received)) |
                        (sample_seed >> received);
    require(read(state_word) == (received == byte_bits ? sample_seed : prefix),
            "Original partial shift equals the independent LSB-first bit prefix and sentinel");
    require(words_ == words_before + (received == byte_bits ? 1U : 0U),
            "Original producer queues no word until all eight actual input samples execute");
    const auto expected_control = received == byte_bits ? capture_control : sampling_control;
    const auto pending_match = bit != 0 && received != byte_bits ? source_match : 0U;
    require(machine_.bus().timers().read(control, control_without_di) ==
                (expected_control | pending_match),
            "Original masks, retained unenabled match and capture rearm stay exact");
  }

  void finish_wire(const WireCompletion completion) {
    const auto byte = completion.byte;
    const auto elapsed = completion.elapsed;
    const auto words_before = completion.words_before;
    machine_.bus().timers().advance_clk2((9 * bit_period) - elapsed);
    drive(true);
    machine_.bus().timers().advance_clk2(bit_period);
    expected_ring_.at(words_before % ring_words) = byte << data_shift;
    if (byte == clock_byte) { expected_timestamp_ = 0; }
    require(read(producer) == ring_base + ((words_before + 1) % ring_words),
            "Original producer advances its real ring cursor");
    for (unsigned index = 0; index < ring_words; ++index) {
      require(read(ring_base + index) == expected_ring_.at(index),
              "Every real ring cell agrees with independent input or its unchanged sentinel");
    }
    require(read(ring_base - 1) == ring_guard && read(ring_base + ring_words) == ring_guard &&
                read(timestamp) == expected_timestamp_,
            "Original writes preserve ring guards and the independent timestamp");
  }

  void send(const unsigned byte) {
    start_wire();
    unsigned elapsed = 0;
    const auto words_before = words_;
    for (unsigned bit = 0; bit < byte_bits; ++bit) {
      prepare_sample(byte, bit, elapsed);
      interrupt(bit == 0 ? status_handler : input_handler);
      verify_sample({.byte = byte, .bit = bit, .words_before = words_before});
    }
    finish_wire({.byte = byte, .elapsed = elapsed, .words_before = words_before});
  }

  void deliver(const unsigned byte, const ExpectedMessage expectation) {
    send(byte);
    consume(expectation);
  }

  void consume(const ExpectedMessage expectation) {
    const auto expected = expectation.word;
    const bool completed = expectation.completed;
    machine_.bus().write(
        {.space = DspSpace::x, .address = harness_selector, .value = parser_entry});
    unsigned entries = 0;
    unsigned writes = 0;
    const auto observer =
        blofeld::DspNativeAccessObserver{.observe = [&](const blofeld::DspNativeAccess &access) {
          watch(access);
          if (access.stage == blofeld::DspNativeAccessStage::after &&
              access.kind == blofeld::DspNativeAccessKind::write &&
              access.target.space == DspSpace::x && access.target.address == field) {
            require(completed && access.value == expected,
                    "Every original pitch field write matches independent signed14-bit MIDI math");
            ++writes;
          }
        }};
    for (unsigned steps = 0; machine_.cpu().state().pc != complete_; ++steps) {
      require(steps < parser_steps,
              "Original parser must consume the actual produced byte and return");
      const auto cpu = machine_.cpu().state();
      if (cpu.pc == callback_entry) {
        require(cpu.sp == 2 && cpu.stack_pc == callback_return,
                "Actual original nested dispatcher JSR enters the genuine pitch callback");
        ++entries;
      }
      machine_.cpu().step(observer);
    }
    require(
        entries == (completed ? 1U : 0U) && writes == entries && read(field) == expected &&
            machine_.cpu().state().sp == 0 && read(consumer) == read(producer) &&
            read(field - 1) == ring_guard && read(field + 1) == ring_guard,
        "Original consumer drains its real producer cursor and returns with the independent value");
    expected_field_ = expected;
    callbacks_ += entries;
    for (unsigned steps = 0; machine_.cpu().state().pc != foreground_; ++steps) {
      require(steps < step_limit, "Own harness returns naturally to the next wire-input boundary");
      machine_.cpu().step();
    }
  }

  void observe(const blofeld::DspNativeAccess &access) {
    if (access.stage != blofeld::DspNativeAccessStage::after ||
        access.kind != blofeld::DspNativeAccessKind::write) {
      return;
    }
    if (access.target.space != DspSpace::x || !access.value.has_value()) {
      throw blofeld::Fault("Original producer writes require one known X-space result");
    }
    const auto address = access.target.address;
    const bool ring = address >= ring_base && address < ring_base + ring_words;
    const bool context =
        address == producer || (address >= saved_accumulator && address <= state_word);
    const bool peripheral = address >= counter && address <= control;
    require(ring || context || peripheral || address == timestamp,
            "Unexpected original producer write at " + blofeld::hex(address));
    if (ring) {
      require(access.value.value() == (expected_byte_ << data_shift) &&
                  address == ring_base + (words_ % ring_words),
              "Every genuine original word write has the independent byte and expected cursor");
      ++words_;
    }
    if (address == timestamp) {
      require(expected_byte_ == clock_byte && access.value.value() == 0,
              "Actual clock timestamp reads the reset-defined timer-two count");
      ++timestamp_writes_;
    }
  }

  void watch(const blofeld::DspNativeAccess &access) {
    if (access.stage != blofeld::DspNativeAccessStage::after ||
        access.kind != blofeld::DspNativeAccessKind::write ||
        access.target.address >= peripheral_base) {
      return;
    }
    const auto same = [&](const blofeld::DspAddress target) {
      return target.space == access.target.space && target.address == access.target.address;
    };
    if (std::ranges::find_if(watched_, same) == watched_.end()) {
      watched_.push_back(access.target);
    }
  }

  [[nodiscard]] std::vector<unsigned> signature() {
    std::vector<unsigned> result;
    result.reserve(watched_.size() + ring_words + 16);
    for (const auto target : watched_) { result.push_back(machine_.bus().read(target)); }
    for (unsigned index = 0; index < ring_words; ++index) {
      result.push_back(read(ring_base + index));
    }
    result.push_back(read(ring_base - 1));
    result.push_back(read(ring_base + ring_words));
    result.push_back(read(field - 1));
    result.push_back(read(field + 1));
    auto &timers = machine_.bus().timers();
    result.push_back(timers.read(control, control_without_di));
    result.push_back(timers.read(compare));
    result.push_back(timers.read(counter));
    for (const auto vector : timers.interrupt_vectors()) { result.push_back(vector); }
    return result;
  }

  struct HostState {
    unsigned expected_field;
    unsigned expected_timestamp;
    unsigned callbacks;
    unsigned expected_byte;
    unsigned words;
    unsigned irqs;
    unsigned returns;
    unsigned timestamp_writes;
    std::uint64_t instructions;
    std::array<unsigned, ring_words> ring;
    std::vector<blofeld::DspAddress> watched;
  };

  [[nodiscard]] HostState host_state() const {
    return {.expected_field = expected_field_,
            .expected_timestamp = expected_timestamp_,
            .callbacks = callbacks_,
            .expected_byte = expected_byte_,
            .words = words_,
            .irqs = irqs_,
            .returns = returns_,
            .timestamp_writes = timestamp_writes_,
            .instructions = instructions_,
            .ring = expected_ring_,
            .watched = watched_};
  }

  void restore_host(const HostState &saved) {
    expected_field_ = saved.expected_field;
    expected_timestamp_ = saved.expected_timestamp;
    callbacks_ = saved.callbacks;
    expected_byte_ = saved.expected_byte;
    words_ = saved.words;
    irqs_ = saved.irqs;
    returns_ = saved.returns;
    timestamp_writes_ = saved.timestamp_writes;
    instructions_ = saved.instructions;
    expected_ring_ = saved.ring;
    watched_ = saved.watched;
  }

  blofeld::DspMachine machine_;
  unsigned foreground_ = 0;
  unsigned complete_ = 0;
  unsigned expected_field_ = field_seed;
  unsigned expected_timestamp_ = timestamp_seed;
  unsigned callbacks_ = 0;
  unsigned expected_byte_ = 0;
  unsigned words_ = 0;
  unsigned irqs_ = 0;
  unsigned returns_ = 0;
  unsigned timestamp_writes_ = 0;
  std::uint64_t instructions_ = 0;
  std::array<unsigned, ring_words> expected_ring_{};
  std::vector<blofeld::DspAddress> watched_;
};
}

int main(const int count, char **arguments) {
  try {
    require(count == 4,
            "Usage: original-dsp-wire firmware.zip private.trace bytes|pitch|checkpoint");
    std::ofstream trace(arguments[2]);
    require(trace.is_open(), "Cannot create private original MIDI wire evidence");
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    Fixture fixture(blofeld::Firmware::load(std::filesystem::path(arguments[1])));
    const std::string scope(arguments[3]);
    if (scope == "bytes") {
      fixture.run_bytes(trace);
    } else if (scope == "pitch") {
      fixture.run_pitch(trace);
    } else if (scope == "checkpoint") {
      fixture.run_checkpoint(trace);
    } else {
      throw blofeld::Fault("Unsupported original MIDI wire test scope: " + scope);
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
