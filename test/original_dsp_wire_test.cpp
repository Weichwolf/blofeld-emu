// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

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
constexpr unsigned download_program = 0x2361;
constexpr unsigned download_entry = download_program;
constexpr unsigned download_target = 0x600;
constexpr unsigned download_buffer = 0x778;
constexpr unsigned download_count = 0x29;
constexpr unsigned append_entry = 0xcaf;
constexpr unsigned append_end = 0xcb9;
constexpr unsigned download_store = 0x23c6;
constexpr unsigned download_return = 0x23ca;
constexpr unsigned context_base = 0x2c25;
constexpr unsigned device_setting = 0x745;
constexpr unsigned launch_return = 0x81e;
constexpr unsigned download_status = 0x76c;
constexpr unsigned completed_status = 19;
constexpr unsigned data_cursor = 0x2d;
constexpr unsigned declared_length = 0x2e;
constexpr unsigned data_end = 0x2f;
constexpr unsigned data_start = 0x30;
constexpr unsigned data_sequence = 0x31;
constexpr unsigned end_sysex = 0xf7;
constexpr unsigned start_sysex = 0xf0;
constexpr unsigned configure_command = 0x73;
constexpr unsigned group_command = 0x75;
constexpr unsigned byte_command = 0x74;
constexpr unsigned expected_cases = 30;
constexpr unsigned expected_matrix_bytes = 1050;
constexpr unsigned expected_matrix_appends = 940;
constexpr unsigned expected_matrix_entries = 51;
constexpr unsigned expected_frames = 54;
constexpr unsigned expected_starts = 55;
constexpr unsigned expected_target_writes = 96;
constexpr unsigned configured_device = 5;
constexpr unsigned other_device = 6;
constexpr unsigned other_manufacturer = 0x3f;
constexpr unsigned other_product = 0x12;
constexpr unsigned final_payload_prefetch = 0x23b4;
constexpr unsigned matrix_prefetches = 24;
constexpr unsigned manufacturer = 0x3e;
constexpr unsigned product = 0x13;
constexpr unsigned all_devices = 0x7f;
constexpr unsigned literal_rts = 12;
constexpr unsigned header_bits = 28;
constexpr unsigned byte_mask = 255;
constexpr unsigned first_residual = 0x87;
constexpr unsigned last_residual = 0xfe;
constexpr unsigned parser_context = 0x22;
constexpr unsigned buffer_limit = 0x920;
constexpr unsigned packet_body_words = 23;
constexpr std::array own_payload{literal_rts, 0x123456U, 0xfedcbaU, 0xa55a11U};
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
          record.address != parameters && record.address != messages &&
          record.address != download_program) {
        continue;
      }
      ++programs;
    } else {
      const auto limit = record.space == 1 ? x_low_end : y_low_end;
      if (record.address + record.words.size() > limit) { continue; }
    }
    for (std::size_t index = 0; index < record.words.size(); ++index) {
      const auto address = record.address + static_cast<unsigned>(index);
      const bool aliased = record.space == 0 && record.address >= 0x1000;
      bus.write({.space = aliased ? DspSpace::x : static_cast<DspSpace>(record.space),
                 .address = aliased ? address + physical_alias_offset : address,
                 .value = record.words[index]});
    }
  }
  require(programs == 5,
          "Five complete unchanged original producer/vector/parser/download records load");
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

struct HeaderFrame {
  unsigned mode;
  unsigned maker = manufacturer;
  unsigned product_id = product;
  unsigned device = all_devices;
};

struct Transfer {
  unsigned mode;
  unsigned command;
  unsigned trailing;
  bool omitted = false;
};

std::vector<unsigned> configuration_bytes(const HeaderFrame header) {
  std::vector<unsigned> result{
      header.maker, header.product_id, header.device, configure_command, header.mode};
  for (const auto field_value : {9U, download_target}) {
    for (unsigned shift = 0; shift < header_bits; shift += 7) {
      result.push_back((field_value >> shift) & midi_data_mask);
    }
  }
  return result;
}

std::vector<unsigned> data_bytes(const Transfer transfer) {
  std::vector<unsigned> decoded;
  for (const auto word : own_payload) {
    for (unsigned shift = 0; shift < blofeld::test::word_bits; shift += 8) {
      decoded.push_back((word >> shift) & byte_mask);
    }
  }
  decoded.push_back(first_residual);
  decoded.push_back(last_residual);
  std::vector<unsigned> result{manufacturer,
                               product,
                               all_devices,
                               transfer.command,
                               0,
                               transfer.command == byte_command ? 16U : 2U};
  for (unsigned offset = 0; offset < decoded.size(); offset += 7) {
    unsigned flags = 0;
    for (unsigned index = 0; index < 7; ++index) {
      flags |= ((decoded[offset + index] >> 7U) & 1U) << (6U - index);
    }
    result.push_back(flags);
    for (unsigned index = 0; index < 7; ++index) {
      result.push_back(decoded[offset + index] & midi_data_mask);
    }
  }
  if (!transfer.omitted) { result.push_back(transfer.trailing); }
  return result;
}

struct InputByte {
  unsigned value;
  unsigned index;
  bool appending;
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

  void run_download(std::ostream &trace) {
    machine_.bus().write(
        {.space = DspSpace::x, .address = device_setting, .value = configured_device});
    for (unsigned mode = 0; mode < 4; ++mode) {
      for (const unsigned command : {byte_command, group_command}) {
        for (const unsigned trailing : {0U, midi_data_mask}) {
          download_case({.mode = mode, .command = command, .trailing = trailing});
          trace << "NORMAL " << mode << ' ' << command << ' ' << trailing << '\n';
        }
        download_case({.mode = mode, .command = command, .trailing = 0, .omitted = true});
        trace << "OMITTED " << mode << ' ' << command << '\n';
      }
    }
    rejected_header({.mode = 0, .maker = other_manufacturer}, false);
    rejected_header({.mode = 0, .product_id = other_product}, false);
    rejected_header({.mode = 0, .device = other_device}, false);
    rejected_header({.mode = 0, .device = configured_device}, true);
    trace << "REJECTED 4\n";
    fragmented_header();
    trace << "FRAGMENTED 1\n";
    restarted_header();
    trace << "RESTARTED 1\n";
    require(words_ == expected_matrix_bytes && irqs_ == expected_matrix_bytes * 9 &&
                returns_ == irqs_ && appended_ == expected_matrix_appends &&
                download_entries_ == expected_matrix_entries && download_launches_ == 6 &&
                frame_starts_ == expected_starts && frame_ends_ == expected_frames &&
                target_writes_ == expected_target_writes && prefetches_ == matrix_prefetches &&
                timestamp_writes_ == 1,
            "Complete independent finite wire/parser/download/header/fragment matrix");
    trace << "RESULT cases=" << expected_cases << " wire=" << words_ << " irqs=" << irqs_
          << " rtis=" << returns_ << " appends=" << appended_ << " entries=" << download_entries_
          << " launches=" << download_launches_ << " starts=" << frame_starts_
          << " frames=" << frame_ends_ << " stores=" << target_writes_
          << " prefetches=" << prefetches_ << " timestamps=" << timestamp_writes_
          << " producer-instructions=" << instructions_ << '\n';
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

  std::array<unsigned, 8> protocol_state() {
    std::array<unsigned, 8> result{};
    for (unsigned i = 0; i < 5; ++i) { result[i] = read(data_cursor + i); }
    result[5] = read(download_status);
    result[6] = machine_.bus().read({.space = DspSpace::p, .address = download_store});
    result[7] = machine_.bus().read({.space = DspSpace::p, .address = download_return});
    return result;
  }

  void download_case(const Transfer transfer) {
    auto &bus = machine_.bus();
    const std::array spaces{DspSpace::p, DspSpace::p, DspSpace::x, DspSpace::y};
    expected_target_ = spaces.at(transfer.mode);
    packet_writes_ = 0;
    expected_trailing_ = transfer.omitted ? blofeld::test::word_mask : transfer.trailing << 16U;
    for (const auto space : {DspSpace::p, DspSpace::x, DspSpace::y}) {
      for (unsigned offset = 0; offset < 6; ++offset) {
        bus.write({.space = space, .address = download_target - 1 + offset, .value = ring_guard});
      }
    }
    const auto before_entries = download_entries_;
    const auto before_launches = download_launches_;
    stream_frame(configuration_bytes({.mode = transfer.mode}));
    require(read(data_cursor) == download_target && read(declared_length) == 9 &&
                read(data_end) == download_target + 3 && read(data_start) == download_target &&
                read(data_sequence) == 0 && read(download_count) == 13 &&
                download_entries_ == before_entries + 1 &&
                bus.read({.space = DspSpace::p, .address = download_store}) ==
                    read(context_base + 8 + transfer.mode),
            "Actual wire/parser/SysEx path configures the original selected download mode");
    stream_frame(data_bytes(transfer));
    for (const auto space : {DspSpace::p, DspSpace::x, DspSpace::y}) {
      require(bus.read({.space = space, .address = download_target - 1}) == ring_guard &&
                  bus.read({.space = space, .address = download_target + 4}) == ring_guard,
              "Wire-delivered data retains exact destination boundary guards");
      for (unsigned index = 0; index < own_payload.size(); ++index) {
        require(bus.read({.space = space, .address = download_target + index}) ==
                    (space == expected_target_ ? own_payload[index] : ring_guard),
                "Only original selected P/X/Y instructions write the exact own payload");
      }
    }
    const auto expected_count = packet_body_words - (transfer.omitted ? 1U : 0U);
    require(read(data_cursor) == download_target + 4 && read(data_sequence) == 0x10000 &&
                read(download_count) == expected_count && packet_writes_ == own_payload.size() &&
                download_entries_ == before_entries + 2 &&
                download_launches_ == before_launches + (transfer.mode == 0 ? 1U : 0U),
            "Original complete frames/padding stores and mode-dependent launches remain exact");
  }

  void rejected_header(const HeaderFrame header, const bool reached_download) {
    const auto before = protocol_state();
    const auto entries = download_entries_;
    const auto launches = download_launches_;
    stream_frame(configuration_bytes(header));
    require(protocol_state() == before && download_launches_ == launches &&
                download_entries_ == entries + (reached_download ? 1U : 0U),
            "Both original header-validation layers retain ignored download state");
  }

  void input_byte(const InputByte input) {
    send(input.value);
    consume_download(input);
  }

  void fragmented_header() {
    const auto body = configuration_bytes({.mode = 0});
    const auto before = protocol_state();
    const auto entries = download_entries_;
    input_byte({.value = start_sysex, .index = 0, .appending = false});
    for (unsigned index = 0; index < body.size(); ++index) {
      if (index == 6) { input_byte({.value = clock_byte, .index = index, .appending = false}); }
      input_byte({.value = body[index], .index = index, .appending = true});
    }
    require(protocol_state() == before && download_entries_ == entries,
            "Partial SysEx plus a real interleaved clock byte never pre-dispatches downloading");
    input_byte(
        {.value = end_sysex, .index = static_cast<unsigned>(body.size()), .appending = false});
    require(download_entries_ == entries + 1 && read(data_cursor) == download_target &&
                read(data_sequence) == 0 && read(download_count) == body.size(),
            "Actual F7 completes the retained frame after ignored realtime interleaving");
  }

  void restarted_header() {
    const auto old = configuration_bytes({.mode = 2, .maker = other_manufacturer});
    const auto before = protocol_state();
    const auto entries = download_entries_;
    input_byte({.value = start_sysex, .index = 0, .appending = false});
    for (unsigned index = 0; index < 6; ++index) {
      input_byte({.value = old[index], .index = index, .appending = true});
    }
    require(protocol_state() == before && download_entries_ == entries && read(download_count) == 6,
            "The abandoned prefix remains pending without any original download call");
    stream_frame(configuration_bytes({.mode = 3}));
    require(download_entries_ == entries + 1 && read(download_count) == 13 &&
                read(data_cursor) == download_target && read(data_sequence) == 0 &&
                machine_.bus().read({.space = DspSpace::p, .address = download_store}) ==
                    read(context_base + 8 + 3),
            "A real new F0 resets the count and replaces the old prefix before actual F7 dispatch");
  }

  void stream_frame(const std::vector<unsigned> &body) {
    send(start_sysex);
    consume_download({.value = start_sysex, .index = 0, .appending = false});
    for (unsigned index = 0; index < body.size(); ++index) {
      send(body[index]);
      consume_download({.value = body[index], .index = index, .appending = true});
    }
    send(end_sysex);
    consume_download(
        {.value = end_sysex, .index = static_cast<unsigned>(body.size()), .appending = false});
  }

  void download_write(const blofeld::DspNativeAccess &access,
                      const InputByte input,
                      unsigned &appended) {
    watch(access);
    if (access.stage == blofeld::DspNativeAccessStage::after &&
        access.kind == blofeld::DspNativeAccessKind::read && access.pc == final_payload_prefetch &&
        access.target.space == DspSpace::x &&
        access.target.address == download_buffer + packet_body_words - 1) {
      require(access.value == expected_trailing_,
              "Actual original final payload prefetch uses checksum or M4 terminator");
      ++prefetches_;
    }
    if (access.stage != blofeld::DspNativeAccessStage::after ||
        access.kind != blofeld::DspNativeAccessKind::write) {
      return;
    }
    const auto address = access.target.address;
    const bool data =
        access.target.space == DspSpace::x &&
        ((address >= parser_context && address <= download_count) ||
         (address >= data_cursor && address <= data_sequence) || address == consumer ||
         address == download_status || (address >= download_buffer && address <= buffer_limit));
    const bool program =
        access.target.space == DspSpace::p &&
        (address == download_store || address == download_return ||
         (address >= download_target && address < download_target + own_payload.size()));
    const bool target = access.pc == download_store && access.target.space == expected_target_ &&
                        address == download_target + packet_writes_;
    if (access.pc == download_store) {
      require(target && packet_writes_ < own_payload.size() &&
                  access.value == own_payload[packet_writes_],
              "Each original destination store matches independent bank/order/value expectations");
      ++packet_writes_;
      ++target_writes_;
    }
    require(data || program || target,
            "Unexpected native SysEx write at " + blofeld::hex(address) +
                " pc=" + blofeld::hex(access.pc));
    if (access.pc >= append_entry && access.pc <= append_end && address >= download_buffer) {
      require(input.appending && access.target.space == DspSpace::x &&
                  address == download_buffer + input.index && access.value == input.value << 16U,
              "Every collected SysEx word comes from the actual original byte-append instruction");
      ++appended;
      ++appended_;
    }
  }

  void consume_download(const InputByte input) {
    machine_.bus().write(
        {.space = DspSpace::x, .address = harness_selector, .value = parser_entry});
    unsigned appended = 0;
    const blofeld::DspNativeAccessObserver observer{
        .observe = [&](const blofeld::DspNativeAccess &access) {
          download_write(access, input, appended);
        }};
    for (unsigned steps = 0; machine_.cpu().state().pc != complete_; ++steps) {
      require(steps < parser_steps, "Actual original SysEx consumer instruction bound");
      const auto state = machine_.cpu().state();
      if (state.pc == download_entry) {
        require(
            input.value == end_sysex && state.sp == 2 && state.stack_pc == launch_return,
            "The real F7 path reaches original downloading with its actual nested caller frame");
        ++download_entries_;
      }
      if (state.pc == download_target) {
        require(
            input.value == end_sysex && state.sp == 2 && state.stack_pc == launch_return &&
                state.r[0] == context_base && read(download_status) == completed_status &&
                machine_.bus().read({.space = DspSpace::p, .address = download_return}) ==
                    literal_rts,
            "Actual original launch restores RTS and passes context without a host PC/frame patch");
        ++download_launches_;
      }
      machine_.cpu().step(observer);
    }
    if (input.value == start_sysex) { ++frame_starts_; }
    if (input.value == end_sysex) { ++frame_ends_; }
    const auto expected_count = input.appending ? input.index + 1 : input.index;
    require(appended == (input.appending ? 1U : 0U) && read(download_count) == expected_count &&
                machine_.cpu().state().sp == 0 && read(consumer) == read(producer),
            "Real byte collection consumes one actual producer word and returns through the "
            "original stack");
    for (unsigned steps = 0; machine_.cpu().state().pc != foreground_; ++steps) {
      require(steps < step_limit, "Own caller returns to the actual next input boundary");
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
  unsigned appended_ = 0;
  unsigned download_entries_ = 0;
  unsigned download_launches_ = 0;
  unsigned frame_starts_ = 0;
  unsigned frame_ends_ = 0;
  unsigned target_writes_ = 0;
  unsigned packet_writes_ = 0;
  unsigned prefetches_ = 0;
  unsigned expected_trailing_ = 0;
  DspSpace expected_target_ = DspSpace::p;
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
            "Usage: original-dsp-wire firmware.zip private.trace bytes|pitch|checkpoint|download");
    std::ofstream trace(arguments[2]);
    require(trace.is_open(), "Cannot create private original MIDI wire evidence");
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    Fixture fixture(blofeld::Firmware::load(std::filesystem::path(arguments[1])));
    const std::string scope(arguments[3]);
    if (scope == "download") {
      fixture.run_download(trace);
    } else if (scope == "bytes") {
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
