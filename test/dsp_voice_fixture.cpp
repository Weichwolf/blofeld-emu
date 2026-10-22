// SPDX-License-Identifier: GPL-3.0-only
// Observe unchanged original 1.25 voice blocks; scalar models remain separate.
#include "dsp_voice_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/pcm_sink.hpp"
#include "blofeld/serial_pcm.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_voice_capture.hpp"
#include "dsp_voice_master.hpp"
#include "dsp_voice_pcm.hpp"
#include "dsp_voice_repeat.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {
using Words = std::vector<unsigned>;
constexpr unsigned fixture_sound = 0x1000, entry = 0xc500, mask = 0xffffff,
                   rounding_mode = 1U << 21U;
constexpr unsigned primary_sound = 0x936;
constexpr unsigned midi_parser = 0xc53;
constexpr unsigned midi_secondary_parser = 0xc5d;
constexpr unsigned response_parser = 0x2407;
constexpr unsigned midi_ring = 0x2f80;
constexpr unsigned midi_ring_size = 64;
constexpr unsigned midi_producer = 0x3b;
constexpr unsigned midi_consumer = 0x3c;
constexpr unsigned midi_channel_setting = 0x744;
constexpr unsigned midi_control_assignments = 0x753;
constexpr unsigned midi_staged_status = 0x24;
constexpr unsigned midi_byte_shift = 16;
constexpr unsigned midi_controller_status = 0xb0;
constexpr unsigned midi_volume = 7;
constexpr unsigned midi_wheel = 1;
constexpr unsigned transition_destination = 0x15;
constexpr unsigned transition_target = 0x16;
constexpr unsigned transition_flag = 0x80;
constexpr unsigned runtime_flags = 6;
constexpr unsigned modulation_controller = 383;
constexpr std::array<unsigned, 8> midi_volumes{127, 0, 1, 64, 96, 127, 32, 0};
constexpr unsigned main_loop = 0x198;
constexpr unsigned master_output = 0x27b;
constexpr unsigned scheduler_tail = 0x22d;
constexpr unsigned active_entry_next = 0x43e;
constexpr unsigned part_stride = 0x1ac;
constexpr unsigned part_count = 16;
constexpr unsigned block_countdown = 0x19;
constexpr unsigned scheduler_period = 0x1d;
constexpr unsigned scheduler_timer = 0x1e;
constexpr unsigned scheduler_pending = 0x1c;
constexpr unsigned phase_word = 5;
constexpr unsigned dma_source = 0xffffef;
constexpr unsigned declared_source = 0x660;
constexpr unsigned declared_period = 0x4801;
constexpr unsigned effect_dispatch = 0x1fc3;
constexpr unsigned reset_parts = 0xd14;
constexpr unsigned event_initializer = 0x1bf4;
constexpr unsigned event_base = 0x175a;
constexpr unsigned event_stride = 83;
constexpr unsigned queued_events = 68;
constexpr unsigned secondary_ring = 0x2d00;
constexpr unsigned secondary_ring_size = 128;
constexpr unsigned event_read = 0x2c34;
constexpr unsigned event_write = 0x2c35;
constexpr unsigned event_ring = 0x2c40;
constexpr unsigned response_read = 0xed97;
constexpr unsigned response_write = 0xed96;
constexpr unsigned response_ring = 0xed98;
constexpr unsigned active_list = 0x6d2;
constexpr unsigned first_effect_type = 128;
constexpr unsigned second_effect_type = 144;
constexpr unsigned master_completed = 0x1aa;
constexpr unsigned mix_completed = 0x228;
constexpr unsigned master_history = 0x46;
constexpr unsigned mixed_bus = 0x620;
constexpr unsigned chain_gain = 0x6a0;
constexpr unsigned idle_burst_pairs = 24; // 48 bytes fit the 64-word input ring and cross its end.
constexpr unsigned midi_stream_blocks = 10; // Retain the delayed output of all idle CC changes.
constexpr unsigned release_input_block = 2;
constexpr unsigned release_retirement_block = 5;
constexpr unsigned sustain_release_block = 4;
constexpr unsigned sustain_stream_blocks = 12;
constexpr unsigned sustain_controller = 64;
constexpr unsigned sustain_scalar = 394;
constexpr unsigned sustain_flag = 395;
constexpr unsigned first_note_latch = 396;
constexpr unsigned note_latch_words = 8;
constexpr unsigned owner_mask = 0x2f1c;
constexpr unsigned owner_base = 0x2f1e;
constexpr unsigned owner_count = 24;
constexpr unsigned first_envelope_callback = 56;
constexpr unsigned envelope_state_stride = 6;
constexpr unsigned release_velocity_field = 19;
constexpr unsigned release_note_field = 9;
constexpr unsigned release_tag_field = 10;
constexpr unsigned note_off_status = 0x80;
constexpr unsigned zero_note_on_status = 0x90;
constexpr std::array<unsigned, 4> release_parameters{205, 217, 229, 241};
constexpr std::array<unsigned, 13> foreground_stages{main_loop,
                                                     midi_parser,
                                                     midi_secondary_parser,
                                                     response_parser,
                                                     master_output,
                                                     0x2a20,
                                                     0x298,
                                                     0x251,
                                                     0x1cc1,
                                                     0x436,
                                                     0x4df,
                                                     scheduler_tail,
                                                     0x246};
constexpr unsigned parameters_copied = 0x51c, processing_chain = 0x59d, lfo_initializer = 0x169d;
constexpr unsigned mixer_gains = 0x6ed, mixer_renderer = 0x707, noise_generator = 0x7e5;

constexpr unsigned memory_mode = 0x600389;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned caller_scratch = 0x700;
constexpr unsigned caller_target = 0x800;
constexpr unsigned caller_status = 0x801;
constexpr unsigned voice_entry = 0x4df;
constexpr unsigned free_sine = 0x1700;
constexpr unsigned envelope_start = 0x167b;
constexpr unsigned envelope_attack = 0x165e;
constexpr unsigned envelope_finish = 0x1668;
constexpr unsigned envelope_decay = 0x1639;
constexpr unsigned envelope_release = 0x160f;
constexpr unsigned envelope_retire = 0x1609;
constexpr unsigned output_dispatch = 0x620;
constexpr unsigned output_dispatch_end = 0x62e;
constexpr unsigned output_accumulator = 0x78d;
constexpr unsigned output_tail = 0x795;
constexpr unsigned output_bus = 0x5a0;
constexpr unsigned routed_buffer = 0x880;
constexpr unsigned output_prefetch = 0x8a0;
constexpr unsigned prefetch_value = 0x123457;
constexpr unsigned coefficient_buffer = 0x860;
constexpr unsigned first_filter_buffer = 0x580;
constexpr unsigned second_filter_buffer = 0x590;
constexpr unsigned noise_buffer = 0x570;
constexpr unsigned first_oscillator_buffer = 0x540;
constexpr unsigned second_oscillator_buffer = 0x550;
constexpr unsigned third_oscillator_buffer = 0x560;
constexpr unsigned filters_begin = 0x5b9;
constexpr unsigned drive_entry = 0x2bce;
constexpr unsigned drive_end = 0x2cd9;
constexpr unsigned drive_table = 0x2f36;
constexpr unsigned first_drive_level = 81;
constexpr unsigned first_drive_curve = 82;
constexpr unsigned second_drive_level = 101;
constexpr unsigned second_drive_curve = 102;
constexpr unsigned word_unit = 65536;
constexpr unsigned drive_mode_flags = 0x0c;
constexpr unsigned first_drive_return = 0x5c9;
constexpr unsigned serial_drive_return = 0x5e4;
constexpr unsigned parallel_drive_return = 0x5ff;
constexpr unsigned lp24_entry = 0x136a;
constexpr unsigned ordinary_biquad = 0x1337;
constexpr unsigned ordinary_biquad_end = 0x134c;
constexpr unsigned wide_biquad = 0x1353;
constexpr unsigned wide_biquad_end = 0x139e;
constexpr unsigned serial_route = 0x5d1;
constexpr unsigned parallel_route = 0x5f8;
constexpr unsigned oscillator_sample = 0x12ab;
constexpr unsigned oscillator_sync = 0x12a5;
constexpr unsigned oscillator_end = 0x12c4;
constexpr unsigned oscillator_wrap = 0x12bc;
constexpr unsigned mixer_continue = 0x6e2;
constexpr unsigned steady_mixer = 0x6c8;
constexpr unsigned mixer_end = 0x72a;
constexpr unsigned noise_end = 0x808;
constexpr unsigned register_prefix = 0x10b;
constexpr unsigned register_prefix_end = 0x11d;
constexpr unsigned waveform_initializer = 0x2ec3;
constexpr unsigned controller_reset = 0xd1e;
constexpr unsigned entry_initializer = 0xe58;
constexpr unsigned entry_list_initializer = 0x44a;
constexpr unsigned pointer_prefix = 0x17e;
constexpr unsigned pointer_prefix_end = 0x185;
constexpr unsigned shared_seed = 0x11;
constexpr unsigned sound_context = 0x13;
constexpr unsigned entry_context = 0x14;
constexpr unsigned previous_gain = 0xba;
constexpr unsigned noise_color = 0xbe;
constexpr unsigned noise_history = 0xbf;
constexpr unsigned noise_descriptor = 0xc0;
constexpr unsigned mixer_callback = 0xb2;
constexpr unsigned first_filter_history = 0x360;
constexpr unsigned second_filter_history = 0x368;
constexpr unsigned factory_sound = 0x1e25;
constexpr unsigned readable_entry_words = 0x106;
constexpr unsigned oscillator_descriptor_end = 0x68;
constexpr unsigned descriptor_ring = 0x40;
constexpr unsigned first_descriptor = 0x48;
constexpr unsigned second_descriptor = 0x50;
constexpr unsigned third_descriptor = 0x58;
constexpr unsigned oscillator_modulo = 0x801f;
constexpr unsigned reset_controller_level = 0x7f0000;
constexpr unsigned converted_high_note = 0x3f0000;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned initial_left = 0x13579b;
constexpr unsigned initial_right = 0x2468ac;
constexpr unsigned positive_limit = 0x7fffff;
constexpr unsigned negative_limit = 0x800000;
constexpr unsigned lfo_state = 0xc518;
constexpr unsigned envelope_state = 0xc538;
constexpr unsigned first_phase = 0x49;
constexpr unsigned second_phase = 0x51;
constexpr unsigned third_phase = 0x59;
constexpr unsigned original_record_count = 127;
constexpr unsigned raw_sound_words = 383;
constexpr unsigned fixture_sound_words = 427;
constexpr unsigned output_controller = 404;
constexpr unsigned secondary_reset_controller = 394;
constexpr unsigned first_controller_depth = 383;
constexpr unsigned second_controller_depth = 385;
constexpr unsigned parameter_max = 127;
constexpr unsigned word_bits = 24;
constexpr unsigned controls_begin = 24;
constexpr unsigned controls_end = 34;
constexpr unsigned lfo_words = 32;
constexpr unsigned stereo_words = 32;
constexpr unsigned caller_latch = 0x2a;
constexpr unsigned shape_parameter = 8;
constexpr unsigned oscillator_parameter_stride = 16;
constexpr unsigned envelope_words = 24;
constexpr unsigned phase_modulo = 47;
constexpr unsigned descriptor_modulo = 23;
constexpr unsigned shared_samples_per_block = 48;
constexpr unsigned instruction_limit = 100000;
constexpr unsigned amplifier_envelope = 63;
constexpr unsigned filter_routing_parameter = 117;
constexpr unsigned noise_level_parameter = 67;
constexpr unsigned noise_color_parameter = 69;
constexpr unsigned oscillator_sync_parameter = 49;
constexpr unsigned mixer_parameters_begin = 61;
constexpr unsigned varied_ring_level = 77;
constexpr unsigned varied_ring_balance = 17;
constexpr unsigned ring_level_parameter = 71;
constexpr unsigned ring_balance_parameter = 72;
constexpr std::array<unsigned, 10> gain_offsets{8, 9, 13, 14, 18, 19, 20, 21, 22, 23};
constexpr std::array<unsigned, 3> high_pitch{112, 76, 127};

struct Setting {
  unsigned field;
  unsigned value;
};

constexpr std::array<Setting, 6> varied_filters{{{.field = 78, .value = 29},
                                                 {.field = 80, .value = 117},
                                                 {.field = 98, .value = 101},
                                                 {.field = 100, .value = 85},
                                                 {.field = 93, .value = 23},
                                                 {.field = 113, .value = 105}}};

void require(bool ok, const std::string &why) {
  if (!ok) { throw blofeld::Fault(why); }
}

struct Record {
  blofeld::DspSpace space;
  unsigned base;
  Words data;
};

struct Source {
  std::vector<Record> records;
  std::map<unsigned, unsigned> program;

  explicit Source(const std::string &path) {
    const auto firmware = blofeld::Firmware::load(path);
    for (const auto &record : firmware.dsp) {
      const auto space = static_cast<blofeld::DspSpace>(record.space);
      if (space == blofeld::DspSpace::p) {
        for (unsigned i = 0; i < record.words.size(); ++i) {
          program[record.address + i] = record.words[i];
        }
      }
      records.push_back({space, record.address, record.words});
    }
    require(records.size() == original_record_count, "Incomplete original runtime records");
  }
};

struct DriveWords {
  unsigned index;
  unsigned address;
};

struct Drive {
  unsigned curve = 0;
  unsigned level = 0;
};

struct Case {
  bool high;
  bool twos;
  unsigned color;
  bool varied;
  bool serial;
  std::array<Drive, 2> drives{};
  unsigned blocks = 8;
  bool drive_case = false;
};

constexpr unsigned stream_contexts = 8;

Case stream_configuration(const unsigned index, const unsigned blocks) {
  require(index < stream_contexts, "Original ring context index exceeds its declared matrix");
  constexpr unsigned pitch_bit = 4;
  constexpr unsigned rounding_bit = 2;
  constexpr unsigned routing_bit = 1;
  const bool high = (index & pitch_bit) != 0;
  return {.high = high,
          .twos = (index & rounding_bit) != 0,
          .color = parameter_max,
          .varied = high,
          .serial = (index & routing_bit) != 0,
          .blocks = blocks};
}

struct Call {
  unsigned target;
  unsigned end;
  bool twos;
};

void harness(blofeld::DspBus &bus,
             unsigned &stop,
             blofeld::test::VoiceMaster &master,
             blofeld::test::VoicePcm &pcm,
             blofeld::test::VoiceRepeat &repeat,
             const unsigned sound) {
  blofeld::test::DspProgram code;
  code.immediate("x1", memory_mode);
  code.emit("move x1,omr");
  for (unsigned i = 0; i < 3; ++i) { code.emit("nop"); }
  const auto begin = code.pc();
  for (unsigned i = 0; i < 8; ++i) { code.immediate("m" + std::to_string(i), mask); }
  code.emit("move x:>$801,x1");
  code.emit("move x1,sr");
  code.immediate("r0", sound);
  code.immediate("r2", entry);
  code.immediate("r5", entry);
  code.immediate("r3", caller_scratch);
  for (const auto *const reg : {"r6", "r7"}) { code.immediate(reg, 0); }
  code.immediate("r4", caller_target);
  code.emit("move x:>$800,r1");
  code.emit("jsr (r1)");
  stop = code.pc();
  code.emit("nop");
  code.word(absolute_jump); // DSP56300FM long absolute JMP plus its target extension.
  code.word(begin);
  master.append(code);
  pcm.append(code);
  repeat.append(code);
  code.install(bus);
}

unsigned initial_bus_word(const unsigned index, const bool varied) {
  if (index == 0) { return initial_left; }
  if (index == 1) { return initial_right; }
  if (varied) { return index % 2 == 0 ? positive_limit : negative_limit; }
  return static_cast<unsigned>(((static_cast<long long>(index) - 16) * 65536) + 0x1000000LL) & mask;
}

struct MemoryRange {
  unsigned base = 0;
  unsigned count = 0;
};

bool stream_profile(const blofeld::test::VoiceInput input) {
  using enum blofeld::test::VoiceInput;
  return input == stream || input == stream_midi || input == stream_release ||
         input == stream_sustain;
}

bool release_callback(const unsigned address) {
  return address >= entry + first_envelope_callback &&
         address < entry + first_envelope_callback + (4 * envelope_state_stride) &&
         (address - entry - first_envelope_callback) % envelope_state_stride == 0;
}

struct Fixture {
  blofeld::DspMachine machine;
  blofeld::DspBus &bus;
  const unsigned sound;
  unsigned stop = 0;
  blofeld::test::VoiceMaster master;
  blofeld::test::VoicePcm pcm;
  blofeld::test::VoiceRepeat repeat;
  blofeld::DspCore &cpu;
  const Source &source;
  std::ostream &trace;
  std::set<unsigned> visited;
  unsigned syncs = 0, shared = 0, continues = 0;
  std::array<unsigned, 3> wraps{};
  unsigned long long total_steps = 0;
  bool output_seen = false;
  bool rendering = false;
  bool master_enabled = false;
  bool pcm_enabled = false;
  blofeld::PcmFrameSink *stream_output = nullptr;
  bool midi_enabled = false;
  bool foreground_enabled = false;
  bool stream_enabled = false;
  bool idle_midi_enabled = false;
  bool release_enabled = false;
  bool sustain_enabled = false;
  blofeld::test::VoicePcmCapture *capture = nullptr;
  Case configuration{};
  bool drive_pending = false;
  unsigned drive_return = 0;
  unsigned drive_buffer = 0;
  unsigned lfo_waves = 0, envelope_starts = 0, envelope_attacks = 0, envelope_finishes = 0,
           envelope_decays = 0, envelope_releases = 0, envelope_retirements = 0;
  unsigned gain_entries = 0, noise_entries = 0, render_entries = 0, lfo_entries = 0,
           core_entries = 0, filter_entries = 0, drive_entries = 0;

  void gain_words(const char *label, bool cache = false) {
    using enum blofeld::DspSpace;
    const auto base = cache ? get(x, entry + 2) : 0;
    trace << label << std::hex;
    for (const auto at : gain_offsets) {
      trace << ' ' << (cache ? get(x, base + at - 4) : get(y, at));
    }
    trace << '\n' << "PARAM_" << label;
    for (unsigned at = controls_begin; at <= controls_end; ++at) {
      trace << ' ' << (cache ? get(x, base + at - 4) : get(y, at));
    }
    trace << '\n' << std::dec;
  }

  static blofeld::DspBus &load(blofeld::DspBus &bus,
                               const Source &s,
                               unsigned &stop,
                               blofeld::test::VoiceMaster &master,
                               blofeld::test::VoicePcm &pcm,
                               blofeld::test::VoiceRepeat &repeat,
                               const unsigned sound_base) {
    harness(bus, stop, master, pcm, repeat, sound_base);
    bus.observe_omr(memory_mode);
    bus.advance_instruction_cycles(3);
    for (const auto &r : s.records) {
      for (unsigned i = 0; i < r.data.size(); ++i) {
        bus.write({.space = r.space, .address = r.base + i, .value = r.data[i]});
      }
    }
    return bus;
  }

  Fixture(const Source &s, std::ostream &output, const blofeld::test::VoiceInput input)
      : bus(machine.bus()),
        sound(input == blofeld::test::VoiceInput::none ? fixture_sound : primary_sound),
        cpu(machine.cpu()),
        source(s),
        trace(output),
        midi_enabled(input != blofeld::test::VoiceInput::none),
        foreground_enabled(input == blofeld::test::VoiceInput::foreground || stream_profile(input)),
        stream_enabled(stream_profile(input)),
        idle_midi_enabled(input == blofeld::test::VoiceInput::stream_midi),
        release_enabled(input == blofeld::test::VoiceInput::stream_release ||
                        input == blofeld::test::VoiceInput::stream_sustain),
        sustain_enabled(input == blofeld::test::VoiceInput::stream_sustain) {
    static_cast<void>(load(bus, s, stop, master, pcm, repeat, sound));
  }

  void put(blofeld::DspSpace space, unsigned at, unsigned v) {
    require(!rendering ||
                (space == blofeld::DspSpace::x &&
                 (at == caller_target || at == caller_status ||
                  (midi_enabled &&
                   (at == midi_producer || (at >= midi_ring && at < midi_ring + midi_ring_size))))),
            "Host synthesis-state repair is forbidden between original voice blocks");
    bus.write({.space = space, .address = at, .value = v});
  }

  unsigned get(blofeld::DspSpace space, unsigned at) {
    return bus.read({.space = space, .address = at});
  }

  [[nodiscard]] unsigned voice_frame(const unsigned nested) const {
    return nested + (foreground_enabled ? 2U : 0U);
  }

  void observe_output(const blofeld::DspState &s) {
    using enum blofeld::DspSpace;

    lfo_waves += static_cast<unsigned>(s.pc == free_sine);
    envelope_starts += static_cast<unsigned>(s.pc == envelope_start);
    envelope_attacks += static_cast<unsigned>(s.pc == envelope_attack);
    envelope_finishes += static_cast<unsigned>(s.pc == envelope_finish);
    envelope_decays += static_cast<unsigned>(s.pc == envelope_decay);
    envelope_releases += static_cast<unsigned>(s.pc == envelope_release);
    envelope_retirements += static_cast<unsigned>(s.pc == envelope_retire);
    if (s.pc == output_dispatch) {
      trace << "PRIOR_BUS" << std::hex;
      for (unsigned i = 0; i < lfo_words; ++i) { trace << ' ' << get(x, output_bus + i); }
      trace << '\n' << std::dec;
      const auto endpoint = cpu.state();
      trace << "FILTER_RETURN " << std::hex << endpoint.a << ' ' << endpoint.b << ' '
            << endpoint.r[0] << ' ' << endpoint.r[2] << ' ' << endpoint.r[3] << ' ' << endpoint.r[4]
            << ' ' << endpoint.r[5] << ' ' << endpoint.r[7] << ' ' << (endpoint.x & mask) << ' '
            << (endpoint.x >> word_bits) << ' ' << (endpoint.y & mask) << '\n';
      trace << "FILTER_ENDPOINT " << endpoint.r[1] << ' ' << endpoint.sp << ' ' << endpoint.m[5]
            << '\n';
      trace << "ROUTE_COEFFICIENTS";
      if (get(x, sound + filter_routing_parameter) != 0) {
        trace << ' ' << (endpoint.y & mask) << ' ' << (endpoint.y >> word_bits);
      } else {
        for (unsigned i = 0; i < 4; ++i) { trace << ' ' << get(y, coefficient_buffer + i); }
      }
      trace << '\n'
            << "OUTPUT_DISPATCH " << get(x, entry + amplifier_envelope) << ' '
            << get(x, entry + previous_gain) << ' ' << get(x, sound + output_controller) << '\n'
            << std::dec;
    }
    if (s.pc == output_accumulator) {
      trace << "OUTPUT_KERNEL " << std::hex << s.a << ' ' << s.b << ' '
            << get(x, entry + previous_gain) << ' ' << s.r[1] << ' ' << s.r[3] << ' ' << s.r[5]
            << ' ' << s.sp << '\n'
            << std::dec;
    }
    if (s.pc == output_tail) {
      require(!output_seen && s.sp == voice_frame(1), "Original output fallthrough frame differs");
      output_seen = true;
      trace << "OUTPUT_END " << std::hex << s.a << ' ' << s.b << ' ' << s.r[1] << ' ' << s.r[3]
            << ' ' << s.r[5] << ' ' << (s.x & mask) << ' ' << (s.x >> word_bits) << ' '
            << (s.y & mask) << ' ' << (s.y >> word_bits) << '\n'
            << std::dec;
    }
  }

  void drive_words(const char *label, const DriveWords input) {
    trace << label << ' ' << std::hex << input.index;
    for (unsigned i = 0; i < 16; ++i) {
      trace << ' ' << get(blofeld::DspSpace::x, input.address + i);
    }
    trace << '\n' << std::dec;
  }

  void observe_drive(const blofeld::DspState &s) {
    using enum blofeld::DspSpace;
    if (drive_pending && s.pc == drive_return) {
      require(s.sp == voice_frame(1), "Actual original Drive return frame differs");
      drive_words("DRIVE_OUTPUT", {.index = drive_entries - 1, .address = drive_buffer});
      trace << "DRIVE_RETURN " << std::hex << drive_entries - 1 << ' ' << s.pc << ' ' << s.sp << ' '
            << s.r[0] << ' ' << (s.x >> word_bits) << ' ' << s.r[5] << '\n'
            << std::dec;
      drive_pending = false;
    }
    if (drive_pending || s.sp != voice_frame(2) || drive_entries >= 2) { return; }
    const auto settings = configuration.drives.at(drive_entries);
    const auto callback = get(x, drive_table + settings.curve);
    if (s.pc != callback) { return; }
    const auto buffer = drive_entries == 0 ? first_filter_buffer : second_filter_buffer;
    require(s.r[5] == buffer && (s.y & mask) == std::uint64_t{settings.level} * word_unit,
            "Original Drive buffer or converted parameter differs");
    require((s.sr & drive_mode_flags) == (settings.level == 0 ? 4U : 8U),
            "Original CLR/CMP Drive entry flags differ");
    if (configuration.drive_case) {
      const auto second_return = configuration.serial ? serial_drive_return : parallel_drive_return;
      const auto expected_return = drive_entries == 0 ? first_drive_return : second_return;
      require(s.stack_pc == expected_return,
              "Original Drive call must retain its independently qualified caller frame");
      trace << "DRIVE_ENTRY " << std::hex << drive_entries << ' ' << settings.curve << ' ' << s.pc
            << ' ' << s.sp << ' ' << s.stack_pc << ' ' << s.r[5] << ' ' << (s.y & mask) << ' '
            << s.sr << '\n'
            << std::dec;
      drive_words("DRIVE_INPUT", {.index = drive_entries, .address = buffer});
      drive_words("DRIVE_OSCILLATOR", {.index = drive_entries, .address = first_oscillator_buffer});
      trace << "DRIVE_LOOKAHEAD " << std::hex << drive_entries;
      for (unsigned i = 0; i < 2; ++i) { trace << ' ' << get(x, buffer + 16 + i); }
      trace << ' ' << get(x, first_oscillator_buffer + 16) << ' ' << (s.x >> word_bits) << '\n'
            << std::dec;
      drive_pending = true;
      drive_return = s.stack_pc;
      drive_buffer = buffer;
    } else {
      require(s.pc == drive_entry && (s.y & mask) == 0,
              "Original zero-drive bypass context differs");
    }
    ++drive_entries;
  }

  void observe_filters(const blofeld::DspState &s) {
    using enum blofeld::DspSpace;
    observe_drive(s);
    if (s.pc == filters_begin) {
      for (const unsigned at : {first_filter_buffer, second_filter_buffer}) {
        trace << "MIX_OUTPUT " << std::hex << at;
        for (unsigned i = 0; i < 16; ++i) { trace << ' ' << get(x, at + i); }
        trace << '\n' << std::dec;
      }
    }
    if (s.pc == lp24_entry) {
      trace << "FILTER " << std::hex << s.r[1] << ' ' << s.r[2] << ' ' << s.r[7] << ' ' << s.sp
            << '\n'
            << std::dec;
      ++filter_entries;
    }
    if (s.pc == ordinary_biquad || s.pc == wide_biquad) {
      trace << "CORE " << std::hex << s.pc << ' ' << s.r[1] << ' ' << s.r[2] << ' ' << s.r[7] << ' '
            << s.sp;
      for (const unsigned at : {1U, 3U, 2U}) { trace << ' ' << get(y, at); }
      trace << ' ' << get(x, 3) << ' ' << get(x, 2) << '\n';
      trace << "CORE_INPUT";
      for (unsigned i = 0; i < 16; ++i) { trace << ' ' << get(x, s.r[1] + i); }
      trace << '\n' << std::dec;
      ++core_entries;
    }
    if (s.pc == serial_route || s.pc == parallel_route) {
      trace << "ROUTE " << (s.pc == serial_route) << '\n';
    }
  }

  void record_coverage(const blofeld::DspState &s) {
    using enum blofeld::DspSpace;
    if ((configuration.drive_case && s.pc >= drive_entry && s.pc < drive_end) ||
        (s.pc >= output_dispatch && s.pc < output_dispatch_end) ||
        (s.pc >= output_accumulator && s.pc < output_tail) ||
        (s.pc >= ordinary_biquad && s.pc < ordinary_biquad_end) ||
        (s.pc >= wide_biquad && s.pc < wide_biquad_end) ||
        (s.pc >= filters_begin && s.pc < output_dispatch) ||
        (s.pc >= oscillator_sync && s.pc < oscillator_end) ||
        (s.pc >= mixer_gains && s.pc < mixer_end) ||
        (s.pc >= noise_generator && s.pc < noise_end)) {
      visited.insert(s.pc);
    }
  }

  void observe_stages(const blofeld::DspState &s) {
    using enum blofeld::DspSpace;
    if (s.pc == parameters_copied) { gain_words("COPIED"); }
    if (s.pc == processing_chain) { gain_words("ROUTED"); }
    if (s.pc == lfo_initializer) {
      trace << "LFO " << std::hex << s.r[0] << ' ' << s.r[1] << ' ' << get(x, shared_seed) << '\n'
            << std::dec;
      ++lfo_entries;
    }
    if (s.pc == mixer_gains) {
      require(s.sp == voice_frame(2), "Actual mixer frame differs");
      gain_words("GAINS");
      ++gain_entries;
    }
    if (s.pc == noise_generator) {
      require(s.sp == voice_frame(3) && s.r[7] == entry + noise_history,
              "Actual noise frame/history differs");
      trace << "NOISE_INPUT " << std::hex << get(x, shared_seed) << ' ' << get(x, s.r[7]) << ' '
            << (s.x & mask) << '\n'
            << std::dec;
      ++noise_entries;
    }
    if (s.pc == mixer_renderer) {
      require(s.sp == voice_frame(2) && s.r[0] == coefficient_buffer &&
                  s.r[7] == entry + noise_descriptor && s.m[0] == 9,
              "Actual coefficient-ring/descriptor frame differs");
      trace << "COEFFICIENTS" << std::hex;
      for (unsigned i = 0; i < 10; ++i) { trace << ' ' << get(y, coefficient_buffer + i); }
      trace << '\n' << std::dec;
      ++render_entries;
    }
  }

  void observe_oscillators(const blofeld::DspState &s, bool twos) {
    using enum blofeld::DspSpace;
    if (s.pc == oscillator_sample) {
      require(s.r[0] == s.r[4] &&
                  (s.r[0] == first_descriptor || s.r[0] == second_descriptor ||
                   s.r[0] == third_descriptor) &&
                  s.n[0] == 3 && s.n[3] == 8 && s.n[4] == 5 && s.n[7] == 16 &&
                  s.sp == voice_frame(6) && s.m[0] == oscillator_modulo && s.m[1] == mask &&
                  s.m[3] == descriptor_modulo && s.m[4] == oscillator_modulo &&
                  s.m[7] == phase_modulo && (s.sr & rounding_mode) == (twos ? rounding_mode : 0),
              "Original oscillator alias/modulo/loop/rounding context differs");
    }
    syncs += static_cast<unsigned>(s.pc == oscillator_sync);
    shared += static_cast<unsigned>(s.pc == oscillator_sample);
    continues += static_cast<unsigned>(s.pc == mixer_continue);
    if (s.pc == oscillator_wrap) {
      require(s.r[0] == second_descriptor || s.r[0] == third_descriptor ||
                  s.r[0] == descriptor_ring,
              "Unexpected wrapped oscillator descriptor");
      unsigned slot = 2;
      if (s.r[0] == second_descriptor) {
        slot = 0;
      } else if (s.r[0] == third_descriptor) {
        slot = 1;
      }
      ++wraps.at(slot);
    }
  }

  void run(const Call call) {
    const auto [target, end, twos] = call;
    using enum blofeld::DspSpace;
    put(x, caller_target, target);
    put(x, caller_status, twos ? rounding_mode : 0);
    unsigned steps = 0;
    while (cpu.state().pc != end || steps == 0) {
      require(steps++ < instruction_limit, "Original sample-path instruction bound");
      const auto s = cpu.state();
      if (target == master.entry) { master.observe(bus, s, trace); }
      if (target == voice_entry) {
        observe_output(s);
        observe_filters(s);
        record_coverage(s);
        observe_stages(s);
        observe_oscillators(s, twos);
      }
      const auto old_seed = get(x, shared_seed);
      try {
        cpu.step();
        if (pcm_enabled && target == pcm.transfer && cpu.state().pc == pcm.enabled) {
          pcm.drain(bus, cpu, trace);
          if (capture != nullptr) { capture->append(pcm.decoded); }
        }
        bus.advance_instruction_cycles(1);
        ++total_steps;
      } catch (const std::exception &e) {
        throw blofeld::Fault("Integration PC=" + std::to_string(s.pc) + ": " + e.what());
      }
      if (target == voice_entry && get(x, shared_seed) != old_seed) {
        trace << "SEED_WRITE " << std::hex << s.pc << ' ' << old_seed << ' ' << get(x, shared_seed)
              << '\n'
              << std::dec;
      }
    }
  }

  void reset_stream(bool twos) {
    using enum blofeld::DspSpace;
    if (stream_enabled) { cpu.reset_spi_boot(); }
    // Load with the source image's switched mapping, then reset CPU and bus
    // together. The real harness performs the next switch with its delay NOPs.
    bus.observe_omr(memory_mode);
    bus.advance_instruction_cycles(3);
    for (const auto &r : source.records) {
      for (unsigned i = 0; i < r.data.size(); ++i) { put(r.space, r.base + i, r.data[i]); }
    }
    cpu.reset_spi_boot();
    run({.target = register_prefix, .end = register_prefix_end, .twos = twos});
    cpu.reset_spi_boot();
    run({.target = waveform_initializer, .end = stop, .twos = twos});
    require(cpu.state().sp == 0, "Original waveform initializer did not return");
    for (unsigned i = 0; i < fixture_sound_words; ++i) {
      put(x, sound + i, i < raw_sound_words ? get(y, factory_sound + i) & byte_mask : 0);
    }
    run({.target = controller_reset, .end = stop, .twos = twos});
    require(cpu.state().sp == 0, "Original controller reset did not return");
    require(get(x, sound + output_controller) == reset_controller_level &&
                get(x, sound + secondary_reset_controller) == reset_controller_level &&
                get(x, sound + first_controller_depth) == 0 &&
                get(x, sound + second_controller_depth) == 0,
            "Original controller reset fields differ");
    for (unsigned i = 8; i < readable_entry_words; ++i) { put(x, entry + i, 0); }
    // Explicit previous state/history and output-scratch readability. The genuine
    // source pointer initializer below replaces the pointer words in this area.
    for (auto space : {x, y}) {
      for (unsigned at = descriptor_ring; at < oscillator_descriptor_end; ++at) {
        put(space, at, 0);
      }
      for (unsigned at = first_oscillator_buffer; at < noise_buffer; ++at) { put(space, at, 0); }
      for (unsigned at = first_filter_buffer; at < output_bus; ++at) { put(space, at, 0); }
    }
    run({.target = pointer_prefix, .end = pointer_prefix_end, .twos = twos});
    cpu.reset_spi_boot();
  }

  void configure_sound(const Case input) {
    using enum blofeld::DspSpace;
    const auto high = input.high;
    const auto twos = input.twos;
    const auto color = input.color;
    const auto varied = input.varied;
    const auto serial = input.serial;
    put(x, entry + 8, sound);
    put(x, entry + 11, 0x400000);
    put(x, entry + 12, high ? converted_high_note : 0U);
    put(x, entry + 13, high ? converted_high_note : 0U);
    put(x, 0, initial_left);
    put(x, sound_context, sound);
    put(x, entry_context, entry);
    put(x, caller_latch, 0);
    put(x, sound + shape_parameter, 3);
    put(x, sound + oscillator_parameter_stride + shape_parameter, 4);
    put(x, sound + (2 * oscillator_parameter_stride) + shape_parameter, 3);
    for (unsigned osc = 0; osc < 3; ++osc) {
      put(x, sound + (16 * osc) + 6, 0); // Actual zero-FM-source path; no substituted callee.
      if (high) {
        put(x, sound + (16 * osc) + 1, high_pitch[0]);
        put(x, sound + (16 * osc) + 2, high_pitch[1]);
        put(x, sound + (16 * osc) + 3, high_pitch[2]);
      }
    }
    put(x, sound + oscillator_sync_parameter, high ? 1U : 0U);
    // The original mixer reads X:$570 even for zero noise gain. Enable its actual
    // noise generator so this input has source-executed provenance in the fixture.
    put(x, sound + noise_level_parameter, 64);
    put(x,
        sound + noise_color_parameter,
        color); // Noise Color, originally default64; caller converts it.
    if (varied) {
      // Mixer fields61..68, ring Level71/Balance72. Preserve original conversion.
      constexpr std::array<unsigned, 8> levels{101, 127, 89, 64, 73, 32, 96, 113};
      for (unsigned i = 0; i < levels.size(); ++i) {
        put(x, sound + mixer_parameters_begin + i, levels[i]);
      }
      put(x, sound + ring_level_parameter, varied_ring_level);
      put(x, sound + ring_balance_parameter, varied_ring_balance);
    }
    put(x, sound + filter_routing_parameter, serial ? 1U : 0U);
    if (varied) {
      for (const auto setting : varied_filters) { put(x, sound + setting.field, setting.value); }
    }
    if (input.drive_case) {
      put(x, sound + first_drive_level, input.drives[0].level);
      put(x, sound + first_drive_curve, input.drives[0].curve);
      put(x, sound + second_drive_level, input.drives[1].level);
      put(x, sound + second_drive_curve, input.drives[1].curve);
    }
    run({.target = entry_initializer, .end = stop, .twos = twos});
    require(cpu.state().sp == 0, "Original entry initializer did not return");
  }

  void prepare_output(const Case input) {
    using enum blofeld::DspSpace;
    const auto high = input.high;
    const auto twos = input.twos;
    const auto color = input.color;
    const auto varied = input.varied;
    const auto serial = input.serial;
    trace << "CASE " << (high ? "sync" : "normal") << ' ' << twos << ' ' << color << ' ' << varied
          << ' ' << serial << '\n';
    if (input.drive_case) {
      trace << "DRIVE_SETTING " << input.drives[0].curve << ' ' << input.drives[0].level << ' '
            << input.drives[1].curve << ' ' << input.drives[1].level << ' ' << input.blocks << '\n';
    }
    gain_words("CACHE", true);
    trace << "SETUP " << std::hex << get(x, shared_seed) << ' ' << get(x, entry + noise_color)
          << ' ' << get(x, entry + noise_history);
    for (unsigned i = 0; i < 4; ++i) { trace << ' ' << get(x, entry + noise_descriptor + i); }
    trace << '\n' << std::dec;
    // Explicit readable neighboring filter/serial-pipeline inputs; no boot-RAM claim.
    for (unsigned i = 0; i < stereo_words; ++i) {
      const unsigned value = initial_bus_word(i, varied);
      put(x, output_bus + i, value);
    }
    put(y, output_prefetch, prefetch_value);
  }

  void begin_block(unsigned block) {
    using enum blofeld::DspSpace;

    trace << "BLOCK " << block << '\n';
    callbacks("CALLBACKS_BEFORE");
    output_seen = false;
    lfo_waves = envelope_starts = envelope_attacks = envelope_finishes = envelope_decays = 0;
    envelope_releases = envelope_retirements = 0;
    trace << "HISTORY_INIT" << std::hex;
    for (const unsigned base : {first_filter_history, second_filter_history}) {
      for (auto space : {x, y}) {
        for (unsigned i = 0; i < 4; ++i) { trace << ' ' << get(space, base + i); }
      }
    }
    trace << '\n' << std::dec;
    gain_entries = noise_entries = render_entries = lfo_entries = core_entries = filter_entries =
        drive_entries = 0;
    syncs = 0;
    shared = 0;
    continues = 0;
    wraps = {};
  }

  void check_block(unsigned block, const unsigned expected_sp = 0) {
    using enum blofeld::DspSpace;

    require(cpu.state().sp == expected_sp && get(x, entry + mixer_callback) == steady_mixer &&
                shared == shared_samples_per_block && continues == 16,
            "Original three-oscillator loop/mixer return differs");
    const auto counter = stream_enabled ? block * 0x1000U : 0U;
    require(get(x, 0xd) == byte_mask && get(x, 4) == counter,
            "Original low-byte mask or declared block counter changed");
    for (const auto &[at, v] : source.program) {
      require(bus.peek_program(at) == v, "Original runtime executable changed");
    }
    require(gain_entries == 1 && noise_entries == 1 && render_entries == 1 &&
                lfo_entries == (block == 0 ? 3U : 0U),
            "Missing original integrated caller stage");
    require(core_entries == 4 && filter_entries == 2 && drive_entries == 2,
            "Missing real filter/drive calls");
    require(output_seen, "Missing original output fallthrough");
    require(!drive_pending, "Original Drive callback has not actually returned");
    check_control_counts(block);
  }

  void check_control_counts(const unsigned block) const {
    const auto request = sustain_enabled ? sustain_release_block : release_input_block;
    const auto retirement = request + (release_retirement_block - release_input_block);
    const bool releasing = release_enabled && block > request;
    const auto release_callbacks = block == retirement ? 2U : 4U;
    const auto releases = releasing ? release_callbacks : 0U;
    const auto retirements = releasing && block == retirement ? 2U : 0U;
    require(lfo_waves == (block == 0 ? 6U : 3U) && envelope_starts == (block == 0 ? 4U : 0U) &&
                envelope_attacks == (block == 0 ? 8U : 0U) &&
                envelope_finishes == (block == 0 ? 4U : 0U) &&
                envelope_decays == (block == 0 || releasing ? 0U : 4U) &&
                envelope_releases == releases && envelope_retirements == retirements,
            "Original per-block control callback counts differ");
  }

  void emit_samples() {
    using enum blofeld::DspSpace;
    trace << std::hex;
    trace << "PITCH " << get(y, 5) << ' ' << get(y, 10) << ' ' << get(y, 15) << '\n';
    for (const unsigned at : {first_oscillator_buffer,
                              second_oscillator_buffer,
                              third_oscillator_buffer,
                              noise_buffer,
                              first_filter_buffer,
                              second_filter_buffer}) {
      trace << "OUTPUT " << at;
      for (unsigned i = 0; i < 16; ++i) { trace << ' ' << get(x, at + i); }
      trace << '\n';
    }
  }

  void emit_state() {
    using enum blofeld::DspSpace;
    callbacks("CALLBACKS_AFTER");
    trace << std::hex;
    trace << "HISTORIES";
    for (const unsigned base : {first_filter_history, second_filter_history}) {
      for (auto space : {x, y}) {
        for (unsigned i = 0; i < 4; ++i) { trace << ' ' << get(space, base + i); }
      }
    }
    trace << '\n' << "ROUTE_OUTPUT";
    for (unsigned i = 0; i < stereo_words; ++i) { trace << ' ' << get(y, routed_buffer + i); }
    trace << '\n'
          << "NOISE_STATE " << get(x, shared_seed) << ' ' << get(x, entry + noise_history) << '\n';
    const auto end = cpu.state();
    trace << "WHOLE_RETURN " << end.a << ' ' << end.b << ' ' << end.r[0] << ' ' << end.r[2] << ' '
          << end.r[3] << ' ' << end.r[4] << ' ' << end.r[5] << ' ' << end.r[7] << ' '
          << (end.x & mask) << ' ' << (end.x >> word_bits) << ' ' << (end.y & mask) << '\n';
    trace << "OUTPUT_BUS";
    for (unsigned i = 0; i < stereo_words; ++i) { trace << ' ' << get(x, output_bus + i); }
    trace << '\n' << "FINAL_GAIN " << get(x, entry + previous_gain) << '\n';
    trace << "FINAL_LFO";
    for (unsigned i = 0; i < lfo_words; ++i) { trace << ' ' << get(x, lfo_state + i); }
    trace << '\n' << "FINAL_ENVELOPES";
    for (unsigned i = 0; i < envelope_words; ++i) { trace << ' ' << get(x, envelope_state + i); }
    trace << '\n'
          << "PHASE " << get(y, first_phase) << ' ' << get(y, second_phase) << ' '
          << get(y, third_phase) << '\n';
    trace << "CURSOR " << get(x, 0xf) << " MASK " << get(x, 0xd) << '\n' << std::dec;
    trace << "WRAPS " << wraps[0] << ' ' << wraps[1] << ' ' << wraps[2] << " SYNC " << syncs
          << '\n';
  }

  void publish_frames(const std::span<const blofeld::SerialPcmFrame> decoded,
                      const unsigned count) const {
    require(count <= decoded.size(), "Decoded frame count exceeds the verified buffer");
    if (stream_output == nullptr) { return; }
    // External output follows the final verified DMA/ESAI replay branch, once.
    // It never becomes part of modeled bus/receiver checkpoints.
    for (unsigned frame = 0; frame < count; ++frame) { stream_output->accept(decoded[frame]); }
  }

  void sample_case(const Case input) {
    if (capture != nullptr) { capture->begin(); }
    if (stream_output != nullptr) {
      stream_output->claim_source(std::make_shared<const bool>(true));
    }
    rendering = false;
    configuration = input;
    reset_stream(input.twos);
    if (release_enabled) {
      run({.target = entry_list_initializer, .end = stop, .twos = input.twos});
    }
    configure_sound(input);
    prepare_output(input);
    if (midi_enabled) { initialize_midi(input.twos); }
    if (master_enabled) { blofeld::test::VoiceMaster::initialize(bus); }
    if (stream_enabled) {
      stream_case(input);
      return;
    }
    if (foreground_enabled) {
      foreground_case(input);
      return;
    }
    if (pcm_enabled) {
      pcm.begin(bus);
      run({.target = pcm.setup, .end = stop, .twos = input.twos});
      pcm.baseline(bus);
    }
    rendering = true;
    for (unsigned block = 0; block < input.blocks; ++block) {
      if (stream_output != nullptr) { stream_output->check(); }
      begin_block(block);
      if (midi_enabled) { midi_block(block, input.twos); }
      run({.target = voice_entry, .end = stop, .twos = input.twos});
      check_block(block);
      emit_samples();
      emit_state();
      if (master_enabled) {
        run({.target = master.entry, .end = stop, .twos = input.twos});
        require(cpu.state().sp == 0, "Original copy/mix/master calls must genuinely return");
        if (pcm_enabled) {
          run({.target = pcm.transfer, .end = stop, .twos = input.twos});
          publish_frames(pcm.decoded, pcm.frames);
        }
      }
    }
    rendering = false;
  }

  void initialize_midi(const bool twos) {
    using enum blofeld::DspSpace;
    // Original single-part dispatch selects X:$936; no relocated parser or firmware patch.
    require(sound == primary_sound, "Original MIDI parser requires its actual part base");
    put(x, midi_channel_setting, 1); // MIDI channel 1, zero-based status nibble 0.
    for (unsigned i = 0; i < 4; ++i) { put(x, midi_control_assignments + i, parameter_max); }
    for (unsigned i = 0; i < midi_ring_size; ++i) { put(x, midi_ring + i, initial_left ^ i); }
    require(get(x, midi_producer) == midi_ring && get(x, midi_consumer) == midi_ring,
            "Original pointer initializer must supply the empty primary ring");
    run({.target = midi_parser, .end = stop, .twos = twos});
  }

  void initialize_foreground(const Case input) {
    using enum blofeld::DspSpace;
    for (unsigned part = 0; part < part_count; ++part) {
      if (part != 0) {
        for (unsigned i = 0; i < fixture_sound_words; ++i) {
          put(x,
              primary_sound + (part * part_stride) + i,
              i < raw_sound_words ? get(y, factory_sound + i) & byte_mask : 0);
        }
      }
      // Explicit Effects Off profile; run the original descriptor initialization.
      put(x, primary_sound + (part * part_stride) + first_effect_type, 0);
      put(x, primary_sound + (part * part_stride) + second_effect_type, 0);
      put(y, event_base + (event_stride * part) + queued_events, 0);
    }
    run({.target = reset_parts, .end = stop, .twos = input.twos});
    run({.target = effect_dispatch, .end = stop, .twos = input.twos});
    for (unsigned i = 0; i < secondary_ring_size; ++i) { put(x, secondary_ring + i, 0); }
    put(x, event_read, event_ring);
    put(x, event_write, event_ring);
    put(x, response_read, response_ring);
    put(x, response_write, response_ring);
    put(x, active_list, entry);
    put(x, active_list + 1, 0);
    run({.target = event_initializer, .end = stop, .twos = input.twos});
    put(x, block_countdown, 2);
    put(x, scheduler_period, declared_period);
    put(x, scheduler_timer, 0);
    put(x, scheduler_pending, 0);
    put(x, phase_word, 0);               // Declared previous half, not a recovered reset value.
    put(x, dma_source, declared_source); // Disabled DMA; no fabricated transfer/clock.
  }

  void foreground_input(const bool commit_then_repeat) {
    using enum blofeld::DspSpace;
    const auto value = commit_then_repeat ? parameter_max : 0U;
    std::vector<unsigned> bytes{midi_controller_status, midi_volume, value};
    if (commit_then_repeat) { bytes.insert(bytes.end(), {midi_wheel, 0, midi_volume, value}); }
    trace << "FOREGROUND_INPUT" << std::hex;
    for (unsigned i = 0; i < bytes.size(); ++i) {
      put(x, midi_ring + i, bytes[i] << midi_byte_shift);
      trace << ' ' << bytes[i];
    }
    put(x, midi_producer, midi_ring + static_cast<unsigned>(bytes.size()));
    trace << '\n' << std::dec;
  }

  void foreground_voice_step() {
    using enum blofeld::DspSpace;
    const auto s = cpu.state();
    observe_output(s);
    observe_filters(s);
    record_coverage(s);
    observe_stages(s);
    observe_oscillators(s, configuration.twos);
    const auto seed = get(x, shared_seed);
    foreground_instruction();
    if (get(x, shared_seed) != seed) {
      trace << "SEED_WRITE " << std::hex << s.pc << ' ' << seed << ' ' << get(x, shared_seed)
            << '\n'
            << std::dec;
    }
  }

  void foreground_state() {
    using enum blofeld::DspSpace;
    const auto s = cpu.state();
    require(s.sp == 1 && s.stack_pc == stop,
            "Original foreground retains the own native entry frame; no fabricated RTS");
    trace << "FOREGROUND_END" << std::hex << ' ' << s.pc << ' ' << s.sp << ' ' << s.a << ' ' << s.b
          << ' ' << s.x << ' ' << s.y << ' ' << s.r[0] << ' ' << s.sr << '\n';
    foreground_words("FOREGROUND_RAM");
    trace << "FOREGROUND_PREVIOUS_OUTPUT" << std::hex;
    for (unsigned i = 0; i < stereo_words; ++i) {
      trace << ' ' << get(x, declared_source + stereo_words + i);
    }
    trace << '\n' << std::dec;
  }

  void foreground_words(const char *label) {
    using enum blofeld::DspSpace;
    trace << label << std::hex;
    for (const auto at : {4U,
                          shared_seed,
                          runtime_flags,
                          transition_destination,
                          transition_target,
                          scheduler_pending,
                          scheduler_period,
                          scheduler_timer,
                          block_countdown,
                          sound + output_controller,
                          midi_producer,
                          midi_consumer,
                          phase_word,
                          dma_source}) {
      trace << ' ' << get(x, at);
    }
    trace << '\n' << std::dec;
  }

  void foreground_idle_input() {
    using enum blofeld::DspSpace;
    auto cursor = get(x, midi_producer);
    require(cursor == get(x, midi_consumer), "Original foreground must consume its initial input");
    trace << "FOREGROUND_IDLE_INPUT" << std::hex;
    for (const auto byte : {midi_volume, 64U}) {
      put(x, cursor, byte << midi_byte_shift);
      cursor = midi_ring + ((cursor - midi_ring + 1) % midi_ring_size);
      trace << ' ' << byte;
    }
    put(x, midi_producer, cursor);
    trace << '\n' << std::dec;
  }

  [[nodiscard]] Words foreground_audio_state(const bool releasing = false) {
    using enum blofeld::DspSpace;
    Words values;
    for (const auto base : {first_filter_history, second_filter_history}) {
      for (const auto space : {x, y}) {
        for (unsigned i = 0; i < 4; ++i) { values.push_back(get(space, base + i)); }
      }
    }
    for (const auto at : {entry + previous_gain, entry + noise_history}) {
      values.push_back(get(x, at));
    }
    for (const auto [base, count] : {std::pair{lfo_state, lfo_words},
                                     std::pair{envelope_state, envelope_words},
                                     std::pair{output_bus, stereo_words},
                                     std::pair{0x620U, stereo_words * 4}}) {
      for (unsigned i = 0; i < count; ++i) {
        // Idle release changes only these callbacks; the release oracle checks their values.
        if (!releasing || !release_callback(base + i)) { values.push_back(get(x, base + i)); }
      }
    }
    for (const auto at : {first_phase, second_phase, third_phase}) { values.push_back(get(y, at)); }
    return values;
  }

  void foreground_idle() {
    const auto audio = foreground_audio_state();
    foreground_idle_input();
    constexpr unsigned iterations = 8;
    for (unsigned iteration = 0; iteration < iterations; ++iteration) {
      trace << "FOREGROUND_IDLE_BEGIN " << iteration << '\n';
      unsigned steps = 0;
      do {
        require(steps++ < instruction_limit, "Original foreground idle iteration bound");
        const auto s = cpu.state();
        require(s.pc != voice_entry && s.pc != master_output && s.pc != scheduler_tail,
                "A stationary DMA cursor cannot dispatch another voice/master/scheduler block");
        if (std::ranges::find(foreground_stages, s.pc) != foreground_stages.end()) {
          trace << "FOREGROUND_IDLE_STAGE " << std::hex << s.pc << ' ' << s.sp << '\n' << std::dec;
        }
        cpu.step();
        bus.advance_instruction_cycles(1);
        ++total_steps;
      } while (cpu.state().pc != main_loop);
      const auto s = cpu.state();
      require(s.sp == 1 && s.stack_pc == stop,
              "Actual idle iteration retains the original parent entry frame");
      trace << "FOREGROUND_IDLE_END" << std::hex << ' ' << s.pc << ' ' << s.sp << ' ' << s.a << ' '
            << s.b << ' ' << s.sr << '\n'
            << std::dec;
      foreground_words("FOREGROUND_IDLE_RAM");
      require(foreground_audio_state() == audio,
              "Idle input polling cannot advance voice/control histories or audio buffers");
    }
  }

  void foreground_instruction() {
    if (stream_enabled) {
      blofeld::test::VoiceRepeat::instruction(machine);
    } else {
      cpu.step();
      bus.advance_instruction_cycles(1);
    }
  }

  void foreground_iteration(const unsigned block) {
    begin_block(block);
    const auto retirement = sustain_enabled ? sustain_release_block + 3 : release_retirement_block;
    const auto expected_voices = release_enabled && block > retirement ? 0U : 1U;
    const auto frozen = expected_voices == 0 ? entry_payload() : Words{};
    bool in_voice = false;
    unsigned voices = 0;
    unsigned steps = 0;
    do {
      require(steps++ < instruction_limit, "Original foreground iteration bound");
      const auto pc = cpu.state().pc;
      if (stream_enabled) { stream_observe(pc); }
      if (std::ranges::find(foreground_stages, pc) != foreground_stages.end()) {
        trace << "FOREGROUND_STAGE " << std::hex << pc << ' ' << cpu.state().sp << '\n' << std::dec;
      }
      if (pc == voice_entry) {
        in_voice = true;
        ++voices;
      }
      if (in_voice && pc == active_entry_next) {
        check_block(block, 2);
        emit_samples();
        emit_state();
        in_voice = false;
      }
      if (in_voice) {
        foreground_voice_step();
      } else {
        foreground_instruction();
      }
      ++total_steps;
    } while (cpu.state().pc != main_loop);
    require(!in_voice && voices == expected_voices,
            "Original foreground must follow the actual retained or retired entry");
    require(frozen.empty() || entry_payload() == frozen,
            "An empty active list must preserve the retired slot's complete payload");
    if (release_enabled) { trace << "RELEASE_RENDERED " << voices << '\n'; }
    foreground_state();
  }

  [[nodiscard]] Words entry_payload() {
    Words words;
    for (unsigned i = 0; i < readable_entry_words; ++i) {
      words.push_back(get(blofeld::DspSpace::x, entry + i));
    }
    return words;
  }

  void stream_row(const char *label, const unsigned base) {
    trace << label << std::hex;
    for (unsigned i = 0; i < stereo_words; ++i) {
      trace << ' ' << get(blofeld::DspSpace::x, base + i);
    }
    trace << '\n' << std::dec;
  }

  void stream_observe(const unsigned pc) {
    using enum blofeld::DspSpace;
    if (pc == master_completed) {
      stream_row("REPEAT_MASTER", declared_source + get(x, phase_word));
      trace << "REPEAT_HISTORY" << std::hex;
      for (const auto space : {x, y}) {
        for (unsigned i = 0; i < 2; ++i) { trace << ' ' << get(space, master_history + i); }
      }
      trace << '\n' << std::dec;
    }
    if (pc == mix_completed) {
      stream_row("REPEAT_MIX", mixed_bus);
      stream_row("REPEAT_CLEARED", output_bus);
      trace << "REPEAT_GAINS " << std::hex << get(x, chain_gain) << ' ' << get(x, chain_gain + 1)
            << '\n'
            << std::dec;
    }
  }

  void foreground_case(const Case input) {
    initialize_foreground(input);
    foreground_input(input.varied);
    run({.target = main_loop, .end = main_loop, .twos = input.twos});
    rendering = true;
    foreground_iteration(0);
    foreground_idle();
    for (const auto &[at, value] : source.program) {
      require(bus.peek_program(at) == value, "Original foreground executable word changed");
    }
    rendering = false;
  }

  [[nodiscard]] std::vector<unsigned> idle_audio_state(const bool releasing = false) {
    using enum blofeld::DspSpace;
    auto values = foreground_audio_state(releasing);
    const auto range = [&](const auto space, const MemoryRange interval) {
      for (unsigned i = 0; i < interval.count; ++i) {
        const auto at = interval.base + i;
        const bool release_write =
            releasing &&
            ((space == x && (release_callback(at) || at == entry + release_velocity_field)) ||
             (space == y && at == coefficient_buffer));
        if (!release_write) { values.push_back(get(space, at)); }
      }
    };
    range(x, {.base = entry, .count = readable_entry_words});
    range(x, {.base = first_oscillator_buffer, .count = 4 * stereo_words});
    range(x, {.base = mixed_bus, .count = 4 * stereo_words});
    range(x, {.base = output_bus + stereo_words, .count = 2});
    range(y, {.base = coefficient_buffer, .count = (2 * stereo_words) + 1});
    for (const auto space : {x, y}) {
      range(space, {.base = master_history, .count = 2});
      range(space, {.base = first_filter_history, .count = 4});
      range(space, {.base = second_filter_history, .count = 4});
    }
    for (const auto at : {sound + output_controller, dma_source - 3, dma_source - 2}) {
      range(x, {.base = at, .count = 1});
    }
    return values;
  }

  void idle_iteration(const unsigned ordinal, const bool releasing = false) {
    using enum blofeld::DspSpace;
    const auto audio = idle_audio_state(releasing);
    const auto wire = repeat;
    trace << "IDLE_BEGIN " << std::hex << ordinal << '\n' << std::dec;
    unsigned steps = 0;
    do {
      require(steps++ < instruction_limit, "Original idle foreground iteration bound");
      const auto pc = cpu.state().pc;
      if (std::ranges::find(foreground_stages, pc) != foreground_stages.end()) {
        require(pc == main_loop || pc == midi_parser || pc == midi_secondary_parser ||
                    pc == response_parser,
                "Original idle foreground must not render or advance the scheduler");
        trace << "IDLE_STAGE " << std::hex << pc << ' ' << cpu.state().sp << '\n' << std::dec;
      }
      foreground_instruction();
      ++total_steps;
    } while (cpu.state().pc != main_loop);
    const auto s = cpu.state();
    require(s.sp == 1 && s.stack_pc == stop && repeat == wire &&
                idle_audio_state(releasing) == audio,
            "Idle MIDI polling must preserve audio state, native frame and serial/DMA phase");
    trace << "IDLE_END " << std::hex << s.pc << ' ' << s.sp << ' ' << s.a << ' ' << s.b << ' '
          << s.sr << '\n';
    trace << "IDLE_MIDI " << get(x, midi_producer) << ' ' << get(x, midi_consumer);
    for (unsigned at = midi_staged_status; at < midi_staged_status + 3; ++at) {
      trace << ' ' << get(x, at);
    }
    trace << ' ' << get(x, transition_destination) << ' ' << get(x, transition_target) << ' '
          << (get(x, runtime_flags) & transition_flag) << ' ' << get(x, sound + output_controller)
          << ' ' << get(x, sound + modulation_controller) << ' ' << s.sp << '\n'
          << std::dec;
    foreground_words("IDLE_RAM");
  }

  void
  idle_chunk(const std::vector<unsigned> &bytes, unsigned &ordinal, const bool releasing = false) {
    using enum blofeld::DspSpace;
    auto cursor = get(x, midi_producer);
    require(cursor == get(x, midi_consumer) && bytes.size() < midi_ring_size,
            "Idle MIDI transport requires an empty bounded input ring");
    trace << "IDLE_BYTES" << std::hex;
    for (const auto byte : bytes) {
      require(byte <= byte_mask, "Idle MIDI transport supplies bytes");
      put(x, cursor, byte << midi_byte_shift);
      cursor = midi_ring + ((cursor - midi_ring + 1) % midi_ring_size);
      trace << ' ' << byte;
    }
    put(x, midi_producer, cursor);
    trace << '\n' << std::dec;
    idle_iteration(ordinal++, releasing);
    idle_iteration(ordinal++, releasing);
  }

  void stream_idle(const unsigned block) {
    unsigned ordinal = 0;
    if (block == 0) {
      idle_chunk({midi_volume, 64}, ordinal);
    } else if (block == 1) {
      idle_chunk({midi_volume}, ordinal);
      idle_chunk({0}, ordinal);
    } else if (block == 2) {
      idle_chunk({midi_controller_status | 1U, midi_volume, parameter_max}, ordinal);
      idle_chunk({midi_controller_status, midi_volume, 1}, ordinal);
    } else if (block == 3) {
      idle_chunk({midi_volume, parameter_max, midi_volume, 32}, ordinal);
    } else if (block == 4) {
      std::vector<unsigned> burst;
      for (unsigned value = 0; value < idle_burst_pairs; ++value) {
        burst.insert(burst.end(), {midi_volume, value});
      }
      idle_chunk(burst, ordinal);
    } else {
      idle_chunk({}, ordinal);
    }
    for (const auto &[at, value] : source.program) {
      require(bus.peek_program(at) == value, "Original idle foreground executable changed");
    }
  }

  void initialize_release_stream() {
    using enum blofeld::DspSpace;
    require(get(x, entry + release_note_field) == 0 && get(x, entry + release_tag_field) == 0 &&
                get(x, owner_mask) == 0,
            "Release stream declares the existing zero note/tag and empty owner mask");
    for (unsigned i = 0; i < release_parameters.size(); ++i) {
      require(get(x, sound + release_parameters[i]) == (i < 2 ? 0U : 64U),
              "Release stream retains the original factory release controls");
    }
    for (unsigned i = 0; i < owner_count; ++i) { put(x, owner_base + i, 0); }
  }

  void release_state(const char *label) {
    using enum blofeld::DspSpace;
    trace << label << std::hex;
    for (const auto at : {entry + release_note_field,
                          entry + release_tag_field,
                          entry + release_velocity_field,
                          active_list,
                          active_list + 1,
                          runtime_flags,
                          owner_mask}) {
      trace << ' ' << get(x, at);
    }
    for (unsigned i = 0; i < envelope_words; ++i) { trace << ' ' << get(x, envelope_state + i); }
    for (unsigned i = 0; i < owner_count; ++i) { trace << ' ' << get(x, owner_base + i); }
    trace << '\n' << std::dec;
  }

  void release_input(const unsigned block) {
    release_state("RELEASE_AFTER_BODY");
    if (sustain_enabled) {
      sustain_input(block);
      return;
    }
    if (block != release_input_block) { return; }
    unsigned ordinal = 0;
    const auto status = configuration.serial ? zero_note_on_status : note_off_status;
    const auto velocity = configuration.serial ? 0U : 64U;
    idle_chunk({status, 0, velocity}, ordinal, true);
    require(get(blofeld::DspSpace::y, coefficient_buffer) == entry,
            "Original release must gather the actual held pointer in its aliased Y scratch");
    release_state("RELEASE_AFTER_INPUT");
  }

  void sustain_state(const char *label) {
    using enum blofeld::DspSpace;
    trace << label << std::hex << ' ' << get(x, sound + sustain_scalar) << ' '
          << get(x, sound + sustain_flag);
    for (unsigned i = 0; i < note_latch_words; ++i) {
      trace << ' ' << get(x, sound + first_note_latch + i);
    }
    trace << '\n' << std::dec;
  }

  void sustain_input(const unsigned block) {
    sustain_state("SUSTAIN_AFTER_BODY");
    unsigned ordinal = 0;
    if (block == 0) {
      idle_chunk(
          {midi_controller_status, sustain_controller, configuration.serial ? parameter_max : 1U},
          ordinal);
    } else if (block == release_input_block) {
      idle_chunk({configuration.serial ? zero_note_on_status : note_off_status,
                  0,
                  configuration.serial ? 0U : 64U},
                 ordinal);
    } else if (block == sustain_release_block) {
      idle_chunk({midi_controller_status, sustain_controller}, ordinal);
      idle_chunk({0}, ordinal, true);
      require(get(blofeld::DspSpace::y, coefficient_buffer) == entry,
              "Pedal release gathers the held entry through the actual aliased scratch");
    } else {
      return;
    }
    release_state("RELEASE_AFTER_INPUT");
    sustain_state("SUSTAIN_AFTER_INPUT");
  }

  [[nodiscard]] unsigned stream_block_count() const {
    if (sustain_enabled) { return sustain_stream_blocks; }
    return idle_midi_enabled || release_enabled ? midi_stream_blocks : 6U;
  }

  void stream_case(const Case input) {
    using enum blofeld::DspSpace;
    initialize_foreground(input);
    foreground_input(input.varied);
    if (release_enabled) { initialize_release_stream(); }
    // This finite reference window has no due part-clock/countdown service.
    put(x, scheduler_period, positive_limit);
    put(x, block_countdown, 0);
    for (unsigned i = 0; i < stereo_words; ++i) { put(x, output_bus + i, 0); }
    run({.target = pcm.setup, .end = stop, .twos = input.twos});
    repeat.begin(bus);
    put(x, caller_target, repeat.entry);
    put(x, caller_status, input.twos ? rounding_mode : 0);
    unsigned steps = 0;
    do {
      require(steps++ < instruction_limit, "Own repeating DMA entry bound");
      blofeld::test::VoiceRepeat::instruction(machine);
    } while (cpu.state().pc != main_loop);
    const auto before = cpu.debug_state();
    bool refused = false;
    try {
      bus.advance_instruction_cycles(1);
    } catch (const blofeld::Fault &error) {
      refused = std::string(error.what()).contains("Unqualified automatic DSP DMA bus timing");
    }
    require(refused && cpu.debug_state() == before,
            "Automatic DMA timing rejection remains intact in the explicit untimed profile");
    trace << "REPEAT_AUTOMATIC_REJECTED 1\n";
    rendering = true;
    for (unsigned block = 0; block < input.blocks; ++block) {
      if (stream_output != nullptr) { stream_output->check(); }
      foreground_iteration(block);
      if (idle_midi_enabled) { stream_idle(block); }
      if (release_enabled) { release_input(block); }
      repeat.drain(bus, cpu, trace);
      publish_frames(repeat.decoded, repeat.frames);
    }
    rendering = false;
  }

  void send_midi(const std::vector<unsigned> &bytes, const bool twos) {
    using enum blofeld::DspSpace;
    auto cursor = get(x, midi_producer);
    require(cursor == get(x, midi_consumer) && bytes.size() < midi_ring_size,
            "Own MIDI transport supplies a bounded empty ring");
    trace << "MIDI_BYTES" << std::hex;
    for (const auto byte : bytes) {
      require(byte <= byte_mask, "Own MIDI transport supplies bytes");
      trace << ' ' << byte;
      put(x, cursor, byte << midi_byte_shift);
      cursor = midi_ring + ((cursor - midi_ring + 1) % midi_ring_size);
    }
    put(x, midi_producer, cursor);
    trace << '\n';
    run({.target = midi_parser, .end = stop, .twos = twos});
    require(cpu.state().sp == 0 && get(x, midi_consumer) == cursor,
            "Original parser must consume supplied bytes and actually return");
    trace << "MIDI_RETURN " << cursor << ' ' << get(x, midi_consumer);
    for (unsigned at = midi_staged_status; at < midi_staged_status + 3; ++at) {
      trace << ' ' << get(x, at);
    }
    trace << ' ' << get(x, transition_destination) << ' ' << get(x, transition_target) << ' '
          << (get(x, runtime_flags) & transition_flag) << ' ' << get(x, sound + output_controller)
          << ' ' << get(x, sound + modulation_controller) << ' ' << cpu.state().sp << '\n'
          << std::dec;
  }

  void midi_block(const unsigned block, const bool twos) {
    const auto value = midi_volumes.at(block);
    if (block == 2) {
      for (const auto byte : {midi_controller_status, midi_volume, value}) {
        send_midi({byte}, twos);
      }
    } else {
      if (block == 3) {
        send_midi({midi_controller_status | 1U, midi_volume, parameter_max}, twos);
      }
      if (block == 1) {
        send_midi({midi_volume, value}, twos); // Running status survives the preceding block.
      } else {
        send_midi({midi_controller_status, midi_volume, value}, twos);
      }
    }
    // A subsequent real wheel command commits the previous volume transition.
    // This tests original destination switching, not the main-loop smoothing clock.
    send_midi({midi_wheel, 0}, twos);
  }

  void callbacks(const char *label) {
    using enum blofeld::DspSpace;
    trace << label << std::hex << ' ' << get(x, entry);
    for (const unsigned offset : {0xacU, 0xaeU, 0xb0U, mixer_callback}) {
      trace << ' ' << get(x, entry + offset);
    }
    trace << '\n' << std::dec;
  }
};
}

namespace {
void baseline(Fixture &fixture) {
  for (const bool high : {false, true}) {
    for (const bool twos : {false, true}) {
      for (const unsigned color : {0U, parameter_max}) {
        for (const bool varied : {false, true}) {
          for (const bool serial : {false, true}) {
            fixture.sample_case(
                {.high = high, .twos = twos, .color = color, .varied = varied, .serial = serial});
          }
        }
      }
    }
  }
}

void drive_matrix(Fixture &fixture) {
  constexpr unsigned selectors = 16;
  constexpr unsigned second_shift = 5;
  constexpr std::array<unsigned, 4> levels{1, 17, 64, 127};
  for (unsigned curve = 0; curve < selectors; ++curve) {
    for (const bool twos : {false, true}) {
      for (const bool serial : {false, true}) {
        const auto second = (curve + second_shift) % selectors;
        fixture.sample_case(
            {.high = true,
             .twos = twos,
             .color = parameter_max,
             .varied = true,
             .serial = serial,
             .drives = {{{.curve = curve, .level = levels[curve % levels.size()]},
                         {.curve = second, .level = levels[second % levels.size()]}}},
             .blocks = 4,
             .drive_case = true});
      }
    }
  }
}

void stream_matrix(Fixture &fixture) {
  for (unsigned index = 0; index < stream_contexts; ++index) {
    fixture.sample_case(stream_configuration(index, fixture.stream_block_count()));
  }
}
}

namespace blofeld::test {
int run_voice_test(const int count,
                   char **arguments,
                   const bool drive,
                   const bool master,
                   const bool pcm,
                   const VoiceOutputOptions output,
                   const VoiceInput input) {
  auto *const stream_output = output.direct;
  const bool ring = stream_profile(input);
  try {
    require(count == 3, "Expected original firmware and private voice trace");
    require(output.direct == nullptr || output.capture == nullptr,
            "Choose direct delivery or the complete offline matrix");
    require(input != VoiceInput::foreground ||
                (!drive && master && !pcm && output.direct == nullptr && output.capture == nullptr),
            "Foreground reference requires declared disabled DMA and no external PCM output");
    require(!ring || (!drive && master && !pcm && output.capture == nullptr),
            "Repeating reference requires declared ring setup and no offline collector");
    require((stream_output == nullptr && output.capture == nullptr &&
             input != VoiceInput::controllers) ||
                (!drive && master && (pcm || ring)),
            "An external source requires the original voice/master/PCM composition");
    require(output.stream_case_index < stream_contexts &&
                (output.stream_case_index == 0 || (stream_output != nullptr && ring)),
            "A selected ring context requires direct stream output");
    if (stream_output != nullptr) { stream_output->check(); }
    const Source source(arguments[1]);
    std::ofstream trace(arguments[2]);
    require(trace.is_open(), "Cannot write private original voice trace");
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    Fixture fixture(source, trace, input);
    fixture.master_enabled = master;
    fixture.pcm_enabled = pcm;
    fixture.capture = output.capture;
    fixture.stream_output = stream_output;
    if (stream_output != nullptr) {
      if (ring) {
        fixture.sample_case(
            stream_configuration(output.stream_case_index, fixture.stream_block_count()));
      } else {
        fixture.sample_case({});
      }
    } else if (ring) {
      stream_matrix(fixture);
    } else if (drive) {
      drive_matrix(fixture);
    } else {
      baseline(fixture);
    }
    trace << "Entry-streams visited PCs:";
    for (auto pc : fixture.visited) { trace << ' ' << std::hex << pc; }
    trace << std::dec << "\nEntry-streams instructions: " << fixture.total_steps << '\n';
    trace.flush();
    trace.close();
    std::cout
        << "Original voice streams executed; independent numerical verification is separate.\n";
    return 0;
  } catch (const std::exception &error) {
    if (stream_output != nullptr) { stream_output->invalidate(); }
    std::cerr << error.what() << '\n';
    return 1;
  }
}

}
