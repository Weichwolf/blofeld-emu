// SPDX-License-Identifier: GPL-3.0-only
// Observe unchanged original 1.25 voice blocks; scalar models remain separate.
#include "dsp_voice_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_voice_master.hpp"
#include "dsp_voice_pcm.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <ostream>
#include <set>
#include <string>
#include <vector>

namespace {
using Words = std::vector<unsigned>;
constexpr unsigned sound = 0x1000, entry = 0xc500, mask = 0xffffff, rounding_mode = 1U << 21U;
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

struct Call {
  unsigned target;
  unsigned end;
  bool twos;
};

void harness(blofeld::DspBus &bus,
             unsigned &stop,
             blofeld::test::VoiceMaster &master,
             blofeld::test::VoicePcm &pcm) {
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
  code.install(bus);
}

unsigned initial_bus_word(const unsigned index, const bool varied) {
  if (index == 0) { return initial_left; }
  if (index == 1) { return initial_right; }
  if (varied) { return index % 2 == 0 ? positive_limit : negative_limit; }
  return static_cast<unsigned>(((static_cast<long long>(index) - 16) * 65536) + 0x1000000LL) & mask;
}

struct Fixture {
  blofeld::DspBus bus;
  unsigned stop = 0;
  blofeld::test::VoiceMaster master;
  blofeld::test::VoicePcm pcm;
  blofeld::DspCore cpu;
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
  Case configuration{};
  bool drive_pending = false;
  unsigned drive_return = 0;
  unsigned drive_buffer = 0;
  unsigned lfo_waves = 0, envelope_starts = 0, envelope_attacks = 0, envelope_finishes = 0,
           envelope_decays = 0;
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
                               blofeld::test::VoicePcm &pcm) {
    harness(bus, stop, master, pcm);
    bus.observe_omr(memory_mode);
    bus.advance_instruction_cycles(3);
    for (const auto &r : s.records) {
      for (unsigned i = 0; i < r.data.size(); ++i) {
        bus.write({.space = r.space, .address = r.base + i, .value = r.data[i]});
      }
    }
    return bus;
  }

  Fixture(const Source &s, std::ostream &output)
      : cpu(load(bus, s, stop, master, pcm)), source(s), trace(output) {}

  void put(blofeld::DspSpace space, unsigned at, unsigned v) {
    require(!rendering ||
                (space == blofeld::DspSpace::x && (at == caller_target || at == caller_status)),
            "Host synthesis-state repair is forbidden between original voice blocks");
    bus.write({.space = space, .address = at, .value = v});
  }

  unsigned get(blofeld::DspSpace space, unsigned at) {
    return bus.read({.space = space, .address = at});
  }

  void observe_output(const blofeld::DspState &s) {
    using enum blofeld::DspSpace;

    lfo_waves += static_cast<unsigned>(s.pc == free_sine);
    envelope_starts += static_cast<unsigned>(s.pc == envelope_start);
    envelope_attacks += static_cast<unsigned>(s.pc == envelope_attack);
    envelope_finishes += static_cast<unsigned>(s.pc == envelope_finish);
    envelope_decays += static_cast<unsigned>(s.pc == envelope_decay);
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
      require(!output_seen && s.sp == 1, "Original output fallthrough frame differs");
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
      require(s.sp == 1, "Actual original Drive return frame differs");
      drive_words("DRIVE_OUTPUT", {.index = drive_entries - 1, .address = drive_buffer});
      trace << "DRIVE_RETURN " << std::hex << drive_entries - 1 << ' ' << s.pc << ' ' << s.sp << ' '
            << s.r[0] << ' ' << (s.x >> word_bits) << ' ' << s.r[5] << '\n'
            << std::dec;
      drive_pending = false;
    }
    if (drive_pending || s.sp != 2 || drive_entries >= 2) { return; }
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
      require(s.sp == 2, "Actual mixer frame differs");
      gain_words("GAINS");
      ++gain_entries;
    }
    if (s.pc == noise_generator) {
      require(s.sp == 3 && s.r[7] == entry + noise_history, "Actual noise frame/history differs");
      trace << "NOISE_INPUT " << std::hex << get(x, shared_seed) << ' ' << get(x, s.r[7]) << ' '
            << (s.x & mask) << '\n'
            << std::dec;
      ++noise_entries;
    }
    if (s.pc == mixer_renderer) {
      require(s.sp == 2 && s.r[0] == coefficient_buffer && s.r[7] == entry + noise_descriptor &&
                  s.m[0] == 9,
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
                  s.n[0] == 3 && s.n[3] == 8 && s.n[4] == 5 && s.n[7] == 16 && s.sp == 6 &&
                  s.m[0] == oscillator_modulo && s.m[1] == mask && s.m[3] == descriptor_modulo &&
                  s.m[4] == oscillator_modulo && s.m[7] == phase_modulo &&
                  (s.sr & rounding_mode) == (twos ? rounding_mode : 0),
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

  void check_block(unsigned block) {
    using enum blofeld::DspSpace;

    require(cpu.state().sp == 0 && get(x, entry + mixer_callback) == steady_mixer &&
                shared == shared_samples_per_block && continues == 16,
            "Original three-oscillator loop/mixer return differs");
    require(get(x, 0xd) == byte_mask && get(x, 4) == 0,
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
    require(lfo_waves == (block == 0 ? 6U : 3U) && envelope_starts == (block == 0 ? 4U : 0U) &&
                envelope_attacks == (block == 0 ? 8U : 0U) &&
                envelope_finishes == (block == 0 ? 4U : 0U) &&
                envelope_decays == (block == 0 ? 0U : 4U),
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

  void sample_case(const Case input) {
    rendering = false;
    configuration = input;
    reset_stream(input.twos);
    configure_sound(input);
    prepare_output(input);
    if (master_enabled) { blofeld::test::VoiceMaster::initialize(bus); }
    if (pcm_enabled) {
      pcm.begin(bus);
      run({.target = pcm.setup, .end = stop, .twos = input.twos});
      pcm.baseline(bus);
    }
    rendering = true;
    for (unsigned block = 0; block < input.blocks; ++block) {
      begin_block(block);
      run({.target = voice_entry, .end = stop, .twos = input.twos});
      check_block(block);
      emit_samples();
      emit_state();
      if (master_enabled) {
        run({.target = master.entry, .end = stop, .twos = input.twos});
        require(cpu.state().sp == 0, "Original copy/mix/master calls must genuinely return");
        if (pcm_enabled) { run({.target = pcm.transfer, .end = stop, .twos = input.twos}); }
      }
    }
    rendering = false;
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
}

namespace blofeld::test {
int run_voice_test(
    const int count, char **arguments, const bool drive, const bool master, const bool pcm) {
  try {
    require(count == 3, "Expected original firmware and private voice trace");
    const Source source(arguments[1]);
    std::ofstream trace(arguments[2]);
    require(trace.is_open(), "Cannot write private original voice trace");
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    Fixture fixture(source, trace);
    fixture.master_enabled = master;
    fixture.pcm_enabled = pcm;
    if (drive) {
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
    std::cerr << error.what() << '\n';
    return 1;
  }
}

}
