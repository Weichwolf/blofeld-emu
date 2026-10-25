// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "gpio_parameter_caller.hpp"
#include "original_gpio_pair_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <ostream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::gpio_pair;
using blofeld::test::require;
using blofeld::test::gpio_parameter::caller_entry_bytes;
constexpr unsigned sound_base = 0x936, noise_parameter = 69, octave_parameter = 1;
constexpr unsigned voice = 0xc000, old_target = 0xe900, result_base = 0x2500;
constexpr unsigned cache_begin = 0x1a12, cache_cursor = 0x19fe, policy_parameter = 44;
constexpr unsigned policy_address = 0x74c, signature_address = 0x76c, signature_value = 127;
constexpr unsigned normalizer = 0x2728, decision = 0x83b, sound_store = 0x83c;
constexpr unsigned noise_callback = 0x11f5, cache_store = 0x11eb, flush_store = 0x11e9;
constexpr unsigned zero_flag = 4, octave_maximum = 112, rejected_return = 0x860;
constexpr unsigned entries = 0x6d2, redirected_offset = 190, ordinary_offset = 196;
constexpr unsigned cached_offset = 17, target_offset = 18, cache_bytes = 40;
constexpr unsigned input_bytes = 40, write_call_bytes = 18;
constexpr unsigned old_canary = 0x135724, ordinary_canary = 0x654321;
constexpr std::array<unsigned, 8> noise_values{0, 1, 7, 63, 64, 126, 127, 42};
constexpr std::array<unsigned, 8> octave_values{17, 18, 27, 29, 63, 65, 113, 127};

struct Profile {
  bool reject;
  unsigned parameter;
  unsigned initial;
  std::array<unsigned, 8> values;

  [[nodiscard]] Case input() const {
    return {.kind = 0,
            .location = 0,
            .parameter = parameter,
            .address = sound_base + parameter,
            .value = initial};
  }

  [[nodiscard]] unsigned readback(const unsigned index) const {
    return reject ? initial : values.at(index);
  }

  [[nodiscard]] unsigned normalized(const unsigned index) const {
    const auto value = values.at(index);
    if (!reject) { return value; }
    const auto clipped = std::clamp(value, 16U, octave_maximum);
    return 16 + (((clipped - 16) / 12) * 12);
  }
};

struct Run {
  Profile profile;
  OriginalGpioPair pair;
  std::vector<unsigned> bytes;
  unsigned produced = 0, consumed = 0, normalized = 0, decisions = 0;
  unsigned stores = 0, callbacks = 0, caches = 0, flushes = 0, policy_reads = 0;

  Run(const Firmware &firmware, const Profile config, std::ostream &trace)
      : profile(config), pair(firmware, config.input(), trace, DspSetup::secondary_parser) {
    for (const auto value : profile.values) {
      for (const auto byte : {0xf5U, 0U, profile.parameter, value, 0xf7U}) {
        bytes.push_back(byte);
      }
    }
    prepare();
    pair.additional_completion.observe = [this](const DspScheduledStep &step) { observe(step); };
  }

  unsigned read(const unsigned address) {
    return pair.dsp.bus().read({.space = DspSpace::x, .address = address});
  }

  void put(const unsigned address, const unsigned value) {
    pair.dsp.bus().write({.space = DspSpace::x, .address = address, .value = value});
  }

  void prepare() {
    put(part_table, sound_base); // Restore the original part0 binding in this declared context.
    put(policy_address, 0);      // Ctrl Send disabled; its real getter must still execute.
    put(entries, voice);
    put(entries + 1, 0);
    put(voice + 8, sound_base);
    put(voice + cached_offset, profile.initial << 16U);
    put(voice + target_offset, old_target);
    put(old_target, old_canary);
    put(voice + redirected_offset, data_canary);
    put(voice + ordinary_offset, ordinary_canary);
    pair.mcu.bus().write(cache_cursor, 0);
    for (unsigned n = 0; n < cache_bytes; ++n) {
      pair.mcu.bus().write(static_cast<std::uint16_t>(cache_begin + n), 0);
    }
  }

  void queue() {
    const auto producer = read(ring_write);
    if (producer != input_ring + produced) {
      require(produced < bytes.size() && producer == input_ring + produced + 1 &&
                  read(input_ring + produced) == bytes.at(produced) << 16U,
              "Original sound bytes must enter through the actual GPIO producer");
      pair.trace << "QUEUE_BYTE index=" << produced << " byte=" << bytes.at(produced)
                 << " cursor=" << producer << '\n';
      ++produced;
    }
    const auto consumer = read(ring_read);
    if (consumer != input_ring + consumed) {
      require(consumed < produced && consumer == input_ring + consumed + 1,
              "Original sound parser must consume each published byte once");
      ++consumed;
    }
  }

  void validation(const DspScheduledStep &step) {
    const auto &cpu = step.plan.before.cpu;
    const auto value = (cpu.a >> word_bits) & word_mask;
    if (cpu.pc == normalizer) {
      require(normalized < profile.values.size() && cpu.n[0] == profile.parameter &&
                  value == profile.values.at(normalized),
              "Original normalizer input differs");
      pair.trace << "NORMALIZER index=" << normalized << " parameter=" << cpu.n[0]
                 << " input=" << value << '\n';
      ++normalized;
    }
    if (cpu.pc != decision) { return; }
    require(decisions < profile.values.size() && value == profile.normalized(decisions) &&
                ((cpu.sr & zero_flag) != 0) == !profile.reject &&
                ((cpu.b >> word_bits) & word_mask) ==
                    (profile.reject ? octave_maximum : profile.parameter) &&
                pair.dsp.cpu().state().pc == (profile.reject ? rejected_return : sound_store),
            "Actual normalization flags/index and original accept/reject branch differ");
    pair.trace << "DECISION index=" << decisions << " normalized=" << value
               << " accepted=" << !profile.reject << '\n';
    ++decisions;
  }

  void callback(const DspScheduledStep &step) {
    const auto &cpu = step.plan.before.cpu;
    const auto value = (cpu.a >> word_bits) & word_mask;
    if (cpu.pc == sound_store) {
      require(!profile.reject && stores < profile.values.size() && cpu.r[0] == sound_base &&
                  cpu.n[0] == profile.parameter && value == profile.values.at(stores) &&
                  read(sound_base + profile.parameter) == value,
              "Original sound store differs");
      pair.trace << "SOUND_STORE index=" << stores << " value=" << value << '\n';
      ++stores;
    }
    if (cpu.pc == noise_callback) {
      require(!profile.reject && callbacks < profile.values.size() && cpu.r[2] == voice &&
                  value == profile.values.at(callbacks),
              "Original noise callback input differs");
      ++callbacks;
    }
    if (cpu.pc == flush_store) {
      const auto address = flushes == 0 ? old_target : voice + redirected_offset;
      const auto previous = flushes == 0 ? profile.initial : profile.values.at(flushes - 1);
      require(!profile.reject && flushes < profile.values.size() && cpu.r[1] == address &&
                  read(address) == previous << 16U,
              "Original callback must flush its old target");
      pair.trace << "FLUSH index=" << flushes << " address=" << address
                 << " value=" << read(address) << '\n';
      ++flushes;
    }
    if (cpu.pc != cache_store) { return; }
    require(!profile.reject && caches < profile.values.size() && cpu.r[2] == voice &&
                read(voice + target_offset) == voice + redirected_offset &&
                read(voice + cached_offset) == profile.values.at(caches) << 16U,
            "Actual callback must redirect and retain the independently shifted target");
    pair.trace << "CALLBACK index=" << caches << " target=" << read(voice + target_offset)
               << " value=" << read(voice + cached_offset) << '\n';
    ++caches;
  }

  void observe(const DspScheduledStep &step) {
    queue();
    if (step.plan.interrupt) { return; }
    validation(step);
    callback(step);
    if (step.plan.before.cpu.pc == source_load && step.plan.before.cpu.r[6] == policy_address) {
      require(policy_reads == 0 && (pair.dsp.cpu().state().x & word_mask) == 0,
              "Notification must obtain the disabled policy through one genuine DSP request");
      ++policy_reads;
    }
  }

  void install(const std::vector<std::uint8_t> &code) {
    for (unsigned n = 0; n < code.size(); ++n) {
      pair.mcu.bus().write(static_cast<std::uint16_t>(caller + n), code[n]);
    }
    pair.input = {.kind = 2,
                  .location = 0,
                  .parameter = policy_parameter,
                  .address = policy_address,
                  .value = 0};
  }

  void execute() {
    unsigned done = 0;
    bool notification_returned = false;
    for (unsigned n = 0; n < step_limit && done < profile.values.size(); ++n) {
      pair.step();
      if (!notification_returned &&
          pair.mcu.cpu().state().pc == caller + (done * caller_entry_bytes) + write_call_bytes) {
        require(policy_reads == 1, "Original notification must complete its real policy lookup");
        notification_returned = true;
        pair.input = profile.input();
        pair.input.value = profile.readback(done);
      }
      const auto value = pair.mcu.bus().read(static_cast<std::uint16_t>(result_base + done));
      if (value == ram_canary) { continue; }
      require(notification_returned && value == profile.readback(done) && decisions == done + 1 &&
                  pair.link.source_register_checks == done + 2 &&
                  pair.link.data_writes == ((done + 1) * 9) + 4 &&
                  pair.mcu.cpu().state().pc == caller + ((done + 1) * caller_entry_bytes),
              "Original sound write/notification/read must complete without host repair");
      pair.trace << "RESULT index=" << done << " value=" << static_cast<unsigned>(value) << '\n';
      ++done;
      notification_returned = false;
      if (done < profile.values.size()) { pair.input.value = profile.readback(done); }
    }
    require(done == profile.values.size(), "All original sound attempts must complete");
    pair.finish();
  }

  void verify(const std::vector<std::uint8_t> &code) {
    const auto expected = profile.reject ? 0U : static_cast<unsigned>(profile.values.size());
    require(normalized == 8 && decisions == 8 && stores == expected && callbacks == expected &&
                caches == expected && flushes == expected && policy_reads == 1 &&
                produced == input_bytes && consumed == produced &&
                pair.link.unresolved_queries == 0 &&
                read(sound_base + profile.parameter) == profile.initial &&
                read(signature_address) == signature_value,
            "Original sound pipeline totals differ");
    require(read(voice + ordinary_offset) == ordinary_canary &&
                read(voice + redirected_offset) ==
                    (profile.reject ? data_canary : noise_values[6] << 16U) &&
                read(voice + target_offset) ==
                    (profile.reject ? old_target : voice + redirected_offset) &&
                read(voice + cached_offset) == profile.initial << 16U &&
                read(old_target) == (profile.reject ? old_canary : profile.initial << 16U),
            "Pending callback target, old-target flush and untouched cells differ");
    require(pair.mcu.bus().read(cache_cursor) == 1,
            "Real policy getter must publish its cache cursor");
    for (unsigned n = 0; n < cache_bytes; ++n) {
      unsigned expected_byte = 0;
      if (n == 5) { expected_byte = 2; }
      if (n == 8) { expected_byte = policy_parameter; }
      require(pair.mcu.bus().read(static_cast<std::uint16_t>(cache_begin + n)) == expected_byte,
              "Real notification lookup must insert exactly its policy key and value");
    }
    for (unsigned n = 0; n < code.size(); ++n) {
      require(pair.mcu.bus().read(static_cast<std::uint16_t>(caller + n)) == code[n],
              "Own caller code changed");
    }
    bool rejected = false;
    try {
      pair.reject_board_qualification();
    } catch (const Fault &) { rejected = true; }
    require(rejected, "Sound response cannot qualify unresolved board voltage");
  }
};

int execute(const int argc, char **argv) {
  require(argc == 4, "Original sound test needs archive/report/trace");
  const auto firmware = Firmware::load(argv[1]);
  std::ofstream report(argv[2]);
  std::ofstream trace(argv[3]);
  require(report.is_open() && trace.is_open(), "Private sound outputs must open");
  report.exceptions(std::ios::badbit | std::ios::failbit);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  for (const auto reject : {false, true}) {
    const Profile config{.reject = reject,
                         .parameter = reject ? octave_parameter : noise_parameter,
                         .initial = reject ? 64U : 42U,
                         .values = reject ? octave_values : noise_values};
    const auto code =
        blofeld::test::gpio_parameter::caller_program(config.input(), config.values, result_base);
    for (const unsigned order : {0U, 1U}) {
      for (const unsigned phase : {0U, phase_offset}) {
        trace << "CASE reject=" << reject << " order=" << order << " phase=" << phase << '\n';
        Run run(firmware, config, trace);
        const auto nvm = run.pair.mcu.bus().power_off_flash_image();
        run.pair.verify_retained(firmware, nvm);
        run.install(code);
        run.pair.start(order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
                       phase,
                       caller + static_cast<unsigned>(config.values.size() * caller_entry_bytes));
        run.execute();
        run.verify(code);
        run.pair.verify_retained(firmware, nvm);
        report << "CASE reject=" << reject << " order=" << order << " phase=" << phase
               << " normalizations=" << run.normalized << " decisions=" << run.decisions
               << " stores=" << run.stores << " callbacks=" << run.callbacks
               << " flushes=" << run.flushes << " policy-reads=" << run.policy_reads
               << " source-loads=" << run.pair.link.source_register_checks
               << " produced=" << run.produced << " consumed=" << run.consumed
               << " queries=" << run.pair.link.pad_queries
               << " unknown-queries=" << run.pair.link.unresolved_queries
               << " unresolved-events=" << run.pair.link.unresolved_events
               << " program-unchanged=1 nvm-unchanged=1 caller-unchanged=1 cache-valid=1 "
                  "board-rejected=1\n";
      }
    }
  }
  return 0;
}
}

int main(const int argc, char **argv) {
  try {
    return execute(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
