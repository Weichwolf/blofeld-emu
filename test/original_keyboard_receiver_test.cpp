// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "keyboard_receiver_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>

namespace {
using namespace blofeld;
using namespace blofeld::test::keyboard_receiver;
constexpr unsigned expected_words = 777, prefix_limit = 150, controller_limit = 150000;
constexpr unsigned own_entry_steps = 2, helper_setup_steps = 8, refill_call_steps = 53;
constexpr unsigned argument_reloads = 2, cached_call_steps = 16;
constexpr unsigned receiver_steps = own_entry_steps + helper_setup_steps + refill_call_steps +
                                    argument_reloads + (2 * cached_call_steps);
constexpr unsigned seed_word = 0x2854a1, replay_word = 0x80ff55;
constexpr unsigned controller_wait = 0x13b0, initial_e_read = 0x1394;
constexpr unsigned delay_entry = 0x1378, delay_body = 0x137c, decrement = 0x1388;
constexpr unsigned indexed_decrement = 0x1382, test_branch = 0x1384;
constexpr unsigned byte_values = 256, register_mask = 0xffff, dbne_cycles = 3;
constexpr unsigned byte_counter = 255, delay_calls = 2;
constexpr std::array<unsigned, 8> corners{
    0, 0xffffff, 0x800000, 0x7fffff, 0xff0000, 0x00ff00, 0x0000ff, replay_word};

void decoded(Fixture &run) {
  if (run.machine.cpu().state().pc != initialized || run.steps != receiver_steps ||
      run.source.reads != samples.size() || run.source.requests != samples.size() ||
      run.source.returned != returns.size()) {
    throw Fault("Original receiver prefix differs: pc=" + hex(run.machine.cpu().state().pc) +
                " steps=" + std::to_string(run.steps) + " expected=" +
                std::to_string(receiver_steps) + " reads=" + std::to_string(run.source.reads) +
                " requests=" + std::to_string(run.source.requests) +
                " returns=" + std::to_string(run.source.returned));
  }
  require(run.machine.bus().read_memory({.address = saved_callback, .size = 2}) == callback_table &&
              run.machine.bus().read_memory({.address = saved_return, .size = 2}) == caller + 6,
          "Real native helper/caller frames retain the callback and return address");
  run.guards();
}

void word(Fixture &run,
          const unsigned value,
          std::ostream &trace,
          std::optional<Saved> &saved,
          std::string &suffix) {
  std::ostringstream events;
  run.reset(value, events);
  run.detailed = value == replay_word;
  trace << "CASE word=" << value << '\n';
  while (run.machine.cpu().state().pc != initialized) {
    require(run.steps < prefix_limit, "Original three-call receiver prefix must finish");
    if (value == replay_word && run.machine.cpu().state().pc == halfway) {
      require(run.source.requests == 3 && run.source.reads == 2 && run.source.returned == 0,
              "Checkpoint lies after two real reads with the third input held");
      saved = run.save();
      trace << events.str() << "CHECKPOINT requests=3 reads=2 returned=0\n";
      events.str({});
      events.clear();
    }
    run.step();
  }
  if (value == replay_word) { suffix = events.str(); }
  trace << events.str() << "WORD steps=" << run.steps << " bytes=" << run.source.returned
        << " requests=" << run.source.requests << " reads=" << run.source.reads << '\n';
  decoded(run);
}

void replay(Fixture &run, const Saved &saved, const std::string &suffix, std::ostream &trace) {
  for (unsigned n = 0; n < 2; ++n) {
    std::ostringstream actual;
    run.restore(saved, actual);
    while (run.machine.cpu().state().pc != initialized) {
      require(run.steps < prefix_limit, "Restored receiver must keep the original bound");
      run.step();
    }
    decoded(run);
    require(actual.str() == suffix,
            "Fresh native owners and external source replay every logged suffix byte");
    trace << "REPLAY index=" << n << " bytes=" << actual.str().size() << '\n';
  }
}

void quarantine(Fixture &run) {
  require(run.machine.timing_faulted() && !run.machine.run_clock().faulted(),
          "Unknown GPIO quarantines the native cycle owner without an unrelated clock fault");
  const auto state = registers(run.machine.cpu().state());
  const auto ticks = run.machine.run_clock().elapsed_ticks();
  const auto source = run.source;
  bool rejected = false;
  try {
    run.step();
  } catch (const Fault &) { rejected = true; }
  require(rejected && registers(run.machine.cpu().state()) == state &&
              run.machine.run_clock().elapsed_ticks() == ticks && run.source == source,
          "A failed helper cannot resume or consume external data without recovery");
  run.guards();
}

void missing_chunks(Fixture &run, std::ostream &trace) {
  for (unsigned n = 0; n < samples.size(); ++n) {
    run.reset(seed_word, trace, n);
    bool failed = false;
    for (unsigned step = 0; step < prefix_limit; ++step) {
      const auto pc = run.machine.cpu().state().pc;
      try {
        run.step();
      } catch (const Fault &error) {
        const std::string_view text(error.what());
        require(pc == samples[n] && text.contains("Unknown MCU GPIO input at 0x0260") &&
                    text.contains("mask=0x3f") && text.contains("pc=" + hex(samples[n])) &&
                    run.source.requests == n + 1 && run.source.reads == n &&
                    run.source.returned == 0,
                "Only the actual read of the withdrawn chunk may fail");
        trace << "MISSING index=" << n << " pc=" << pc << " fault=" << text << '\n';
        quarantine(run);
        failed = true;
        break;
      }
    }
    require(failed, "Withdrawn chunk must never become a successful return");
  }
}

struct Delay {
  unsigned calls = 0, bodies = 0, decrements = 0, indexed = 0, branches = 0;

  void observe(const CpuState &before, Fixture &run, const std::uint64_t cycles) {
    const auto after = run.machine.cpu().state();
    if (before.pc == delay_entry) {
      require(before.d == byte_counter, "Disabled reset TIM supplies zero then byte-minus-one");
      ++calls;
    }
    if (before.pc == delay_body) { ++bodies; }
    if (before.pc == decrement) {
      const auto expected = ((before.d & high_mask) + high_mask) & high_mask;
      require(after.d == ((before.d & ~high_mask) | expected) && after.ccr == before.ccr &&
                  after.sp == before.sp &&
                  after.pc == (expected == 0 ? decrement + 3 : decrement) &&
                  run.machine.run_clock().bus_cycles() - cycles == dbne_cycles,
              "Independent HC12 DBNE byte wrap, branch, unchanged CCR and three cycles differ");
      ++decrements;
    }
    if (before.pc == indexed_decrement) {
      require(after.y == ((static_cast<unsigned>(before.x) + register_mask) & register_mask) &&
                  after.x == before.x && after.sp == before.sp,
              "LEAY independently decrements the old X, including zero wrap");
      ++indexed;
    }
    if (before.pc == test_branch) {
      require(registers(after) == std::tuple(static_cast<std::uint16_t>(
                                                 before.x != 0 ? delay_body : test_branch + 3),
                                             before.d,
                                             before.x,
                                             before.y,
                                             before.sp,
                                             before.ccr),
              "TBNE tests without decrementing X or changing CCR/stack");
      ++branches;
    }
  }
};

struct ControllerInputs {
  unsigned word;
  unsigned e_levels;
  bool known;
};

void controller(Fixture &run, const ControllerInputs input, std::ostream &trace) {
  run.reset(input.word, trace);
  run.machine.bus().set_pin_drive(
      McuPort::e,
      {.mask = static_cast<std::uint8_t>(input.known ? e_inputs : 3U),
       .levels = static_cast<std::uint8_t>(input.known ? input.e_levels : 3U)});
  require(run.machine.bus().read(hc12::tscr1) == 0,
          "This reference fixture explicitly retains the disabled reset timer");
  Delay delay;
  for (unsigned n = 0; n < controller_limit; ++n) {
    const auto before = run.machine.cpu().state();
    const auto cycles = run.machine.run_clock().bus_cycles();
    try {
      run.step();
    } catch (const Fault &error) {
      const auto site = input.known ? controller_wait : initial_e_read;
      const std::string_view text(error.what());
      require(before.pc == site && text.contains("Unknown MCU GPIO input at 0x0008") &&
                  text.contains(input.known ? "mask=0x80" : "mask=0x6c") &&
                  text.contains("pc=" + hex(site)) && run.source.returned == returns.size() &&
                  run.source.requests == samples.size() && run.source.reads == samples.size(),
              "Controller setup stops only on the actual unsupplied PE input");
      if (input.known) {
        require(delay.calls == delay_calls && delay.bodies == delay_calls * byte_counter &&
                    delay.decrements == delay_calls * byte_counter * byte_values &&
                    delay.indexed == delay_calls * (byte_counter + 1) &&
                    delay.branches == delay.indexed && run.machine.bus().read(hc12::ddre) == 0x10,
                "Both original delay nests execute completely before PE7 is released and read");
      } else {
        require(delay.decrements == 0, "Unknown initial PE inputs cannot reach the delay");
      }
      trace << "CONTROLLER known=" << input.known << " levels=" << input.e_levels << " pc=" << site
            << " steps=" << run.steps << " dbne=" << delay.decrements << " fault=" << text << '\n';
      quarantine(run);
      return;
    }
    delay.observe(before, run, cycles);
  }
  throw Fault("Original controller initialization exceeded its declared native bound");
}

int execute(const int argc, char **argv) {
  require(argc == 5, "Expected original firmware, helper, checksum word and private trace");
  const auto firmware = Firmware::load(argv[1]);
  const auto helper = bytes(argv[2], helper_bytes);
  const auto data = bytes(argv[3], 3);
  const auto checksum =
      (static_cast<unsigned>(data[0]) << 16U) | (static_cast<unsigned>(data[1]) << 8U) | data[2];
  std::ofstream trace(argv[4]);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  Fixture run(firmware, helper);
  missing_chunks(run, trace);
  for (const unsigned levels : {3U, e_inputs}) {
    controller(run, {.word = checksum, .e_levels = levels, .known = true}, trace);
  }
  controller(run, {.word = checksum, .e_levels = 3, .known = false}, trace);
  std::optional<Saved> saved;
  std::string suffix;
  unsigned cases = 0;
  for (unsigned index = 0; index < 3; ++index) {
    const auto shift = 8U * index;
    for (unsigned value = 0; value < byte_values; ++value) {
      word(run, (seed_word & ~(high_mask << shift)) | (value << shift), trace, saved, suffix);
      ++cases;
    }
  }
  for (const auto value : corners) {
    word(run, value, trace, saved, suffix);
    ++cases;
  }
  word(run, checksum, trace, saved, suffix);
  require(++cases == expected_words, "Every directed receiver word ran");
  if (!saved.has_value()) { throw Fault("Directed receiver checkpoint missing"); }
  replay(run, saved.value(), suffix, trace);
  trace << "TOTAL words=" << cases << " bytes=" << cases * 3 << " missing=4 replays=2 prefixes=3\n";
  std::cout << "Original Keyboard receiver and controller-input frontiers verified.\n";
  return 0;
}
} // namespace

int main(const int argc, char **argv) {
  try {
    return execute(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
