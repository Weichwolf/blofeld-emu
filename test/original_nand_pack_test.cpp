// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "nand_gpio_fixture.hpp"
#include "nand_program_fixture.hpp"
#include "nand_read_timing_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_read_timing.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>

namespace blofeld::test::nand_pack {
namespace {
using nand_read_reference::install_stage;
using nand_read_reference::Link;
constexpr unsigned extal_hz = 12000000;
constexpr unsigned timebase_hz = 1080000000;
constexpr unsigned buffer = 0xc000;
constexpr unsigned words = 176;
constexpr unsigned destination = 0x100;
constexpr unsigned guard = buffer + (2 * words) - 1;
constexpr unsigned guard_value = 0x345678;
constexpr unsigned y_guard_value = 0x654321;
constexpr unsigned packing_entry = 0xbd;
constexpr unsigned getter_entry = 0x9c;
constexpr unsigned shared_prefetch = 0xc5;
constexpr unsigned after_read_edge = 0x94;
constexpr unsigned first_refill_f = 0xa3;
constexpr unsigned original_omr = 0x600389;
constexpr unsigned prior_lc = 0x57;
constexpr unsigned prior_la = 0x1357;
constexpr unsigned saved_column = 264;
constexpr unsigned maximum_steps = 10000;
constexpr unsigned bcr_reset = 0x1fffff;
constexpr unsigned first_initial_seed = 0x13579b;
constexpr unsigned second_initial_seed = 0x2468ac;
constexpr unsigned negative_bias = 0x96;
constexpr std::uint64_t word_scale = 0x1000000;

struct Input {
  unsigned bias = 0;
  unsigned old_first_word = 0;
};

struct Ledger {
  unsigned packed = 0;
  unsigned buffered = 0;
};

struct Joint {
  DspCheckpoint machine;
  Nand nand;
  Link::State link;
  DspDebugState debug;
  Ledger ledger;
  std::string prefix;
};

DspRunContract contract() {
  return {.clock = {.extal_hz = extal_hz,
                    .timebase_hz = timebase_hz,
                    .switch_timing = DspClockSwitch::reference_instruction_boundary},
          .timing = DspRunTiming::native_internal_estimates};
}

std::array<std::uint8_t, Nand::page_bytes> page(const Input input) {
  std::array<std::uint8_t, Nand::page_bytes> data{};
  constexpr unsigned stride = 29;
  for (unsigned index = 0; index < data.size(); ++index) {
    data[index] =
        static_cast<std::uint8_t>((input.bias + index * stride + (index >> 8U)) & byte_mask);
  }
  return data;
}

std::array<std::uint32_t, words> expected_words(const Input input) {
  const auto data = page(input);
  std::array<std::uint32_t, words> result{};
  constexpr unsigned high_shift = 16;
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = (static_cast<unsigned>(data[index * 3]) << high_shift) |
                    (static_cast<unsigned>(data[(index * 3) + 1]) << 8U) | data[(index * 3) + 2];
  }
  return result;
}

void prepare(Nand &nand, const Input input) {
  nand.observe_bytes(0, 0, page(input));
  nand.observe_power_ready();
  nand.command(0);
  for (unsigned index = 0; index < 4; ++index) { nand.address(0); }
  nand.observe_completion();
}

struct Store {
  unsigned pointer;
  unsigned value;
  char space;
};

void store(DspProgram &program, const Store target) {
  program.immediate("r0", target.pointer);
  program.immediate("x0", target.value);
  program.emit("move x0," + std::string(1, target.space) + ":(r0)");
}

unsigned caller(DspBus &bus, const Input input) {
  DspProgram program;
  setup_gpio(program);
  movep(program, input_f, port_f_direction, 'y');
  movep(program, all_inputs_f, port_f_control, 'y');
  program.immediate("omr", original_omr);
  store(program, {.pointer = buffer, .value = input.old_first_word, .space = 'x'});
  store(program, {.pointer = guard, .value = guard_value, .space = 'x'});
  store(program, {.pointer = buffer, .value = y_guard_value, .space = 'y'});
  program.immediate("r0", buffer);
  program.immediate("r2", buffer);
  program.immediate("r3", destination);
  program.immediate("lc", prior_lc);
  program.immediate("la", prior_la);
  program.emit("jsr " + number(packing_entry));
  program.emit("move a1,x:(r3)+");
  for (unsigned index = 1; index < words; ++index) {
    program.emit("jsr " + number(getter_entry));
    program.emit("move a1,x:(r3)+");
  }
  program.install(bus);
  return program.pc();
}

std::uint64_t accumulator(const unsigned raw) {
  auto signed_word = static_cast<std::int64_t>(raw);
  constexpr unsigned sign = 0x800000;
  if ((raw & sign) != 0) { signed_word -= static_cast<std::int64_t>(word_scale); }
  return static_cast<std::uint64_t>(signed_word * static_cast<std::int64_t>(word_scale)) &
         accumulator_mask;
}

void step(DspMachine &machine, Link &link, const Input input, Ledger &ledger, std::ostream &trace) {
  const auto before = machine.cpu().state();
  const auto expected = expected_words(input);
  unsigned next_word = 0;
  unsigned prefetched = 0;
  const bool packing = before.sp == 3;
  if (before.pc == shared_prefetch) {
    if (packing) {
      require(ledger.packed < words && before.r[4] == buffer && before.lc == words - ledger.packed,
              "Original DOR retains the independent remaining count and reset prefetch cursor");
      next_word = expected.at(ledger.packed);
      prefetched = ledger.packed == 0 ? input.old_first_word : expected[0];
    } else {
      require(before.sp == 1 && ledger.buffered < words - 1 &&
                  before.r[4] == buffer + ledger.buffered + 1,
              "Buffered-only original call retains its real return frame and next cursor");
      next_word = before.r[4];
      prefetched = expected.at(ledger.buffered + 1);
    }
  }
  unsigned reads = 0;
  link.step({.observe = [&](const DspNativeAccess &access) {
    if (access.pc == shared_prefetch && access.kind == DspNativeAccessKind::read &&
        access.stage == DspNativeAccessStage::after) {
      require(access.target.space == DspSpace::x && access.target.address == before.r[4] &&
                  access.value == prefetched,
              "Actual original parallel prefetch reads the independent OLD RAM word exactly");
      ++reads;
    }
  }});
  const auto after = machine.cpu().state();
  if (before.pc == shared_prefetch) {
    require(reads == 1 && after.a == accumulator(prefetched) &&
                after.b == static_cast<std::uint64_t>(next_word) * word_scale &&
                after.r[4] == before.r[4] + 1,
            "Original parallel ADD/prefetch keeps full accumulators and exactly one cursor update");
    if (packing) {
      ++ledger.packed;
    } else {
      ++ledger.buffered;
    }
  }
  const auto controls = machine.bus().gpio(DspPort::c).observe_pads(nand_controls);
  const auto data = machine.bus().gpio(DspPort::f).observe_pads(nand_data);
  trace << "state pc=" << hex(after.pc, 6) << " sr=" << hex(after.sr, 6)
        << " omr=" << hex(after.omr, 6) << " a=" << hex(after.a, 14) << " b=" << hex(after.b, 14)
        << " sp=" << after.sp << " lc=" << after.lc << " la=" << after.la
        << " r0=" << hex(after.r[0], 6) << " r3=" << hex(after.r[3], 6)
        << " r4=" << hex(after.r[4], 6) << " packed=" << ledger.packed
        << " buffered=" << ledger.buffered << " C-known=" << hex(controls.known, 3)
        << " C-levels=" << hex(controls.levels, 3) << " F-known=" << hex(data.known, 2)
        << " F-levels=" << hex(data.levels, 2) << " F-conflicts=" << hex(data.conflicts, 2) << '\n';
}

void finish(DspMachine &machine, Nand &nand, Link &link, const Input input, const Ledger ledger) {
  const auto expected = expected_words(input);
  for (unsigned index = 0; index < words; ++index) {
    require(
        machine.bus().read({.space = DspSpace::x, .address = buffer + index}) == expected[index] &&
            machine.bus().read({.space = DspSpace::x, .address = destination + index}) ==
                expected[index],
        "Every original packed and subsequently returned word has independent big-endian bytes");
    if (index != 0) {
      require(machine.bus().read({.space = DspSpace::x, .address = buffer + words + index - 1}) ==
                  buffer + index,
              "The original shared tail stores its OLD read cursor beyond the filled buffer");
    }
  }
  const auto state = machine.cpu().debug_state();
  require(
      state.cpu.r[0] == guard && state.cpu.r[4] == buffer + words &&
          state.cpu.r[3] == destination + words && state.cpu.sp == 0 && state.sc == 0 &&
          state.cpu.lc == prior_lc && state.cpu.la == prior_la && ledger.packed == words &&
          ledger.buffered == words - 1 && link.reads() == Nand::page_bytes &&
          nand.column() == Nand::page_bytes && !nand.ready().has_value(),
      "Original loops/call frames retire and preserve old loop state without an extra NAND read");
  require(machine.bus().read({.space = DspSpace::x, .address = guard}) == guard_value &&
              machine.bus().read({.space = DspSpace::y, .address = buffer}) == y_guard_value,
          "The exact tail endpoint and the independent Y-space guard remain untouched");
}

void run(const Firmware &firmware, const Input input, std::ostream &out) {
  DspMachine machine(contract());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  install_stage(machine.bus(), firmware);
  const auto end = caller(machine.bus(), input);
  Nand nand;
  prepare(nand, input);
  std::ostringstream trace;
  Link link(machine, nand, trace);
  Ledger ledger;
  std::optional<Joint> saved;
  for (unsigned count = 0; machine.cpu().state().pc != end; ++count) {
    require(count < maximum_steps,
            "Original NAND pack/getter exceeded its native instruction budget");
    step(machine, link, input, ledger, trace);
    if (!saved && link.reads() == saved_column && machine.cpu().state().pc == after_read_edge) {
      auto timing = link.state().timing;
      require(timing.output(machine.run_clock().elapsed_ticks()) == NandReadOutput{},
              "The joint checkpoint actually suspends an unresolved read-output timing window");
      saved.emplace(Joint{.machine = machine.checkpoint(),
                          .nand = nand,
                          .link = link.state(),
                          .debug = machine.cpu().debug_state(),
                          .ledger = ledger,
                          .prefix = trace.str()});
    }
  }
  if (!saved) { throw Fault("The original loop must reach the selected mid-read snapshot"); }
  finish(machine, nand, link, input, ledger);
  const auto result = machine.cpu().debug_state();
  out << "input=" << input.bias << " initial-X=" << hex(input.old_first_word, 6) << '\n'
      << trace.str();
  DspMachine branch(contract());
  branch.restore(saved->machine);
  Nand peer = saved->nand;
  std::ostringstream replay;
  Link wire(branch, peer, replay, saved->link);
  require(branch.cpu().debug_state() == saved->debug && peer.column() == saved_column,
          "Fresh wiring of joint timing/NAND/CPU state cannot advance the saved read");
  auto repeated = saved->ledger;
  for (unsigned count = 0; branch.cpu().state().pc != end; ++count) {
    require(count < maximum_steps, "Original NAND restored suffix exceeded its instruction budget");
    step(branch, wire, input, repeated, replay);
  }
  finish(branch, peer, wire, input, repeated);
  require(trace.str().starts_with(saved->prefix) &&
              replay.str() == trace.str().substr(saved->prefix.size()) &&
              branch.cpu().debug_state() == result,
          "Every restored native state/time/pin row and the full final debug state match");
  out << "verified words=" << words << " buffered=" << ledger.buffered << " reads=" << link.reads()
      << " restored-column=" << saved_column << " suffix-bytes=" << replay.str().size() << '\n';
}

void missing_biu(const Firmware &firmware, std::ostream &trace) {
  DspMachine machine(contract());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  install_stage(machine.bus(), firmware);
  DspProgram program;
  program.immediate("sr", masked_status);
  program.immediate("omr", original_omr);
  movep(program, nand_controls, port_c_direction, 'x');
  movep(program, nand_data, port_c_control, 'x');
  movep(program, idle_c, port_c_data, 'x');
  program.immediate("r4", buffer + words);
  program.immediate("r2", buffer);
  program.immediate("r5", 0);
  program.emit("jsr " + number(getter_entry));
  program.install(machine.bus());
  Nand nand;
  prepare(nand, {});
  Link link(machine, nand, trace);
  std::string error;
  for (unsigned count = 0; error.empty(); ++count) {
    require(count < maximum_steps, "Unqualified BIU original refill must fail within its budget");
    try {
      link.step();
    } catch (const Fault &failure) { error = failure.what(); }
  }
  require(error.contains("Port F requires BCR=0x01ffff") &&
              error.contains("pc=" + hex(first_refill_f, 6)) &&
              error.contains("write at Y:" + hex(port_f_control, 6)) && link.reads() == 0 &&
              machine.bus().read({.space = DspSpace::x, .address = bus_control}) == bcr_reset &&
              machine.bus().gpio(DspPort::f).read(DspGpioRegister::control) == 0,
          "The original refill has no inherited qualified BIU setup; its first F write fails: " +
              error);
  trace << "missing-original-BIU=" << error << '\n';
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-nand-pack firmware.zip private.trace");
    }
    const auto firmware = blofeld::Firmware::load(std::filesystem::path(arguments[1]));
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    for (const auto input :
         {blofeld::test::nand_pack::Input{.old_first_word =
                                              blofeld::test::nand_pack::first_initial_seed},
          blofeld::test::nand_pack::Input{.bias = blofeld::test::nand_pack::negative_bias,
                                          .old_first_word =
                                              blofeld::test::nand_pack::second_initial_seed}}) {
      blofeld::test::nand_pack::run(firmware, input, trace);
    }
    blofeld::test::nand_pack::missing_biu(firmware, trace);
    std::cout << "Original NAND pack/getter: two full pages, shared-tail side effects, joint "
                 "mid-read replay and strict BIU frontier. No complete boot proof.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
