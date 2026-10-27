// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "nand_gpio_fixture.hpp"
#include "nand_program_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld::test {
namespace {
constexpr unsigned helper_base = 0x2e00, alias = 0x11e00, memory_mode = 0x600389;
constexpr unsigned absolute_jsr = 0x0bf080, instruction_limit = 1000, sentinel = 0x123456;
constexpr std::array<std::uint8_t, 4> identity{0xec, 0x76, 0x5a, 0x3f};
constexpr unsigned packed_identity = 0xec763f, shifted_prefix = 0xec7600;
constexpr unsigned observed_row = 37, observed_column = 17, observed_cookie = 0x42;
constexpr std::uint8_t read_identification_command = 0x90;

struct Helper {
  unsigned block;
  unsigned words;
  unsigned identify;
  unsigned packing;
  unsigned reader;
  unsigned first_re;
};

constexpr std::array<Helper, 2> helpers{{{.block = 3,
                                          .words = 388,
                                          .identify = 0x2f6c,
                                          .packing = 0x2f77,
                                          .reader = 0x2ecb,
                                          .first_re = 0x2f75},
                                         {.block = 5,
                                          .words = 400,
                                          .identify = 0x2f78,
                                          .packing = 0x2f83,
                                          .reader = 0x2ed7,
                                          .first_re = 0x2f81}}};

struct WireState {
  std::optional<unsigned> previous;
  unsigned commands = 0, addresses = 0, reads = 0, steps = 0;
  bool switched = false, faulted = false;
};

// Selected reference chip: CE is held active separately; F10 is not inferred as CE.
class Wire {
public:
  Wire(DspMachine &machine,
       Nand &device,
       std::ostream &trace,
       const unsigned switch_boundary,
       const WireState state = {})
      : machine_(machine),
        device_(device),
        trace_(trace),
        switch_boundary_(switch_boundary),
        state_(state) {
    if (!state.previous) { machine_.bus().gpio(DspPort::f).set_drive({.mask = nand_other_f}); }
  }

  void step() {
    if (state_.faulted) { throw Fault("NAND ID owner requires coordinated recovery"); }
    const auto before = machine_.cpu().state();
    try {
      if (!state_.switched && before.pc == switch_boundary_) {
        // Three real own NOPs precede this explicit unclocked switch observation.
        machine_.bus().advance_instruction_cycles(3);
        state_.switched = true;
      }
      machine_.cpu().step();
      ++state_.steps;
      observe();
      trace_ << "STEP " << before.pc << ' ' << machine_.cpu().state().pc << ' ' << state_.commands
             << ' ' << state_.addresses << ' ' << state_.reads << '\n';
    } catch (const Fault &error) {
      state_.faulted = true;
      throw Fault("NAND ID at X:" + hex(port_c_data, 6) + " dsp-pc=" + hex(before.pc, 6) +
                  " omr=" + hex(before.omr, 6) + ": " + error.what());
    }
  }

  [[nodiscard]] WireState state() const { return state_; }

private:
  void latch(const unsigned controls) {
    const auto output = machine_.bus().gpio(DspPort::f).outputs();
    require((output.mask & output.known & nand_data) == nand_data,
            "ID command/address needs eight actual known data outputs");
    const auto byte = static_cast<std::uint8_t>(output.levels & nand_data);
    switch (controls & (nand_cle | nand_ale)) {
      case nand_cle:
        require(byte == read_identification_command,
                "Original/own ID query must issue literal 90h");
        device_.command(byte);
        ++state_.commands;
        break;
      case nand_ale:
        require(byte == 0, "ID query must supply exactly address 00h");
        device_.address(byte);
        ++state_.addresses;
        break;
      default: throw Fault("Unsupported latch in ID-only reference wire");
    }
    trace_ << "LATCH " << (controls & (nand_cle | nand_ale)) << ' ' << unsigned(byte) << '\n';
  }

  void observe() {
    auto &bus = machine_.bus();
    const auto control = bus.gpio(DspPort::c).observe_pads(nand_controls);
    if (control.known != nand_controls) {
      require(!state_.previous, "Established ID control pins cannot become unknown");
      return;
    }
    if (state_.previous) {
      const auto changed = *state_.previous ^ control.levels;
      require((changed & (nand_re | nand_we)) != (nand_re | nand_we),
              "ID RE/WE changes require an explicit order");
      if ((changed & control.levels & nand_we) != 0) { latch(control.levels); }
      if ((changed & ~control.levels & nand_re) != 0) {
        if ((bus.gpio(DspPort::f).outputs().mask & nand_data) != 0) {
          throw Fault("Unqualified NAND read turnaround: DSP data pins still driven");
        }
        const auto value = device_.read();
        require(state_.reads < identity.size(), "Original ID path consumes only four observations");
        require(value == identity[state_.reads], "Every native ID cycle matches independent data");
        bus.gpio(DspPort::f).set_drive({.mask = nand_other_f | nand_data, .levels = value});
        trace_ << "READ " << state_.reads << ' ' << unsigned(value) << '\n';
        ++state_.reads;
      }
      if ((changed & control.levels & nand_re) != 0) {
        bus.gpio(DspPort::f).set_drive({.mask = nand_other_f});
      }
    }
    state_.previous = control.levels;
  }

  DspMachine &machine_;
  Nand &device_;
  std::ostream &trace_;
  unsigned switch_boundary_;
  WireState state_;
};

struct Entry {
  unsigned switch_boundary;
  unsigned returned;
};

Entry install(DspBus &bus, const Firmware &firmware, const Helper helper, const bool full) {
  const auto bytes = std::span(firmware.update_blocks.at(helper.block));
  require(bytes.size() == static_cast<std::size_t>(helper.words) * 3U,
          "Pinned updater object size");
  // FM13-37: independently decode the signed nine-bit short-BSR displacement.
  constexpr unsigned short_bsr_mask = 0xfffc20;
  constexpr unsigned short_bsr_prefix = 0x050800;
  constexpr unsigned high_displacement = 0x1e0;
  constexpr unsigned low_displacement = 0x1f;
  constexpr unsigned displacement_sign = 0x100;
  constexpr unsigned displacement_range = 0x200;
  const auto opcode = dsp_boot_word(std::span<const std::uint8_t, 3>(
      bytes.subspan(static_cast<std::size_t>(helper.packing - helper_base) * 3U, 3)));
  const auto raw = ((opcode >> 1U) & high_displacement) | (opcode & low_displacement);
  const auto displacement =
      static_cast<std::int64_t>(raw) - (raw >= displacement_sign ? displacement_range : 0U);
  require((opcode & short_bsr_mask) == short_bsr_prefix &&
              static_cast<std::int64_t>(helper.packing) + displacement == helper.reader,
          "Declared packing entry begins with the original short BSR to the byte reader");
  for (unsigned index = 0; index < helper.words; ++index) {
    bus.write({.space = DspSpace::x,
               .address = alias + index,
               .value = dsp_boot_word(std::span<const std::uint8_t, 3>(
                   bytes.subspan(static_cast<std::size_t>(index) * 3U, 3)))});
  }
  DspProgram code;
  setup_gpio(code);
  code.immediate("a", 0);
  code.immediate("b", 0);
  code.immediate("x1", sentinel);
  code.immediate("x0", sentinel);
  code.immediate("omr", memory_mode);
  for (unsigned index = 0; index < 3; ++index) { code.emit("nop"); }
  const auto switch_boundary = code.pc();
  if (!full) {
    // Own legal setup supplies the explicitly declared entry of the original packing tail.
    latch(code, {.value = read_identification_command, .mode = nand_cle});
    latch(code, {.value = 0, .mode = nand_ale});
    movep(code, idle_c, port_c_data, 'x');
    movep(code, nand_other_f, port_f_control, 'y');
    movep(code, input_f, port_f_direction, 'y');
    movep(code, all_inputs_f, port_f_control, 'y');
  }
  code.word(absolute_jsr);
  code.word(full ? helper.identify : helper.packing);
  const auto returned = code.pc();
  code.emit("nop");
  code.install(bus);
  return {.switch_boundary = switch_boundary, .returned = returned};
}

Nand device(const std::optional<unsigned> missing = std::nullopt) {
  Nand result;
  NandIdentification observed;
  for (unsigned index = 0; index < identity.size(); ++index) {
    if (missing != index) {
      observed.bytes[index] = {.value = identity[index], .known = byte_mask};
    }
  }
  result.observe_identification(observed);
  result.observe_cell({.page = observed_row, .column = observed_column},
                      {.value = observed_cookie, .known = byte_mask});
  result.observe_power_ready();
  return result;
}

std::vector<std::uint8_t> cold_image(Nand model) {
  model.power_loss();
  return encode_nand_image(model.persistent_image());
}

void returned(const DspMachine &machine, const Wire &wire, const Entry entry) {
  const auto cpu = machine.cpu().state();
  require(cpu.pc == entry.returned && cpu.sp == 0 && cpu.omr == memory_mode &&
              cpu.a == (std::uint64_t{packed_identity} << word_bits) &&
              cpu.b == (std::uint64_t{shifted_prefix} << word_bits) &&
              cpu.x == ((std::uint64_t{sentinel} << word_bits) | identity[2]),
          "Original four-byte pack returns full independent A/B/X and real JSR/RTS frame");
  require(wire.state().commands == 1 && wire.state().addresses == 1 && wire.state().reads == 4 &&
              !wire.state().faulted,
          "Actual native pins perform one ID command/address and four data reads");
}

struct Saved {
  DspCheckpoint machine;
  Nand nand;
  WireState wire;
};

void packing(const Firmware &firmware, const Helper helper, std::ostream &trace) {
  DspMachine machine;
  const auto entry = install(machine.bus(), firmware, helper, false);
  auto nand = device();
  const auto initial = cold_image(nand);
  std::ostringstream steps;
  auto wire = std::make_unique<Wire>(machine, nand, steps, entry.switch_boundary);
  std::optional<Saved> saved;
  std::size_t prefix_bytes = 0;
  while (machine.cpu().state().pc != entry.returned) {
    require(wire->state().steps < instruction_limit, "Original ID packing must return");
    wire->step();
    if (wire->state().reads == 2 && !saved) {
      saved = Saved{.machine = machine.checkpoint(), .nand = nand, .wire = wire->state()};
      prefix_bytes = steps.str().size();
    }
  }
  returned(machine, *wire, entry);
  if (!saved) { throw Fault("Original ID path did not reach its joint checkpoint"); }
  const auto &joint = saved.value();
  require(cold_image(nand) == initial, "Original ID path retains NVM");
  const auto expected = machine.cpu().debug_state();
  const auto suffix = steps.str().substr(prefix_bytes);
  trace << "PACK " << helper.block << ' ' << wire->state().steps << ' ' << packed_identity << '\n';
  trace << "BEGIN_PACK " << helper.block << '\n'
        << steps.str() << "END_PACK " << helper.block << '\n';
  for (unsigned replay = 0; replay < 2; ++replay) {
    wire.reset();
    machine.restore(joint.machine);
    nand = joint.nand;
    steps.str({});
    steps.clear();
    wire = std::make_unique<Wire>(machine, nand, steps, entry.switch_boundary, joint.wire);
    while (machine.cpu().state().pc != entry.returned) {
      require(wire->state().steps < instruction_limit, "Original restored ID path must return");
      wire->step();
    }
    returned(machine, *wire, entry);
    require(machine.cpu().debug_state() == expected && steps.str() == suffix &&
                cold_image(nand) == initial,
            "Joint native/ID/edge-cursor restore reproduces full debug state, events and NVM");
    trace << "REPLAY " << helper.block << ' ' << replay << ' ' << wire->state().steps << '\n';
    trace << "BEGIN_REPLAY " << helper.block << ' ' << replay << '\n'
          << steps.str() << "END_REPLAY " << helper.block << ' ' << replay << '\n';
  }
}

void frontier(const Firmware &firmware,
              const Helper helper,
              const bool full,
              const unsigned missing,
              std::ostream &trace) {
  DspMachine machine;
  const auto entry = install(machine.bus(), firmware, helper, full);
  auto nand = device(full ? std::nullopt : std::optional{missing});
  const auto initial = cold_image(nand);
  Wire wire(machine, nand, trace, entry.switch_boundary);
  std::string reason;
  try {
    while (wire.state().steps < instruction_limit) { wire.step(); }
  } catch (const Fault &fault) { reason = fault.what(); }
  require(wire.state().faulted && wire.state().commands == 1 && wire.state().addresses == 1 &&
              wire.state().reads == (full ? 0 : missing) && cold_image(nand) == initial &&
              nand.ready() == true && nand.pending() == NandPending::none,
          "Real original setup or missing-data failure must retain ID cursor/array facts");
  require(reason.contains("dsp-pc=" + hex(full ? helper.first_re : helper.reader, 6)) &&
              reason.contains(full ? "turnaround" : "unknown bits byte=" + std::to_string(missing)),
          "Original ID faults must retain their independent PC and exact cause: " + reason);
  const auto before = machine.cpu().debug_state();
  try {
    wire.step();
    throw Fault("Faulted ID wire unexpectedly resumed");
  } catch (const Fault &fault) {
    require(std::string(fault.what()).contains("coordinated recovery"), "ID owner is fail-stop");
  }
  require(machine.cpu().debug_state() == before && cold_image(nand) == initial,
          "Refused continuation cannot execute another CPU instruction or NVM effect");
  trace << (full ? "TURNAROUND " : "MISSING ") << helper.block << ' ' << missing << ' '
        << wire.state().reads << ' ' << reason << '\n';
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) { throw blofeld::Fault("Usage: original-nand-id firmware.zip private.trace"); }
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    for (const auto helper : blofeld::test::helpers) {
      blofeld::test::packing(firmware, helper, trace);
      blofeld::test::frontier(firmware, helper, true, 0, trace);
      for (unsigned missing = 0; missing < blofeld::test::identity.size(); ++missing) {
        blofeld::test::frontier(firmware, helper, false, missing, trace);
      }
    }
    std::cout << "Original NAND ID: two packing paths, four joint replays, eight missing-data "
                 "failures, two retained turnaround frontiers\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
