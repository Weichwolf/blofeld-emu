// SPDX-License-Identifier: GPL-3.0-only
#include "board/nand_bus_decode.hpp"
#include "board/nand_peer.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "nand_native_peer.hpp"
#include "nand_native_run.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_image.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned boot = 0xff0000;
constexpr unsigned c_data = 0x07f43d, c_direction = 0x07f43e, c_function = 0x07f43f;
constexpr unsigned f_data = 0x09f4b5, f_direction = 0x09f4b6, f_function = 0x09f4b7;
constexpr unsigned b_control = 0x08f4bb, f_read_x0 = 0x094435, bus_value = 0x1ffff;
constexpr unsigned inactive = 0x300, read_active = 0x200, all_c_outputs = 0xfff;
constexpr unsigned input_functions = 0xff, control_outputs = 0xf00;
constexpr unsigned f_output = 0x4ff, f_input = 0x400, f_output_functions = 0x300;
constexpr unsigned f_input_functions = 0x3ff, host_command = 13, byte_mask = 0xff;
constexpr unsigned read_wait = 256, release_wait = 16, maximum_cycles = 100000;
constexpr unsigned first_byte = 7, final_byte = 0xba, second_byte = 0x24, next_row_byte = 0xb7;
constexpr std::uint64_t inherited_tick = 1000000000;
constexpr DspRunClockInputs clocks{.extal_hz = 12000000,
                                   .timebase_hz = nand_timebase_hz,
                                   .switch_timing = DspClockSwitch::reference_cycle_boundary};

void require(const bool condition, const std::string &reason) {
  if (!condition) { throw Fault(reason); }
}

Nand image(const NandReadContinuation part) {
  NandImage result;
  NandArray::Page page;
  constexpr unsigned multiplier = 29;
  for (unsigned column = 0; column < Nand::page_bytes; ++column) {
    page.cells.at(column) = {
        .value = static_cast<std::uint8_t>((column * multiplier + first_byte) & byte_mask),
        .known = byte_mask};
  }
  result.pages.emplace(0, page);
  page.cells.fill({.value = next_row_byte, .known = byte_mask});
  result.pages.emplace(1, page);
  return {result, part};
}

enum class HostDirection : std::uint8_t { full_byte, partial_byte };

class Program {
public:
  explicit Program(const unsigned count,
                   const HostDirection host_direction = HostDirection::full_byte) {
    write(b_control, bus_value);
    write(c_function, input_functions);
    write(c_direction, control_outputs);
    write(c_data, inactive);
    output();
    select_row(0);
    input();
    nops(read_wait);
    for (unsigned i = 0; i < count; ++i) {
      read();
      if (i + 1 < count) { write(c_data, inactive); }
    }
    markers_.active = pc();
    write(c_function, 0);
    write(c_direction,
          host_direction == HostDirection::full_byte ? all_c_outputs : control_outputs | 1U);
    write(c_data, host_command);
    markers_.neutral = pc();
    nops(release_wait);
    markers_.released = pc();
    for (unsigned value = 0; value <= byte_mask; ++value) { write(c_data, read_active | value); }
    constexpr unsigned qualifiers = 16;
    constexpr unsigned control_shift = 8;
    for (unsigned controls = 0; controls < qualifiers; ++controls) {
      write(c_data, (controls << control_shift) | host_command);
    }
    write(c_data, read_active | host_command);
    markers_.host_done = pc();
    write(c_function, input_functions);
    write(c_direction, control_outputs);
    nops(release_wait);
    markers_.host_input = pc();
    write(c_data, inactive);
    markers_.armed = pc();
    if (count == Nand::page_bytes) {
      output();
      select_row(1);
      input();
      nops(read_wait);
    }
    read();
    write(c_data, inactive);
    markers_.stop = pc();
    nops(release_wait);
  }

  void install(DspBus &bus) const { bus.install_rom({.start = boot, .words = words_}); }

  struct Markers {
    unsigned active = 0, neutral = 0, released = 0, host_done = 0, host_input = 0, armed = 0,
             stop = 0;
  };

  [[nodiscard]] const Markers &markers() const { return markers_; }

private:
  [[nodiscard]] unsigned pc() const { return boot + static_cast<unsigned>(words_.size()); }

  void write(const unsigned opcode, const unsigned value) {
    words_.push_back(opcode);
    words_.push_back(value);
  }

  void nops(const unsigned count) { words_.insert(words_.end(), count, 0); }

  void output() {
    write(f_function, f_output_functions);
    write(f_direction, f_output);
  }

  void input() {
    write(f_direction, f_input);
    write(f_function, f_input_functions);
  }

  void latch(const unsigned value, const bool command) {
    constexpr unsigned command_mode = 0x700;
    constexpr unsigned address_mode = 0xb00;
    constexpr unsigned write_strobe = 0x200;
    const auto controls = command ? command_mode : address_mode;
    write(c_data, controls);
    write(f_data, value);
    write(c_data, controls & ~write_strobe);
    nops(1);
    write(c_data, controls);
  }

  void select_row(const unsigned row) {
    latch(0, true);
    for (const auto value : {0U, row, 0U, 0U}) { latch(value, false); }
    write(c_data, inactive);
  }

  void read() {
    write(c_data, read_active);
    nops(2);
    words_.push_back(f_read_x0);
  }

  Markers markers_;
  std::vector<std::uint32_t> words_;
};

struct Fixture {
  Fixture(const NandReadContinuation part,
          const unsigned count,
          const HostDirection host_direction = HostDirection::full_byte)
      : program(count, host_direction),
        peer(machine, image(part), NandCellProfile{}, NandBusContract::functional_transactions) {
    program.install(machine.bus());
    runtime = std::make_unique<NandNativeRun>(
        machine,
        peer,
        NandPeerEvents{
            .identity = std::make_shared<const PeripheralEventIdentity>(),
            .next_ticks = []() -> std::optional<std::uint64_t> { return inherited_tick; },
            .observe = [](std::uint64_t) { throw Fault("Inherited event remains pending"); }});
  }

  void until(const unsigned target) {
    for (;;) {
      require(native_cycles_ < maximum_cycles, "Authored read/host/read native suffix is bounded");
      ++native_cycles_;
      const auto result = runtime->tick();
      if (result.native && result.native->retired && machine.cpu().registers().pc == target) {
        return;
      }
    }
  }

  OwnDspMachine machine{clocks};
  Program program;
  NandPeer peer;
  std::unique_ptr<NandNativeRun> runtime;
  unsigned native_cycles_ = 0;
};

void neutral_endpoint(const Fixture &f, const unsigned count, const bool released) {
  const auto state = f.peer.detailed();
  require(state.neutral_bus && state.page == 0 && state.column == count &&
              state.base.ready == true && state.base.pending == NandPending::none,
          "Read handoff preserves consumed pointer and Ready without P-row advance or write");
  if (released) {
    require(state.output.driven == false && !state.base.next,
            "Released NAND neither drives data nor schedules an unwanted row while host owns bus");
  }
}

void finish(Fixture &f, const unsigned count) {
  f.until(f.program.markers().neutral);
  neutral_endpoint(f, count, false);
  f.until(f.program.markers().released);
  neutral_endpoint(f, count, true);
  f.until(f.program.markers().host_done);
  neutral_endpoint(f, count, true);
  require(f.machine.bus().gpio(DspPort::c).read(DspGpioRegister::data) ==
              (read_active | host_command),
          "Functional NAND isolation leaves the actual GPIO host word unchanged");
  f.until(f.program.markers().host_input);
  neutral_endpoint(f, count, true);
  f.until(f.program.markers().armed);
  const auto armed = f.peer.detailed();
  require(!armed.neutral_bus && armed.page == 0 && armed.column == count &&
              armed.base.ready == true && armed.base.pending == NandPending::none,
          "Only a real inactive control phase arms NAND again, without a phantom byte or Busy");
  f.until(f.program.markers().stop);
  const auto state = f.peer.detailed();
  const bool full = count == Nand::page_bytes;
  const auto expected = full ? next_row_byte : second_byte;
  require((f.machine.cpu().registers().x & byte_mask) == expected && !state.neutral_bus &&
              state.page == (full ? 1U : 0U) && state.column == (full ? 1U : 2U) &&
              state.base.ready == true && state.base.pending == NandPending::none,
          "Actual native MOVEP consumes the independently expected resumed or newly selected byte");
  require(f.runtime->state().next_external == inherited_tick &&
              f.runtime->state().external_events == 0,
          "Read/host/read keeps unrelated inherited events intact");
}

void partial_guard() {
  Fixture f(NandReadContinuation::k9f1208_j, 1, HostDirection::partial_byte);
  f.until(f.program.markers().active);
  bool refused = false;
  try {
    f.until(f.program.markers().neutral);
  } catch (const Fault &error) {
    const std::string message = error.what();
    refused = message.contains("partial GPIO output mask=0x001") &&
              message.contains("Joined NAND pins at X:0xffffbd/Y:0xfffff5 pc=");
  }
  require(refused && f.peer.faulted(),
          "An unqualified partial-byte owner fails visibly and quarantines its native peer");
  std::cout << "PASS partial-byte owner refusal with native PC/address context\n";
}

void run(const NandReadContinuation part, const unsigned count) {
  Fixture f(part, count);
  f.until(f.program.markers().active);
  const auto before = f.peer.detailed();
  const auto expected = count == Nand::page_bytes ? final_byte : first_byte;
  require((f.machine.cpu().registers().x & byte_mask) == expected && before.page == 0 &&
              before.column == count && before.previous_re == false && !before.neutral_bus &&
              before.base.ready == true,
          "Independent literal byte and still-selected native read precede host handoff");
  const auto peer_cut = f.peer.checkpoint();
  const auto runtime_cut = f.runtime->checkpoint();
  finish(f, count);
  const auto endpoint = native_cpu(f.machine);
  const auto peer_endpoint = f.peer.detailed();
  const auto calendar_endpoint = f.runtime->peripheral_state();
  f.peer.restore(peer_cut);
  f.runtime->restore(runtime_cut);
  require(f.peer.detailed() == before, "Active read payload restores before native replay");
  finish(f, count);
  require(native_cpu(f.machine) == endpoint && f.peer.detailed() == peer_endpoint &&
              f.runtime->peripheral_state() == calendar_endpoint,
          "Joint active-read restore repeats complete native CPU, cell and calendar suffix");
  std::cout << "PASS read-host-read part=" << static_cast<unsigned>(part) << " consumed=" << count
            << " host-bytes=256 host-control-words=16 retained-read-input-pause=1 replay=1\n";
}
}

int main() {
  try {
    for (const auto part : {NandReadContinuation::k9f1208_j, NandReadContinuation::k9f1208_p}) {
      for (const auto count : {1U, Nand::page_bytes}) { run(part, count); }
    }
    partial_guard();
    std::cout
        << "Native J/P mid-page/final-byte read handoff, host ownership and restore passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
