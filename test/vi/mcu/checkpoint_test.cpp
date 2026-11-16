// SPDX-License-Identifier: GPL-3.0-only
#include "aot/mcu.hpp"
#include "io/dsp_reset.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "mcu/boot_port.hpp"
#include "mcu/devices.hpp"
#include "mcu/io.hpp"
#include "mcu/memory.hpp"
#include "mcu/program.hpp"
#include "storage/flash_image.hpp"
#include "support/mcu.hpp"

#include <algorithm>
#include <array>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <vector>

namespace {
namespace vi = blofeld::vi;
namespace mcu = vi::mcu;
constexpr std::uint8_t datum = 0xa5;
constexpr std::uint8_t other = 0x5a;
constexpr std::uint8_t all_bits = 0xff;
constexpr std::uint16_t system = 0x1800;
constexpr std::uint16_t queue = 0x0400;
constexpr std::uint16_t remapped = 0x2400;
constexpr std::uint16_t initmq = 0x12;
constexpr std::uint16_t initrm = 0x10;
constexpr std::uint8_t queue_map = 0x21;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

using Observation = std::array<std::optional<std::uint8_t>, cpu12::address_bytes>;

Observation observe_ram(const mcu::Memory &memory) {
  Observation result{};
  for (std::size_t address = 0; address < result.size(); ++address) {
    if (memory.locate(static_cast<std::uint16_t>(address)).space == mcu::Space::flash) { continue; }
    std::uint8_t value = 0;
    if (memory.read(static_cast<std::uint16_t>(address), value)) { result[address] = value; }
  }
  return result;
}

void memory_state() {
  vi::FlashImage flash;
  flash.bytes[0] = datum;
  flash.known[0] = 1;
  mcu::Memory memory(flash);
  require(memory.write({.address = system, .value = datum}) &&
              memory.write({.address = queue, .value = other}) &&
              memory.write_mapping({.address = initrm, .value = 1}) &&
              memory.write_mapping({.address = initmq, .value = queue_map}),
          "Prepare real RAM, knowledge and a staged mapping write");
  const auto saved = memory.save();
  const auto before = observe_ram(memory);
  memory.power_on();
  flash.bytes[0] = other;
  const auto image = flash.encode();
  memory.restore(saved);
  require(observe_ram(memory) == before && flash.encode() == image && memory.program_changed(),
          "Volatile bytes/knowledge restore without rewinding persistent storage; code bindings "
          "are revoked");
  std::uint8_t value = 0;
  require(memory.commit_mapping() && memory.read(remapped, value) && value == other &&
              !memory.read(queue, value) && !memory.write_mapping({.address = initrm, .value = 0}),
          "The restored pending map commits once, retaining write-once register history");
}

void code_binding() {
  vi::FlashImage flash;
  mcu::Memory memory(flash);
  constexpr auto nop = cpu12::isa::nop;
  constexpr cpu12::Byte corrupt = 0;
  require(memory.write({.address = system, .value = nop[0]}), "Initialize authored RAM code");
  const std::array spans{cpu12::CodeSpan{.address = system, .words = nop}};
  const std::array bindings{mcu::CodeBinding{.identity = 1, .code = spans}};
  require(memory.bind_program(bindings), "Protect the known RAM code extent");
  const auto saved = memory.save();
  require(memory.write({.address = system, .value = corrupt}) && memory.program_changed(),
          "A real code write revokes its prepared view");
  memory.restore(saved);
  require(!memory.bind_program(bindings) && memory.commit_mapping() &&
              memory.bind_program(bindings) && !memory.program_changed(),
          "Restore requires a new owner binding before the RAM code is executable again");
  require(memory.write({.address = system, .value = corrupt}) && memory.program_changed(),
          "The restored binding still detects later code changes");
}

void retained_code_binding() {
  vi::FlashImage flash;
  mcu::Memory memory(flash);
  constexpr auto nop = cpu12::isa::nop;
  constexpr cpu12::Address data = system + 16;
  require(memory.write({.address = system, .value = nop[0]}), "Initialize actual RAM code");
  const std::array spans{cpu12::CodeSpan{.address = system, .words = nop}};
  const std::array bindings{mcu::CodeBinding{.identity = 1, .code = spans}};
  require(memory.bind_program(bindings), "Bind a real protected code extent");
  const auto saved = memory.save();
  const auto grants = memory.regions();
  require(memory.write({.address = data, .value = datum}) && memory.restore_bound(saved) &&
              !memory.program_changed() && !memory.commit_mapping(),
          "Data rollback retains the established code protection and actual mapping");
  const auto restored = memory.regions();
  require(std::ranges::equal(grants,
                             restored,
                             [](const auto &a, const auto &b) {
                               return a.first == b.first && a.bytes.data() == b.bytes.data() &&
                                      a.bytes.size() == b.bytes.size() &&
                                      a.known.data() == b.known.data() &&
                                      a.known.size() == b.known.size() &&
                                      a.writable == b.writable && a.identity == b.identity;
                             }),
          "Every retained code/data/knowledge descriptor has identical storage and rights");
  require(memory.write({.address = system, .value = other}) && memory.program_changed(),
          "A later real code write still revokes the retained binding");
  const auto changed = observe_ram(memory);
  require(!memory.restore_bound(saved) && observe_ram(memory) == changed &&
              memory.program_changed(),
          "An invalidated code binding refuses transactionally before restoring any byte");
  memory.restore(saved);
  require(memory.commit_mapping() && memory.bind_program(bindings) &&
              memory.write_mapping({.address = initmq, .value = queue_map}) &&
              memory.commit_mapping(),
          "Commit an actual queue remap");
  const auto remap = observe_ram(memory);
  require(!memory.restore_bound(saved) && observe_ram(memory) == remap,
          "An active mapping change cannot retain the old Core descriptors");
}

void pending_handoff() {
  namespace isa = cpu12::isa;
  constexpr cpu12::Address from = 0x8000;
  constexpr cpu12::Address to = 0x8100;
  constexpr std::uint64_t first_id = 7;
  constexpr std::uint64_t second_id = 9;
  constexpr auto jump = isa::program(isa::opcode<isa::Opcode::jmp_extended>(),
                                     std::array<cpu12::Byte, 2>{to >> 8U, to & all_bits});
  constexpr auto target =
      isa::program(isa::opcode<isa::Opcode::inca>(), std::array<cpu12::Byte, 2>{0x20, 0xfe});
  blofeld::test::McuImage image;
  image.load(from, jump);
  image.load(to, target);
  const std::array first_code{cpu12::CodeSpan{.address = from, .words = jump}};
  const std::array second_code{cpu12::CodeSpan{.address = to, .words = target}};
  const std::array first_entries{from};
  const std::array<cpu12::Address, 2> second_entries{to, to + 1};
  const std::array variants{
      cpu12::CodeVariant{.id = first_id, .code = first_code, .entries = first_entries},
      cpu12::CodeVariant{.id = second_id, .code = second_code, .entries = second_entries}};
  const std::array transitions{
      cpu12::CodeTransition{.from = first_id, .to = second_id, .entry = to},
      cpu12::CodeTransition{.from = first_id, .to = second_id, .entry = to + 1},
      cpu12::CodeTransition{.from = second_id, .to = first_id, .entry = from}};
  const auto program = blofeld::app::aot::prepare_mcu_program({.variants = variants}, transitions);
  mcu::Memory memory(*image);
  const auto grants = memory.regions();
  cpu12::Core core(cpu12::Memory::mapped(grants));
  const std::array mounted{first_id};
  const std::array both{first_id, second_id};
  core.set_registers({.pc = from});
  program.install(core, mounted);
  const auto saved = core.save();
  const auto jumped = core.run(1);
  require(jumped.retired == 1 && jumped.fault == cpu12::Fault::none && core.registers().pc == to &&
              program.handoff(mounted, to) && !program.handoff(mounted, from),
          "An actual exit to an unmounted prepared view is a possible pending handoff");
  const auto boundary = core.save();
  const auto pending = core.run(1);
  require(pending.retired == 0 && pending.fault == cpu12::Fault::none && core.save() == boundary,
          "A declared handoff yields without retiring another instruction or inventing a fault");
  core.restore(saved);
  require(core.run(1).retired == 0 && core.save() == saved,
          "Processor restoration does not retire the pending prepared-code handoff");
  program.install(core, both);
  require(!program.handoff(both, to) && core.run(1).retired == 1 && core.run(1).retired == 1 &&
              core.registers().d == (std::uint16_t{1} << 8U),
          "A fresh install clears the stale handoff; an already mounted target remains native");
}

void boot_shifter() {
  constexpr std::uint32_t control = 0xffff91;
  constexpr std::uint32_t receive = 0xffff94;
  constexpr std::uint32_t fifo24 = 0x29;
  constexpr std::uint32_t word = 0xa591e7;
  constexpr unsigned width = 24;
  constexpr unsigned halfway = width / 2;
  constexpr unsigned data_bit = 0x10;
  vi::Shi shi;
  mcu::BootPort port(shi);
  require(shi.write({.address = control, .value = fifo24}), "Configure the real SPI FIFO");
  const auto drive = [&](const std::uint8_t value) {
    require(port.write({.direction = all_bits, .known = all_bits, .value = value}),
            "Drive a real SPI clock/data edge");
  };
  const auto bits = [&](const unsigned first, const unsigned last) {
    for (unsigned bit = first; bit > last; --bit) {
      const auto data = static_cast<std::uint8_t>(1 | (((word >> (bit - 1)) & 1) * data_bit));
      drive(data);
      drive(static_cast<std::uint8_t>(data | 4));
      drive(data);
    }
  };
  drive(3);
  drive(1);
  bits(width, halfway);
  const auto saved = port.save();
  port.reset();
  port.restore(saved);
  bits(halfway, 0);
  require(port.pending() && port.complete({}) && !port.pending(),
          "The partial word resumes from its saved bit count, value, mode and clock phase");
  std::uint32_t actual = 0;
  require(shi.read(receive, actual) && actual == word && !shi.read(receive, actual),
          "Restoring local shifter state neither consumes nor duplicates shared FIFO data");
}

void device_continuations() {
  constexpr std::uint16_t baud = 0xc8;
  constexpr std::uint16_t sci_control = 0xcb;
  constexpr std::uint16_t sci_data = 0xcf;
  constexpr std::uint16_t sci_status = 0xcc;
  constexpr std::uint16_t empty = 0x80;
  constexpr std::uint16_t tscr1 = 0x46;
  constexpr std::uint16_t tcnt = 0x44;
  constexpr std::uint16_t enabled = 0x80;
  constexpr std::uint16_t umcr = 0x300;
  constexpr std::uint16_t usb_clock = 0x4000;
  constexpr std::uint16_t usb_select = 0x30e;
  constexpr std::uint16_t usb_data = 0x310;
  constexpr std::uint16_t selected = 0x100;
  constexpr std::uint16_t high = 0x1234;
  constexpr std::uint16_t low = 0x5678;
  constexpr std::uint64_t before = 154;
  constexpr std::uint64_t after = 320;
  const std::stop_token stop;
  mcu::Devices devices(stop);
  std::uint16_t value = 0;
  require(devices.sci.write({.offset = baud, .value = 1, .bytes = 2}) &&
              devices.sci.write({.offset = sci_control, .value = 8}) &&
              devices.sci.read({.offset = sci_status}, value) && value == empty &&
              devices.sci.write({.offset = sci_data, .value = datum}) &&
              devices.sci.advance(before) && devices.clock.advance(before) &&
              devices.clock.write({.offset = umcr, .value = usb_clock, .bytes = 2}) &&
              devices.timer.write({.offset = tscr1, .value = enabled}) &&
              devices.timer.advance(before) &&
              devices.usb.write({.offset = usb_select, .value = selected, .bytes = 2}, usb_clock) &&
              devices.usb.write({.offset = usb_data, .value = high, .bytes = 2}, usb_clock),
          "Prepare an in-flight UART, pending PLL lock, running timer and staged USB word");
  devices.idle = before;
  const auto saved = devices.save();
  const auto lock = devices.clock.deadline();
  if (!lock) { throw std::runtime_error("PLL qualification must actually be pending"); }
  require(devices.sci.advance(after) && devices.clock.advance(after) &&
              devices.timer.advance(after),
          "Advance beyond all saved deadlines");
  const auto original = devices.sci.take_output();
  require(original && original->word == datum && original->cycle == after,
          "The original UART setup must complete its real queued character");
  require(devices.usb.write({.offset = usb_data + 2, .value = 0, .bytes = 2}, usb_clock),
          "Commit the wrong USB low word before rollback");
  devices.idle = 0;
  devices.restore(saved);
  require(devices.idle == before && devices.clock.deadline() == lock &&
              devices.clock.advance(lock.value()) && devices.sci.advance(after) &&
              devices.timer.advance(after),
          "Restore reinstates deadlines, phases and owner idle accounting");
  const auto output = devices.sci.take_output();
  require(output && output->cycle == after && output->word == datum && output->bits == 8 &&
              !devices.sci.take_output(),
          "The real pending UART character completes exactly once");
  require(devices.timer.read({.offset = tcnt, .bytes = 2}, value) && value == after &&
              devices.usb.write({.offset = usb_data + 2, .value = low, .bytes = 2}, usb_clock) &&
              devices.usb.read({.offset = usb_data, .bytes = 2}, value) && value == high &&
              devices.usb.read({.offset = usb_data + 2, .bytes = 2}, value) && value == low,
          "Timer state and the staged USB high word survive rollback");
}

struct Machine {
  static constexpr std::uint8_t write_control = 0x40, read_control = 0x61, enable = 0x10;
  blofeld::test::McuImage image;
  mcu::Memory memory{*image};
  const std::stop_token stop;
  mcu::Devices devices{stop};
  vi::Shi shi;
  vi::DspReset reset;
  mcu::Io io{memory, *image, devices, shi, reset, stop, nullptr, nullptr, nullptr};
  cpu12::Io callbacks = io.callbacks();
  std::vector<cpu12::Ram> grants;
  cpu12::Core core{cpu12::Memory::external(), &callbacks};

  Machine() { io.bind(core); }

  void write(const cpu12::ByteWrite access) const {
    require(callbacks.write(callbacks.owner, access), "Perform an actual MCU register store");
  }

  std::uint8_t read(const std::uint16_t address) const {
    std::uint8_t value = 0;
    require(callbacks.read(callbacks.owner, address, value), "Read a defined MCU register");
    return value;
  }

  void send(const std::uint8_t value, const bool data = false) const {
    write({.address = 1, .value = static_cast<std::uint8_t>(write_control | (data ? 1U : 0U))});
    write({.address = 2, .value = all_bits});
    write({.address = 0, .value = value});
    write({.address = 8, .value = enable});
    write({.address = 8});
  }

  void begin_read() const {
    write({.address = 2});
    write({.address = 1, .value = read_control});
    write({.address = 8, .value = enable});
  }

  void bind(const vi::mcu::Program &program) {
    static_cast<void>(memory.commit_mapping());
    const auto admitted = program.admit(memory, core.registers().pc);
    auto next = memory.regions();
    core.set_memory(cpu12::Memory::mapped(next));
    program.install(core, admitted);
    grants.swap(next);
  }
};

void lcd_and_flash() {
  constexpr std::uint8_t outputs = 0xe1;
  constexpr std::uint8_t page_two = 0xb2;
  constexpr std::uint8_t column = 3;
  constexpr std::uint16_t divider = 0x100;
  constexpr std::uint16_t status = 0x105;
  constexpr std::uint16_t flash_first = 0x8000;
  constexpr std::uint8_t ready = 0xc0;
  constexpr std::uint8_t access_error = 0x10;
  constexpr std::uint8_t loaded = 0x80;
  Machine machine;
  machine.write({.address = 3, .value = outputs});
  machine.write({.address = divider, .value = column});
  machine.send(page_two);
  machine.send(column);
  machine.send(datum, true);
  machine.send(column);
  machine.begin_read();
  require(machine.read(0) == column, "Observe the real LCD dummy-read holder");
  const auto saved = machine.io.save();
  machine.write({.address = 8});
  machine.begin_read();
  require(machine.read(0) == datum, "Finish the first LCD dummy-read phase");
  machine.write({.address = 8});
  machine.write({.address = flash_first});
  require((machine.read(status) & access_error) != 0,
          "Generate a real flash-controller access error");
  machine.io.restore(saved);
  require(machine.read(status) == ready && machine.read(divider) == (loaded | column) &&
              machine.read(0) == column,
          "Flash flags and the active LCD read phase restore exactly");
  machine.write({.address = 8});
  machine.begin_read();
  require(machine.read(0) == datum,
          "The restored LCD dummy-read completes once at the saved column");
}

void native_continuation(const bool retained) {
  namespace isa = cpu12::isa;
  using Opcode = isa::Opcode;
  constexpr cpu12::Address entry = 0x8000;
  constexpr cpu12::Address stack = 0x1fff;
  constexpr cpu12::Address port = 0x240;
  constexpr cpu12::Byte first = 0x0a;
  constexpr cpu12::Byte second = first + 1;
  constexpr auto code = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                     std::array<cpu12::Byte, 1>{all_bits},
                                     isa::opcode<Opcode::staa_extended>(),
                                     std::array<cpu12::Byte, 2>{2, 0x42},
                                     isa::opcode<Opcode::ldaa_immediate>(),
                                     std::array<cpu12::Byte, 1>{first},
                                     isa::opcode<Opcode::staa_extended>(),
                                     std::array<cpu12::Byte, 2>{2, 0x40},
                                     isa::opcode<Opcode::inca>(),
                                     isa::opcode<Opcode::staa_extended>(),
                                     std::array<cpu12::Byte, 2>{2, 0x40},
                                     isa::wai,
                                     std::array<cpu12::Byte, 2>{0x20, 0xfe});
  Machine machine;
  machine.image.load(entry, code);
  const auto program = machine.image.program();
  machine.core.set_registers({.pc = entry, .sp = stack});
  machine.bind(*program);
  require(machine.core.run(4).retired == 4 && machine.io.complete_events(),
          "Run prepared GPIO setup");
  machine.devices.activity(machine.core.state().execution);
  const auto cpu = machine.core.save();
  const auto memory = machine.memory.save();
  const auto devices = machine.devices.save();
  const auto io = machine.io.save();
  require(machine.read(port) == first, "Observe the first original GPIO latch");
  const auto resume = [&] {
    const auto result = machine.core.run(3);
    require(result.retired == 3 && result.state == cpu12::ExecutionState::waiting &&
                machine.io.complete_events(),
            "Continue through a real WAI and its stack stores");
    machine.devices.activity(machine.core.state().execution);
  };
  resume();
  const auto completed = machine.core.save();
  const auto observed = observe_ram(machine.memory);
  if (retained) {
    require(machine.memory.restore_bound(memory), "Restore data without replacing a valid binding");
  } else {
    machine.memory.power_on();
    machine.memory.restore(memory);
  }
  machine.devices.restore(devices);
  machine.io.restore(io);
  machine.core.restore(cpu);
  if (!retained) { machine.bind(*program); }
  resume();
  require(machine.core.save() == completed && observe_ram(machine.memory) == observed &&
              machine.read(port) == second &&
              machine.devices.clock.cycles() == completed.progress.estimated_cycles,
          "Prepared host fusion resumes with identical CPU, RAM knowledge, GPIO and device time");
}
}

int main() try {
  memory_state();
  code_binding();
  retained_code_binding();
  pending_handoff();
  boot_shifter();
  device_continuations();
  lcd_and_flash();
  native_continuation(false);
  native_continuation(true);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
