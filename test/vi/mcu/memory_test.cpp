// SPDX-License-Identifier: GPL-3.0-only
#include "interrupt_model.hpp"
#include "isa.hpp"
#include "mcu/memory.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <vector>

namespace {
namespace isa = cpu12::isa;
constexpr cpu12::Address overlay_first = 0x1600;
constexpr std::size_t overlay_bytes = 0x210;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void mapping() {
  using blofeld::vi::mcu::Space;
  blofeld::vi::FlashImage flash;
  flash.known.fill(1);
  blofeld::vi::mcu::Memory memory(flash);
  constexpr std::uint16_t system = 0x1200;
  constexpr std::uint16_t old_queue = 0x0400;
  constexpr std::uint16_t queue_reserved = 0x0600;
  constexpr std::uint16_t new_queue = 0x2400;
  constexpr std::uint16_t queue_register = 0x12;
  constexpr std::uint8_t queue_at_2000 = 0x21;
  constexpr std::uint8_t data = 0x55;
  std::uint8_t value = 0;
  require(memory.locate(0).space == Space::registers, "Registers override QRAM");
  require(memory.locate(system).space == Space::system_ram &&
              memory.locate(old_queue).space == Space::queue_ram &&
              memory.locate(queue_reserved).space == Space::reserved,
          "Reset map has exact SRAM, QRAM and reserved ranges");
  require(!memory.read(system, value), "Power-on RAM is unknown");
  require(memory.write({.address = system, .value = data}) &&
              memory.write({.address = old_queue, .value = data}),
          "Actual RAM stores");
  require(memory.write_mapping({.address = queue_register, .value = queue_at_2000}),
          "Stage QRAM remap");
  require(memory.read_mapping(queue_register) == 1 && memory.read(old_queue, value),
          "Mapping remains unchanged inside the instruction");
  require(memory.commit_mapping() && memory.read(new_queue, value) && value == data,
          "Remap exposes the same backing bytes");
  require(!memory.read(old_queue, value), "Old QRAM address is no longer granted");
  memory.reset();
  require(memory.read(system, value) && value == data && memory.read(old_queue, value),
          "Warm reset retains RAM while restoring the reset map");
  memory.power_on();
  require(!memory.read(system, value) && !memory.read(old_queue, value),
          "Power cycle retires volatile knowledge");
  constexpr std::uint16_t initrm = 0x10;
  require(memory.write_mapping({.address = initrm, .value = 0}) && memory.commit_mapping(),
          "First INITRM write");
  require(!memory.write_mapping({.address = initrm, .value = 9}) &&
              memory.read_mapping(initrm) == 0,
          "INITRM is write-once in normal mode");
  constexpr std::uint16_t primary = 0x8000;
  constexpr std::uint16_t alias = 0x4000;
  flash.bytes[0] = data;
  require(memory.read(primary, value) && value == data && memory.read(alias, value) &&
              value == data,
          "Primary and lower flash mapping alias physical storage");
  require(!memory.write({.address = primary, .value = 0}),
          "Flash writes require the flash controller");
}

void verify_regions(blofeld::vi::mcu::Memory &memory, const blofeld::vi::FlashImage &flash) {
  using blofeld::vi::mcu::Space;
  const auto views = memory.regions();
  std::size_t next = 0;
  unsigned end = 0;
  for (const auto &view : views) {
    require(!view.bytes.empty() && view.first >= end &&
                view.bytes.size() <= cpu12::address_bytes - view.first,
            "Direct grants are nonempty, sorted, disjoint and addressable");
    end = static_cast<unsigned>(view.first + view.bytes.size());
  }
  for (unsigned address = 0; address < cpu12::address_bytes; ++address) {
    const auto at = static_cast<cpu12::Address>(address);
    const auto location = memory.locate(at);
    const bool ram = location.space == Space::system_ram || location.space == Space::queue_ram;
    const bool readable_flash = location.space == Space::flash && flash.known[location.offset] != 0;
    while (next < views.size() && address >= views[next].first + views[next].bytes.size()) {
      ++next;
    }
    const bool granted = next < views.size() && address >= views[next].first;
    require(granted == (ram || readable_flash),
            "Every address agrees with the functional mapping and actual flash knowledge");
    if (!granted) { continue; }
    const auto &view = views[next];
    const auto offset = address - view.first;
    std::uint8_t value = 0;
    if (readable_flash) {
      require(!view.writable && view.known.empty() && view.identity == 0 &&
                  view.bytes.data() + offset == flash.bytes.data() + location.offset,
              "Flash grants borrow exact physical bytes, including both aliases and holes");
    } else {
      require(view.writable && view.identity == 0 && view.known.size() == view.bytes.size() &&
                  memory.read(at, value) == (view.known[offset] != 0),
              "Direct RAM grants preserve writability and every initializedness bit");
      if (view.known[offset] != 0) {
        require(view.bytes[offset] == value, "Direct and functional RAM reads agree");
      }
    }
  }
}

void mapping_regions() {
  blofeld::vi::FlashImage flash;
  for (std::size_t index = 0; index < flash.bytes.size(); ++index) {
    flash.bytes[index] = static_cast<std::uint8_t>(index ^ (index >> 8U));
  }
  const auto check = [&flash](const std::array<std::uint8_t, 4> &mapping) {
    blofeld::vi::mcu::Memory memory(flash);
    constexpr cpu12::Address mapping_first = 0x10;
    for (std::size_t index = 0; index < mapping.size(); ++index) {
      static_cast<void>(
          memory.write_mapping({.address = static_cast<cpu12::Address>(mapping_first + index),
                                .value = mapping[index]}));
    }
    static_cast<void>(memory.commit_mapping());
    constexpr unsigned stride = 8;
    for (unsigned address = 0; address < cpu12::address_bytes; address += stride) {
      static_cast<void>(
          memory.write({.address = static_cast<cpu12::Address>(address),
                        .value = static_cast<std::uint8_t>(address ^ (address >> 8U))}));
    }
    verify_regions(memory, flash);
  };
  flash.known.fill(1);
  check({9, 0, 1, 0x0d});
  flash.known.fill(0);
  check({9, 0, 1, 0x0d});
  constexpr unsigned hole_mask = 0x7f;
  for (std::size_t index = 0; index < flash.known.size(); ++index) {
    flash.known[index] = (index & hole_mask) == 0 ? 0 : static_cast<std::uint8_t>(index | 1U);
  }
  // Every raw byte for each independent map register, including masked bits.
  for (unsigned value = 0; value <= UINT8_MAX; ++value) {
    const auto byte = static_cast<std::uint8_t>(value);
    check({byte, 0, 1, 0x0d});
    check({9, byte, 1, 0x0d});
    check({9, 0, byte, 0x0d});
    check({9, 0, 1, byte});
    // Intersect all windows, including the last byte of the address space.
    check({static_cast<std::uint8_t>(byte | 1U), byte, static_cast<std::uint8_t>(byte | 1U), 1});
  }
}

void native_memory() {
  namespace isa = cpu12::isa;
  constexpr std::uint16_t program_address = 0x8200;
  constexpr std::size_t program_offset = 0x200;
  constexpr std::uint16_t destination = 0x1200;
  constexpr std::uint16_t stack = 0x1fff;
  constexpr std::uint16_t reset_vector = 0xfffe;
  constexpr std::uint8_t data = 0x55;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::ldaa_immediate>(),
                                        std::array{data},
                                        isa::opcode<isa::Opcode::staa_extended>(),
                                        std::array{static_cast<std::uint8_t>(destination >> 8U),
                                                   static_cast<std::uint8_t>(destination)},
                                        isa::stop,
                                        std::array<cpu12::Byte, 2>{0x20, 0xfe});
  blofeld::vi::FlashImage image;
  image.known.fill(1);
  std::ranges::copy(program, image.bytes.begin() + program_offset);
  image.bytes[blofeld::vi::FlashImage::byte_count - 2] =
      static_cast<std::uint8_t>(program_address >> 8U);
  image.bytes[blofeld::vi::FlashImage::byte_count - 1] = static_cast<std::uint8_t>(program_address);
  blofeld::vi::mcu::Memory memory(image);
  const auto grants = memory.regions();
  cpu12::Core core(cpu12::Memory::mapped(grants));
  require(core.reset(reset_vector).accepted && core.registers().pc == program_address,
          "Native reset vector comes from the borrowed flash image");
  auto registers = core.registers();
  registers.sp = stack;
  registers.ccr = 0;
  core.set_registers(registers);
  const std::array code{cpu12::CodeSpan{.address = program_address, .words = program}};
  const std::array entries{program_address};
  const std::array variants{cpu12::CodeVariant{.id = 1, .code = code, .entries = entries}};
  const auto native = cpu12::Core::compile({.variants = variants},
                                           {.memory = {},
                                            .interrupts = {},
                                            .transitions = {},
                                            .callbacks = {},
                                            .bindings = {},
                                            .topology = cpu12::MemoryTopology::dynamic});
  core.install(native, 1);
  const auto result = core.run();
  require(result.fault == cpu12::Fault::none && result.state == cpu12::ExecutionState::stopped,
          "Authored native program stops without an execution budget");
  std::uint8_t value = 0;
  require(memory.read(destination, value) && value == data,
          "Native RAM writes and knownness reach the consumer directly");
}

void unknown_memory() {
  namespace isa = cpu12::isa;
  constexpr std::uint16_t program_address = 0x8200;
  constexpr std::size_t program_offset = 0x200;
  constexpr std::uint16_t unreadable = 0x1210;
  constexpr std::uint16_t flash_hole = 0x8300;
  constexpr std::size_t physical_hole = 0x300;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::ldaa_extended>(),
                                        std::array{static_cast<std::uint8_t>(unreadable >> 8U),
                                                   static_cast<std::uint8_t>(unreadable)},
                                        std::array<cpu12::Byte, 2>{0x20, 0xfe});
  blofeld::vi::FlashImage image;
  image.known.fill(1);
  image.known[physical_hole] = 0;
  std::ranges::copy(program, image.bytes.begin() + program_offset);
  blofeld::vi::mcu::Memory memory(image);
  const auto grants = memory.regions();
  for (const auto grant : grants) {
    require(flash_hole < grant.first || flash_hole - grant.first >= grant.bytes.size(),
            "Unknown flash never becomes an immutable native grant");
  }
  cpu12::Core core(cpu12::Memory::mapped(grants));
  core.set_registers({.pc = program_address});
  const std::array code{cpu12::CodeSpan{.address = program_address, .words = program}};
  const std::array entries{program_address};
  const std::array variants{cpu12::CodeVariant{.id = 1, .code = code, .entries = entries}};
  const auto native = cpu12::Core::compile({.variants = variants},
                                           {.memory = {},
                                            .interrupts = {},
                                            .transitions = {},
                                            .callbacks = {},
                                            .bindings = {},
                                            .topology = cpu12::MemoryTopology::dynamic});
  core.install(native, 1);
  const auto result = core.run();
  require(result.fault == cpu12::Fault::memory_read && result.address == unreadable &&
              result.pc == program_address && result.retired == 0,
          "Uninitialized native load fails with its actual PC and address");
}

cpu12::Ram grant(blofeld::vi::mcu::Memory &memory, const cpu12::Address address) {
  for (const auto view : memory.regions()) {
    if (address >= view.first && std::size_t{address} - view.first < view.bytes.size()) {
      return view;
    }
  }
  throw std::runtime_error("Expected an actual borrowed memory grant");
}

bool bind(blofeld::vi::mcu::Memory &memory,
          const std::span<const cpu12::CodeSpan> code,
          const std::uint64_t identity) {
  const std::array bindings{blofeld::vi::mcu::CodeBinding{.identity = identity, .code = code}};
  return memory.bind_program(bindings);
}

void program_bindings() {
  blofeld::vi::FlashImage image;
  blofeld::vi::mcu::Memory memory(image);
  constexpr auto original = cpu12::isa::nop[0];
  constexpr cpu12::Byte replacement = 0x5a;
  std::array<cpu12::Byte, overlay_bytes> bytes;
  bytes.fill(original);
  const std::array code{cpu12::CodeSpan{.address = overlay_first, .words = bytes}};
  require(!bind(memory, code, 1), "Unknown RAM cannot become immutable program");
  for (std::size_t index = 0; index < overlay_bytes - 1; ++index) {
    require(memory.write(
                {.address = static_cast<cpu12::Address>(overlay_first + index), .value = original}),
            "Initialize independently authored program bytes");
  }
  require(!bind(memory, code, 1), "One unknown tail byte prevents executable binding");
  require(memory.write({.address = overlay_first + overlay_bytes - 1, .value = original}) &&
              bind(memory, code, 1),
          "Only complete actual source bytes receive an immutable binding");
  const auto first = grant(memory, overlay_first);
  require(!first.writable && first.known.empty() && first.identity == 1 &&
              first.bytes.size() == overlay_bytes && first.first == overlay_first,
          "The exact code extent has initialized read-only native grants");
  require(grant(memory, overlay_first - 1).writable &&
              grant(memory, overlay_first + overlay_bytes).writable && bind(memory, code, 1),
          "Adjacent data stays writable and repeated source admission is valid");
  require(memory.write({.address = overlay_first, .value = original}) &&
              !memory.program_changed() && !memory.commit_mapping(),
          "An unchanged real write preserves the executable binding");
  require(memory.write({.address = overlay_first, .value = replacement}) &&
              memory.program_changed() && !bind(memory, code, 1) && memory.commit_mapping() &&
              !memory.program_changed(),
          "A code change releases ownership at its completed instruction boundary");
  require(!bind(memory, code, 1), "Changed bytes cannot reuse the declared original source");
  bytes[0] = replacement;
  require(bind(memory, code, 2) && grant(memory, overlay_first).identity == 2,
          "A separately declared changed source has its own identity");
  require(memory.write({.address = overlay_first, .value = original}) && memory.commit_mapping(),
          "The original store restores its real byte");
  bytes[0] = original;
  require(bind(memory, code, 1) && grant(memory, overlay_first).identity == first.identity,
          "Restoring identical bytes admits the original declared source");
  memory.power_on();
  require(!bind(memory, code, 1) && grant(memory, overlay_first).writable,
          "Power-on removes executable ownership without initializing RAM");
  require(memory.commit_mapping(), "Power-on retires the preceding executable grant");
  bytes.fill(replacement);
  for (std::size_t index = 0; index < overlay_bytes; ++index) {
    require(memory.write({.address = static_cast<cpu12::Address>(overlay_first + index),
                          .value = replacement}),
            "Initialize different code after power-on");
  }
  require(bind(memory, code, 3) && grant(memory, overlay_first).identity == 3,
          "Power-on admits only the separately declared matching source");
}

void sparse_program_bindings() {
  blofeld::vi::FlashImage flash;
  blofeld::vi::mcu::Memory memory(flash);
  constexpr std::array<cpu12::Address, 3> bases{0x1201, 0x1600, 0x0401};
  constexpr auto bytes = cpu12::isa::nop;
  for (const auto base : bases) {
    const std::array code{
        cpu12::CodeSpan{.address = base, .words = bytes},
        cpu12::CodeSpan{.address = static_cast<cpu12::Address>(base + 2), .words = bytes}};
    require(
        memory.write({.address = base, .value = bytes[0]}) &&
            memory.write({.address = static_cast<cpu12::Address>(base + 2), .value = bytes[0]}) &&
            bind(memory, code, 1),
        "Explicit initialized code binds at helper, UI and queue addresses");
    require(!grant(memory, base).writable && !grant(memory, base + 2).writable &&
                grant(memory, base).bytes.size() == 1 && grant(memory, base + 1).writable &&
                grant(memory, base - 1).writable && grant(memory, base + 3).writable,
            "Only instruction bytes are protected; interior data and both adjacent bytes stay "
            "writable");
    require(memory.write({.address = static_cast<cpu12::Address>(base + 1), .value = bytes[0]}) &&
                memory.write({.address = base, .value = bytes[0]}) && !memory.program_changed(),
            "A real data-hole store and same-value code store preserve execution ownership");
    require(
        memory.write({.address = base, .value = 0}) && memory.program_changed() &&
            !grant(memory, base).writable && memory.commit_mapping() &&
            grant(memory, base).writable && !bind(memory, code, 1),
        "Changed code stays protected through the instruction boundary and old source is refused");
  }
}

struct Writer {
  blofeld::vi::mcu::Memory &memory;
  cpu12::Core *core = nullptr;

  static bool write(void *owner, cpu12::ByteWrite access) noexcept {
    auto &self = *static_cast<Writer *>(owner);
    const bool accepted = self.memory.write(access);
    if (self.memory.program_changed()) { self.core->request_yield(); }
    return accepted;
  }

  static bool word(void *owner, cpu12::WordWrite access) noexcept {
    return write(owner,
                 {.address = access.address,
                  .value = static_cast<cpu12::Byte>(access.value >> 8U)}) &&
           write(owner,
                 {.address = static_cast<cpu12::Address>(access.address + 1U),
                  .value = static_cast<cpu12::Byte>(access.value)});
  }
};

void run_program_transition(const std::span<const cpu12::Byte> original,
                            const std::span<const cpu12::Byte> replacement,
                            const cpu12::NativeImage &native,
                            const bool word) {
  for (unsigned limit = 0; limit <= 4; ++limit) {
    blofeld::vi::FlashImage flash;
    blofeld::vi::mcu::Memory memory(flash);
    for (std::size_t i = 0; i < original.size(); ++i) {
      require(memory.write({.address = static_cast<cpu12::Address>(overlay_first + i),
                            .value = original[i]}),
              "Actual original source stores");
    }
    const std::array first{cpu12::CodeSpan{.address = overlay_first, .words = original}};
    const std::array second{cpu12::CodeSpan{.address = overlay_first, .words = replacement}};
    require(bind(memory, first, 1), "Actual original instructions bind");
    auto grants = memory.regions();
    Writer writer{.memory = memory};
    cpu12::Io io{.owner = &writer, .write = Writer::write, .write_word = Writer::word};
    cpu12::Core core(cpu12::Memory::mapped(grants), &io);
    writer.core = &core;
    constexpr cpu12::Address stack = 0x1fff;
    core.set_registers({.pc = overlay_first, .sp = stack});
    core.install(native, 1);
    const auto prefix = core.run(limit);
    const auto completed = std::min(limit, 2U);
    require(prefix.fault == cpu12::Fault::none && prefix.retired == completed,
            "Exact requested native prefix before code-changing store");
    if (completed < 2) {
      require(core.run().retired == 2 - completed,
              "Only remaining prefix completes before code invalidation");
    }
    require(memory.program_changed() && core.registers().pc == overlay_first + (word ? 6 : 5),
            "Real full byte/word store yields before changed operand executes");
    const auto state = core.state();
    const auto progress = core.progress();
    require(memory.commit_mapping() && !bind(memory, first, 1) && bind(memory, second, 2),
            "Only prepared replacement matches after original complete store");
    auto next = memory.regions();
    core.set_memory(cpu12::Memory::mapped(next));
    grants.swap(next);
    bool refused = false;
    try {
      core.install(native, 1);
    } catch (const std::invalid_argument &) { refused = true; }
    require(refused && core.state() == state && core.progress() == progress,
            "Mismatched native source cannot change architecture/time");
    core.install(native, 2);
    require(core.state() == state && core.progress() == progress,
            "New stopped source admission never replays a guest instruction");
    const auto suffix = core.run();
    constexpr cpu12::Address word_expected = 0x5a96;
    constexpr cpu12::Address byte_expected = 0x5a5a;
    require(suffix.fault == cpu12::Fault::none && suffix.retired == 2 &&
                suffix.state == cpu12::ExecutionState::waiting &&
                core.registers().d == (word ? word_expected : byte_expected) &&
                core.statistics().compiled_entries == 0,
            "Prepared replacement executes original changed operand once without new translation");
    for (std::size_t i = 0; i < replacement.size(); ++i) {
      std::uint8_t actual = 0;
      require(memory.read(static_cast<cpu12::Address>(overlay_first + i), actual) &&
                  actual == replacement[i],
              "Every actual source byte equals original store result");
    }
  }
}

void native_program_transition(const bool word) {
  constexpr auto byte_code = isa::program(isa::opcode<isa::Opcode::ldaa_immediate>(),
                                          std::array<cpu12::Byte, 1>{0x5a},
                                          isa::opcode<isa::Opcode::staa_extended>(),
                                          std::array<cpu12::Byte, 2>{0x16, 0x06},
                                          isa::opcode<isa::Opcode::ldab_immediate>(),
                                          std::array<cpu12::Byte, 1>{0x12},
                                          isa::wai,
                                          std::array<cpu12::Byte, 2>{0x20, 0xfe});
  constexpr auto word_code = isa::program(isa::opcode<isa::Opcode::ldd_immediate>(),
                                          std::array<cpu12::Byte, 2>{0x5a, 0x96},
                                          isa::opcode<isa::Opcode::std_extended>(),
                                          std::array<cpu12::Byte, 2>{0x16, 0x07},
                                          isa::opcode<isa::Opcode::ldd_immediate>(),
                                          std::array<cpu12::Byte, 2>{0x12, 0x34},
                                          isa::wai,
                                          std::array<cpu12::Byte, 2>{0x20, 0xfe});
  const std::vector<cpu12::Byte> original =
      word ? std::vector<cpu12::Byte>(word_code.begin(), word_code.end())
           : std::vector<cpu12::Byte>(byte_code.begin(), byte_code.end());
  auto replacement = original;
  constexpr cpu12::Byte replacement_high = 0x5a;
  constexpr cpu12::Byte replacement_low = 0x96;
  replacement[word ? 7 : 6] = replacement_high;
  if (word) { replacement[8] = replacement_low; }
  const std::array first{cpu12::CodeSpan{.address = overlay_first, .words = original}};
  const std::array second{cpu12::CodeSpan{.address = overlay_first, .words = replacement}};
  const std::array<cpu12::Address, 5> entries{
      overlay_first,
      static_cast<cpu12::Address>(overlay_first + (word ? 3 : 2)),
      static_cast<cpu12::Address>(overlay_first + (word ? 6 : 5)),
      static_cast<cpu12::Address>(overlay_first + (word ? 9 : 7)),
      static_cast<cpu12::Address>(overlay_first + (word ? 10 : 8))};
  const std::array variants{cpu12::CodeVariant{.id = 1, .code = first, .entries = entries},
                            cpu12::CodeVariant{.id = 2, .code = second, .entries = entries}};
  const auto native = cpu12::Core::compile({.variants = variants},
                                           {.memory = {},
                                            .interrupts = {},
                                            .transitions = {},
                                            .callbacks = {},
                                            .bindings = {},
                                            .topology = cpu12::MemoryTopology::dynamic});
  run_program_transition(original, replacement, native, word);
}

void interrupt_program_transition() {
  constexpr auto code = isa::program(isa::nop,
                                     isa::nop,
                                     isa::nop,
                                     isa::nop,
                                     isa::nop,
                                     isa::nop,
                                     isa::nop,
                                     isa::nop,
                                     isa::nop,
                                     std::array<cpu12::Byte, 2>{0x20, 0xfe});
  const std::array spans{cpu12::CodeSpan{.address = overlay_first, .words = code}};
  std::array<cpu12::Address, 10> entries{};
  for (unsigned i = 0; i < entries.size(); ++i) {
    entries[i] = static_cast<cpu12::Address>(overlay_first + i);
  }
  const std::array variants{cpu12::CodeVariant{.id = 1, .code = spans, .entries = entries}};
  constexpr cpu12::Address vector = 0xfff0;
  const std::array events{cpu12::InterruptRequest{.vector = vector}};
  const auto native = cpu12::Core::compile({.variants = variants},
                                           {.memory = {},
                                            .interrupts = events,
                                            .transitions = {},
                                            .callbacks = {},
                                            .bindings = {},
                                            .topology = cpu12::MemoryTopology::dynamic});
  blofeld::vi::FlashImage flash;
  flash.known.fill(1);
  constexpr std::size_t vector_offset = 0x7ff0;
  flash.bytes[vector_offset] = static_cast<cpu12::Byte>(overlay_first >> 8U);
  flash.bytes[vector_offset + 1] = 0;
  blofeld::vi::mcu::Memory memory(flash);
  for (std::size_t i = 0; i < code.size(); ++i) {
    require(
        memory.write({.address = static_cast<cpu12::Address>(overlay_first + i), .value = code[i]}),
        "Real pre-interrupt source writes");
  }
  require(bind(memory, spans, 1), "Original code admits before IRQ stack stores");
  auto grants = memory.regions();
  Writer writer{.memory = memory};
  cpu12::Io io{.owner = &writer, .write = Writer::write, .write_word = Writer::word};
  cpu12::Core core(cpu12::Memory::mapped(grants), &io);
  writer.core = &core;
  const cpu12::Registers seed{.pc = overlay_first,
                              .d = 0x1234,
                              .x = 0x5678,
                              .y = 0x9abc,
                              .sp = overlay_first + 9,
                              .ccr = 0};
  core.set_registers(seed);
  core.install(native, 1);
  const auto entered = core.interrupt(events[0]);
  require(entered.accepted && entered.fault == cpu12::Fault::none &&
              core.registers().pc == overlay_first && core.registers().sp == overlay_first &&
              memory.program_changed(),
          "Original complete IRQ stack entry invalidates overlapping instructions before target "
          "executes");
  const auto frame = cpu12::test::interrupts::encode(seed);
  for (std::size_t i = 0; i < frame.size(); ++i) {
    std::uint8_t actual = 0;
    require(memory.read(static_cast<cpu12::Address>(overlay_first + i), actual) &&
                actual == frame[i],
            "Every original IRQ frame byte and knownness reaches actual RAM");
  }
  const auto state = core.state();
  const auto progress = core.progress();
  require(memory.commit_mapping() && !bind(memory, spans, 1),
          "IRQ-modified code cannot reuse original prepared bytes");
  auto next = memory.regions();
  core.set_memory(cpu12::Memory::mapped(next));
  grants.swap(next);
  bool refused = false;
  try {
    core.install(native, 1);
  } catch (const std::invalid_argument &) { refused = true; }
  require(refused && core.state() == state && core.progress() == progress &&
              core.statistics().compiled_entries == 0,
          "Failed post-IRQ source admission neither replays frame nor compiles stale instructions");
}

}

int main() try {
  mapping();
  mapping_regions();
  native_memory();
  unknown_memory();
  program_bindings();
  sparse_program_bindings();
  native_program_transition(false);
  native_program_transition(true);
  interrupt_program_transition();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
