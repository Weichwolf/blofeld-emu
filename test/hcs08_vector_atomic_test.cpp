// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_vector_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_known_interrupts.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_system_options.hpp"
#include "keyboard/hcs08_vector_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::vectors;
using blofeld::test::hcs08::require;
constexpr unsigned setup_instructions = 8, final_phase = 16;
constexpr unsigned ram_first = 0x80, ram_end = 0x480;

// Untimed CPU test bus:real FLASH/SPI/SOPT components,owned raw SRAM masks.
// No ICG/COP clock model is claimed;unexpected accesses fail rather than stub.
class Memory final : public Bus,
                     public Hcs08KnownMemory,
                     public Hcs08KnownInterrupts,
                     public Hcs08VectorMemory {
public:
  explicit Memory(const Hcs08FlashImage &image) : flash(image, Hcs08FlashContract::candidate_gt8a) {
    for (auto &byte : ram) { byte.known = 0; }
  }

  using Bus::interrupt_request;

  std::uint8_t read(const std::uint16_t address) override {
    if (resetting) { return flash.read_reset_vector_byte(address); }
    if (address >= hcs08_flash::first) { return flash.read_array(address); }
    if (address >= ram_first && address < ram_end) {
      const auto byte = ram.at(address - ram_first);
      if (byte.known != Hcs08KnownByte::all) { fail("read", address, "unknown SRAM"); }
      return byte.value;
    }
    if (Hcs08Spi::contains(address)) { return spi.read(address); }
    if (address == Hcs08SystemOptions::address) { return options.read(); }
    fail("read", address, "unimplemented fixture address");
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (address >= ram_first && address < ram_end) {
      ram.at(address - ram_first) = {.value = value};
    } else if (Hcs08Spi::contains(address)) {
      spi.write({.address = address, .value = value});
    } else if (address == Hcs08SystemOptions::address) {
      options.write({.address = address, .value = value}, context);
    } else {
      fail("write", address, "unimplemented fixture address");
    }
    ++generation;
  }

  Hcs08KnownByte read_known_byte(const std::uint16_t address) override {
    if (address >= ram_first && address < ram_end) { return ram.at(address - ram_first); }
    return {.value = read(address)};
  }

  void write_known_byte(const ByteWrite access, const std::uint8_t known) override {
    if (access.address < ram_first || access.address >= ram_end) {
      fail("masked write", access.address, "requires SRAM");
    }
    ram.at(access.address - ram_first) = {.value = static_cast<std::uint8_t>(access.value & known),
                                          .known = known};
    ++generation;
  }

  std::uint8_t peek(const std::uint16_t address) const override {
    if (address >= hcs08_flash::first) { return flash.read_array(address); }
    fail("peek", address, "fixture code requires FLASH");
  }

  std::uint64_t revision() const override { return generation; }

  void check_execute(const InstructionFetch fetch) const override {
    if (fetch.length == 0 || fetch.address < hcs08_flash::first) {
      fail("fetch", fetch.address, "requires own FLASH code");
    }
    for (unsigned i = 0; i < fetch.length; ++i) {
      static_cast<void>(flash.read_array(static_cast<std::uint16_t>(fetch.address + i)));
    }
  }

  void begin_instruction(const ExecutionContext value) override {
    context = value;
    flash.context(value);
    spi.context(value);
  }

  void begin_reset(const ExecutionContext value) override {
    begin_instruction(value);
    resetting = true;
  }

  void end_reset() override { resetting = false; }

  std::optional<InterruptRequest> interrupt_request(const Hcs08State & /*state*/) const override {
    if (spi.interrupt_requested()) {
      return InterruptRequest{.vector = vector, .kind = InterruptKind::maskable};
    }
    return {};
  }

  void begin_interrupt(const InterruptRequest request, const bool reuse) override {
    require(request.vector == vector && !reuse && spi.interrupt_requested(),
            "Only actual SPI requests enter a native IRQ frame");
  }

  void acknowledge_interrupt(const std::uint16_t selected) override {
    require(selected == vector && spi.interrupt_requested(),
            "Logical SPI acknowledgement must not use relocated memory addresses");
  }

  std::uint8_t read_interrupt_vector_byte(const std::uint16_t address) override {
    vector_reads.push_back(address);
    return flash.read_interrupt_vector_byte(address);
  }

  [[noreturn]] void
  fail(const std::string &operation, const unsigned address, const std::string &reason) const {
    throw Fault("Atomic IRQ fixture " + operation + " address=" + hex(address) +
                " pc=" + hex(context.pc) + ": " + reason);
  }

  Hcs08Flash flash;
  Hcs08Spi spi{Hcs08SpiContract::candidate_gt8a_reference_master_edges};
  Hcs08SystemOptions options{Hcs08FlashContract::candidate_gt8a};
  std::array<Hcs08KnownByte, ram_end - ram_first> ram{};
  std::vector<std::uint16_t> vector_reads;
  ExecutionContext context;
  std::uint64_t generation = 0;
  bool resetting = false;
};

struct Saved {
  Memory memory;
  Hcs08Checkpoint cpu;
  unsigned phase;
};

struct Unit {
  Memory memory;
  Hcs08 cpu;

  explicit Unit(const Hcs08FlashImage &image)
      : memory(image), cpu(memory, BLOFELD_HCS08_SLA, Hcs08RtiIrqContract::reference_immediate) {
    cpu.reset(Hcs08ResetEntry::normal, Hcs08ResetCause::power_on);
  }

  explicit Unit(const Saved &saved)
      : memory(saved.memory),
        cpu(memory, BLOFELD_HCS08_SLA, Hcs08RtiIrqContract::reference_immediate) {
    cpu.restore_checkpoint(saved.cpu);
  }
};

void advance(Unit &unit, const unsigned phase, const Configuration config) {
  if (phase == setup_instructions) {
    require(unit.cpu.state() == caller(),
            "Literal own setup establishes the genuine caller before entry");
    require(unit.cpu.service_interrupt() && unit.cpu.state().pc == config.handler,
            "One-shot native stack entry uses the same actual memory-owned vector routing");
  } else {
    unit.cpu.step();
  }
}

void endpoint_checks(const Unit &unit, const Configuration config, const Hcs08FlashImage &image) {
  require(
      unit.cpu.state() == caller() && unit.memory.flash.image() == image &&
          unit.memory.ram.at(marker - ram_first).value == config.marker_value &&
          unit.memory.ram.at(marker - ram_first).known == Hcs08KnownByte::all &&
          unit.memory.ram.at(ordinary_read - ram_first).value == original_vector_high &&
          unit.memory.spi.state().control1 == quiet_spi &&
          unit.memory.vector_reads == std::vector<std::uint16_t>{vector, vector + 1},
      "One-shot ISR/RTI keeps independent caller,real logical vector bytes,ordinary read and NVM");
}

void run(const Configuration config) {
  const auto image = own_image(config);
  std::optional<Unit> original(std::in_place, image);
  std::vector<Saved> saved;
  for (unsigned phase = 0; phase < final_phase; ++phase) {
    saved.push_back({.memory = original.value().memory,
                     .cpu = original.value().cpu.checkpoint(),
                     .phase = phase});
    advance(original.value(), phase, config);
  }
  endpoint_checks(original.value(), config, image);
  original.reset();
  for (const auto &snapshot : saved) {
    for (unsigned copy = 0; copy < 2; ++copy) {
      Unit fresh(snapshot);
      for (unsigned phase = snapshot.phase; phase < final_phase; ++phase) {
        advance(fresh, phase, config);
      }
      endpoint_checks(fresh, config, image);
    }
  }
}

void swi_controls() {
  constexpr unsigned known_pc = 0xe007;
  constexpr unsigned partial_pc = 0xe004;
  constexpr std::array<std::uint8_t, 10> known{
      0x45, 4, 0x80, 0x94, 0xa6, 0x68, 0x84, 0x83, 0x9d, 0x9d};
  constexpr std::array<std::uint8_t, 7> partial{0x45, 4, 0x80, 0x94, 0x83, 0x9d, 0x9d};
  for (const bool full : {true, false}) {
    auto image = own_image(cases.front());
    const std::span<const std::uint8_t> code =
        full ? std::span<const std::uint8_t>(known) : std::span<const std::uint8_t>(partial);
    for (unsigned i = 0; i < code.size(); ++i) {
      const auto offset = fixture::entry - hcs08_flash::first + i;
      image.data.at(offset) = code[i];
      image.fresh.reset(offset);
    }
    Unit unit(image);
    const unsigned setup = full ? 4U : 2U;
    for (unsigned i = 0; i < setup; ++i) { unit.cpu.step(); }
    const auto before_cpu = unit.cpu.state();
    const auto before_memory = unit.memory;
    require(before_cpu.pc == (full ? known_pc : partial_pc) &&
                (before_cpu.ccr_known == Hcs08State::all_ccr) == full,
            "Actual LDHX/TXS and optional LDA/TAP establish the SWI caller knowledge");
    bool refused = false;
    std::string error;
    try {
      unit.cpu.step();
    } catch (const Fault &failure) {
      refused = true;
      error = failure.what();
    }
    require(refused,
            "SWI was silently accepted: actual PC=" + hex(unit.cpu.state().pc) +
                " SP=" + hex(unit.cpu.state().sp));
    require(error.contains("SWI") && error.contains("address=0xfffc") &&
                error.contains("pc=" + hex(before_cpu.pc)) && error.contains("agent=CPU") &&
                unit.cpu.state() == before_cpu &&
                unit.memory.generation == before_memory.generation &&
                unit.memory.flash.image() == image &&
                unit.memory.spi.state() == before_memory.spi.state(),
            "Unqualified SWI refuses before CPU/frame/device/NVM effects,with actual context");
    for (unsigned i = 0; i < unit.memory.ram.size(); ++i) {
      require(unit.memory.ram[i].value == before_memory.ram[i].value &&
                  unit.memory.ram[i].known == before_memory.ram[i].known,
              "Refused SWI preserves every SRAM byte and knowledge mask");
    }
    require(blofeld::test::hcs08::rejected([&] {
              static_cast<void>(unit.cpu.checkpoint());
            }).contains("healthy"),
            "Refused SWI cannot checkpoint a successful CPU boundary");
  }
}
} // namespace

int main() {
  try {
    swi_controls();
    for (const auto config : cases) { run(config); }
    std::cout << "Verified one-shot native vector routing,real ISR/RTI and192fresh CPU/memory "
                 "suffixes.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
