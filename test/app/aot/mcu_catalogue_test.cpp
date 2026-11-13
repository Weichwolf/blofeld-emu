// SPDX-License-Identifier: GPL-3.0-only
#include "aot/mcu.hpp"
#include "isa.hpp"
#include "mcu/memory.hpp"
#include "storage/firmware.hpp"
#include "storage/flash_image.hpp"

#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/JSON.h"
#include "llvm/Support/SHA256.h"
#include "llvm/Support/raw_ostream.h"

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
#include <string>
#include <utility>

namespace {
using Object = llvm::json::Object;
using Array = llvm::json::Array;
namespace isa = cpu12::isa;
constexpr cpu12::Address rom = 0x8000;
constexpr cpu12::Address ram = 0x1600;
constexpr cpu12::Address stack = 0x1fff;
constexpr cpu12::Byte datum = 0x5a;
constexpr auto resident = isa::program(isa::opcode<isa::Opcode::jsr_extended>(),
                                       std::array<cpu12::Byte, 2>{0x16, 0},
                                       isa::opcode<isa::Opcode::bra_relative>(),
                                       std::array<cpu12::Byte, 1>{0xfe});
constexpr auto overlay = isa::program(
    isa::opcode<isa::Opcode::ldaa_immediate>(), std::array{datum}, isa::opcode<isa::Opcode::rts>());

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::string hash(const std::span<const std::uint8_t> bytes) {
  return llvm::toHex(llvm::SHA256::hash({bytes.data(), bytes.size()}), true);
}

std::string document(Object root) {
  std::string text;
  llvm::raw_string_ostream output(text);
  output << llvm::json::Value(std::move(root));
  return text;
}

Object catalogue(const blofeld::vi::FirmwareUpdate &firmware) {
  Array variants;
  variants.push_back(Object{{.K = "id", .V = 1},
                            {.K = "source", .V = "mcu"},
                            {.K = "address", .V = rom},
                            {.K = "byte_offset", .V = 0},
                            {.K = "bytes", .V = firmware.mcu_flash.size()},
                            {.K = "instructions", .V = Array{Array{rom, 3}, Array{rom + 3, 2}}}});
  variants.push_back(Object{{.K = "id", .V = 2},
                            {.K = "source", .V = "ui"},
                            {.K = "address", .V = ram},
                            {.K = "byte_offset", .V = 0},
                            {.K = "bytes", .V = overlay.size()},
                            {.K = "instructions", .V = Array{Array{ram, 2}, Array{ram + 2, 1}}}});
  Object input{{.K = "mcu", .V = hash(firmware.mcu_flash)},
               {.K = "blobs",
                .V = Array{Object{{.K = "name", .V = "ui"}, {.K = "sha256", .V = hash(overlay)}}}}};
  return Object{{.K = "format", .V = 1},
                {.K = "inputs", .V = std::move(input)},
                {.K = "cross_domain_entries", .V = true},
                {.K = "variants", .V = std::move(variants)}};
}

void qualify() {
  blofeld::vi::FirmwareUpdate firmware;
  std::ranges::copy(resident, firmware.mcu_flash.begin());
  const std::array sources{blofeld::app::aot::McuSource{.name = "ui", .bytes = overlay}};
  const auto prepared =
      blofeld::app::aot::prepare_mcu_catalogue(document(catalogue(firmware)), firmware, sources);
  require(prepared.report().prepared_entries == 4 && prepared.report().unresolved_entries == 0,
          "Both declared domains and every entry are prepared before execution");
  for (unsigned instance = 0; instance < 2; ++instance) {
    blofeld::vi::FlashImage flash;
    flash.bytes = firmware.mcu_flash;
    flash.known.fill(1);
    blofeld::vi::mcu::Memory memory(flash);
    auto mounted = prepared.admit(memory, rom);
    require(mounted.size() == 1 && mounted[0] == 1,
            "Absent guest RAM code is never manufactured by preparation");
    auto grants = memory.regions();
    cpu12::Core core(cpu12::Memory::mapped(grants));
    core.set_registers({.pc = rom, .sp = stack});
    prepared.install(core, mounted);
    const auto transfer = core.run();
    require(transfer.fault == cpu12::Fault::none && transfer.retired == 1 &&
                core.registers().pc == ram,
            "A real JSR completes once before admitting the unloaded destination");
    bool refused = false;
    try {
      static_cast<void>(prepared.admit(memory, ram));
    } catch (const std::runtime_error &) { refused = true; }
    require(refused, "Declared but absent executable bytes cannot run");
    for (std::size_t index = 0; index < overlay.size(); ++index) {
      require(memory.write(
                  {.address = static_cast<cpu12::Address>(ram + index), .value = overlay[index]}),
              "Only actual guest stores initialize executable RAM");
    }
    mounted = prepared.admit(memory, ram);
    auto next = memory.regions();
    core.set_memory(cpu12::Memory::mapped(next));
    grants.swap(next);
    prepared.install(core, mounted);
    const auto returned = core.run(2);
    require(
        mounted.size() == 2 && returned.fault == cpu12::Fault::none && returned.retired == 2 &&
            core.registers().pc == rom + 3 &&
            core.registers().d == static_cast<cpu12::Address>(datum << 8U) &&
            core.statistics().compiled_entries == 0,
        "Two independent owners retain original call/return, data and zero runtime translation");
    require(memory.write({.address = ram + 1, .value = datum + 1}) && memory.program_changed() &&
                memory.commit_mapping(),
            "A genuine operand write invalidates the immutable source");
    refused = false;
    try {
      static_cast<void>(prepared.admit(memory, ram));
    } catch (const std::runtime_error &) { refused = true; }
    require(refused, "Undeclared changed source fails without translating new code");
  }
  const auto reject = [&](Object root) {
    bool refused = false;
    try {
      static_cast<void>(
          blofeld::app::aot::prepare_mcu_catalogue(document(std::move(root)), firmware, sources));
    } catch (const std::invalid_argument &) { refused = true; }
    require(refused, "Malformed catalogue or source identity is refused before native execution");
  };
  auto root = catalogue(firmware);
  root["format"] = 2;
  reject(std::move(root));
  root = catalogue(firmware);
  root["cross_domain_entries"] = false;
  reject(std::move(root));
  root = catalogue(firmware);
  (*root.getObject("inputs"))["mcu"] = "wrong";
  reject(std::move(root));
  root = catalogue(firmware);
  root["variants"] = Array{};
  reject(std::move(root));
  root = catalogue(firmware);
  (*(*root.getArray("variants"))[0].getAsObject())["address"] = 65536;
  reject(std::move(root));
  root = catalogue(firmware);
  (*(*root.getArray("variants"))[0].getAsObject())["instructions"] = Array{Array{rom, 7}};
  reject(std::move(root));
  root = catalogue(firmware);
  (*(*root.getArray("variants"))[1].getAsObject())["id"] = 1;
  reject(std::move(root));
}
}

int main() try { qualify(); } catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
