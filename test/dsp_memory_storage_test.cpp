// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp56kEmu/memory.h"
#include "dsp56kEmu/types.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {
using dsp56k::EMemArea;
using dsp56k::Memory;
using dsp56k::TWord;
constexpr TWord program_words = 0x1000000, data_words = 0x18000;
constexpr TWord high_address = 0xffffff, payload = 0xabcdef;
constexpr std::array areas{dsp56k::MemArea_P, dsp56k::MemArea_X, dsp56k::MemArea_Y};

void require(const bool value, const char *message) {
  if (!value) { throw blofeld::Fault(message); }
}

struct Access {
  enum class Kind : std::uint8_t { read, write, opcode, extension };
  Kind kind;
  EMemArea area;
  TWord address;
  TWord value = 0;
  bool operator==(const Access &) const = default;
};

class Bus final : public dsp56k::IMemoryValidator {
public:
  bool memValidateAccess(EMemArea /*area*/, TWord /*address*/, bool /*write*/) const override {
    throw blofeld::Fault("Bus-owned storage reached legacy validation");
  }

  bool readWord(const EMemArea area, const TWord address, TWord &value) const override {
    accesses.push_back({.kind = Access::Kind::read, .area = area, .address = address});
    value = payload;
    return accepts;
  }

  bool writeWord(const EMemArea area, const TWord address, const TWord value) const override {
    accesses.push_back(
        {.kind = Access::Kind::write, .area = area, .address = address, .value = value});
    return accepts;
  }

  bool readOpcode(const TWord address, TWord &first, TWord &second) const override {
    accesses.push_back(
        {.kind = Access::Kind::opcode, .area = dsp56k::MemArea_P, .address = address});
    first = payload;
    second = high_address;
    return accepts;
  }

  bool readExtension(const TWord address, TWord &value) const override {
    accesses.push_back(
        {.kind = Access::Kind::extension, .area = dsp56k::MemArea_P, .address = address});
    value = high_address;
    return accepts;
  }

  bool accepts = true;
  mutable std::vector<Access> accesses;
};

template <class F> void rejects(F &&action, const std::string_view operation) {
  try {
    action();
  } catch (const std::logic_error &error) {
    const std::string_view message(error.what());
    require(message.starts_with("Bus-owned DSP memory ") && message.contains(operation) &&
                message.contains("area=") && message.contains("address="),
            "Storage bypass reports the operation, area and address");
    return;
  }
  throw blofeld::Fault("Bus-owned memory allowed an array fallback");
}

void successful_hooks() {
  const Bus bus;
  Memory memory(bus, Memory::BusOwned{}, program_words, data_words);
  require(!memory.hasArrayStorage() && !memory.hasMmuSupport() && memory.sizeP() == program_words &&
              memory.sizeXY() == data_words && memory.getBridgedMemoryAddress() == 0,
          "Full logical bounds remain without backing arrays, MMU or SDK address bridging");
  std::vector<Access> expected;
  for (const auto area : areas) {
    for (const auto address : {0U, data_words - 1, data_words, high_address}) {
      require(memory.get(area, address) == payload && memory.set(area, address, high_address),
              "The owning bus handles each space and full 24-bit address unchanged");
      expected.push_back({.kind = Access::Kind::read, .area = area, .address = address});
      expected.push_back(
          {.kind = Access::Kind::write, .area = area, .address = address, .value = high_address});
    }
  }
  TWord first = 0;
  TWord second = 0;
  memory.getOpcode(high_address, first, second);
  require(first == payload && second == high_address &&
              memory.getExtension(high_address) == high_address,
          "Opcode lookahead and extension values come only from their respective hooks");
  expected.push_back(
      {.kind = Access::Kind::opcode, .area = dsp56k::MemArea_P, .address = high_address});
  expected.push_back(
      {.kind = Access::Kind::extension, .area = dsp56k::MemArea_P, .address = high_address});
  require(bus.accesses == expected, "Each operation invokes its owning hook exactly once");
}

void rejected_fallbacks() {
  Bus bus;
  bus.accepts = false;
  Memory memory(bus, Memory::BusOwned{}, program_words, data_words);
  for (const auto area : areas) {
    for (const auto address : {0U, high_address}) {
      rejects([&] { static_cast<void>(memory.get(area, address)); }, "read");
      rejects([&] { static_cast<void>(memory.set(area, address, payload)); }, "write");
    }
    rejects([&] { static_cast<void>(memory.getMemAreaPtr(area)); }, "direct pointer");
  }
  TWord first = 0;
  TWord second = 0;
  rejects([&] { memory.getOpcode(high_address, first, second); }, "opcode");
  rejects([&] { static_cast<void>(memory.getExtension(high_address)); }, "extension");
  constexpr std::size_t expected_calls = 14;
  require(bus.accesses.size() == expected_calls &&
              bus.accesses.back().kind == Access::Kind::extension,
          "Declining extension hooks cannot fall through to another hook or legacy validation");
  rejects([&] { memory.setExternalMemory(0, false); }, "external-memory remap");
  rejects([&] { memory.setExternalMemory(high_address, true); }, "external-memory remap");
  rejects([&] { static_cast<void>(memory.saveAssembly("/", 0, 1)); }, "assembly export");
  require(!memory.hasArrayStorage() && !memory.hasMmuSupport() &&
              memory.getBridgedMemoryAddress() == 0 && bus.accesses.size() == expected_calls,
          "Rejected pointer, remap and export operations cannot create storage or bus traffic");
}

void legacy_arrays() {
  const dsp56k::DefaultMemoryValidator validator;
  constexpr TWord words = 8;
  Memory owned(validator, words);
  constexpr std::size_t external_words = static_cast<std::size_t>(words) * dsp56k::MemArea_COUNT;
  std::array<TWord, external_words> external{};
  std::array<TWord, external_words> separate_external{};
  Memory borrowed(validator, words, external.data());
  Memory separate_owned(validator, words, words, words);
  Memory separate_borrowed(validator, words, words, words, separate_external.data());
  for (auto *memory : {&owned, &borrowed, &separate_owned, &separate_borrowed}) {
    require(memory->hasArrayStorage(), "Legacy constructors retain their array storage");
    TWord value = 1;
    for (const auto area : areas) {
      require(memory->set(area, words - 1, value) && memory->get(area, words - 1) == value &&
                  memory->getMemAreaPtr(area)[words - 1] == value,
              "Legacy owned and borrowed arrays retain independent X/Y/P words and pointers");
      ++value;
    }
    require(memory->set(dsp56k::MemArea_P, 0, payload) && memory->getExtension(0) == payload,
            "Legacy extension reads retain the ordinary program-array fallback");
    TWord first = 0;
    TWord second = 0;
    memory->getOpcode(0, first, second);
    require(first == payload && second == 0, "Legacy opcode reads retain their array lookahead");
  }
  Memory bridged(validator, words, words, 0);
  require(bridged.set(dsp56k::MemArea_X, 0, payload) &&
              bridged.get(dsp56k::MemArea_P, 0) == payload &&
              bridged.get(dsp56k::MemArea_Y, 0) == payload &&
              bridged.getMemAreaPtr(dsp56k::MemArea_P)[0] == payload &&
              bridged.getMemAreaPtr(dsp56k::MemArea_X)[0] == 0 &&
              bridged.getMemAreaPtr(dsp56k::MemArea_Y)[0] == 0,
          "The legacy zero threshold still translates all SDK accesses to P without MMU aliases");
}
}

int main() {
  try {
    successful_hooks();
    rejected_fallbacks();
    legacy_arrays();
    std::cout
        << "Bus-owned DSP storage rejects fallback and pointers; legacy arrays remain usable.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
