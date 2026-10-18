// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_core.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "dsp56kEmu/disasm.h"
#include "dsp56kEmu/dsp.h"
#include "dsp56kEmu/memory.h"
#include "dsp56kEmu/opcodetypes.h"
#include "dsp56kEmu/peripherals.h"
#include "dsp56kEmu/registers.h"
#include "dsp56kEmu/types.h"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>

namespace blofeld {
namespace {
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned address_space = 0x1000000;
constexpr unsigned native_xy_words = 0x18000;
constexpr unsigned spi_mode = 5;
constexpr unsigned boot_entry = 0xff0000;
constexpr std::uint64_t data_mask = 0xffffffffffff;
constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;

DspSpace space(const dsp56k::EMemArea area) {
  switch (area) {
    case dsp56k::MemArea_P: return DspSpace::p;
    case dsp56k::MemArea_X: return DspSpace::x;
    case dsp56k::MemArea_Y: return DspSpace::y;
    case dsp56k::MemArea_COUNT: throw Fault("Invalid native DSP memory-space count sentinel");
    default: throw Fault("Unsupported native DSP memory space");
  }
}

DspBitOperation bit_operation(const dsp56k::PeripheralBitOp operation) {
  switch (operation) {
    case dsp56k::PeripheralBitOp::Test: return DspBitOperation::test;
    case dsp56k::PeripheralBitOp::Clear: return DspBitOperation::clear;
    case dsp56k::PeripheralBitOp::Set: return DspBitOperation::set;
    case dsp56k::PeripheralBitOp::Change: return DspBitOperation::change;
    default: throw Fault("Invalid native DSP peripheral bit operation");
  }
}

class MemoryAccess final : public dsp56k::IMemoryValidator {
public:
  explicit MemoryAccess(DspBus &bus) : bus_(bus) {}

  [[nodiscard]] bool memValidateAccess(dsp56k::EMemArea /*area*/,
                                       dsp56k::TWord /*address*/,
                                       bool /*write*/) const override {
    throw Fault("Native DSP array access bypassed the strict board bus");
  }

  bool readWord(const dsp56k::EMemArea area,
                const dsp56k::TWord address,
                dsp56k::TWord &value) const override {
    value = bus_.read({.space = space(area), .address = address});
    return true;
  }

  [[nodiscard]] bool writeWord(const dsp56k::EMemArea area,
                               const dsp56k::TWord address,
                               const dsp56k::TWord value) const override {
    bus_.write({.space = space(area), .address = address, .value = value});
    return true;
  }

  bool readOpcode(const dsp56k::TWord address,
                  dsp56k::TWord &first,
                  dsp56k::TWord &second) const override {
    first = bus_.read({.address = address});
    second = bus_.peek_program(address + 1);
    return true;
  }

private:
  DspBus &bus_;
};

class Peripherals final : public dsp56k::IPeripherals {
public:
  Peripherals(DspBus &bus, const DspSpace area)
      : IPeripherals(dsp56k::PeripheralType::PeripheralsCustom), bus_(bus), area_(area) {}

  dsp56k::TWord read(const dsp56k::TWord address, dsp56k::Instruction /*instruction*/) override {
    return bus_.read({.space = area_, .address = address});
  }

  bool bitAccess(const dsp56k::TWord address,
                 const dsp56k::TWord bit,
                 const dsp56k::PeripheralBitOp operation,
                 bool &old) override {
    const auto result = bus_.access_peripheral_bit(
        {.space = area_, .address = address, .bit = bit, .operation = bit_operation(operation)});
    if (!result.has_value()) { return false; }
    old = *result;
    return true;
  }

  const dsp56k::TWord *readAsPtr(dsp56k::TWord /*address*/,
                                 dsp56k::Instruction /*instruction*/) override {
    throw Fault("Native DSP direct peripheral pointer bypassed the board bus");
  }

  void write(const dsp56k::TWord address, const dsp56k::TWord value) override {
    bus_.write({.space = area_, .address = address, .value = value});
  }

  std::uint32_t exec() override { return MaxDelayCycles; }

  void reset() override {
    if (area_ == DspSpace::x) { bus_.reset_peripherals(); }
  }

  void setSymbols(dsp56k::Disassembler & /*disassembler*/) const override {}

  void terminate() override {}

private:
  DspBus &bus_;
  DspSpace area_;
};

std::uint32_t word(const dsp56k::TReg24 &value) {
  return static_cast<std::uint32_t>(value.var) & word_mask;
}

std::uint64_t register_value(const dsp56k::DSP &cpu, const dsp56k::EReg reg) {
  std::int64_t value = 0;
  if (!cpu.readRegToInt(reg, value)) { throw Fault("Cannot read native DSP register"); }
  return std::bit_cast<std::uint64_t>(value);
}
}

class DspCore::Impl {
public:
  explicit Impl(DspBus &board)
      : bus(board),
        access(board),
        memory(access, address_space, native_xy_words, 0),
        peripherals_x(board, DspSpace::x),
        peripherals_y(board, DspSpace::y),
        cpu(memory, &peripherals_x, &peripherals_y) {
    static_assert(!dsp56k::g_useJIT);
  }

  void invalidate() {
    const auto changes = bus.take_code_changes();
    if (changes.all) {
      cpu.clearOpcodeCache();
    } else {
      for (const auto address : changes.addresses) { cpu.clearOpcodeCache(address); }
    }
  }

  DspBus &bus;
  MemoryAccess access;
  dsp56k::Memory memory;
  Peripherals peripherals_x;
  Peripherals peripherals_y;
  dsp56k::DSP cpu;
  bool faulted = false;
};

DspCore::DspCore(DspBus &bus) : impl_(std::make_unique<Impl>(bus)) {
  reset_spi_boot();
}

DspCore::~DspCore() = default;

void DspCore::reset_spi_boot() {
  impl_->bus.reset();
  impl_->cpu.resetHW();
  impl_->cpu.regs().omr.var = spi_mode;
  impl_->cpu.setPC(boot_entry);
  impl_->faulted = false;
  impl_->invalidate();
}

DspState DspCore::state() const {
  const auto &cpu = impl_->cpu;
  const auto &regs = cpu.regs();
  DspState result{.pc = word(regs.pc),
                  .sr = static_cast<std::uint32_t>(register_value(cpu, dsp56k::Reg_SR)) & word_mask,
                  .omr = word(regs.omr),
                  .sp = word(regs.sp),
                  .la = word(regs.la),
                  .lc = word(regs.lc),
                  .x = register_value(cpu, dsp56k::Reg_X) & data_mask,
                  .y = register_value(cpu, dsp56k::Reg_Y) & data_mask,
                  .a = register_value(cpu, dsp56k::Reg_A) & accumulator_mask,
                  .b = register_value(cpu, dsp56k::Reg_B) & accumulator_mask};
  for (std::size_t index = 0; index < result.r.size(); ++index) {
    result.r[index] = word(regs.r[index]);
    result.n[index] = word(regs.n[index]);
    result.m[index] = word(regs.m[index]);
  }
  return result;
}

void DspCore::step() {
  if (impl_->faulted) { throw Fault("DSP requires reset after a rejected reference step"); }
  const auto before = state();
  try {
    impl_->invalidate();
    impl_->cpu.execReference();
    impl_->bus.observe_omr(word(impl_->cpu.regs().omr));
  } catch (const std::exception &error) {
    impl_->faulted = true;
    throw Fault("DSP reference step pc=" + hex(before.pc, 6) + " omr=" + hex(before.omr, 6) + ": " +
                error.what());
  }
}

}
