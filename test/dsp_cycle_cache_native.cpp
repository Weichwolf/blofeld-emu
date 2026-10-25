// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp56kEmu/assembler.h"
#include "dsp56kEmu/dsp.h"
#include "dsp56kEmu/jit.h"
#include "dsp56kEmu/memory.h"
#include "dsp56kEmu/opcodetypes.h"
#include "dsp56kEmu/peripherals.h"
#include "dsp56kEmu/registers.h"
#include "dsp56kEmu/types.h"
#include "dsp_cycle_cache_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <map>

namespace blofeld::test {
namespace {
constexpr unsigned words = 0x1000000, xy_words = 0x18000, site = 0xff0000;
constexpr unsigned cold_address = 0x1000;
constexpr unsigned core_priority = 0xffffff, peripheral_priority = 0xfffffe;

// SDK metadata only: reset/priority writes use the real board bus; data/execution reject.
class Peripheral final : public dsp56k::IPeripherals {
public:
  Peripheral(DspBus &bus, const DspSpace space)
      : IPeripherals(dsp56k::PeripheralType::PeripheralsCustom), bus_(bus), space_(space) {}

  std::uint32_t exec() override { throw Fault("Unexpected cycle-test peripheral execution"); }

  dsp56k::TWord read(dsp56k::TWord /*address*/, dsp56k::Instruction /*instruction*/) override {
    throw Fault("Unexpected cycle-test peripheral read");
  }

  const dsp56k::TWord *readAsPtr(dsp56k::TWord /*address*/,
                                 dsp56k::Instruction /*instruction*/) override {
    throw Fault("Unexpected cycle-test peripheral pointer");
  }

  void write(const dsp56k::TWord address, const dsp56k::TWord value) override {
    bus_.write({.space = space_, .address = address, .value = value});
    ++writes_;
  }

  void reset() override {
    if (space_ == DspSpace::x) { bus_.reset_peripherals(); }
    ++resets_;
  }

  void setSymbols(dsp56k::Disassembler & /*disassembler*/) const override {}

  void terminate() override { ++terminations_; }

  [[nodiscard]] unsigned resets() const { return resets_; }

  [[nodiscard]] unsigned terminations() const { return terminations_; }

  [[nodiscard]] unsigned writes() const { return writes_; }

private:
  DspBus &bus_;
  DspSpace space_;
  unsigned resets_ = 0, terminations_ = 0, writes_ = 0;
};

class Code final : public dsp56k::IMemoryValidator {
public:
  void set(const unsigned address, const char *source) {
    const dsp56k::Assembler assembler;
    const auto encoded = assembler.assemble(source);
    require(encoded.success() && encoded.wordCount > 0 && encoded.wordCount <= 2,
            "Own cycle-cache opcode must assemble");
    program_[address] = {encoded.word[0], encoded.wordCount == 2 ? encoded.word[1] : 0U};
  }

  bool readOpcode(const dsp56k::TWord address,
                  dsp56k::TWord &first,
                  dsp56k::TWord &second) const override {
    const auto found = program_.find(address);
    if (found == program_.end()) {
      throw Fault("Missing own cycle-diagnostic opcode at P:" + hex(address, 6));
    }
    ++reads_;
    first = found->second[0];
    second = found->second[1];
    return true;
  }

  bool memValidateAccess(dsp56k::EMemArea /*area*/,
                         dsp56k::TWord /*address*/,
                         bool /*write*/) const override {
    throw Fault("Unexpected cycle-test fallback array access");
  }

  bool readWord(dsp56k::EMemArea /*area*/,
                dsp56k::TWord /*address*/,
                dsp56k::TWord & /*value*/) const override {
    throw Fault("Unexpected cycle-test data read");
  }

  bool writeWord(dsp56k::EMemArea /*area*/,
                 dsp56k::TWord /*address*/,
                 dsp56k::TWord /*value*/) const override {
    throw Fault("Unexpected cycle-test data write");
  }

  bool readExtension(dsp56k::TWord /*address*/, dsp56k::TWord & /*value*/) const override {
    throw Fault("Unexpected cycle-test extension execution");
  }

  [[nodiscard]] unsigned reads() const { return reads_; }

private:
  std::map<unsigned, std::array<dsp56k::TWord, 2>> program_;
  mutable unsigned reads_ = 0;
};
}

void native_cycle_cache() {
  Code code;
  DspBus bus;
  Peripheral x(bus, DspSpace::x);
  Peripheral y(bus, DspSpace::y);
  dsp56k::Memory memory(code, dsp56k::Memory::BusOwned{}, words, xy_words);
  require(!memory.hasArrayStorage() && !memory.hasMmuSupport(),
          "Actual SDK metadata operations use bus-owned storage without fallback arrays");
  dsp56k::DSP cpu(memory, &x, &y);
  static_assert(!dsp56k::g_useJIT);
  require(cpu.getJitEntries() == nullptr && cpu.getJit().getRuntime() == nullptr,
          "Forced interpreter construction must not create JIT tables or its optional runtime");
  constexpr unsigned loop_end = 0x123;
  require(cpu.writeReg(dsp56k::Reg_M0, dsp56k::TReg24{7}) &&
              cpu.writeReg(dsp56k::Reg_SR,
                           dsp56k::TReg24{static_cast<dsp56k::TInt32>(masked_status)}) &&
              cpu.writeReg(dsp56k::Reg_LA, dsp56k::TReg24{static_cast<dsp56k::TInt32>(loop_end)}) &&
              cpu.regs().m[0].toWord() == 7 && cpu.getSR().toWord() == masked_status &&
              cpu.regs().la.toWord() == loop_end && cpu.getJitEntries() == nullptr,
          "Real modifier, SR and LA writes retain their values without activating a JIT chain");
  code.set(site, "nop");
  require(cpu.getOpcodeCycles(site) == 1 && code.reads() == 1,
          "FM NOP has one diagnostic cycle at the high program address");
  const auto saved = cpu.referenceState();
  require(saved.opcodeCycleCache.size() == words && saved.opcodeCycleCache.read(site) == 1 &&
              saved.opcodeCycleCache.resident_pages() == 1,
          "Actual SDK snapshots retain the nonzero cached value and sparse logical space");
  code.set(site, "move #>$123456,x0");
  require(cpu.getOpcodeCycles(site) == 1 && code.reads() == 1,
          "Existing memoization retains its count until explicit code invalidation");
  cpu.clearOpcodeCache(site);
  require(cpu.getOpcodeCycles(site) == 2 && code.reads() == 2 &&
              saved.opcodeCycleCache.read(site) == 1,
          "FM immediate MOVE costs two cycles and invalidation cannot mutate a captured snapshot");
  cpu.restoreReferenceState(saved);
  require(cpu.getOpcodeCycles(site) == 1 && code.reads() == 2,
          "SDK restore retains the captured cycle value without recomputing from changed code");
  cpu.clearOpcodeCache(cold_address);
  require(cpu.referenceState().opcodeCycleCache.resident_pages() == 1,
          "SDK invalidation of a cold program page must not allocate cycle storage");
  cpu.clearOpcodeCache();
  require(cpu.referenceState().opcodeCycleCache.resident_pages() == 0 &&
              cpu.getOpcodeCycles(site) == 2 && code.reads() == 3 && x.resets() == 1 &&
              y.resets() == 1 && x.terminations() == 0 && y.terminations() == 0 &&
              x.writes() == 2 && y.writes() == 0 &&
              bus.read({.space = DspSpace::x, .address = core_priority}) == 0 &&
              bus.read({.space = DspSpace::x, .address = peripheral_priority}) == 0,
          "Global invalidation clears every cycle memo and leaves peripheral reset history alone");
  cpu.getJit().destroyAllBlocks();
  require(cpu.getJitEntries() == nullptr && cpu.getJit().getRuntime() == nullptr &&
              cpu.getJit().getLoops().empty() && cpu.getJit().getLoopEnds().empty(),
          "Program writes, restore and JIT cleanup cannot activate JIT state in the interpreter");
}
}
