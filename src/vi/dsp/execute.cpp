// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/execute.hpp"

#include "execution.hpp"

#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>

namespace blofeld::vi::dsp {
namespace {
constexpr dsp56300::Word boot_first = 0xff0000;
constexpr dsp56300::Word reset_status = 0xc00300;
constexpr dsp56300::Word reset_mode = 0x305;
constexpr dsp56300::Word once_data_address = 0xfffffc;

class Io final : public dsp56300::Io {
public:
  Io(const std::span<const dsp56300::Word> boot, const std::stop_token &stop)
      : boot_(boot), stop_(stop) {}

  void bind(dsp56300::Core &core) noexcept { core_ = &core; }

  [[nodiscard]] std::optional<dsp56300::Word>
  inspect_program(const dsp56300::Word address) const noexcept override {
    if (address < boot_first || address - boot_first >= boot_.size()) { return {}; }
    return boot_[address - boot_first];
  }

  bool read(const dsp56300::Space space,
            const dsp56300::Word address,
            dsp56300::Word /*pc*/,
            dsp56300::Word &value) noexcept override {
    if (stop_.stop_requested()) {
      core_->request_yield();
      return false;
    }
    const auto word = space == dsp56300::Space::program ? inspect_program(address) : std::nullopt;
    if (!word) { return false; }
    value = *word;
    return true;
  }

  bool write(const dsp56300::Space space,
             const dsp56300::Word address,
             dsp56300::Word /*pc*/,
             const dsp56300::Word value) noexcept override {
    if (space != dsp56300::Space::x || address != once_data_address) { return false; }
    // DSP56300FM Rev.5, 7.2.4: OGDBR is a 24-bit output latch, readable
    // only through JTAG. A core write also enables the ROM-secured OnCE port.
    once_data_ = value;
    return true;
  }

  bool bit(dsp56300::Space /*space*/,
           dsp56300::Word /*address*/,
           dsp56300::Word /*pc*/,
           dsp56300::Word /*bit*/,
           dsp56300::BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

private:
  std::span<const dsp56300::Word> boot_;
  const std::stop_token &stop_;
  dsp56300::Core *core_ = nullptr;
  std::optional<dsp56300::Word> once_data_;
};

Access operation(const dsp56300::FaultKind fault) {
  switch (fault) {
    case dsp56300::FaultKind::io_read:
    case dsp56300::FaultKind::memory_read: return Access::read;
    case dsp56300::FaultKind::io_write:
    case dsp56300::FaultKind::memory_write: return Access::write;
    case dsp56300::FaultKind::io_bit: return Access::bit;
    case dsp56300::FaultKind::none:
    case dsp56300::FaultKind::unsupported_instruction:
    case dsp56300::FaultKind::unsupported_mode:
    case dsp56300::FaultKind::stack:
    case dsp56300::FaultKind::invalid_instruction:
    case dsp56300::FaultKind::host_event:
    default: return Access::execute;
  }
}
}

ExecutionReport execute(const std::span<const dsp56300::Word> boot, const std::stop_token &stop) {
  if (boot.empty() || boot.size() > dsp56300::address_words - boot_first) {
    throw std::invalid_argument("DSP boot ROM has invalid bounds");
  }
  Io io(boot, stop);
  dsp56300::Core core(dsp56300::Memory::external(), &io);
  io.bind(core);
  core.reset();
  auto registers = core.registers();
  registers.pc = boot_first;
  registers.sr = reset_status;
  registers.omr = reset_mode;
  core.set_registers(registers);
  ExecutionReport report;
  while (!stop.stop_requested()) {
    const auto result = core.run();
    report.instructions += result.retired;
    if (stop.stop_requested()) { break; }
    if (result.fault.kind != dsp56300::FaultKind::none) {
      const auto &fault = result.fault;
      char space = 'P';
      if (fault.space == dsp56300::Space::x) { space = 'X'; }
      if (fault.space == dsp56300::Space::y) { space = 'Y'; }
      report.failure = Failure{.operation = operation(fault.kind),
                               .pc = fault.pc,
                               .address = fault.address,
                               .value = fault.value,
                               .code = static_cast<std::uint32_t>(fault.kind),
                               .space = space,
                               .reason = "DSP access or instruction is not implemented"};
      break;
    }
    if (result.state != dsp56300::ExecutionState::running) { break; }
  }
  return report;
}
}
