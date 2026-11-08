// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/execute.hpp"

#include "execution.hpp"
#include "io/shi.hpp"
#include "mcu/memory.hpp"
#include "mcu/ports.hpp"
#include "storage/flash_image.hpp"

#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <stop_token>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint16_t reset_vector = 0xfffe;
constexpr std::uint16_t mapping_first = 0x10;
constexpr std::uint16_t mapping_last = 0x13;
constexpr std::uint8_t all_bits = 0xff;

class Io {
public:
  Io(Memory &memory, Shi &shi, const std::stop_token &stop)
      : memory_(memory), ports_(shi), stop_(stop) {}

  void bind(cpu12::Core &core) noexcept { core_ = &core; }

  cpu12::Io callbacks() noexcept;

private:
  bool interrupted() noexcept {
    if (!stop_.stop_requested()) { return false; }
    core_->request_yield();
    return true;
  }

  bool read(const std::uint16_t address,
            std::uint8_t &value,
            const std::uint8_t mask = all_bits) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(address);
    if (at.space == Space::registers && Ports::contains(at.offset)) {
      return ports_.read(at.offset, value, mask);
    }
    if (at.space == Space::registers && at.offset >= mapping_first && at.offset <= mapping_last) {
      value = memory_.read_mapping(at.offset);
      return true;
    }
    return memory_.read(address, value);
  }

  bool write(const cpu12::ByteWrite access) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(access.address);
    if (at.space == Space::registers && Ports::contains(at.offset)) {
      return ports_.write({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && at.offset >= mapping_first && at.offset <= mapping_last) {
      if (memory_.write_mapping({.address = at.offset, .value = access.value})) {
        core_->request_yield();
      }
      return true;
    }
    return memory_.write(access);
  }

  Memory &memory_;
  Ports ports_;
  const std::stop_token &stop_;
  cpu12::Core *core_ = nullptr;
};

cpu12::Io Io::callbacks() noexcept {
  return {.owner = this,
          .read =
              [](void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
                return static_cast<Io *>(owner)->read(address, value);
              },
          .write =
              [](void *owner, const cpu12::ByteWrite access) noexcept {
                return static_cast<Io *>(owner)->write(access);
              },
          .fetch =
              [](void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
                return static_cast<Io *>(owner)->read(address, value);
              },
          .read_masked =
              [](void *owner, const cpu12::MaskedByteRead access, std::uint8_t &value) noexcept {
                return static_cast<Io *>(owner)->read(access.address, value, access.mask);
              }};
}

Access operation(const cpu12::Fault fault) {
  switch (fault) {
    case cpu12::Fault::memory_read: return Access::read;
    case cpu12::Fault::memory_write: return Access::write;
    case cpu12::Fault::instruction_fetch: return Access::fetch;
    case cpu12::Fault::none:
    case cpu12::Fault::invalid_input:
    case cpu12::Fault::unsupported_instruction:
    case cpu12::Fault::unsupported_host:
    case cpu12::Fault::reentrant:
    case cpu12::Fault::unsupported_state:
    default: return Access::execute;
  }
}
}

ExecutionReport execute(FlashImage &image, Shi &shi, const std::stop_token &stop) {
  Memory memory(image);
  auto grants = memory.regions();
  Io io(memory, shi, stop);
  auto callbacks = io.callbacks();
  cpu12::Core core(cpu12::Memory::mapped(grants), &callbacks);
  io.bind(core);
  ExecutionReport report;
  const auto reset = core.reset(reset_vector);
  if (!reset.accepted) {
    report.failure = Failure{.operation = Access::reset,
                             .pc = reset.pc,
                             .address = reset.address,
                             .value = reset.value,
                             .code = static_cast<std::uint32_t>(reset.fault),
                             .reason = "MCU reset vector access failed"};
    return report;
  }
  while (!stop.stop_requested()) {
    const auto result = core.run();
    report.instructions += result.retired;
    if (stop.stop_requested()) { break; }
    if (result.fault != cpu12::Fault::none) {
      report.failure = Failure{.operation = operation(result.fault),
                               .pc = result.pc,
                               .address = result.address,
                               .value = result.value,
                               .code = static_cast<std::uint32_t>(result.fault),
                               .reason = "MCU access or instruction is not implemented"};
      break;
    }
    if (memory.commit_mapping()) {
      grants = memory.regions();
      core.set_memory(cpu12::Memory::mapped(grants));
    }
    if (result.state != cpu12::ExecutionState::running) { break; }
  }
  return report;
}
}
