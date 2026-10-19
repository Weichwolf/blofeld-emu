// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_machine.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <memory>
#include <utility>
#include <vector>

namespace blofeld {
namespace {
std::vector<std::uint8_t> read_specification(const std::filesystem::path &path) {
  constexpr std::uintmax_t maximum_size = 16777216;
  const auto size = std::filesystem::file_size(path);
  if (size == 0 || size > maximum_size) { throw Fault("Invalid MCU checkpoint backend size"); }
  std::ifstream stream(path, std::ios::binary);
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  stream.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!stream || stream.peek() != std::ifstream::traits_type::eof()) {
    throw Fault("Cannot read complete MCU checkpoint backend: " + path.string());
  }
  return bytes;
}
}

struct McuCheckpoint::Storage {
  Storage(std::unique_ptr<McuBus> bus_state,
          const Hc12Checkpoint &cpu_state,
          const std::vector<std::uint8_t> &backend)
      : bus(std::move(bus_state)), cpu(cpu_state), specification(backend) {}

  const std::unique_ptr<const McuBus> bus;
  const Hc12Checkpoint cpu;
  const std::vector<std::uint8_t> specification;
};

struct McuMachine::State {
  State(std::unique_ptr<McuBus> bus_state, const std::filesystem::path &specification)
      : bus(std::move(bus_state)), cpu(*bus, specification) {}

  std::unique_ptr<McuBus> bus;
  Hc12 cpu;
};

McuCheckpoint::McuCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

McuCheckpoint::~McuCheckpoint() = default;

McuMachine::McuMachine(const McuFlashImage &image, const std::filesystem::path &specification)
    : specification_(specification),
      specification_bytes_(read_specification(specification)),
      state_(std::make_unique<State>(std::make_unique<McuBus>(image), specification)) {
  check_specification();
  state_->cpu.reset();
}

McuMachine::~McuMachine() = default;

McuBus &McuMachine::bus() {
  return *state_->bus;
}

Hc12 &McuMachine::cpu() {
  return state_->cpu;
}

const McuBus &McuMachine::bus() const {
  return *state_->bus;
}

const Hc12 &McuMachine::cpu() const {
  return state_->cpu;
}

void McuMachine::check_specification() const {
  if (read_specification(specification_) != specification_bytes_) {
    throw Fault("MCU checkpoint backend changed after machine construction");
  }
}

McuCheckpoint McuMachine::checkpoint() const {
  const auto cpu_state = state_->cpu.checkpoint();
  check_specification();
  return McuCheckpoint(std::make_shared<const McuCheckpoint::Storage>(
      state_->bus->clone(), cpu_state, specification_bytes_));
}

void McuMachine::restore(const McuCheckpoint &checkpoint) {
  if (state_->cpu.executing() || state_->bus->notifying_) {
    throw Fault("Cannot restore during active MCU execution or bus observation");
  }
  if (!checkpoint.storage_) { throw Fault("Cannot restore an empty MCU checkpoint"); }
  if (checkpoint.storage_->specification != specification_bytes_) {
    throw Fault("MCU checkpoint has an incompatible instruction backend");
  }
  check_specification();
  auto replacement = std::make_unique<State>(checkpoint.storage_->bus->clone(), specification_);
  replacement->cpu.restore_checkpoint(checkpoint.storage_->cpu);
  check_specification();
  state_.swap(replacement);
}

}
