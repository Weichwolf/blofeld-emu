// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_store.hpp"

#include "common/bus.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"
#include "storage/private_file.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>

namespace blofeld {
namespace {
std::filesystem::path destination(const std::filesystem::path &path) {
  if (path.extension() != ".bfnand") {
    throw Fault("NAND store requires the private .bfnand extension");
  }
  const auto parent = std::filesystem::canonical(
      path.parent_path().empty() ? std::filesystem::path(".") : path.parent_path());
  return parent / path.filename();
}

class Updating {
public:
  explicit Updating(bool &flag) : flag_(flag) { flag_ = true; }

  ~Updating() { flag_ = false; }

  Updating(const Updating &) = delete;
  Updating &operator=(const Updating &) = delete;

private:
  bool &flag_;
};
}

class NandStore::Impl {
public:
  Impl(const std::filesystem::path &file, const NandImage *initial)
      : path(destination(file)),
        directory(path.parent_path()),
        lease(directory, path.filename().string() + ".lock") {
    if (initial != nullptr) {
      initial->validate();
      if (directory.contains(path.filename())) {
        throw Fault("NAND store destination already exists: " + path.string());
      }
      directory.write(path.filename(), encode_nand_image(*initial));
      model = Nand(*initial);
    } else {
      model = Nand(decode_nand_image(directory.read(path.filename(), nand_image_max_bytes)));
    }
  }

  void usable() const {
    if (failed) {
      throw Fault("NAND store requires reload after failed persistence: " + path.string());
    }
    if (updating) { throw Fault("Reentrant NAND store mutation: " + path.string()); }
  }

  void transact(const std::function<void(Nand &)> &action) {
    usable();
    const Updating guard(updating);
    auto next = model;
    action(next);
    auto cut = next;
    cut.power_loss();
    const auto image = cut.persistent_image();
    try {
      directory.write(path.filename(), encode_nand_image(image), observer);
    } catch (...) {
      failed = true;
      throw;
    }
    static_assert(std::is_nothrow_move_assignable_v<Nand>);
    model = std::move(next);
  }

  std::filesystem::path path;
  PrivateDirectory directory;
  PrivateFileLease lease;
  Nand model;
  NandFileObserver observer;
  bool updating = false;
  bool failed = false;
};

NandStore::NandStore(const std::filesystem::path &path)
    : impl_(std::make_unique<Impl>(path, nullptr)) {}

NandStore::NandStore(const std::filesystem::path &path, const NandImage &initial)
    : impl_(std::make_unique<Impl>(path, &initial)) {}

NandStore::~NandStore() = default;

void NandStore::set_observer(NandFileObserver observer) {
  impl_->usable();
  impl_->observer = std::move(observer);
}

void NandStore::command(const std::uint8_t value) {
  impl_->usable();
  constexpr unsigned program_confirm = 0x10;
  constexpr unsigned erase_confirm = 0xd0;
  constexpr unsigned reset = 0xff;
  if (value == program_confirm || value == erase_confirm || value == reset) {
    impl_->transact([value](Nand &next) { next.command(value); });
  } else {
    impl_->model.command(value);
  }
}

void NandStore::address(const std::uint8_t value) {
  impl_->usable();
  impl_->model.address(value);
}

void NandStore::write(const std::uint8_t value) {
  impl_->usable();
  impl_->model.write(value);
}

void NandStore::observe_power_ready() {
  impl_->usable();
  impl_->model.observe_power_ready();
}

void NandStore::observe_wp(const std::optional<bool> high) {
  impl_->usable();
  impl_->model.observe_wp(high);
}

void NandStore::observe_completion() {
  impl_->usable();
  impl_->model.observe_completion();
}

void NandStore::observe_completion(const NandOutcome outcome) {
  impl_->transact([outcome](Nand &next) { next.observe_completion(outcome); });
}

void NandStore::power_loss() {
  impl_->transact([](Nand &next) { next.power_loss(); });
}

void NandStore::observe_bytes(const unsigned page,
                              const unsigned column,
                              const std::span<const std::uint8_t> bytes) {
  impl_->transact([page, column, bytes](Nand &next) { next.observe_bytes(page, column, bytes); });
}

void NandStore::observe_cell(const NandLocation location, const NandByte cell) {
  impl_->transact([location, cell](Nand &next) { next.observe_cell(location, cell); });
}

void NandStore::observe_block(const unsigned block, const NandBlock state) {
  impl_->transact([block, state](Nand &next) { next.observe_block(block, state); });
}

std::uint8_t NandStore::read(const std::uint8_t mask) {
  impl_->usable();
  return impl_->model.read(mask);
}

const Nand &NandStore::device() const {
  if (impl_->failed) {
    throw Fault("NAND store requires reload after failed persistence: " + impl_->path.string());
  }
  return impl_->model;
}

bool NandStore::faulted() const {
  return impl_->failed;
}
}
