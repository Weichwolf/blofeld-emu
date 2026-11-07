// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_write_store.hpp"

#include "common/bus.hpp"
#include "storage/nand.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_operation_clock.hpp"
#include "storage/nand_write_medium.hpp"
#include "storage/private_file.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>

namespace blofeld {
namespace {
std::filesystem::path destination(const std::filesystem::path &path) {
  if (path.extension() != ".bfnand") { throw Fault("Timed NAND store requires .bfnand"); }
  const auto parent = std::filesystem::canonical(
      path.parent_path().empty() ? std::filesystem::path(".") : path.parent_path());
  return parent / path.filename();
}

NandWriteMedium load_medium(const PrivateDirectory &directory,
                            const std::filesystem::path &name,
                            const NandImage *seed,
                            const NandReadContinuation continuation,
                            const std::uint64_t timebase,
                            NandCellProfile profile) {
  if (directory.contains(name)) {
    const auto image = decode_nand_image(directory.read(name, nand_image_max_bytes));
    return {Nand(image, continuation), timebase, std::move(profile)};
  }
  if (seed == nullptr) { throw Fault("Timed NAND store has no existing image or initial seed"); }
  NandWriteMedium model(Nand(*seed, continuation), timebase, std::move(profile));
  directory.write(name, encode_nand_image(*seed));
  return model;
}

class Updating {
public:
  explicit Updating(bool &updating) : flag_(updating) { flag_ = true; }

  ~Updating() { flag_ = false; }

  Updating(const Updating &) = delete;
  Updating &operator=(const Updating &) = delete;

private:
  bool &flag_;
};
}

class NandWriteStore::Impl {
public:
  Impl(const std::filesystem::path &file,
       const NandImage *seed,
       const NandReadContinuation continuation,
       const std::uint64_t timebase,
       NandCellProfile profile)
      : path(destination(file)),
        directory(path.parent_path()),
        lease(directory, path.filename().string() + ".lock"),
        model(load_medium(
            directory, path.filename(), seed, continuation, timebase, std::move(profile))),
        timebase_hz(timebase),
        read_continuation(continuation) {}

  void readable() const {
    if (failed) {
      throw Fault("Timed NAND store requires reload after persistence failure: " + path.string());
    }
  }

  void usable() const {
    readable();
    if (updating) { throw Fault("Reentrant timed NAND store mutation: " + path.string()); }
  }

  void transact(const std::function<void(NandWriteMedium &)> &action) {
    usable();
    const Updating guard(updating);
    auto next = model;
    action(next);
    auto cut = next;
    cut.power_loss();
    try {
      directory.write(
          path.filename(), encode_nand_image(cut.device().persistent_image()), observer);
    } catch (...) {
      failed = true;
      throw;
    }
    static_assert(std::is_nothrow_move_assignable_v<NandWriteMedium>);
    model = std::move(next);
  }

  std::filesystem::path path;
  PrivateDirectory directory;
  PrivateFileLease lease;
  NandWriteMedium model;
  std::uint64_t timebase_hz;
  NandReadContinuation read_continuation;
  NandFileObserver observer;
  bool updating = false;
  bool failed = false;
};

NandWriteStore::NandWriteStore(const std::filesystem::path &path,
                               const NandReadContinuation continuation,
                               const std::uint64_t timebase,
                               NandCellProfile profile)
    : impl_(std::make_unique<Impl>(path, nullptr, continuation, timebase, std::move(profile))) {}

NandWriteStore::NandWriteStore(const std::filesystem::path &path,
                               const NandImage &seed,
                               const NandReadContinuation continuation,
                               const std::uint64_t timebase,
                               NandCellProfile profile)
    : impl_(std::make_unique<Impl>(path, &seed, continuation, timebase, std::move(profile))) {}

NandWriteStore::~NandWriteStore() = default;

void NandWriteStore::set_observer(NandFileObserver observer) {
  impl_->usable();
  impl_->observer = std::move(observer);
}

const Nand &NandWriteStore::device() const {
  impl_->readable();
  return impl_->model.device();
}

std::uint64_t NandWriteStore::ticks() const {
  impl_->readable();
  return impl_->model.ticks();
}

std::uint64_t NandWriteStore::timebase_hz() const {
  return impl_->timebase_hz;
}

NandReadContinuation NandWriteStore::continuation() const {
  return impl_->read_continuation;
}

bool NandWriteStore::faulted() const {
  return impl_->failed || impl_->model.faulted();
}

std::optional<NandCompletionEvent> NandWriteStore::next_event() const {
  impl_->readable();
  return impl_->model.next_event();
}

std::optional<NandCompletionEvent> NandWriteStore::advance(const std::uint64_t at) {
  impl_->usable();
  const auto event = impl_->model.next_event();
  if (event && at >= event->ticks &&
      (event->operation == NandPending::program || event->operation == NandPending::erase)) {
    std::optional<NandCompletionEvent> result;
    impl_->transact([&](NandWriteMedium &next) { result = next.advance(at); });
    return result;
  }
  return impl_->model.advance(at);
}

void NandWriteStore::command(const std::uint8_t value) {
  impl_->usable();
  constexpr unsigned program = 0x10;
  constexpr unsigned erase = 0xd0;
  constexpr unsigned reset = 0xff;
  const auto pending = impl_->model.device().pending();
  const bool aborts_write =
      value == reset && (pending == NandPending::program || pending == NandPending::erase);
  if (value == program || value == erase || aborts_write) {
    impl_->transact([value](NandWriteMedium &next) { next.command(value); });
  } else {
    impl_->model.command(value);
  }
}

void NandWriteStore::observe_power_ready() {
  impl_->usable();
  impl_->model.observe_power_ready();
}

void NandWriteStore::observe_identification(const NandIdentification identification) {
  impl_->usable();
  impl_->model.observe_identification(identification);
}

void NandWriteStore::observe_wp(const std::optional<bool> high) {
  impl_->usable();
  impl_->model.observe_wp(high);
}

void NandWriteStore::address(const std::uint8_t value) {
  impl_->usable();
  impl_->model.address(value);
}

void NandWriteStore::write(const std::uint8_t value) {
  impl_->usable();
  impl_->model.write(value);
}

std::uint8_t NandWriteStore::read(const std::uint8_t mask) {
  impl_->usable();
  return impl_->model.read(mask);
}

void NandWriteStore::observe_read_end(const NandReadEnd edge) {
  impl_->usable();
  impl_->model.observe_read_end(edge);
}

void NandWriteStore::power_loss() {
  impl_->transact([](NandWriteMedium &next) { next.power_loss(); });
}
}
