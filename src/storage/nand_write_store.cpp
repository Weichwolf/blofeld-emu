// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_write_store.hpp"

#include "common/bus.hpp"
#include "storage/image_journal.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
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

PrivateFile open_image(const PrivateDirectory &directory,
                       const std::filesystem::path &name,
                       const NandImage *seed) {
  if (!directory.contains(name)) {
    if (seed == nullptr) { throw Fault("NAND store has no existing image or initial seed"); }
    directory.write(name, encode_nand_image(*seed));
  }
  return {directory, name, PrivateFileAccess::read_write};
}

NandWriteMedium load_medium(const PrivateFile &file,
                            ImageJournalPosition &position,
                            const NandReadContinuation continuation,
                            const std::uint64_t timebase,
                            NandCellProfile profile) {
  const auto image = load_image_journal(file, position);
  return {Nand(image, continuation), timebase, std::move(profile)};
}

NandRegion affected(const Nand &device, const NandPending operation) {
  if (operation == NandPending::program) { return {.first_page = device.page()}; }
  if (operation == NandPending::erase) {
    return {.first_page = (device.page() / Nand::block_pages) * Nand::block_pages,
            .page_count = Nand::block_pages};
  }
  throw Fault("Persistent image mutation has no program/erase region");
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
  Impl(const std::filesystem::path &image_path,
       const NandImage *seed,
       const NandReadContinuation continuation,
       const std::uint64_t timebase,
       NandCellProfile profile)
      : path(destination(image_path)),
        directory(path.parent_path()),
        lease(directory, path.filename().string() + ".lock"),
        file(open_image(directory, path.filename(), seed)),
        model(load_medium(file, position, continuation, timebase, std::move(profile))),
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

  void transact(const NandRegion region, const std::function<void(NandWriteMedium &)> &action) {
    usable();
    const Updating guard(updating);
    auto next = model;
    action(next);
    auto cut = next;
    cut.power_loss();
    try {
      append_image_journal(file, position, cut.device().persistent_image(region), observer);
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
  PrivateFile file;
  ImageJournalPosition position;
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
    impl_->transact(affected(impl_->model.device(), event->operation),
                    [&](NandWriteMedium &next) { result = next.advance(at); });
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
    auto operation = pending;
    if (value == program) { operation = NandPending::program; }
    if (value == erase) { operation = NandPending::erase; }
    impl_->transact(affected(impl_->model.device(), operation),
                    [value](NandWriteMedium &next) { next.command(value); });
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
  impl_->usable();
  const auto pending = impl_->model.device().pending();
  if (pending == NandPending::program || pending == NandPending::erase) {
    impl_->transact(affected(impl_->model.device(), pending),
                    [](NandWriteMedium &next) { next.power_loss(); });
  } else {
    impl_->model.power_loss();
  }
}
}
