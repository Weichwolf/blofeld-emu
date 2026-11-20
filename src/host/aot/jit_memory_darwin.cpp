// SPDX-License-Identifier: GPL-3.0-only
#include "aot/jit_memory_darwin.hpp"

#if defined(__aarch64__)
#include "llvm/ExecutionEngine/JITLink/JITLink.h"
#include "llvm/ExecutionEngine/JITLink/JITLinkDylib.h"
#include "llvm/ExecutionEngine/JITLink/JITLinkMemoryManager.h"
#include "llvm/ExecutionEngine/Orc/ExecutorProcessControl.h"
#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/ExecutionEngine/Orc/Shared/AllocationActions.h"
#include "llvm/ExecutionEngine/Orc/Shared/ExecutorAddress.h"
#include "llvm/ExecutionEngine/Orc/Shared/MemoryFlags.h"
#include "llvm/ExecutionEngine/Orc/Shared/WrapperFunctionUtils.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/MathExtras.h"

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <libkern/OSCacheControl.h>
#include <map>
#include <memory>
#include <pthread.h>
#include <ranges>
#include <span>
#include <stdexcept>
#include <sys/mman.h>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

// LLVM 19 Expected instantiates the C++23-deprecated std::aligned_union.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

namespace blofeld::host::aot::detail {
namespace {
using llvm::Error;
using llvm::jitlink::BasicLayout;
using llvm::jitlink::JITLinkMemoryManager;
using llvm::orc::AllocGroup;
using llvm::orc::ExecutorAddr;
using llvm::orc::MemLifetime;
using llvm::orc::MemProt;
using llvm::orc::shared::AllocActions;
using llvm::orc::shared::WrapperFunctionCall;

Error system_error(const char *action) {
  return llvm::createStringError(std::error_code(errno, std::generic_category()), "%s", action);
}

struct Protection {
  int flags;
  bool temporary;
};

std::map<AllocGroup, Protection> protections(const llvm::jitlink::LinkGraph &graph) {
  std::map<AllocGroup, Protection> result;
  for (const auto &section : graph.sections()) {
    const auto protection = section.getMemProt();
    const auto lifetime = section.getMemLifetime();
    if (lifetime == MemLifetime::NoAlloc) { continue; }
    const auto has = [protection](const MemProt flag) {
      return (std::to_underlying(protection) & std::to_underlying(flag)) != 0;
    };
    const int flags = (has(MemProt::Read) ? PROT_READ : 0) |
                      (has(MemProt::Write) ? PROT_WRITE : 0) | (has(MemProt::Exec) ? PROT_EXEC : 0);
    result.try_emplace(AllocGroup(protection, lifetime),
                       Protection{.flags = flags, .temporary = lifetime == MemLifetime::Finalize});
  }
  return result;
}

struct CodeWrite {
  char *address;
  std::span<const char> content;
};

int write_code(void *context) noexcept {
  const auto &write = *static_cast<const CodeWrite *>(context);
  std::memcpy(write.address, write.content.data(), write.content.size());
  return 0;
}

PTHREAD_JIT_WRITE_ALLOW_CALLBACKS_NP(write_code);

class Segment {
public:
  Segment(const std::size_t size, const Protection access) : protection(access), content(size) {}

  Segment(const Segment &) = delete;
  Segment &operator=(const Segment &) = delete;

  ~Segment() { llvm::consumeError(release()); }

  Error release() {
    if (address == nullptr) { return Error::success(); }
    if (munmap(address, content.size()) != 0) { return system_error("Unmap AOT segment"); }
    address = nullptr;
    return Error::success();
  }

  Error protect() {
    if ((protection.flags & PROT_EXEC) != 0) {
      CodeWrite write{.address = address, .content = content};
      if (pthread_jit_write_with_callback_np(write_code, &write) != 0) {
        return llvm::createStringError(std::errc::io_error, "Write native AOT code");
      }
      sys_icache_invalidate(address, content.size());
      return Error::success();
    }
    std::memcpy(address, content.data(), content.size());
    if (mprotect(address, content.size(), protection.flags) != 0) {
      return system_error("Protect AOT data");
    }
    return Error::success();
  }

  const Protection protection;
  std::vector<char> content;
  char *address = nullptr;
};

struct Allocation {
  std::vector<std::unique_ptr<Segment>> segments;
  std::vector<WrapperFunctionCall> deallocate;

  Error release(const bool temporary_only) {
    Error error = Error::success();
    for (auto &segment : segments) {
      if (!temporary_only || segment->protection.temporary) {
        error = llvm::joinErrors(std::move(error), segment->release());
      }
    }
    return error;
  }
};

class InFlight final : public JITLinkMemoryManager::InFlightAlloc {
public:
  InFlight(std::unique_ptr<Allocation> allocation, AllocActions &actions)
      : allocation_(std::move(allocation)), actions_(actions) {}

  void abandon(OnAbandonedFunction finish) override { finish(allocation_->release(false)); }

  void finalize(OnFinalizedFunction finish) override {
    for (const auto &segment : allocation_->segments) {
      if (auto error = segment->protect()) {
        finish(std::move(error));
        return;
      }
    }
#pragma clang diagnostic push
#pragma clang diagnostic ignored                                                                   \
    "-Wdeprecated-declarations" // LLVM 19 Expected uses std::aligned_union.
    auto deallocate = llvm::orc::shared::runFinalizeActions(actions_);
    if (!deallocate) {
      finish(deallocate.takeError());
      return;
    }
    allocation_->deallocate = std::move(*deallocate);
    if (auto error = allocation_->release(true)) {
      finish(llvm::joinErrors(std::move(error),
                              llvm::orc::shared::runDeallocActions(allocation_->deallocate)));
      return;
    }
    finish(JITLinkMemoryManager::FinalizedAlloc(ExecutorAddr::fromPtr(allocation_.release())));
  }

private:
  std::unique_ptr<Allocation> allocation_;
  AllocActions &actions_;
};

class Memory final : public JITLinkMemoryManager {
public:
  void allocate(const llvm::jitlink::JITLinkDylib * /*dylib*/,
                llvm::jitlink::LinkGraph &graph,
                OnAllocatedFunction finish) override {
    const auto page = static_cast<std::uint64_t>(getpagesize());
    BasicLayout layout(graph);
#pragma clang diagnostic push
#pragma clang diagnostic ignored                                                                   \
    "-Wdeprecated-declarations" // LLVM 19 Expected uses std::aligned_union.
    auto sizes = layout.getContiguousPageBasedLayoutSizes(page);
    if (!sizes) {
      finish(sizes.takeError());
      return;
    }
    const auto access = protections(graph);
    auto allocation = std::make_unique<Allocation>();
    void *hint = nullptr;
    for (auto &[group, planned] : layout.segments()) {
      const auto permissions = access.find(group);
      if (permissions == access.end()) {
        finish(
            llvm::createStringError(std::errc::invalid_argument, "Missing AOT segment protection"));
        return;
      }
      const auto flags = permissions->second.flags;
      const bool exec = (flags & PROT_EXEC) != 0;
      if (exec && flags != (PROT_READ | PROT_EXEC)) {
        finish(llvm::createStringError(std::errc::invalid_argument,
                                       "Expected read/execute AOT segment"));
        return;
      }
      const auto size = llvm::alignTo(planned.ContentSize + planned.ZeroFillSize, page);
      if (size == 0) { continue; }
      auto segment = std::make_unique<Segment>(size, permissions->second);
      const int protection = PROT_READ | PROT_WRITE | (exec ? PROT_EXEC : 0);
      const int mapping = MAP_PRIVATE | MAP_ANON | (exec ? MAP_JIT : 0);
      void *const address = mmap(hint, size, protection, mapping, -1, 0);
      if (address == MAP_FAILED) {
        finish(system_error("Map AOT segment"));
        return;
      }
      segment->address = static_cast<char *>(address);
      hint = segment->address + size;
      planned.Addr = ExecutorAddr::fromPtr(segment->address);
      planned.WorkingMem = segment->content.data();
      allocation->segments.push_back(std::move(segment));
    }
    if (auto error = layout.apply()) {
      finish(std::move(error));
      return;
    }
    finish(std::make_unique<InFlight>(std::move(allocation), layout.graphAllocActions()));
  }

  void deallocate(std::vector<FinalizedAlloc> allocations, OnDeallocatedFunction finish) override {
    Error error = Error::success();
    for (auto &allocation : allocations | std::views::reverse) {
      if (!allocation) { continue; }
      const std::unique_ptr<Allocation> owner(allocation.release().toPtr<Allocation *>());
      error = llvm::joinErrors(std::move(error),
                               llvm::orc::shared::runDeallocActions(owner->deallocate));
      error = llvm::joinErrors(std::move(error), owner->release(false));
    }
    finish(std::move(error));
  }
};
}

void use_jit_memory(llvm::orc::LLJITBuilder &builder) {
  if (pthread_jit_write_protect_supported_np() == 0) {
    throw std::runtime_error("The Darwin AOT mapper requires per-thread write protection");
  }
#pragma clang diagnostic push
#pragma clang diagnostic ignored                                                                   \
    "-Wdeprecated-declarations" // LLVM 19 Expected uses std::aligned_union.
  auto process =
      llvm::orc::SelfExecutorProcessControl::Create(nullptr, nullptr, std::make_unique<Memory>());
  if (!process) { throw std::runtime_error(llvm::toString(process.takeError())); }
  builder.setExecutorProcessControl(std::move(*process));
}
}

#pragma clang diagnostic pop
#endif
