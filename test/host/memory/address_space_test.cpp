// SPDX-License-Identifier: GPL-3.0-only
// POSIX signal/wait extensions precede glibc's duplicate stdlib macros.
extern "C" {
#include <signal.h>
#if defined(__APPLE__)
#include <sys/signal.h>
#endif
#include <sys/wait.h>
}

#include "memory/address_space.hpp"
#include "memory/faults.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <sys/resource.h>
#include <system_error>
#include <unistd.h>

namespace {
using namespace blofeld::host::memory;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

// Each child performs exactly one real host access; its parent remains usable.
void forbidden(std::byte *address, const bool writing) {
  const auto child = fork();
  if (child < 0) { throw std::system_error(errno, std::generic_category(), "Fork memory probe"); }
  if (child == 0) {
    const rlimit no_core{.rlim_cur = 0, .rlim_max = 0};
    if (setrlimit(RLIMIT_CORE, &no_core) != 0) { _exit(2); }
    if (writing) {
      *static_cast<volatile std::byte *>(address) = std::byte{1};
    } else {
      const auto value = *static_cast<const volatile std::byte *>(address);
      static_cast<void>(value);
    }
    _exit(3);
  }
  int status = 0;
  require(waitpid(child, &status, 0) == child && WIFSIGNALED(status) &&
              (WTERMSIG(status) == SIGSEGV || WTERMSIG(status) == SIGBUS),
          "Host protection rejects the exact forbidden read or write");
}

void mappings(const std::size_t page) {
  const Backing backing(page * 2);
  AddressSpace read(page * 4);
  AddressSpace write(page * 4);
  read.map({.offset = page, .size = page}, &backing);
  write.map({.offset = page, .size = page, .writable = true}, &backing);
  write.map({.address = page * 2, .size = page, .writable = true});
  read.map({.address = page * 2, .size = page});
  auto *const physical = static_cast<volatile std::byte *>(backing.bytes().data());
  auto *const writable = static_cast<volatile std::byte *>(write.bytes().data());
  const auto *const readable = static_cast<const volatile std::byte *>(read.bytes().data());
  physical[page] = std::byte{7};
  require(readable[0] == std::byte{7}, "The read mapping shares the selected backing offset");
  writable[0] = std::byte{9};
  require(physical[page] == std::byte{9} && readable[0] == std::byte{9},
          "Distinct read/write virtual addresses preserve real physical identity");
  writable[page * 2] = std::byte{11};
  require(readable[page * 2] == std::byte{0},
          "Discarded writes cannot change declared constant read-only zeros");
  forbidden(read.bytes().data(), true);
  forbidden(read.bytes().data() + page, false);
  forbidden(write.bytes().data() + page, true);
  forbidden(read.bytes().data() + (page * 2), true);
  for (const auto region : std::array{AddressSpace::Region{.size = 0},
                                      AddressSpace::Region{.address = 1, .size = page},
                                      AddressSpace::Region{.offset = 1, .size = page},
                                      AddressSpace::Region{.address = page * 4, .size = page},
                                      AddressSpace::Region{.offset = page * 2, .size = page},
                                      AddressSpace::Region{.size = page + 1}}) {
    bool rejected = false;
    try {
      write.map(region, &backing);
    } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Malformed geometry is rejected before replacing any host pages");
  }
  require(readable[0] == std::byte{9}, "Rejected mappings preserve live RAM aliases");
}
}

int main() try {
  const Faults faults([](std::uintptr_t, bool, std::uintptr_t) noexcept { return false; });
  const auto page = sysconf(_SC_PAGESIZE);
  require(page > 1, "Query the actual native host page size");
  mappings(static_cast<std::size_t>(page));
  std::cout << "Shared VM backing, read/write views, scratch and protected holes passed\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
