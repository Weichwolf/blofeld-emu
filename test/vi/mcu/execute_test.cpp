// SPDX-License-Identifier: GPL-3.0-only
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "mcu/execute.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cpu12/memory.hpp>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void firmware_controls_reset() {
  namespace isa = cpu12::isa;
  constexpr cpu12::Address marker = 0x3000;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::clr_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x78},
                                        isa::opcode<isa::Opcode::ldaa_immediate>(),
                                        std::array<cpu12::Byte, 1>{1},
                                        isa::opcode<isa::Opcode::staa_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x7a},
                                        isa::opcode<isa::Opcode::brclr_extended>(),
                                        std::array<cpu12::Byte, 4>{2, 0x78, 1, 3},
                                        isa::opcode<isa::Opcode::ldaa_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 2},
                                        isa::opcode<isa::Opcode::clr_extended>(),
                                        std::array<cpu12::Byte, 2>{2, 0x7a},
                                        isa::opcode<isa::Opcode::brset_extended>(),
                                        std::array<cpu12::Byte, 4>{2, 0x78, 1, 3},
                                        isa::opcode<isa::Opcode::ldaa_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 1},
                                        isa::opcode<isa::Opcode::ldaa_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 0});
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(program, image->bytes.begin());
  image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
  image->bytes.back() = 0;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::ExecutionReport report;
  std::atomic<bool> done{false};
  std::jthread worker([&](const std::stop_token &stop) {
    try {
      report = blofeld::vi::mcu::execute(*image, shi, reset, stop);
    } catch (...) { report.exception = std::current_exception(); }
    done.store(true, std::memory_order_release);
  });
  while (!reset.state().held) {
    require(!done.load(std::memory_order_acquire), "The MCU must assert reset through U0");
    std::this_thread::yield();
  }
  require(reset.state().epoch == 1 && reset.acknowledge(1), "The DSP owner applies the reset");
  worker.join();
  require(!report.exception && report.failure && report.failure->address == marker &&
              report.failure->operation == blofeld::vi::Access::read && !reset.state().held,
          "Native GPIO reads see driven low, then the released RESET pull-up");
}
}

int main() try { firmware_controls_reset(); } catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
