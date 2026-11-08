// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/execute.hpp"
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/shi.hpp"
#include "support/isa.hpp"

#include <array>
#include <atomic>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void reset_running_core() {
  namespace isa = dsp56300::isa;
  constexpr dsp56300::Word start = 0xff0000;
  constexpr dsp56300::Word configuration = 0x29;
  constexpr dsp56300::Word x0 = 4;
  constexpr dsp56300::Word immediate = 0x34;
  constexpr dsp56300::Word write = 0x8000;
  constexpr dsp56300::Word control_port = 0x11;
  constexpr dsp56300::Word receive_ready_bit = 17;
  constexpr dsp56300::Word receive_port = 0x14;
  constexpr dsp56300::Word absolute = 0x30;
  constexpr auto enable =
      isa::opcode<isa::Opcode::movep_register_low>()[0] | (x0 << 8U) | write | control_port;
  constexpr auto poll =
      isa::opcode<isa::Opcode::jclr_port_low>()[0] | (control_port << 8U) | receive_ready_bit;
  constexpr auto read =
      isa::opcode<isa::Opcode::movep_program_low>()[0] | (absolute << 8U) | receive_port;
  constexpr auto program = isa::program(
      std::array{isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | write,
                 configuration,
                 enable,
                 poll,
                 start + 3,
                 read,
                 dsp56300::Word{0}},
      isa::opcode<isa::Opcode::jmp_absolute_short>());
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::ExecutionReport report;
  std::atomic<bool> done{false};
  std::jthread worker([&](const std::stop_token &stop) {
    try {
      report = blofeld::vi::dsp::execute(program, shi, reset, stop);
    } catch (...) { report.exception = std::current_exception(); }
    done.store(true, std::memory_order_release);
  });
  const auto wait_boot = [&] {
    while (!shi.select(true)) {
      require(!done.load(std::memory_order_acquire), "DSP reset code must enable its own SHI");
      std::this_thread::yield();
    }
  };
  wait_boot();
  require(reset.assert_reset() && reset.release({}), "A running DSP acknowledges real reset");
  require(reset.state().epoch == 1, "Reset advances the execution lifetime");
  wait_boot();
  require(shi.receive(isa::stop[0]), "Send a real instruction after the new boot enables SHI");
  worker.join();
  require(!report.exception && !report.failure && report.instructions > 0,
          "A fresh Core boots, loads PRAM and executes STOP after reset");
}
}

int main() try { reset_running_core(); } catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
