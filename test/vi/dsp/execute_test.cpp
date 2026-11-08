// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/execute.hpp"
#include "dsp/ports.hpp"
#include "execution.hpp"
#include "files/read.hpp"
#include "io/dsp_reset.hpp"
#include "io/shi.hpp"
#include "storage/media_image.hpp"
#include "support/isa.hpp"

#include <array>
#include <atomic>
#include <dsp56300/memory.hpp>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void native_clock_register() {
  namespace isa = dsp56300::isa;
  constexpr dsp56300::Word control = 0x03620f;
  constexpr dsp56300::Word expected = 0x03620d;
  constexpr dsp56300::Word pctl_port = 0x3d;
  constexpr dsp56300::Word x0 = 4;
  constexpr dsp56300::Word immediate = 0x34;
  constexpr dsp56300::Word absolute = 0x30;
  constexpr dsp56300::Word write = 0x8000;
  constexpr dsp56300::Word unmapped = 0xc000;
  constexpr auto configure =
      isa::opcode<isa::Opcode::movep_memory_high>()[0] | (immediate << 8U) | write | pctl_port;
  constexpr auto clear = isa::opcode<isa::Opcode::bclr_port_high>()[0] | (pctl_port << 8U) | 1;
  constexpr auto read = isa::opcode<isa::Opcode::movep_register_high>()[0] | (x0 << 8U) | pctl_port;
  constexpr auto output = isa::opcode<isa::Opcode::move_x_effective>()[0] | (absolute << 8U);
  constexpr dsp56300::Word ram = 0x40;
  constexpr auto load = output | write;
  constexpr auto zero = isa::opcode<isa::Opcode::move_immediate_x0>()[0];
  constexpr auto sink = isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0;
  constexpr auto program = isa::program(
      std::array{configure, control, clear, read, output, ram, zero, load, ram, sink, unmapped});
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
  require(report.failure && report.failure->operation == blofeld::vi::Access::write &&
              report.failure->space == 'P' && report.failure->address == unmapped &&
              report.failure->value == expected,
          "Native MOVEP/BCLR share PCTL and native RAM stores/loads retain its actual value");
}

void native_memory_switch() {
  namespace isa = dsp56300::isa;
  constexpr dsp56300::Word immediate = 0x34;
  constexpr dsp56300::Word absolute = 0x30;
  constexpr dsp56300::Word write_control = 0x8000;
  constexpr dsp56300::Word x0 = 4;
  constexpr dsp56300::Word omr_selector = 0x1a;
  constexpr dsp56300::Word mode = 0x600385;
  constexpr dsp56300::Word source = 0x10000;
  constexpr dsp56300::Word destination = 0x1000;
  constexpr auto literal =
      isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | write_control;
  constexpr auto store = isa::opcode<isa::Opcode::move_x_effective>()[0] | (absolute << 8U);
  constexpr auto switch_mode =
      isa::opcode<isa::Opcode::movec_register>()[0] | (x0 << 8U) | write_control | omr_selector;
  constexpr auto jump = isa::opcode<isa::Opcode::jmp_effective>()[0] | (absolute << 8U);
  constexpr auto program =
      isa::program(std::array{literal, isa::stop[0], store, source, literal, mode, switch_mode},
                   isa::nop,
                   isa::nop,
                   isa::nop,
                   std::array{jump, destination});
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
  require(!report.failure && report.instructions == 9,
          "Guest OMR remaps retained X RAM into executable P without copying or a new Core");
}

void native_media_read(const std::filesystem::path &root) {
  namespace isa = dsp56300::isa;
  constexpr dsp56300::Word immediate = 0x34;
  constexpr dsp56300::Word absolute = 0x30;
  constexpr dsp56300::Word load = 0x8000;
  constexpr dsp56300::Word c_data = 0xffffbd;
  constexpr dsp56300::Word c_direction = 0xffffbe;
  constexpr dsp56300::Word c_function = 0xffffbf;
  constexpr dsp56300::Word f_data = 0xfffff5;
  constexpr dsp56300::Word f_direction = 0xfffff6;
  constexpr dsp56300::Word f_function = 0xfffff7;
  constexpr dsp56300::Word sink = 0xc000;
  std::vector<dsp56300::Word> program;
  const auto put = [&](const blofeld::vi::dsp::Ports::Write access) {
    const auto store = access.space == dsp56300::Space::x ? isa::Opcode::move_x_effective
                                                          : isa::Opcode::move_y_effective;
    program.push_back(isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | load);
    program.push_back(access.value);
    program.push_back(static_cast<dsp56300::Word>(store) | (absolute << 8U));
    program.push_back(access.address);
  };
  const auto control = [&](const dsp56300::Word value) {
    put({.space = dsp56300::Space::x, .address = c_data, .value = value});
  };
  constexpr dsp56300::Word directions = 0xf00;
  constexpr dsp56300::Word inputs = 0xff;
  constexpr dsp56300::Word word_output = 0x4ff;
  constexpr dsp56300::Word upper_inputs = 0x300;
  put({.space = dsp56300::Space::x, .address = c_direction, .value = directions});
  put({.space = dsp56300::Space::x, .address = c_function, .value = inputs});
  put({.space = dsp56300::Space::y, .address = f_function, .value = upper_inputs});
  put({.space = dsp56300::Space::y, .address = f_direction, .value = word_output});
  put({.space = dsp56300::Space::y, .address = f_data, .value = 0});
  constexpr dsp56300::Word command_high = 0x700;
  constexpr dsp56300::Word command_low = 0x500;
  constexpr dsp56300::Word address_low = 0x900;
  constexpr dsp56300::Word address_high = 0xb00;
  control(command_high);
  control(command_low);
  control(command_high);
  put({.space = dsp56300::Space::y, .address = f_data, .value = 1});
  control(address_low);
  control(address_high);
  for (unsigned byte = 0; byte < 3; ++byte) {
    control(address_low);
    put({.space = dsp56300::Space::y, .address = f_data, .value = 0});
    control(address_high);
  }
  control(upper_inputs);
  constexpr dsp56300::Word active_read = 0x200;
  constexpr dsp56300::Word receive_inputs = 0x3ff;
  constexpr dsp56300::Word single_output = 0x400;
  control(active_read);
  put({.space = dsp56300::Space::y, .address = f_function, .value = receive_inputs});
  put({.space = dsp56300::Space::y, .address = f_direction, .value = single_output});
  constexpr dsp56300::Word y_space = 0x10000;
  constexpr dsp56300::Word x0 = 4;
  constexpr dsp56300::Word data_port = 0x35;
  program.push_back(isa::opcode<isa::Opcode::movep_register_high>()[0] | y_space | (x0 << 8U) |
                    data_port);
  program.push_back(isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0);
  program.push_back(sink);
  const blofeld::vi::MediaImage image(
      blofeld::host::read_file(root / "media.bfnand", blofeld::vi::MediaImage::max_bytes));
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {}, &image);
  const bool matched = report.failure && report.failure->space == 'P' &&
                       report.failure->address == sink && report.failure->value == 1;
  if (!matched && report.failure) {
    const auto &failure = *report.failure;
    std::cerr << "pc=" << failure.pc << " address=" << failure.address << " value=" << failure.value
              << " code=" << failure.code << " reason=" << failure.reason << '\n';
  }
  require(matched,
          "Native firmware register transactions read an actual image byte through Port F");
}

void consecutive_receive_registers() {
  namespace isa = dsp56300::isa;
  constexpr dsp56300::Word receive_port = 0x14;
  constexpr dsp56300::Word a0 = 8;
  constexpr dsp56300::Word r0 = 16;
  constexpr dsp56300::Word absolute = 0x30;
  constexpr dsp56300::Word control = 0xffff91;
  constexpr dsp56300::Word receive = 0xffff94;
  constexpr dsp56300::Word fifo24 = 0x29;
  constexpr auto read_a0 =
      isa::opcode<isa::Opcode::movep_register_low>()[0] | (a0 << 8U) | receive_port;
  constexpr auto read_r0 =
      isa::opcode<isa::Opcode::movep_register_low>()[0] | (r0 << 8U) | receive_port;
  constexpr auto store =
      isa::opcode<isa::Opcode::movep_program_low>()[0] | (absolute << 8U) | receive_port;
  constexpr auto program = isa::program(std::array{read_a0, read_r0, store, dsp56300::Word{0}},
                                        isa::opcode<isa::Opcode::jmp_absolute_short>());
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  require(shi.write({.address = control, .value = fifo24}) && shi.select(true).has_value(),
          "Configure actual 24-bit boot reception");
  require(shi.receive(1) && shi.receive(0) && shi.receive(isa::stop[0]),
          "Queue separate length, destination and code words");
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
  dsp56300::Word word = 0;
  require(!report.failure && report.instructions == 5 && !shi.read(receive, word),
          "Each native MOVEP consumes exactly one word across A0, R0 and program destinations");
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

int main(const int argc, char **argv) try {
  require(argc == 3, "Supply artifact and fixture directories");
  native_media_read(argv[2]);
  native_clock_register();
  native_memory_switch();
  consecutive_receive_registers();
  reset_running_core();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
