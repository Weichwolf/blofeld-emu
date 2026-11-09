// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/execute.hpp"
#include "dsp/ports.hpp"
#include "execution.hpp"
#include "files/read.hpp"
#include "io/dsp_reset.hpp"
#include "io/pcm_output.hpp"
#include "io/shi.hpp"
#include "storage/media_image.hpp"
#include "support/isa.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <exception>
#include <filesystem>
#include <iostream>
#include <span>
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

void unknown_boot_tail() {
  constexpr dsp56300::Word next = 0xff0001;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(dsp56300::isa::nop, shi, reset, {});
  const bool correct =
      report.instructions == 1 && report.failure &&
      report.failure->operation == blofeld::vi::Access::read && report.failure->space == 'P' &&
      report.failure->pc == next && report.failure->address == next &&
      report.failure->code == static_cast<std::uint32_t>(dsp56300::FaultKind::memory_read) &&
      report.stopped_pc && *report.stopped_pc == next;
  if (!correct && report.failure) {
    std::cerr << "Boot boundary: retired=" << report.instructions
              << " operation=" << static_cast<unsigned>(report.failure->operation)
              << " pc=" << std::hex << report.failure->pc << " address=" << report.failure->address
              << " code=" << report.failure->code << std::dec << '\n';
  }
  require(correct,
          "A known boot prefix executes before an unprovided ROM word fails at its actual PC");
}

template <dsp56300::Word reserved> void native_reserved_y_read() {
  namespace isa = dsp56300::isa;
  constexpr dsp56300::Word absolute = 0x30;
  constexpr dsp56300::Word read = 0x8000;
  constexpr dsp56300::Word x0 = 4;
  constexpr dsp56300::Word marker = 0xc000;
  constexpr auto load = isa::opcode<isa::Opcode::move_y_effective>()[0] | (absolute << 8U) | read;
  constexpr auto store = isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0;
  constexpr auto program = isa::program(std::array{load, reserved, store, marker});
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
  require(report.failure && report.failure->operation == blofeld::vi::Access::write &&
              report.failure->space == 'P' && report.failure->address == marker &&
              report.failure->value == 0,
          "The native Y load executes before its result reaches the independent fault sink");
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

void native_dma_program_write() {
  namespace isa = dsp56300::isa;
  constexpr dsp56300::Word immediate = 0x34;
  constexpr dsp56300::Word absolute = 0x30;
  constexpr dsp56300::Word load = 0x8000;
  constexpr dsp56300::Word x0 = 4;
  constexpr dsp56300::Word source = 0x40;
  constexpr dsp56300::Word function = 0x100;
  constexpr dsp56300::Word marker = 0xc000;
  constexpr dsp56300::Word replacement = 0x123456;
  constexpr dsp56300::Word source_port = 0x2f;
  constexpr dsp56300::Word destination_port = 0x2e;
  constexpr dsp56300::Word count_port = 0x2d;
  constexpr dsp56300::Word control_port = 0x2c;
  constexpr dsp56300::Word software_x_to_p = 0x980248;
  constexpr auto literal =
      isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | load;
  constexpr auto store_p = isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0;
  constexpr auto store_x = isa::opcode<isa::Opcode::move_x_effective>()[0] | (absolute << 8U);
  constexpr auto call = isa::opcode<isa::Opcode::jsr_effective>()[0] | (absolute << 8U);
  constexpr auto port_store =
      isa::opcode<isa::Opcode::movep_memory_high>()[0] | (immediate << 8U) | load;
  const std::array program{literal,
                           literal,
                           store_p,
                           function,
                           literal,
                           dsp56300::Word{0},
                           store_p,
                           function + 1,
                           literal,
                           isa::rts[0],
                           store_p,
                           function + 2,
                           call,
                           function,
                           literal,
                           replacement,
                           store_x,
                           source,
                           port_store | source_port,
                           source,
                           port_store | destination_port,
                           function + 1,
                           port_store | count_port,
                           dsp56300::Word{0},
                           port_store | control_port,
                           software_x_to_p,
                           call,
                           function,
                           store_p,
                           marker};
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
  if (report.failure && report.failure->address != marker) {
    std::cerr << "DMA native failure pc=" << report.failure->pc
              << " address=" << report.failure->address << " reason=" << report.failure->reason
              << '\n';
  }
  require(report.failure && report.failure->space == 'P' && report.failure->address == marker &&
              report.failure->value == replacement,
          "A real software DMA transfer changes a previously translated function's literal");
}

void native_dma_esai_write() {
  namespace isa = dsp56300::isa;
  using dsp56300::Word;
  constexpr Word immediate = 0x34;
  constexpr Word absolute = 0x30;
  constexpr Word load = 0x8000;
  constexpr Word x0 = 4;
  constexpr Word source = 0x40;
  constexpr Word marker = 0xc000;
  constexpr Word payload = 0x123456;
  constexpr Word software_x_to_y = 0x980244;
  constexpr Word done_port = 0x34;
  constexpr Word all_done = 0x3f;
  std::vector<Word> program;
  const auto put = [&](const blofeld::vi::dsp::Ports::Write access) {
    const auto store = access.space == dsp56300::Space::x ? isa::Opcode::move_x_effective
                                                          : isa::Opcode::move_y_effective;
    program.push_back(isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | load);
    program.push_back(access.value);
    program.push_back(static_cast<Word>(store) | (absolute << 8U));
    program.push_back(access.address);
  };
  using Write = blofeld::vi::dsp::Ports::Write;
  constexpr std::array configuration{
      Write{.space = dsp56300::Space::y, .address = 0xffff9f, .value = 0x83b},
      Write{.space = dsp56300::Space::y, .address = 0xffff9e, .value = 0xebc},
      Write{.space = dsp56300::Space::y, .address = 0xffff96, .value = 0x640301},
      Write{.space = dsp56300::Space::y, .address = 0xffff95, .value = 0x7d01},
      Write{.space = dsp56300::Space::x, .address = source, .value = payload},
      Write{.space = dsp56300::Space::x, .address = 0xffffef, .value = source},
      Write{.space = dsp56300::Space::x, .address = 0xffffee, .value = 0xffff80},
      Write{.space = dsp56300::Space::x, .address = 0xffffed, .value = 0},
      Write{.space = dsp56300::Space::x, .address = 0xffffec, .value = software_x_to_y}};
  for (const auto access : configuration) { put(access); }
  program.push_back(isa::opcode<isa::Opcode::movep_register_high>()[0] | (x0 << 8U) | done_port);
  program.push_back(isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0);
  program.push_back(marker);
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
  require(report.failure && report.failure->space == 'P' && report.failure->address == marker &&
              report.failure->value == all_done,
          "Native software DMA completes only after the connected ESAI TX register accepts data");
}

void native_dma_dax_write() {
  namespace isa = dsp56300::isa;
  using dsp56300::Word;
  constexpr Word immediate = 0x34;
  constexpr Word absolute = 0x30;
  constexpr Word load = 0x8000;
  constexpr Word x0 = 4;
  constexpr Word marker = 0xc000;
  constexpr Word done_port = 0x34;
  constexpr Word all_done = 0x3f;
  constexpr auto literal =
      isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | load;
  constexpr auto store = isa::opcode<isa::Opcode::move_x_effective>()[0] | (absolute << 8U);
  using Write = blofeld::vi::dsp::Ports::Write;
  constexpr std::array configuration{Write{.address = 0xffffd1, .value = 0x9400},
                                     Write{.address = 0xffffd2, .value = 0x123456},
                                     Write{.address = 0xffffd3, .value = 0xfedcba},
                                     Write{.address = 0xffffd0, .value = 8},
                                     Write{.address = 0xffffd7, .value = 3},
                                     Write{.address = 0xffffd6, .value = 3},
                                     Write{.address = 0x40, .value = 0x345678},
                                     Write{.address = 0xffffef, .value = 0x40},
                                     Write{.address = 0xffffee, .value = 0xffffd2},
                                     Write{.address = 0xffffed, .value = 0},
                                     Write{.address = 0xffffec, .value = 0x980240}};
  std::vector<Word> program;
  for (const auto access : configuration) {
    const std::array words{literal, access.value, store, access.address};
    program.insert(program.end(), words.begin(), words.end());
  }
  program.push_back(isa::opcode<isa::Opcode::movep_register_high>()[0] | (x0 << 8U) | done_port);
  program.push_back(isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0);
  program.push_back(marker);
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
  require(report.failure && report.failure->space == 'P' && report.failure->address == marker &&
              report.failure->value == all_done,
          "Native writes bind DAX and Port D, then software DMA transfers an actual XADR word");
}

void native_timer_events() {
  namespace isa = dsp56300::isa;
  using dsp56300::Word;
  constexpr Word immediate = 0x34;
  constexpr Word absolute = 0x30;
  constexpr Word write = 0x8000;
  constexpr Word x0 = 4;
  constexpr Word marker = 0xc000;
  constexpr Word control_port = 15;
  constexpr Word load_port = 14;
  constexpr Word compare_port = 13;
  constexpr Word expected = 0x200001;
  constexpr auto configure =
      isa::opcode<isa::Opcode::movep_memory_low>()[0] | (immediate << 8U) | write;
  constexpr auto read =
      isa::opcode<isa::Opcode::movep_register_low>()[0] | (x0 << 8U) | control_port;
  constexpr auto sink = isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0;
  constexpr std::array configuration{configure | load_port,
                                     Word{0},
                                     configure | compare_port,
                                     Word{3},
                                     configure | control_port,
                                     Word{1}};
  std::vector<Word> program(configuration.begin(), configuration.end());
  program.insert(program.end(), 32, isa::nop[0]);
  program.push_back(read);
  program.push_back(sink);
  program.push_back(marker);
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
  require(report.failure && report.failure->space == 'P' && report.failure->address == marker &&
              report.failure->value == expected,
          "Native execution reaches a real timer compare deadline and reads its sticky TCF");
}

void native_timer_interrupt(const bool waiting) {
  namespace isa = dsp56300::isa;
  using dsp56300::Word;
  constexpr Word immediate = 0x34;
  constexpr Word absolute = 0x30;
  constexpr Word write = 0x8000;
  constexpr Word x0 = 4;
  constexpr Word vba_selector = 0x10;
  constexpr Word vba = 0x200;
  constexpr Word vector = vba + 0x54;
  constexpr Word handler = 0x100;
  constexpr Word result_address = 0x40;
  constexpr Word marker = 0xc000;
  constexpr Word timer_port = 15;
  constexpr Word load_port = 14;
  constexpr Word compare_port = 13;
  constexpr Word priority_port = 0x3e;
  constexpr Word priority = 0x100;
  constexpr Word timer_enable = 5;
  constexpr Word clear_mask = 0xfc;
  constexpr Word repeat = 64;
  constexpr Word sleeping_compare = 512;
  constexpr auto literal =
      isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | write;
  constexpr auto store_p = isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0;
  constexpr auto store_x = isa::opcode<isa::Opcode::move_x_effective>()[0] | (absolute << 8U);
  constexpr auto port = isa::opcode<isa::Opcode::movep_memory_low>()[0] | (immediate << 8U) | write;
  constexpr auto read_status =
      isa::opcode<isa::Opcode::movep_register_low>()[0] | (x0 << 8U) | timer_port;
  constexpr auto unmask = isa::opcode<isa::Opcode::andi_control>()[0] | (clear_mask << 8U);
  constexpr auto call = isa::opcode<isa::Opcode::jsr_effective>()[0] | (absolute << 8U);
  std::vector<Word> program;
  const auto put_code = [&](const Word address, const Word value) {
    const std::array words{literal, value, store_p, address};
    program.insert(program.end(), words.begin(), words.end());
  };
  put_code(vector, call);
  put_code(vector + 1, handler);
  constexpr std::array interrupt_handler{
      read_status, store_x, result_address, port | timer_port, Word{0}, isa::rti[0]};
  for (unsigned index = 0; index < interrupt_handler.size(); ++index) {
    put_code(handler + index, interrupt_handler[index]);
  }
  const std::array setup{
      literal,
      vba,
      isa::opcode<isa::Opcode::movec_register>()[0] | (x0 << 8U) | write | vba_selector,
      isa::opcode<isa::Opcode::movep_memory_high>()[0] | (immediate << 8U) | write | priority_port,
      priority,
      port | load_port,
      Word{0},
      port | compare_port,
      waiting ? sleeping_compare : Word{16},
      port | timer_port,
      timer_enable};
  program.insert(program.end(), setup.begin(), setup.end());
  if (waiting) {
    program.push_back(unmask);
    program.push_back(isa::wait[0]);
  } else {
    program.push_back(isa::opcode<isa::Opcode::rep_immediate>()[0] | (repeat << 8U));
    program.push_back(isa::nop[0]);
    constexpr unsigned masked_work = 512;
    program.insert(program.end(), masked_work, isa::opcode<isa::Opcode::add_x0_b>()[0]);
    program.push_back(unmask);
    program.push_back(isa::nop[0]);
    program.push_back(isa::nop[0]);
  }
  program.push_back(store_x | write);
  program.push_back(result_address);
  program.push_back(store_p);
  program.push_back(marker);
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
  constexpr Word boot = 0xff0000;
  require(report.failure && report.failure->space == 'P' && report.failure->address == marker &&
              report.failure->value == timer_enable &&
              report.failure->pc == boot + program.size() - 2,
          "A real timer IRQ observes masks/REP or wakes WAIT, uses VBA, acknowledges TCF and "
          "returns via RTI");
}

void native_audio_dma(const bool throwing) {
  namespace isa = dsp56300::isa;
  using dsp56300::Word;
  constexpr Word immediate = 0x34;
  constexpr Word absolute = 0x30;
  constexpr Word write = 0x8000;
  constexpr Word base = 0x660;
  constexpr Word sign = 0x800000;
  constexpr Word boot = 0xff0000;
  constexpr auto literal =
      isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | write;
  std::vector<Word> program;
  const auto put = [&](const blofeld::vi::dsp::Ports::Write access) {
    const auto opcode = access.space == dsp56300::Space::x ? isa::Opcode::move_x_effective
                                                           : isa::Opcode::move_y_effective;
    const std::array words{
        literal, access.value, static_cast<Word>(opcode) | (absolute << 8U), access.address};
    program.insert(program.end(), words.begin(), words.end());
  };
  for (unsigned index = 0; index < 64; ++index) {
    put({.address = base + index, .value = ((index + 1) << 16U) | ((index & 1) != 0 ? sign : 0)});
  }
  using Write = blofeld::vi::dsp::Ports::Write;
  constexpr std::array configuration{
      Write{.space = dsp56300::Space::y, .address = 0xffff9f, .value = 0x83b},
      Write{.space = dsp56300::Space::y, .address = 0xffff9e, .value = 0xebc},
      Write{.space = dsp56300::Space::y, .address = 0xffff96, .value = 0x640301},
      Write{.space = dsp56300::Space::y, .address = 0xffff95, .value = 0x7d00},
      Write{.space = dsp56300::Space::y, .address = 0xffff80, .value = 0x10000},
      Write{.address = 0xfffff3, .value = 1},
      Write{.address = 0xfffff2, .value = 0xffffc1},
      Write{.address = 0xffffef, .value = base + 1},
      Write{.address = 0xffffee, .value = 0xffff80},
      Write{.address = 0xffffed, .value = 0xfc0},
      Write{.address = 0xffffec, .value = 0xaab604},
      Write{.space = dsp56300::Space::y, .address = 0xffff95, .value = 0x7d01}};
  for (const auto access : configuration) { put(access); }
  const auto loop = boot + static_cast<Word>(program.size());
  program.push_back(isa::opcode<isa::Opcode::jmp_effective>()[0] | (absolute << 8U));
  program.push_back(loop);

  struct Consumer {
    std::stop_source stop;
    bool correct = false;
    bool submitted = false;
    bool throwing = false;
    unsigned closed = 0;
    std::thread::id thread = std::this_thread::get_id();
    bool closed_on_owner = false;
  } consumer;

  consumer.throwing = throwing;

  const blofeld::vi::PcmOutput output{
      .owner = &consumer,
      .write =
          [](void *owner, const blofeld::vi::PcmOutput::Block block, const std::stop_token &) {
            auto &result = *static_cast<Consumer *>(owner);
            result.submitted = true;
            if (result.throwing) { throw std::runtime_error("Expected PCM consumer exception"); }
            result.correct = block.frames.size() == 16 &&
                             block.ticks.size() == block.frames.size() && block.first_frame == 0 &&
                             block.underruns == 0;
            constexpr std::int32_t scale = 65536;
            constexpr std::int32_t negative = 8388608;
            for (unsigned index = 0; index < block.frames.size(); ++index) {
              const auto left = static_cast<std::int32_t>((index * 2) + 1) * scale;
              const auto right = (static_cast<std::int32_t>((index * 2) + 2) * scale) - negative;
              result.correct = result.correct &&
                               block.frames[index] == blofeld::vi::PcmOutput::Frame{left, right} &&
                               block.ticks.size() == block.frames.size() &&
                               block.ticks[index] > 0 &&
                               (index == 0 || block.ticks[index] > block.ticks[index - 1]);
            }
            result.stop.request_stop();
            return true;
          },
      .close =
          [](void *owner) noexcept {
            auto &result = *static_cast<Consumer *>(owner);
            ++result.closed;
            result.closed_on_owner = result.thread == std::this_thread::get_id();
          }};
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::ExecutionReport report;
  bool caught = false;
  try {
    report =
        blofeld::vi::dsp::execute(program, shi, reset, consumer.stop.get_token(), nullptr, output);
  } catch (const std::runtime_error &) { caught = true; }
  require(consumer.submitted && caught == throwing && consumer.closed == 1 &&
              consumer.closed_on_owner,
          "PCM ownership closes once on its execution thread after stop or exception");
  require(throwing || (!report.failure && consumer.correct),
          "Native guest RAM, real timed DMA and ESAI deliver sixteen exact stereo frames to the "
          "consumer");
}

void native_transmit_registers() {
  namespace isa = dsp56300::isa;
  constexpr dsp56300::Word immediate = 0x34;
  constexpr dsp56300::Word absolute = 0x30;
  constexpr dsp56300::Word write = 0x8000;
  constexpr dsp56300::Word status_port = 0x11;
  constexpr dsp56300::Word transmit_port = 0x13;
  constexpr dsp56300::Word x0 = 4;
  constexpr dsp56300::Word sink = 0xc000;
  constexpr dsp56300::Word mode = 9;
  constexpr dsp56300::Word first = 0xa591e7;
  constexpr dsp56300::Word second = 0x5391e7;
  constexpr dsp56300::Word expected_status = 0x209;
  constexpr auto store =
      isa::opcode<isa::Opcode::movep_memory_low>()[0] | (immediate << 8U) | write;
  constexpr auto read = isa::opcode<isa::Opcode::movep_register_low>()[0] | (x0 << 8U);
  constexpr auto output = isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0;
  constexpr auto program = isa::program(std::array{store | status_port,
                                                   mode,
                                                   store | transmit_port,
                                                   first,
                                                   store | transmit_port,
                                                   second,
                                                   read | status_port,
                                                   output,
                                                   sink});
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
  require(report.failure && report.failure->space == 'P' && report.failure->address == sink &&
              report.failure->value == expected_status,
          "Native HTX writes occupy both real transmit stages; HCSR reports HTX full");
}

void native_interrupt_priorities() {
  namespace isa = dsp56300::isa;
  constexpr dsp56300::Word immediate = 0x34;
  constexpr dsp56300::Word absolute = 0x30;
  constexpr dsp56300::Word write = 0x8000;
  constexpr dsp56300::Word peripheral_port = 0x3e;
  constexpr dsp56300::Word core_port = 0x3f;
  constexpr dsp56300::Word x0 = 4;
  constexpr dsp56300::Word sink = 0xc000;
  constexpr dsp56300::Word peripheral_expected = 0x030fce;
  constexpr dsp56300::Word core_expected = 0xfffffe;
  constexpr auto configure =
      isa::opcode<isa::Opcode::movep_memory_high>()[0] | (immediate << 8U) | write;
  constexpr auto clear = isa::opcode<isa::Opcode::bclr_port_high>()[0];
  constexpr auto read = isa::opcode<isa::Opcode::movep_register_high>()[0] | (x0 << 8U);
  constexpr auto output = isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0;
  for (const auto port : {peripheral_port, core_port}) {
    const std::array program{
        configure | port, dsp56300::word_mask, clear | (port << 8U), read | port, output, sink};
    blofeld::vi::Shi shi;
    blofeld::vi::DspReset reset;
    const auto report = blofeld::vi::dsp::execute(program, shi, reset, {});
    const auto expected = port == core_port ? core_expected : peripheral_expected;
    require(report.failure && report.failure->space == 'P' && report.failure->address == sink &&
                report.failure->value == expected,
            "Native priority writes, bit updates and reads retain only implemented fields");
    const std::array reset_program{read | port, output, sink};
    const auto after_reset = blofeld::vi::dsp::execute(reset_program, shi, reset, {});
    require(after_reset.failure && after_reset.failure->address == sink &&
                after_reset.failure->value == 0,
            "A new reset instance starts with every programmable priority disabled");
  }
}

enum class MediaCase : std::uint8_t { read, released_read, erase, program };

struct MediaGuest {
  using Word = dsp56300::Word;
  using Write = blofeld::vi::dsp::Ports::Write;
  static constexpr Word c_data = 0xffffbd, f_data = 0xfffff5, absolute = 0x30;
  static constexpr Word sink = 0xc000;
  static constexpr Word f_function = 0xfffff7, f_direction = 0xfffff6;
  static constexpr std::array<Word, 3> payload{0xa5, 0x36, 0};
  std::vector<Word> code;

  MediaGuest() {
    constexpr std::array setup{
        Write{.address = 0xffffbe, .value = 0xf00},
        Write{.address = 0xffffbf, .value = 0xff},
        Write{.space = dsp56300::Space::y, .address = 0xfffff7, .value = 0x300},
        Write{.space = dsp56300::Space::y, .address = 0xfffff6, .value = 0x4ff}};
    for (const auto access : setup) { put(access); }
  }

  void put(const Write access) {
    namespace isa = dsp56300::isa;
    constexpr Word immediate = 0x34;
    constexpr Word load = 0x8000;
    const auto store = access.space == dsp56300::Space::x ? isa::Opcode::move_x_effective
                                                          : isa::Opcode::move_y_effective;
    const std::array words{isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) |
                               load,
                           access.value,
                           static_cast<Word>(store) | (absolute << 8U),
                           access.address};
    code.insert(code.end(), words.begin(), words.end());
  }

  void control(const Word value) { put({.address = c_data, .value = value}); }

  void command(const Word value) {
    constexpr Word high = 0x700;
    constexpr Word low = 0x500;
    control(high);
    put({.space = dsp56300::Space::y, .address = f_data, .value = value});
    control(low);
    control(high);
  }

  void address(const Word value) {
    constexpr Word low = 0x900;
    constexpr Word high = 0xb00;
    control(low);
    put({.space = dsp56300::Space::y, .address = f_data, .value = value});
    control(high);
  }

  void data(const Word value) {
    constexpr Word low = 0x100;
    constexpr Word high = 0x300;
    control(low);
    put({.space = dsp56300::Space::y, .address = f_data, .value = value});
    control(high);
  }

  void input() {
    constexpr Word idle = 0x300;
    constexpr Word active_read = 0x200;
    control(idle);
    control(active_read);
    constexpr Word receive_inputs = 0x3ff;
    constexpr Word single_output = 0x400;
    put({.space = dsp56300::Space::y, .address = f_function, .value = receive_inputs});
    put({.space = dsp56300::Space::y, .address = f_direction, .value = single_output});
  }

  void ready() {
    namespace isa = dsp56300::isa;
    constexpr Word boot = 0xff0000;
    constexpr Word ready_port = 0x0b;
    constexpr Word ready_bit = 12;
    const auto target = boot + static_cast<Word>(code.size());
    code.push_back(isa::opcode<isa::Opcode::jclr_port_low>()[0] | (ready_port << 8U) | ready_bit);
    code.push_back(target);
  }

  void observe() {
    namespace isa = dsp56300::isa;
    constexpr Word y_space = 0x10000;
    constexpr Word x0 = 4;
    constexpr Word data_port = 0x35;
    code.push_back(isa::opcode<isa::Opcode::movep_register_high>()[0] | y_space | (x0 << 8U) |
                   data_port);
    code.push_back(isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0);
    code.push_back(sink);
  }

  void transaction(const MediaCase operation) {
    const bool erasing = operation == MediaCase::erase;
    const bool programming = operation == MediaCase::program;
    const bool writing = erasing || programming;
    constexpr dsp56300::Word erase_command = 0x60;
    constexpr dsp56300::Word program_command = 0x80;
    if (programming) {
      command(0); // Explicitly select the first main-array half.
      command(program_command);
    } else {
      command(erasing ? erase_command : 0);
    }
    constexpr std::array<dsp56300::Word, 4> read_address{1, 0, 0, 0};
    constexpr std::array<dsp56300::Word, 4> program_address{0, 0, 0, 0};
    constexpr std::array<dsp56300::Word, 3> erase_address{31, 0, 0};
    const auto page_address =
        programming ? std::span<const Word>(program_address) : std::span<const Word>(read_address);
    const auto addresses = erasing ? std::span<const Word>(erase_address) : page_address;
    for (const auto byte : addresses) { address(byte); }
    if (programming) {
      for (const auto byte : payload) { data(byte); }
    }
    if (writing) {
      constexpr dsp56300::Word confirm_erase = 0xd0;
      constexpr dsp56300::Word confirm_program = 0x10;
      constexpr dsp56300::Word status = 0x70;
      command(erasing ? confirm_erase : confirm_program);
      command(status);
      input(); // Hold a busy status sample through actual completion.
    }
    ready();
    if (!writing) { input(); }
    if (operation == MediaCase::released_read) {
      constexpr Word released = 0x300;
      control(released);
    }
    observe();
  }
};

void native_media(const std::filesystem::path &root, const MediaCase operation) {
  const bool erasing = operation == MediaCase::erase;
  const bool programming = operation == MediaCase::program;
  const bool writing = erasing || programming;
  MediaGuest guest;
  guest.transaction(operation);
  blofeld::vi::MediaImage image(
      blofeld::host::read_file(root / (writing ? "media-writable.bfnand" : "media.bfnand"),
                               blofeld::vi::MediaImage::max_bytes));
  if (programming) { require(image.erase(0) == true, "Prepare a qualified erased program target"); }
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(guest.code, shi, reset, {}, &image);
  constexpr dsp56300::Word complete_status = 0xc0;
  const auto expected = writing ? complete_status : 1U;
  const bool matched = report.failure && report.failure->space == 'P' &&
                       report.failure->address == MediaGuest::sink &&
                       report.failure->value == expected;
  if (!matched && report.failure) {
    const auto &failure = *report.failure;
    std::cerr << "pc=" << failure.pc << " address=" << failure.address << " value=" << failure.value
              << " code=" << failure.code << " reason=" << failure.reason << '\n';
  }
  std::uint8_t byte = 0;
  constexpr std::uint8_t erased_byte = 0xff;
  require(!erasing || (image.read({}, byte) && byte == erased_byte),
          "The native erase operation changes the real selected image");
  if (programming) {
    for (unsigned index = 0; index < MediaGuest::payload.size(); ++index) {
      require(image.read({.column = index}, byte) && byte == MediaGuest::payload[index],
              "Native data/confirm writes program every actual payload byte");
    }
  }
  require(matched, "Native media operations refresh held status/data at actual completion");
}

void native_missing_media(const std::filesystem::path &root) {
  constexpr dsp56300::Word column = 17;
  constexpr dsp56300::Word page = 32;
  MediaGuest guest;
  guest.command(0);
  for (const auto byte : std::array{column, page, dsp56300::Word{0}, dsp56300::Word{0}}) {
    guest.address(byte);
  }
  guest.ready();
  guest.input();
  guest.observe();
  blofeld::vi::MediaImage image(
      blofeld::host::read_file(root / "media.bfnand", blofeld::vi::MediaImage::max_bytes));
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = blofeld::vi::dsp::execute(guest.code, shi, reset, {}, &image);
  require(report.failure && report.failure->operation == blofeld::vi::Access::write &&
              report.failure->space == 'X' && report.failure->address == MediaGuest::c_data &&
              report.failure->media && report.failure->media->page == page &&
              report.failure->media->column == column,
          "Native missing-data failure retains both DSP register access and physical NAND address");
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
  require(shi.transfer(1).accepted && shi.transfer(0).accepted &&
              shi.transfer(isa::stop[0]).accepted,
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
  require(shi.transfer(isa::stop[0]).accepted,
          "Send a real instruction after the new boot enables SHI");
  worker.join();
  require(!report.exception && !report.failure && report.instructions > 0,
          "A fresh Core boots, loads PRAM and executes STOP after reset");
}
}

int main(const int argc, char **argv) try {
  require(argc == 3, "Supply artifact and fixture directories");
  native_media(argv[2], MediaCase::read);
  native_media(argv[2], MediaCase::released_read);
  native_media(argv[2], MediaCase::erase);
  native_media(argv[2], MediaCase::program);
  native_missing_media(argv[2]);
  native_clock_register();
  unknown_boot_tail();
  constexpr dsp56300::Word delay_boundary = 0x16000;
  native_reserved_y_read<delay_boundary>();
  native_reserved_y_read<dsp56300::word_mask>();
  native_transmit_registers();
  native_interrupt_priorities();
  native_memory_switch();
  native_dma_program_write();
  native_dma_esai_write();
  native_dma_dax_write();
  native_audio_dma(false);
  native_audio_dma(true);
  native_timer_events();
  native_timer_interrupt(false);
  native_timer_interrupt(true);
  consecutive_receive_registers();
  reset_running_core();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
