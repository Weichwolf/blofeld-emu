// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/coupling.hpp"
#include "dsp/execute.hpp"
#include "dsp/io.hpp"
#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#endif
#include "dsp/ports.hpp"
#include "dsp/program.hpp"
#include "execution.hpp"
#include "files/read.hpp"
#include "io/coupling.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "mcu/epoch.hpp"
#include "storage/media_image.hpp"
#include "support/isa.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <exception>
#include <filesystem>
#include <future>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

blofeld::vi::dsp::Program prepare(const blofeld::vi::dsp::BootRom boot) {
  using namespace dsp56300;
  constexpr std::array entries{blofeld::vi::dsp::BootRom::main_first};
  std::vector<CodeSpan> code{{.address = entries[0], .words = boot.main}};
  std::vector<Word> failed{entries[0] + static_cast<Word>(boot.main.size())};
  if (!boot.eeprom.empty()) {
    constexpr Word eeprom = blofeld::vi::dsp::BootRom::eeprom_first;
    code.push_back({.address = eeprom, .words = boot.eeprom});
    failed.push_back(eeprom + static_cast<Word>(boot.eeprom.size()));
  }
  const std::array variants{
      CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = failed}};
  const std::array choices{
      blofeld::vi::dsp::ProgramChoice{.variant = 0, .resumes = entries, .words = {}}};
  return blofeld::app::aot::prepare_dsp_program({.variants = variants}, choices);
}

blofeld::vi::dsp::Program
prepare_loaded(const blofeld::vi::dsp::BootRom boot,
               const std::span<const dsp56300::CodeSpan> loaded,
               const dsp56300::Word handoff,
               const std::span<const dsp56300::InterruptRequest> interrupts = {}) {
  using namespace dsp56300;
  constexpr Word first = blofeld::vi::dsp::BootRom::main_first;
  const auto split =
      handoff >= first && handoff - first < boot.main.size() ? handoff - first : boot.main.size();
  const std::array initial{CodeSpan{.address = first, .words = boot.main.first(split)}};
  std::vector<CodeSpan> final(loaded.begin(), loaded.end());
  if (split < boot.main.size()) {
    final.push_back({.address = handoff, .words = boot.main.subspan(split)});
  }
  constexpr std::array initial_entries{first};
  const std::array final_entries{handoff};
  std::vector<Word> failed;
  for (const auto request : interrupts) { failed.push_back(request.vector); }
  failed.push_back(first + static_cast<Word>(boot.main.size()));
  const std::array final_failed{first + static_cast<Word>(boot.main.size())};
  const std::array variants{
      CodeVariant{.id = 0, .code = initial, .entries = initial_entries, .failed_fetches = failed},
      CodeVariant{
          .id = 1, .code = final, .entries = final_entries, .failed_fetches = final_failed}};
  const std::array choices{
      blofeld::vi::dsp::ProgramChoice{.variant = 0, .resumes = initial_entries, .words = {}},
      blofeld::vi::dsp::ProgramChoice{.variant = 1, .resumes = final_entries, .words = {}}};
  const std::array transfers{CodeTransition{.from = 0, .to = 1, .entry = handoff}};
  return blofeld::app::aot::prepare_dsp_program(
      {.variants = variants}, choices, {}, interrupts, transfers);
}

blofeld::vi::ExecutionReport execute(const blofeld::vi::dsp::BootRom boot,
                                     blofeld::vi::Shi &shi,
                                     blofeld::vi::DspReset &reset,
                                     const std::stop_token &stop,
                                     blofeld::vi::MediaImage *media = nullptr,
                                     const blofeld::vi::PcmOutput output = {},
                                     blofeld::vi::PeerPort *peer = nullptr,
                                     const blofeld::vi::dsp::Program *prepared = nullptr) {
  const auto run = [&](const blofeld::vi::dsp::Program &image) {
    std::cerr << "Execute prepared fixture: words=" << boot.main.size() << '\n';
    return blofeld::vi::dsp::execute(
        boot, shi, reset, stop, media, output, peer, nullptr, nullptr, nullptr, &image);
  };
  return prepared != nullptr ? run(*prepared) : run(prepare(boot));
}

void native_clock_register() {
#if defined(BLOFELD_CENSUS)
  using blofeld::vi::Census;
  Census census(true);
  census.interval(1, std::numeric_limits<std::uint64_t>::max());
  const Census::Scope owner(census, Census::Actor::dsp);
#endif
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
  const auto report = execute({.main = program, .eeprom = {}}, shi, reset, {});
  require(report.failure && report.failure->operation == blofeld::vi::Access::write &&
              report.failure->space == 'P' && report.failure->address == unmapped &&
              report.failure->value == expected,
          "Native MOVEP/BCLR share PCTL and native RAM stores/loads retain its actual value");
#if defined(BLOFELD_CENSUS)
  Census::Counts counts{};
  for (const auto &phase : census.intervals(Census::Actor::dsp)) {
    for (unsigned index = 0; index < counts.size(); ++index) { counts[index] += phase[index]; }
  }
  require(counts[static_cast<unsigned>(Census::Event::retired)] == report.instructions &&
              counts[static_cast<unsigned>(Census::Event::fault)] == 1 &&
              counts[static_cast<unsigned>(Census::Event::audio_change)] >= 1,
          "Actual native I/O yields and faults retain exact whole-run retirement accounting");
#endif
}

void native_timed_peer_input() {
  namespace isa = dsp56300::isa;
  using dsp56300::Word;
  constexpr Word data = 0xffffbd;
  constexpr Word direction = 0xffffbe;
  constexpr Word function = 0xffffbf;
  constexpr Word immediate = 0x34;
  constexpr Word absolute = 0x30;
  constexpr Word loading = 0x8000;
  constexpr Word x0 = 4;
  constexpr Word marker = 0xc000;
  constexpr Word byte_mask = 0xff;
  constexpr Word before = 0x12;
  constexpr Word after = 0x34;
  constexpr auto literal =
      isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | loading;
  constexpr auto store = isa::opcode<isa::Opcode::move_x_effective>()[0] | (absolute << 8U);
  constexpr auto read = store | loading;
  constexpr auto sink = isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0;
  constexpr auto program = isa::program(std::array{literal,
                                                   Word{0},
                                                   store,
                                                   data,
                                                   store,
                                                   direction,
                                                   literal,
                                                   byte_mask,
                                                   store,
                                                   function,
                                                   read,
                                                   data,
                                                   sink,
                                                   marker});
  constexpr std::uint64_t later = 1000000;
  for (const std::uint64_t second_tick : {std::uint64_t{1}, later}) {
    blofeld::vi::PeerPort peer;
    require(peer.mcu_data({.mask = byte_mask, .known = byte_mask, .value = before}, 0) &&
                peer.mcu_data({.mask = byte_mask, .known = byte_mask, .value = after}, second_tick),
            "Queue two real MCU data states before native DSP execution");
    blofeld::vi::Shi shi;
    blofeld::vi::DspReset reset;
    const auto report =
        execute({.main = program, .eeprom = {}}, shi, reset, {}, nullptr, {}, &peer);
    const auto expected = second_tick == later ? before : after;
    require(report.failure && report.failure->operation == blofeld::vi::Access::write &&
                report.failure->space == 'P' && report.failure->address == marker &&
                report.failure->value == expected,
            "Native Port C reads use their completed prefix time without leaking future data");
  }
}

std::vector<dsp56300::Word> peer_reply_program() {
  namespace isa = dsp56300::isa;
  using dsp56300::Word;
  constexpr Word immediate = 0x34;
  constexpr Word absolute = 0x30;
  constexpr Word load = 0x8000;
  constexpr Word x0 = 4;
  constexpr Word status_port = 0x11;
  constexpr Word receive_port = 0x14;
  constexpr Word receive_ready = 17;
  constexpr Word boot = 0xff0000;
  constexpr Word data = 0xffffbd;
  constexpr Word direction = 0xffffbe;
  constexpr unsigned delay = 64;
  std::vector<Word> program;
  const auto put = [&](const blofeld::vi::dsp::Ports::Write access) {
    const auto operation = access.space == dsp56300::Space::x ? isa::Opcode::move_x_effective
                                                              : isa::Opcode::move_y_effective;
    program.insert(program.end(),
                   {isa::opcode<isa::Opcode::move_x_effective>()[0] | (immediate << 8U) | load,
                    access.value,
                    static_cast<Word>(operation) | (absolute << 8U),
                    access.address});
  };
  using Write = blofeld::vi::dsp::Ports::Write;
  constexpr std::array configuration{
      Write{.address = 0xffff91, .value = 0x29},
      Write{.address = data, .value = 0x0d},
      Write{.address = direction, .value = 0xff},
      Write{.space = dsp56300::Space::y, .address = 0xffff9d, .value = 0},
      Write{.space = dsp56300::Space::y, .address = 0xffff9e, .value = 0x200}};
  for (const auto access : configuration) { put(access); }
  const auto polling = boot + static_cast<Word>(program.size());
  program.insert(
      program.end(),
      {isa::opcode<isa::Opcode::jclr_port_low>()[0] | (status_port << 8U) | receive_ready,
       polling,
       isa::opcode<isa::Opcode::movep_register_low>()[0] | (x0 << 8U) | receive_port});
  program.insert(program.end(), delay, isa::nop[0]);
  constexpr Word bytes = 16;
  for (Word index = 0; index < bytes; ++index) { put({.address = data, .value = index}); }
  put({.address = direction, .value = 0});
  program.push_back(isa::stop[0]);
  return program;
}

std::optional<blofeld::vi::Coupling::Input>
next_publication(const blofeld::vi::Coupling &coupling,
                 const std::uint64_t serial,
                 const std::chrono::steady_clock::time_point end) {
  while (std::chrono::steady_clock::now() < end) {
    if (const auto input = coupling.input(serial)) { return input; }
    std::this_thread::yield();
  }
  return {};
}

void port_publication_boundary(const blofeld::vi::dsp::Program &program,
                               const bool reset_at_boundary) {
  namespace vi = blofeld::vi;
  constexpr std::uint64_t origin = 180;
  constexpr std::uint8_t mask = 0xff;
  constexpr std::uint8_t report = 0x0d;
  constexpr std::uint8_t payload = 0xa5;
  constexpr unsigned bytes = 16;
  constexpr dsp56300::Word port_c = 0xffffbd;
  constexpr dsp56300::Word direction = port_c + 1;
  constexpr dsp56300::Word driven = 0xfff;
  constexpr dsp56300::Word media_idle = 0x300;
  constexpr dsp56300::Word media_command = 0x700;
  constexpr auto space = dsp56300::Space::x;
  vi::PeerPort peer;
  vi::Shi shi;
  vi::DspReset reset;
  vi::ExecutionTime time;
  vi::MidiPort midi;
  std::stop_source stopping;
  const auto stop = stopping.get_token();
  vi::Coupling coupling(peer, shi, reset, time, midi, stop, vi::Coupling::default_window);
  vi::dsp::Io io(
      {}, shi, reset, stop, nullptr, {}, &peer, &time, &midi, nullptr, program, &coupling);
  dsp56300::Core core(dsp56300::Memory::external(), &io);
  time.release_dsp(origin);
  io.bind(core, 0);
  require(io.write(space, direction, 0, driven) && io.write(space, port_c, 0, media_idle | payload),
          "Configure the actual Port C drivers");
  peer.dsp_data({.mask = mask, .known = mask, .value = report});
  require(peer.dsp_ready(true) && peer.mcu_strobe(true) && peer.mcu_strobe(false),
          "Announce and accept the actual report request");
  constexpr vi::PeerPort::Sample byte{.mask = mask, .known = mask, .value = payload};
  for (unsigned index = 0; index < bytes; ++index) {
    require(io.write(space, port_c, 0, media_idle | payload),
            "Record each equal timed Port C write");
  }
  peer.dsp_data({});
  for (unsigned index = 0; index < bytes - 1; ++index) {
    require(peer.read_dsp_data({.mask = mask, .ticks = origin}) == byte,
            "Read all but the last original reply byte");
  }
  require(io.write(space, port_c, 0, media_idle | report),
          "Drive both Port C groups while the last reply byte remains available");
  const vi::dsp::Coupled coupled(&coupling, io, core);
  std::array<vi::Coupling::Input, 2> seen{};
  bool applied = false;
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds{2};
  std::jthread observer([&] {
    std::uint64_t serial = 0;
    for (auto &sample : seen) {
      const auto input = next_publication(coupling, serial, end);
      if (!input) {
        stopping.request_stop();
        return;
      }
      sample = *input;
      if (serial == 0) {
        if (reset_at_boundary) {
          peer.hold_reset();
          applied = reset.assert_reset();
        } else {
          applied = peer.read_dsp_data({.mask = mask, .ticks = origin}) == byte;
        }
      }
      serial = sample.serial;
      coupling.acknowledge(serial);
    }
  });
  const bool written = io.write(space, port_c, 0, media_command | payload);
  observer.join();
  using Signal = vi::mcu::Epoch::Signal;
  constexpr auto reply_signal = static_cast<std::size_t>(Signal::reply);
  constexpr auto data_signal = static_cast<std::size_t>(Signal::data);
  require(written && applied && seen[0].serial == 1 && seen[1].serial == 2 &&
              seen[0].ticks == origin && seen[1].ticks == origin &&
              seen[0].values[reply_signal] == 1 && (seen[0].values[data_signal] & mask) == payload,
          "A simultaneous media/peer write retains the original first publication");
  require(reset_at_boundary ? seen[1].values[data_signal] == 0 && reset.pending(0)
                            : seen[1].values[reply_signal] == 0 &&
                                  seen[1].values[data_signal] == seen[0].values[data_signal],
          "The second original observation sees reset or the actual last-byte read");
}

void native_reply_time() {
  const auto program = peer_reply_program();
  const auto prepared = prepare({.main = program, .eeprom = {}});
  blofeld::vi::PeerPort peer;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  std::promise<blofeld::vi::ExecutionReport> promise;
  auto completed = promise.get_future();
  std::jthread worker([&](const std::stop_token &stop) {
    blofeld::vi::ExecutionReport report;
    try {
      report =
          execute({.main = program, .eeprom = {}}, shi, reset, stop, nullptr, {}, &peer, &prepared);
    } catch (...) { report.exception = std::current_exception(); }
    promise.set_value(report);
  });
  const auto guard = std::chrono::steady_clock::now() + std::chrono::seconds{2};
  while (!peer.mcu_interrupt() &&
         completed.wait_for(std::chrono::seconds{0}) != std::future_status::ready &&
         std::chrono::steady_clock::now() < guard) {
    std::this_thread::yield();
  }
  const bool accepted = peer.mcu_interrupt() && peer.mcu_strobe(true) && peer.mcu_strobe(false) &&
                        shi.select(true) && shi.transfer(0).accepted;
  const auto status = completed.wait_for(std::chrono::seconds{2});
  if (status != std::future_status::ready) { worker.request_stop(); }
  worker.join();
  const auto report = completed.get();
  require(
      accepted && status == std::future_status::ready && !report.exception && !report.failure,
      "Native DSP code announces a report and waits for the actual host request before writing");
  constexpr std::uint64_t delay = 64;
  constexpr std::uint64_t reset_top = 12500;
  constexpr std::uint64_t reset_bottom = 17052;
  constexpr auto earliest = (delay * reset_top) / reset_bottom;
  constexpr std::uint8_t byte_mask = 0xff;
  require(!peer.read_dsp_data({.mask = byte_mask, .ticks = earliest - 1}) &&
              peer.error() != nullptr,
          "Sixty-four native NOPs at the reset PLL forbid reply data before floor(64*12500/17052)");
  constexpr unsigned bytes = 16;
  for (unsigned index = 0; index < bytes; ++index) {
    const auto sample =
        peer.read_dsp_data({.mask = byte_mask, .ticks = std::numeric_limits<std::uint64_t>::max()});
    require(sample && sample->mask == byte_mask && sample->known == byte_mask &&
                sample->value == index,
            "The real native producer's timestamps preserve all sixteen distinct data writes");
  }
  for (const bool reset_at_boundary : {false, true}) {
    port_publication_boundary(prepared, reset_at_boundary);
  }
}

void unknown_boot_tail() {
  constexpr dsp56300::Word next = 0xff0001;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report = execute({.main = dsp56300::isa::nop, .eeprom = {}}, shi, reset, {});
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

void sparse_boot_rom() {
  namespace isa = dsp56300::isa;
  using dsp56300::Word;
  constexpr Word absolute = 0x30;
  constexpr Word x0 = 4;
  constexpr Word loading = 0x8000;
  constexpr Word data_address = 0xff0002;
  constexpr Word eeprom_address = 0xffff00;
  constexpr Word marker = 0xc000;
  constexpr Word value = 0x123456;
  constexpr auto jump = isa::opcode<isa::Opcode::jmp_effective>()[0] | (absolute << 8U);
  constexpr auto sink = isa::opcode<isa::Opcode::movem_effective>()[0] | (absolute << 8U) | x0;
  constexpr auto main = isa::program(std::array{jump, eeprom_address, value});
  constexpr auto eeprom = isa::program(std::array{sink | loading, data_address, sink, marker});
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  constexpr std::array entries{blofeld::vi::dsp::BootRom::main_first};
  const std::array code{
      dsp56300::CodeSpan{.address = entries[0], .words = std::span(main).first(2)},
      dsp56300::CodeSpan{.address = eeprom_address, .words = eeprom}};
  const std::array failed{entries[0] + static_cast<Word>(main.size()),
                          eeprom_address + static_cast<Word>(eeprom.size())};
  const std::array variants{
      dsp56300::CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = failed}};
  const std::array choices{
      blofeld::vi::dsp::ProgramChoice{.variant = 0, .resumes = entries, .words = {}}};
  const auto prepared = blofeld::app::aot::prepare_dsp_program({.variants = variants}, choices);
  const auto report =
      execute({.main = main, .eeprom = eeprom}, shi, reset, {}, nullptr, {}, nullptr, &prepared);
  const bool correct = report.instructions == 2 && report.failure &&
                       report.failure->operation == blofeld::vi::Access::write &&
                       report.failure->space == 'P' && report.failure->address == marker &&
                       report.failure->pc == eeprom_address + 2 && report.failure->value == value &&
                       report.stopped_pc == eeprom_address + eeprom.size();
  if (!correct && report.failure) {
    std::cerr << "Sparse ROM: retired=" << report.instructions
              << " operation=" << static_cast<unsigned>(report.failure->operation)
              << " pc=" << std::hex << report.failure->pc << " address=" << report.failure->address
              << " value=" << report.failure->value << " code=" << report.failure->code << std::dec
              << '\n';
  }
  require(
      correct,
      "Native EEPROM-region code reads original main-ROM data and faults at its actual next write");
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
  const auto report = execute({.main = program, .eeprom = {}}, shi, reset, {});
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
  constexpr dsp56300::Word prefix_words = 7;
  const std::array loaded{dsp56300::CodeSpan{.address = destination, .words = isa::stop}};
  const auto prepared = prepare_loaded({.main = program, .eeprom = {}},
                                       loaded,
                                       blofeld::vi::dsp::BootRom::main_first + prefix_words);
  const auto report =
      execute({.main = program, .eeprom = {}}, shi, reset, {}, nullptr, {}, nullptr, &prepared);
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
  constexpr dsp56300::Word boot = blofeld::vi::dsp::BootRom::main_first;
  constexpr unsigned first_call = 12;
  constexpr unsigned second_call = 26;
  const std::span words(program);
  const std::array before{dsp56300::CodeSpan{.address = boot, .words = words.first(first_call)}};
  const std::array original{literal, dsp56300::Word{0}, isa::rts[0]};
  const std::array changed{literal, replacement, isa::rts[0]};
  const std::array middle{
      dsp56300::CodeSpan{.address = boot + first_call,
                         .words = words.subspan(first_call, second_call - first_call)},
      dsp56300::CodeSpan{.address = function, .words = original}};
  const std::array after{
      dsp56300::CodeSpan{.address = boot + second_call, .words = words.subspan(second_call)},
      dsp56300::CodeSpan{.address = function, .words = changed}};
  constexpr std::array first_entries{boot};
  constexpr std::array middle_entries{boot + first_call};
  constexpr std::array last_entries{boot + second_call};
  const std::array failed{boot + static_cast<dsp56300::Word>(program.size())};
  const std::array variants{
      dsp56300::CodeVariant{
          .id = 0, .code = before, .entries = first_entries, .failed_fetches = failed},
      dsp56300::CodeVariant{
          .id = 1, .code = middle, .entries = middle_entries, .failed_fetches = failed},
      dsp56300::CodeVariant{
          .id = 2, .code = after, .entries = last_entries, .failed_fetches = failed}};
  const std::array choices{
      blofeld::vi::dsp::ProgramChoice{.variant = 0, .resumes = first_entries, .words = {}},
      blofeld::vi::dsp::ProgramChoice{.variant = 1, .resumes = middle_entries, .words = {}},
      blofeld::vi::dsp::ProgramChoice{.variant = 2, .resumes = last_entries, .words = {}}};
  constexpr std::array transfers{
      dsp56300::CodeTransition{.from = 0, .to = 1, .entry = middle_entries[0]},
      dsp56300::CodeTransition{.from = 1, .to = 2, .entry = last_entries[0]}};
  const auto prepared =
      blofeld::app::aot::prepare_dsp_program({.variants = variants}, choices, {}, {}, transfers);
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const auto report =
      execute({.main = program, .eeprom = {}}, shi, reset, {}, nullptr, {}, nullptr, &prepared);
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
  const auto report = execute({.main = program, .eeprom = {}}, shi, reset, {});
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
  const auto report = execute({.main = program, .eeprom = {}}, shi, reset, {});
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
  const auto report = execute({.main = program, .eeprom = {}}, shi, reset, {});
  require(report.failure && report.failure->space == 'P' && report.failure->address == marker &&
              report.failure->value == expected,
          "Native execution reaches a real timer compare deadline and reads its sticky TCF");
}

void native_timer_interrupt(const bool waiting, const bool late_vba = false) {
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
  const auto handoff = blofeld::vi::dsp::BootRom::main_first + static_cast<Word>(program.size());
  const std::array set_base{literal,
                            vba,
                            isa::opcode<isa::Opcode::movec_register>()[0] | (x0 << 8U) | write |
                                vba_selector};
  const std::array set_priority{isa::opcode<isa::Opcode::movep_memory_high>()[0] |
                                    (immediate << 8U) | write | priority_port,
                                priority};
  const auto base = [&] { program.insert(program.end(), set_base.begin(), set_base.end()); };
  if (!late_vba) { base(); }
  program.insert(program.end(), set_priority.begin(), set_priority.end());
  if (late_vba) { base(); }
  const std::array setup{port | load_port,
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
  const std::array vector_words{call, handler};
  const std::array loaded{dsp56300::CodeSpan{.address = vector, .words = vector_words},
                          dsp56300::CodeSpan{.address = handler, .words = interrupt_handler}};
  constexpr std::array interrupts{dsp56300::InterruptRequest{.vector = vector, .level = 0}};
  const auto prepared =
      prepare_loaded({.main = program, .eeprom = {}}, loaded, handoff, interrupts);
  const auto report =
      execute({.main = program, .eeprom = {}}, shi, reset, {}, nullptr, {}, nullptr, &prepared);
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
    report = execute(
        {.main = program, .eeprom = {}}, shi, reset, consumer.stop.get_token(), nullptr, output);
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
  const auto report = execute({.main = program, .eeprom = {}}, shi, reset, {});
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
    const auto report = execute({.main = program, .eeprom = {}}, shi, reset, {});
    const auto expected = port == core_port ? core_expected : peripheral_expected;
    require(report.failure && report.failure->space == 'P' && report.failure->address == sink &&
                report.failure->value == expected,
            "Native priority writes, bit updates and reads retain only implemented fields");
    const std::array reset_program{read | port, output, sink};
    const auto after_reset = execute({.main = reset_program, .eeprom = {}}, shi, reset, {});
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
  const auto report = execute({.main = guest.code, .eeprom = {}}, shi, reset, {}, &image);
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
  const auto report = execute({.main = guest.code, .eeprom = {}}, shi, reset, {}, &image);
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
  const std::array loaded{dsp56300::CodeSpan{.address = 0, .words = isa::stop}};
  const auto prepared = prepare_loaded({.main = program, .eeprom = {}}, loaded, 0);
  const auto report =
      execute({.main = program, .eeprom = {}}, shi, reset, {}, nullptr, {}, nullptr, &prepared);
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
  const std::array loaded{dsp56300::CodeSpan{.address = 0, .words = isa::stop}};
  const auto prepared = prepare_loaded({.main = program, .eeprom = {}}, loaded, 0);
  std::atomic<bool> done{false};
  std::jthread worker([&](const std::stop_token &stop) {
    try {
      report = execute(
          {.main = program, .eeprom = {}}, shi, reset, stop, nullptr, {}, nullptr, &prepared);
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
  const auto run = [](const char *name, const auto action) {
    std::cerr << name << '\n';
    action();
  };
  run("native_media(MediaCase::read)", [&] { native_media(argv[2], MediaCase::read); });
  run("native_media(MediaCase::released_read)",
      [&] { native_media(argv[2], MediaCase::released_read); });
  run("native_media(MediaCase::erase)", [&] { native_media(argv[2], MediaCase::erase); });
  run("native_media(MediaCase::program)", [&] { native_media(argv[2], MediaCase::program); });
  run("native_missing_media(argv[2])", [&] { native_missing_media(argv[2]); });
  run("native_clock_register()", native_clock_register);
  run("native_timed_peer_input()", native_timed_peer_input);
  run("native_reply_time()", native_reply_time);
  run("unknown_boot_tail()", unknown_boot_tail);
  run("sparse_boot_rom()", sparse_boot_rom);
  constexpr dsp56300::Word delay_boundary = 0x16000;
  run("native_reserved_y_read<delay_boundary>()", native_reserved_y_read<delay_boundary>);
  run("native_reserved_y_read<dsp56300::word_mask>()", native_reserved_y_read<dsp56300::word_mask>);
  run("native_transmit_registers()", native_transmit_registers);
  run("native_interrupt_priorities()", native_interrupt_priorities);
  run("native_memory_switch()", native_memory_switch);
  run("native_dma_program_write()", native_dma_program_write);
  run("native_dma_esai_write()", native_dma_esai_write);
  run("native_dma_dax_write()", native_dma_dax_write);
  run("native_audio_dma(false)", [&] { native_audio_dma(false); });
  run("native_audio_dma(true)", [&] { native_audio_dma(true); });
  run("native_timer_events()", native_timer_events);
  run("native_timer_interrupt(false)", [&] { native_timer_interrupt(false); });
  run("native_timer_interrupt(true)", [&] { native_timer_interrupt(true); });
  run("native_timer_interrupt(false, true)", [&] { native_timer_interrupt(false, true); });
  run("native_timer_interrupt(true, true)", [&] { native_timer_interrupt(true, true); });
  run("consecutive_receive_registers()", consecutive_receive_registers);
  run("reset_running_core()", reset_running_core);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
