// SPDX-License-Identifier: GPL-3.0-only
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_programmer;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr auto contract = Hcs08FlashContract::candidate_gt8a;
constexpr unsigned strict_site = 0xb2, target_pointer = 0x80, cursor = 0x83;
constexpr unsigned first_stack_low = 0x77, later_stack = 0x477, ready_flags = 0x68;
constexpr unsigned record_count = 7, protected_value = 0xfc, protected_status = 0xe0;
constexpr unsigned vector_low = 0xffff, vector_high = 0xfffe, reset_high = 0xe0;
constexpr unsigned option_unsecured = 0xc2, second_h = 0x1200;
constexpr unsigned slow_delay = 97, fast_delay = 5, options_value = 0x53;

void inputs(Memory &memory, const std::span<const std::uint8_t> record) {
  memory.install(target_pointer, record.first(2));
  memory.install(buffer, record.subspan(2));
}

void strict(const std::vector<std::uint8_t> &program, const unsigned seed, std::ostream &trace) {
  Memory memory(Hcs08IdleLaunch::unqualified, slow_delay);
  memory.install(algorithm, program);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = algorithm,
                 .hx = static_cast<std::uint16_t>(seed),
                 .sp = ram_end - 4,
                 .ccr = ready_flags});
  for (unsigned n = 0; n < 8; ++n) { step(cpu, memory, trace); }
  require(cpu.state().pc == strict_site && memory.retired == 8 && memory.transfers == 0,
          "Original programmer reaches idle FSTAT FF after eight real instructions");
  const auto saved = cpu.checkpoint();
  const auto bus = memory;
  for (unsigned replay = 0; replay < 2; ++replay) {
    Memory restored = bus;
    Hcs08 other(restored, BLOFELD_HCS08_SLA);
    other.restore_checkpoint(saved);
    const auto error = rejected([&] { other.step(); });
    const auto failed = other.state();
    require(error.contains("idle FCBEF") && error.contains("pc=0x00b2") &&
                error.contains("address=0x1825") && other.faulted() && restored.transfers == 0 &&
                restored.retired == 8 && restored.flash.image() == memory.flash.image(),
            "Trusted native prefix restoration retains the strict actual idle-launch frontier");
    require(
        rejected([&] { static_cast<void>(other.checkpoint()); }).contains("healthy completed") &&
            rejected([&] { other.step(); }).contains("coordinated recovery") &&
            other.state() == failed,
        "Failed native CPU retry/checkpoint never clears the unknown-peripheral quarantine");
    trace << "STRICT seed=" << seed << " replay=" << replay << " pc=" << failed.pc
          << " sp=" << failed.sp << " retired=" << restored.retired << " transfers=0\n";
  }
}

void replay(const std::vector<Saved> &saved,
            const Hcs08 &expected_cpu,
            const Memory &expected,
            std::ostream &trace) {
  for (unsigned cut = 0; cut < saved.size(); ++cut) {
    for (unsigned repetition = 0; repetition < 2; ++repetition) {
      Memory memory = saved[cut].memory;
      Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
      cpu.restore_checkpoint(saved[cut].cpu);
      std::ostringstream actual;
      while (!cpu.background_requested()) {
        require(memory.retired < maximum_steps,
                "Restored native page suffix keeps its independent bound");
        step(cpu, memory, actual);
      }
      require(actual.str() == saved[cut].suffix && cpu.state() == expected_cpu.state() &&
                  memory.flash.state() == expected.flash.state() &&
                  memory.flash.image() == expected.flash.image() &&
                  memory.retired == expected.retired &&
                  memory.transfer_due == expected.transfer_due &&
                  memory.complete_due == expected.complete_due &&
                  memory.options.read() == expected.options.read(),
              "Fresh CPU/bus/FLASH/schedule owners reproduce every suffix byte and final "
              "native/model state");
      trace << "REPLAY cut=" << cut << " index=" << repetition << " bytes=" << actual.str().size()
            << " retired=" << memory.retired << " identical=1\n";
    }
  }
}

void page(Hcs08 &cpu, Memory &memory, const bool checkpoint, std::ostream &trace) {
  std::ostringstream events;
  std::vector<Saved> saved;
  bool erase_saved = false;
  bool queued_saved = false;
  const auto start = memory.retired;
  while (!cpu.background_requested()) {
    require(memory.retired - start < maximum_steps,
            "Original programmer reaches actual BGND within its bound");
    step(cpu, memory, events);
    const auto state = memory.flash.state();
    const bool erase = !erase_saved && state.active && state.active->command == hf::erase_page;
    const bool queued = !queued_saved && state.active && state.active->command == hf::burst &&
                        state.buffer == Hcs08FlashBuffer::launched;
    if (checkpoint && (erase || queued)) {
      saved.push_back({.cpu = cpu.checkpoint(),
                       .memory = memory,
                       .offset = static_cast<std::size_t>(events.tellp()),
                       .suffix = {}});
      erase_saved = erase_saved || erase;
      queued_saved = queued_saved || queued;
    }
  }
  require(memory.bytes[marker] == marker_value &&
              memory.read_memory({.address = cursor, .size = 2}) == buffer + page_bytes &&
              memory.options.read() == options_value && memory.sopt_writes >= 1,
          "Original loop advances its full buffer, writes marker72 and actually disables COP");
  const auto complete = events.str();
  trace << complete;
  for (auto &cut : saved) { cut.suffix = complete.substr(cut.offset); }
  replay(saved, cpu, memory, trace);
  const auto native = cpu.checkpoint();
  Memory copy = memory;
  Hcs08 held(copy, BLOFELD_HCS08_SLA);
  held.restore_checkpoint(native);
  require(held.background_requested() && held.state() == cpu.state() &&
              rejected([&] { held.step(); }).contains("owning BDC decision"),
          "Actual BGND request survives trusted restoration without fabricated BDC status");
}

void vector_cut(const Memory &memory, std::ostream &trace) {
  const auto state = memory.flash.state();
  require(state.active && state.active->address == vector_low && memory.complete_due,
          "The actual last vector byte remains active when slow reference execution reaches BGND");
  Hcs08Flash restarted(memory.flash.power_off_image(Hcs08FlashAbort::reference_command_range),
                       contract);
  restarted.context({.pc = algorithm, .ccr = ready_flags});
  require(restarted.read_array(vector_high) == reset_high &&
              rejected([&] {
                static_cast<void>(restarted.read_array(vector_low));
              }).contains("unknown nonvolatile"),
          "A BGND marker cannot prove a bootable vector after loss of the pending final program");
  trace << "VECTOR_CUT address=65535 unknown=1 marker=114\n";
}

struct OriginalInputs {
  const std::vector<std::uint8_t> &program;
  const std::vector<std::uint8_t> &records;
};

void profile(const OriginalInputs original,
             const unsigned seed,
             const unsigned delay,
             const std::string &path,
             std::ostream &trace) {
  Memory memory(Hcs08IdleLaunch::reference_ignore_empty, delay);
  memory.install(algorithm, original.program);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  auto expected = memory.flash.image();
  Hcs08State state{.pc = algorithm,
                   .hx = static_cast<std::uint16_t>(seed),
                   .sp = ram_end - 4,
                   .ccr = ready_flags};
  trace << "CASE seed=" << seed << " delay=" << delay << '\n';
  for (unsigned n = 0; n < record_count; ++n) {
    const auto record = std::span(original.records)
                            .subspan(static_cast<std::size_t>(n) * record_bytes, record_bytes);
    const auto address = (static_cast<unsigned>(record[0]) << 8U) | record[1];
    inputs(memory, record);
    state.pc = algorithm;
    state.sp = ram_end - 4;
    cpu.set_state(state); // Explicit own debug entry, not an emulated BDC GO command.
    trace << "PAGE index=" << n << " address=" << address << '\n';
    page(cpu, memory, n == 0 && delay == slow_delay, trace);
    require(std::ranges::equal(std::span(memory.bytes).subspan(algorithm, algorithm_bytes),
                               original.program),
            "Every original algorithm byte remains unchanged through native page execution");
    const auto expected_sp = n == 0 ? seed | first_stack_low : later_stack;
    require(cpu.state().sp == expected_sp &&
                memory.read_memory({.address = target_pointer, .size = 2}) ==
                    ((address + page_bytes) & vector_low),
            "Actual H retention and destination increments agree through all seven pages");
    if (n == record_count - 1 && delay == slow_delay) { vector_cut(memory, trace); }
    const bool pending = memory.flash.state().active.has_value();
    finish_pending(memory);
    for (unsigned byte = 0; byte < page_bytes; ++byte) {
      expected.data[address - hf::first + byte] = record[byte + 2];
    }
    require(memory.flash.image() == expected && memory.array_writes == (n + 1) * (page_bytes + 1) &&
                memory.command_writes == memory.array_writes &&
                memory.status_writes == (n + 1) * (page_bytes + 2) &&
                memory.transfers == memory.array_writes && memory.completions == memory.transfers,
            "Every original erase/program command produces exact page bytes and no omitted-region "
            "changes");
    trace << "PAGE_END index=" << n << " retired=" << memory.retired << " sp=" << cpu.state().sp
          << " transfers=" << memory.transfers << " completed=" << memory.completions
          << " pending-at-bgnd=" << pending << " cuts=" << memory.cuts << '\n';
    state = cpu.state();
  }
  memory.flash.reset();
  memory.flash.context({.pc = algorithm, .ccr = ready_flags});
  require(memory.flash.read_array(vector_high) == reset_high &&
              memory.flash.read_array(vector_low) == 0 &&
              memory.flash.read(hf::options) == option_unsecured,
          "Completed programmed image retains its genuine reset vector and option byte");
  write_private_file(path, memory.flash.image().data);
  trace << "PROFILE_END seed=" << seed << " delay=" << delay << " pages=7 bytes=3584 retained=1\n";
}

void protected_page(const OriginalInputs original, std::ostream &trace) {
  Memory memory(Hcs08IdleLaunch::reference_ignore_empty, slow_delay);
  memory.flash.context({.pc = algorithm, .ccr = ready_flags}, Hcs08FlashAgent::background);
  memory.flash.write({.address = hf::protection, .value = protected_value});
  const auto before = memory.flash.image();
  memory.install(algorithm, original.program);
  inputs(memory, std::span(original.records).last(record_bytes));
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = algorithm, .sp = ram_end - 4, .ccr = ready_flags});
  trace << "PROTECTED\n";
  page(cpu, memory, false, trace);
  require(memory.flash.read(hf::status) == protected_status &&
              memory.flash.read(hf::protection) == protected_value &&
              memory.flash.image() == before && memory.transfers == 0 &&
              memory.bytes[marker] == marker_value,
          "Original CPU FPROT FF cannot remove protection: native code still reaches BGND without "
          "writing the page");
  trace << "PROTECTED_END status=224 marker=114 transfers=0 nvm=1 retired=" << memory.retired
        << '\n';
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Expected original programmer, seven supplied records and private trace");
    const auto program = read_input(arguments[1], algorithm_bytes);
    const auto records = read_input(arguments[2], record_count * record_bytes);
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    for (const unsigned seed : {0U, 0x1200U}) { strict(program, seed, trace); }
    const OriginalInputs original{.program = program, .records = records};
    profile(original, 0, fast_delay, std::string(arguments[3]) + ".fast.bin", trace);
    profile(original, second_h, slow_delay, std::string(arguments[3]) + ".slow.bin", trace);
    protected_page(original, trace);
    std::cout << "Original HCS08 page programmer, strict frontier and explicit reference FLASH "
                 "paths verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
