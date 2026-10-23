// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/private_file.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_bdc;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
namespace hp = blofeld::test::hcs08_programmer;
namespace hf = blofeld::hcs08_flash;
constexpr auto contract = Hcs08FlashContract::candidate_gt8a;
constexpr unsigned seed = 0x1200, stack = 0x47c, ready_flags = 0x68;
constexpr unsigned page_count = 7, trim = 0xffbe, strict_site = 0xb2;
constexpr unsigned slow_delay = 97, vector_low = 0xffff;
constexpr unsigned running_status = 0x88, active_status = 0xc8, first_stack_low = 0x77;
constexpr unsigned later_stack = 0x477, commands_per_page = 513, target_high = 0x80,
                   target_low = 0x81;

std::uint8_t status_reply(Hcs08Bdc &bdc) {
  constexpr std::array<std::uint8_t, 1> frame{0xe4};
  command(bdc, frame);
  return bdc.read_reply();
}

void upload(Hcs08Bdc &bdc, const std::span<const std::uint8_t> programmer) {
  for (unsigned address = hp::ram_first; address < hp::algorithm + hp::algorithm_bytes; ++address) {
    const auto byte = address < hp::algorithm ? 0 : programmer[address - hp::algorithm];
    write(
        bdc,
        {.address = static_cast<std::uint16_t>(address), .value = static_cast<std::uint8_t>(byte)});
  }
}

void start(Hcs08Bdc &bdc) {
  constexpr std::array<std::uint8_t, 3> pc{0x4b, 0, hp::algorithm};
  constexpr std::array<std::uint8_t, 3> sp{0x4f, 4, 0x7c};
  constexpr std::array<std::uint8_t, 1> go{0x08};
  command(bdc, pc);
  command(bdc, sp);
  command(bdc, go);
}

void inputs(Hcs08Bdc &bdc,
            const std::span<const std::uint8_t> record,
            const std::uint8_t saved_trim) {
  constexpr std::array<std::uint8_t, 1> background{0x90};
  command(bdc, background);
  const auto destination = (static_cast<unsigned>(record[0]) << 8U) | record[1];
  for (unsigned n = 0; n < hp::page_bytes; ++n) {
    const auto value = destination + n == trim ? saved_trim : record[n + 2];
    write(bdc, {.address = static_cast<std::uint16_t>(hp::buffer + n), .value = value});
  }
  write(bdc, {.address = target_high, .value = record[0]});
  write(bdc, {.address = target_low, .value = record[1]});
}

void strict(const std::vector<std::uint8_t> &program, std::ostream &trace) {
  hp::Memory memory(Hcs08IdleLaunch::unqualified, slow_delay);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = hp::algorithm, .hx = seed, .sp = stack, .ccr = ready_flags});
  Memory target(memory);
  Hcs08Bdc bdc(cpu, target, contract, Hcs08BdcReset::bkgd_held_low);
  upload(bdc, program);
  start(bdc);
  require(status_reply(bdc) == running_status, "GO reports a genuinely running native target");
  for (unsigned n = 0; n < 8; ++n) { bdc.step_cpu(); }
  require(cpu.state().pc == strict_site && memory.transfers == 0,
          "BDC-uploaded unchanged program reaches its actual unstaged FSTAT store");
  const auto control = bdc.checkpoint();
  const auto native = cpu.checkpoint();
  const auto saved = memory;
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto restored = saved;
    Hcs08 other_cpu(restored, BLOFELD_HCS08_SLA);
    other_cpu.restore_checkpoint(native);
    Memory backend(restored);
    Hcs08Bdc other(other_cpu, backend, contract, Hcs08BdcReset::normal);
    other.restore_checkpoint(control);
    const auto error = rejected([&] { other.step_cpu(); });
    require(error.contains("address=0x1825") && error.contains("pc=0x00b2") && other.faulted() &&
                other_cpu.faulted() && restored.retired == 8 && restored.transfers == 0 &&
                restored.flash.image() == saved.flash.image() &&
                rejected([&] { other.receive(0x08); }).contains("coordinated recovery") &&
                rejected([&] { other.restore_checkpoint(control); }).contains("healthy completed"),
            "Neither GO nor controller-only restore can clear a native unknown-access failure");
    trace << "STRICT replay=" << replay << " pc=" << other_cpu.state().pc
          << " retired=" << restored.retired << " transfers=0\n";
  }
}

struct OriginalInputs {
  const std::vector<std::uint8_t> &program;
  const std::vector<std::uint8_t> &records;
};

void profile(const OriginalInputs original, const std::string &image_path, std::ostream &trace) {
  const auto &program = original.program;
  const auto &records = original.records;
  hp::Memory memory(Hcs08IdleLaunch::reference_ignore_empty, slow_delay);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = hp::algorithm, .hx = seed, .sp = stack, .ccr = ready_flags});
  Memory target(memory);
  Hcs08Bdc bdc(cpu, target, contract, Hcs08BdcReset::bkgd_held_low);
  const auto saved_trim = read(bdc, trim);
  require(saved_trim == hp::data_canary, "Read existing target trim through an actual BDC reply");
  upload(bdc, program);
  auto expected = memory.flash.image();
  for (unsigned page = 0; page < page_count; ++page) {
    const auto record = std::span(records).subspan(
        static_cast<std::size_t>(page) * hp::record_bytes, hp::record_bytes);
    const auto destination = (static_cast<unsigned>(record[0]) << 8U) | record[1];
    inputs(bdc, record, saved_trim);
    start(bdc);
    require(status_reply(bdc) == running_status, "Programmer is running before actual native BGND");
    const auto begin = memory.retired;
    while (!bdc.state().active) {
      require(memory.retired - begin < hp::maximum_steps, "Original BDC-launched page bound");
      const auto before = cpu.state();
      bdc.step_cpu();
      trace << "STEP n=" << memory.retired << " pc=" << before.pc << " next=" << cpu.state().pc
            << " mode=" << bdc.state().active << '\n';
    }
    require(status_reply(bdc) == active_status && memory.bytes[hp::marker] == hp::marker_value &&
                (cpu.state().sp == (page == 0 ? seed | first_stack_low : later_stack)),
            "BDC status follows genuine BGND, including original H-preserving TXS");
    require(std::ranges::equal(std::span(memory.bytes).subspan(hp::algorithm, hp::algorithm_bytes),
                               program),
            "Every BDC-uploaded original instruction remains unchanged");
    require(memory.flash.state().active.has_value(),
            "The reference final byte is still executing despite actual active BDC status");
    if (page == page_count - 1) {
      require(memory.flash.state().active->address == vector_low,
              "The last active command is the genuine low reset-vector byte");
    }
    hp::finish_pending(
        memory); // Explicit external reference event, never a status-read side effect.
    for (unsigned n = 0; n < hp::page_bytes; ++n) {
      expected.data[destination - hf::first + n] =
          destination + n == trim ? saved_trim : record[n + 2];
    }
    require(memory.flash.image() == expected &&
                memory.transfers == (page + 1) * commands_per_page &&
                memory.completions == memory.transfers,
            "Native target, rather than the BDC host, performs every page erase/program");
    trace << "PAGE index=" << page << " address=" << destination << " retired=" << memory.retired
          << " serviced=" << bdc.state().serviced
          << " status=" << static_cast<unsigned>(bdc.status())
          << " trim=" << static_cast<unsigned>(saved_trim) << " transfers=" << memory.transfers
          << '\n';
  }
  write_private_file(image_path, memory.flash.image().data);
  trace << "END pages=7 serviced=" << bdc.state().serviced << " retired=" << memory.retired
        << " trim=" << static_cast<unsigned>(saved_trim) << '\n';
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Expected pinned programmer, records and private trace");
    const auto program = hp::read_input(arguments[1], hp::algorithm_bytes);
    const auto records = hp::read_input(arguments[2], page_count * hp::record_bytes);
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    strict(program, trace);
    profile({.program = program, .records = records}, std::string(arguments[3]) + ".bin", trace);
    std::cout << "Original HCS08 execution through BDC transactions verified under explicit "
                 "reference contracts.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
