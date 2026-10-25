// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_bdc;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
namespace hp = blofeld::test::hcs08_programmer;
namespace hf = blofeld::hcs08_flash;
constexpr auto contract = Hcs08FlashContract::candidate_gt8a;
constexpr unsigned initial_hx = 0x1234, initial_sp = 0x47c, initial_a = 0x5a;
constexpr unsigned initial_ccr = 0x79, next_pc = 0xa1, result_a = 0x33;
constexpr unsigned secure_option = 0xfd, protection_value = 0xfc;
constexpr unsigned active_clock_status = 0xc8, active_status = 0xc0, modified_sp = 0x312;
constexpr unsigned test_address = 0x90, invalid_opcode = 0xff, hx_high = 0x12, hx_low = 0x34;
constexpr unsigned write_opcode = 0xc0;

struct Fixture {
  hp::Memory bus{Hcs08IdleLaunch::unqualified, 5};
  Hcs08 cpu{bus, BLOFELD_HCS08_SLA};
  Memory memory{bus};

  Fixture() {
    constexpr std::array<std::uint8_t, 4> code{0x9d, 0x82, 0xa6, result_a};
    bus.install(hp::algorithm, code);
    cpu.set_state({.pc = hp::algorithm,
                   .hx = initial_hx,
                   .sp = initial_sp,
                   .a = initial_a,
                   .ccr = initial_ccr});
  }
};

void mode_and_registers() {
  Fixture fixture;
  Hcs08Bdc bdc(fixture.cpu, fixture.memory, contract, Hcs08BdcReset::bkgd_held_low);
  const auto initial = fixture.cpu.state();
  require(bdc.status() == active_clock_status &&
              rejected([&] { bdc.step_cpu(); }).contains("halted CPU") &&
              fixture.cpu.state() == initial && !bdc.faulted(),
          "Sampled active-reset mode owns the CPU without inventing its register values");
  constexpr std::array<std::uint8_t, 2> clear_control{0xc4, 0};
  command(bdc, clear_control);
  require(bdc.status() == active_status,
          "Active mode preserves ENBDM while allowing a CLKSW change; no clock is synthesized");
  constexpr std::array<std::uint8_t, 3> set_pc{0x4b, 0, next_pc};
  command(bdc, set_pc);
  constexpr std::array<std::uint8_t, 3> set_sp{0x4f, 0x03, hx_high};
  command(bdc, set_sp);
  require(fixture.cpu.state() == Hcs08State{.pc = next_pc,
                                            .hx = initial_hx,
                                            .sp = modified_sp,
                                            .a = initial_a,
                                            .ccr = initial_ccr},
          "Big-endian WRITE_PC/WRITE_SP preserve every unrelated architectural register");
  constexpr std::array<std::uint8_t, 1> get_hx{0x6c};
  command(bdc, get_hx);
  require(bdc.read_reply() == hx_high && bdc.read_reply() == hx_low,
          "READ_HX emits independently expected high then low bytes");
  constexpr std::array<std::uint8_t, 1> go{0x08};
  command(bdc, go);
  require(bdc.status() == 0x80, "GO releases the actual CPU, not a success flag");
  bdc.step_cpu();
  require(bdc.status() == active_status && fixture.cpu.background_requested() &&
              fixture.cpu.state().pc == next_pc + 1,
          "Native BGND requests genuine background admission at the following PC");
  command(bdc, go);
  bdc.step_cpu();
  require(bdc.status() == 0x80 && fixture.cpu.state().a == result_a &&
              fixture.cpu.state().hx == initial_hx && fixture.cpu.state().sp == modified_sp,
          "A subsequent GO consumes BGND and resumes real native instructions/registers");
  constexpr std::array<std::uint8_t, 1> background{0x90};
  constexpr std::array<std::uint8_t, 3> repeat_load{0x4b, 0, hp::algorithm + 2};
  constexpr unsigned changed_a = 0x44;
  command(bdc, background);
  write(bdc, {.address = hp::algorithm + 3, .value = changed_a});
  command(bdc, repeat_load);
  command(bdc, go);
  bdc.step_cpu();
  require(fixture.cpu.state().a == changed_a && fixture.cpu.state().hx == initial_hx,
          "A privileged BDC RAM-code write invalidates the previous native translation");
}

void frame_abort_and_restore() {
  Fixture fixture;
  Hcs08Bdc bdc(fixture.cpu, fixture.memory, contract, Hcs08BdcReset::bkgd_held_low);
  const auto before = fixture.bus.bytes;
  for (const auto byte : {write_opcode, 0U, test_address}) {
    bdc.receive(static_cast<std::uint8_t>(byte));
  }
  const auto saved = bdc.checkpoint();
  const auto native = fixture.cpu.checkpoint();
  const auto memory = fixture.bus;
  bdc.communication_timeout();
  require(fixture.bus.bytes == before && bdc.state().serviced == 0 &&
              bdc.status() == active_clock_status,
          "Timed-out incomplete WRITE_BYTE has no target memory or mode effects");
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto restored = memory;
    Hcs08 cpu(restored, BLOFELD_HCS08_SLA);
    cpu.restore_checkpoint(native);
    Memory target(restored);
    Hcs08Bdc other(cpu, target, contract, Hcs08BdcReset::normal);
    other.restore_checkpoint(saved);
    other.receive(result_a);
    require(restored.bytes == before, "Final command byte alone cannot acquire a service grant");
    other.service();
    require(restored.bytes[test_address] == result_a && other.state().serviced == 1 &&
                other.status() == active_clock_status && cpu.state() == fixture.cpu.state(),
            "Fresh joint restoration commits the original partial frame exactly once");
    require(read(other, test_address) == result_a,
            "Actual BDC read returns the independently written byte");
    other.communication_timeout();
    require(restored.bytes[test_address] == result_a,
            "A later communication timeout never rolls back a completed target write");
  }
}

void remaining_registers_and_quarantine() {
  Fixture fixture;
  Hcs08Bdc bdc(fixture.cpu, fixture.memory, contract, Hcs08BdcReset::bkgd_held_low);
  constexpr std::array<std::uint8_t, 2> set_a{0x48, result_a};
  constexpr std::array<std::uint8_t, 2> set_ccr{0x49, 0};
  constexpr std::array<std::uint8_t, 3> set_hx{0x4c, 0xab, 0xcd};
  command(bdc, set_a);
  command(bdc, set_ccr);
  command(bdc, set_hx);
  constexpr std::array<std::uint8_t, 3> get_a{0x68, 0x69, 0x6c};
  constexpr std::array<std::uint8_t, 3> expected{result_a, 0x60, 0xab};
  for (unsigned n = 0; n < get_a.size(); ++n) {
    command(bdc, std::span(get_a).subspan(n, 1));
    require(bdc.read_reply() == expected[n], "Independent accumulator/CCR/HX read expectations");
  }
  constexpr unsigned low_x = 0xcd;
  require(bdc.read_reply() == low_x, "Read word retains the independent low byte");
  constexpr std::array<std::uint8_t, 1> get_pc{0x6b};
  constexpr std::array<std::uint8_t, 1> get_sp{0x6f};
  command(bdc, get_pc);
  require(bdc.read_reply() == 0 && bdc.read_reply() == hp::algorithm, "Independent PC reply");
  command(bdc, get_sp);
  constexpr unsigned stack_low = 0x7c;
  require(bdc.read_reply() == 4 && bdc.read_reply() == stack_low, "Independent SP reply");
  const auto native = fixture.cpu.checkpoint();
  const auto held = bdc.checkpoint();
  constexpr std::array<std::uint8_t, 1> go{0x08};
  command(bdc, go);
  fixture.bus.known.reset(hp::algorithm);
  require(rejected([&] { bdc.step_cpu(); }).contains("fetch address=0x00a0") &&
              fixture.cpu.faulted() && bdc.faulted() &&
              rejected([&] { bdc.receive(go[0]); }).contains("coordinated recovery") &&
              rejected([&] { bdc.restore_checkpoint(held); }).contains("healthy completed"),
          "BDC GO/controller-only restore cannot clear a native failed instruction");
  fixture.bus.known.set(hp::algorithm);
  fixture.cpu.restore_checkpoint(native);
  bdc.restore_checkpoint(held);
  require(bdc.status() == active_clock_status,
          "Only explicit joint CPU, memory and controller recovery reestablishes a trusted owner");
}

void security_and_privilege() {
  Fixture fixture;
  auto image = fixture.bus.flash.image();
  image.data[hf::nv_options - hf::first] = secure_option;
  fixture.bus.flash = Hcs08Flash(image, contract);
  Hcs08Bdc bdc(fixture.cpu, fixture.memory, contract, Hcs08BdcReset::bkgd_held_low);
  const auto before = fixture.bus.bytes;
  write(bdc, {.address = test_address, .value = result_a});
  require(read(bdc, test_address) == 0 && fixture.bus.bytes == before &&
              read(bdc, hf::nv_options) == 0,
          "Secure target mapping blocks BDC RAM/FLASH data without exposing retained contents");
  write(bdc, {.address = hf::protection, .value = protection_value});
  require(read(bdc, hf::protection) == protection_value,
          "Real BDC WRITE_BYTE has the privilege to alter FPROT even on a secure target");
  constexpr std::array<std::uint8_t, 1> go{0x08};
  constexpr std::array<std::uint8_t, 1> background{test_address};
  constexpr std::array<std::uint8_t, 2> enable{0xc4, 0x80};
  command(bdc, go);
  command(bdc, enable);
  command(bdc, background);
  require(bdc.status() == 0x80 && fixture.cpu.state().pc == hp::algorithm,
          "Secure running target refuses serial BACKGROUND even when ENBDM is set");
  Fixture disabled;
  Hcs08Bdc normal(disabled.cpu, disabled.memory, contract, Hcs08BdcReset::normal);
  command(normal, background);
  require(normal.status() == 0, "Normal-reset ENBDM zero ignores BACKGROUND");
  command(normal, enable);
  command(normal, background);
  require(normal.status() == active_status,
          "Enabled unsecured BACKGROUND halts the current boundary");
}

void arbitration_and_failures() {
  Fixture fixture;
  Hcs08Bdc bdc(fixture.cpu, fixture.memory, contract, Hcs08BdcReset::normal);
  constexpr std::array<std::uint8_t, 3> frame{0xe0, 0, test_address};
  for (const auto byte : frame) { bdc.receive(byte); }
  const auto native = fixture.cpu.state();
  const auto error = rejected([&] { bdc.service(); });
  require(error.contains("bus arbitration") && error.contains("address=0x0090") &&
              error.contains("pc=0x00a0") && bdc.faulted() && fixture.cpu.state() == native &&
              rejected([&] { bdc.communication_timeout(); }).contains("coordinated recovery"),
          "Unqualified non-intrusive access fails visibly and cannot retry through timeout");
  Fixture granted;
  Hcs08Bdc running(granted.cpu, granted.memory, contract, Hcs08BdcReset::normal);
  for (const auto byte : frame) { running.receive(byte); }
  running.service(Hcs08BdcGrant::reference_instruction_boundary);
  require(running.read_reply() == 0 && running.status() == 0,
          "Only the explicit reference boundary admits a running-target memory access");
  for (unsigned failure = 0; failure < 6; ++failure) {
    Fixture target;
    Hcs08Bdc controller(target.cpu, target.memory, contract, Hcs08BdcReset::bkgd_held_low);
    const auto saved = controller.checkpoint();
    switch (failure) {
      case 0:
        require(rejected([&] { controller.receive(invalid_opcode); }).contains("unsupported"),
                "Opcode");
        break;
      case 1:
        require(
            rejected([&] { static_cast<void>(controller.read_reply()); }).contains("no serviced"),
            "Reply");
        break;
      case 2:
        require(rejected([&] { controller.service(); }).contains("no complete"), "Frame");
        break;
      case 3:
        target.bus.known.reset(test_address);
        require(rejected([&] {
                  static_cast<void>(read(controller, test_address));
                }).contains("address=0x0090"),
                "Unknown byte");
        break;
      case 4: {
        constexpr std::array<std::uint8_t, 1> get_status{0xe4};
        command(controller, get_status);
        require(rejected([&] { controller.receive(get_status[0]); }).contains("reply"),
                "An unread response cannot be silently replaced by another command");
        break;
      }
      case 5: {
        auto unknown = target.bus.flash.image();
        unknown.known.reset(hf::nv_options - hf::first);
        target.bus.flash = Hcs08Flash(unknown, contract);
        const auto reason = rejected([&] { static_cast<void>(read(controller, test_address)); });
        require(reason.contains("address=0x1821") && reason.contains("pc=0x00a0") &&
                    reason.contains("unknown reset-latched"),
                "BDC memory access cannot guess a missing security latch or stale PC context");
        break;
      }
      default: throw Fault("Invalid own BDC failure case");
    }
    require(controller.faulted() && rejected([&] {
                                      static_cast<void>(controller.checkpoint());
                                    }).contains("coordinated recovery"),
            "Failed BDC execution cannot become trusted state");
    controller.restore_checkpoint(saved);
    require(controller.status() == active_clock_status && controller.state().serviced == 0,
            "An explicit trusted controller restore is distinct from implicit retry");
  }
}
} // namespace

int main() {
  try {
    mode_and_registers();
    frame_abort_and_restore();
    remaining_registers_and_quarantine();
    security_and_privilege();
    arbitration_and_failures();
    std::cout << "HCS08 BDC modes, native register ownership, framing, security and strict "
                 "failures passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
