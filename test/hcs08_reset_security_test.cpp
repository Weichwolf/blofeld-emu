// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_flash_fixture.hpp"
#include "hcs08_wire_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
namespace cmds = blofeld::test::hcs08_bdc;
namespace wire = blofeld::test::hcs08_wire;
namespace nvm = blofeld::test::hcs08_nvm;
namespace hf = hcs08_flash;
constexpr unsigned previous_pc = 0x1234, previous_hx = 0x3322, previous_sp = 0x4567;
constexpr unsigned retained_a = 0xa5, retained_x = 0x22, reset_ccr = 0x68, load_known = 0xee;
constexpr unsigned vector_high = 0xfffe, vector_low = 0xffff, own_entry = 0xa0, loaded_pc = 0xa2;
constexpr unsigned loaded_a = 0x42, active_status = 0xc8, idle_status = 0xc0;
constexpr unsigned option_prefix = 0xfc, unsecured = 2, option_value = 0xc2;
constexpr std::array<std::uint8_t, 3> own_code{0xa6, loaded_a, 0x9d};
constexpr auto held = Hcs08BdcReset::bkgd_held_low;

void warm(Hcs08 &cpu, const unsigned pc = previous_pc) {
  cpu.set_state({.pc = static_cast<std::uint16_t>(pc),
                 .hx = previous_hx,
                 .sp = previous_sp,
                 .a = retained_a,
                 .ccr = byte_mask});
}

Hcs08FlashImage preimage(const unsigned sec, const bool options, const bool protection) {
  auto image = nvm::initial();
  image.data[hf::nv_options - hf::first] = static_cast<std::uint8_t>(option_prefix | sec);
  image.known.set(hf::nv_options - hf::first, options);
  image.known.set(hf::nv_protection - hf::first, protection);
  image.data[vector_high - hf::first] = 0;
  image.data[vector_low - hf::first] = own_entry;
  return image;
}

void normal_vectors() {
  constexpr std::array<unsigned, 3> origins{previous_pc, own_entry, nvm::example};
  unsigned cases = 0;
  for (unsigned sec = 0; sec < 4; ++sec) {
    for (const bool options : {true, false}) {
      for (const bool protection : {false, true}) {
        for (const auto origin : origins) {
          wire::Target target;
          const auto image = preimage(sec, options, protection);
          target.memory.flash = Hcs08Flash(image, wire::chip);
          target.memory.install(own_entry, own_code);
          warm(target.cpu, origin);
          target.cpu.reset();
          require(
              target.cpu.state() == Hcs08State{.pc = own_entry,
                                               .hx = retained_x,
                                               .sp = byte_mask,
                                               .a = retained_a,
                                               .ccr_known = reset_ccr,
                                               .ccr = reset_ccr},
              "Normal CPU reset must fetch the actual vector independent of SEC/old PC; actual=" +
                  hex(target.cpu.state().pc));
          require(target.memory.accesses ==
                          std::vector<Access>{
                              {.address = vector_high, .value = 0, .write = false},
                              {.address = vector_low, .value = own_entry, .write = false}} &&
                      target.memory.retired == 0 && !target.memory.resetting &&
                      target.memory.flash.image() == image,
                  "Normal reset performs two actual ordered vector reads without changing NVM "
                  "knowledge");
          target.cpu.step();
          require(target.cpu.state() == Hcs08State{.pc = loaded_pc,
                                                   .hx = retained_x,
                                                   .sp = byte_mask,
                                                   .a = loaded_a,
                                                   .ccr_known = load_known,
                                                   .ccr = reset_ccr} &&
                      target.memory.retired == 1,
                  "Known reset entry executes the actual native LDA, with independent full state");
          target.memory.flash.context({.pc = previous_pc, .ccr = reset_ccr});
          if (!options) {
            require(rejected([&] {
                      static_cast<void>(target.memory.flash.read_array(nvm::example));
                    }).contains("unknown reset-latched"),
                    "Reset vector privilege cannot unlock unknown ordinary CPU access permissions");
          } else {
            require(target.memory.flash.read_array(nvm::example) ==
                        (sec == unsecured ? nvm::seed_data : 0),
                    "Unsecured-origin CPU reads retain actual SEC blocking after reset closes");
          }
          ++cases;
        }
      }
    }
  }
  constexpr unsigned expected_cases = 48;
  require(cases == expected_cases, "All SEC/options/protection/old-origin cases were exercised");
}

void missing_vectors() {
  for (const bool options : {false, true}) {
    for (unsigned known = 0; known < 3; ++known) {
      wire::Target target;
      auto image = preimage(0, options, false);
      image.known.set(vector_high - hf::first, (known & 1U) != 0);
      image.known.set(vector_low - hf::first, (known & 2U) != 0);
      target.memory.flash = Hcs08Flash(image, wire::chip);
      warm(target.cpu);
      const auto failure = rejected([&] { target.cpu.reset(); });
      require(
          failure.contains("unknown nonvolatile") &&
              failure.contains("address=" + hex(known == 1 ? vector_low : vector_high)) &&
              failure.contains("pc=" + hex(previous_pc)) && target.cpu.faulted() &&
              target.memory.retired == 0 && !target.memory.resetting &&
              target.memory.flash.image() == image,
          "Normal reset cannot use secured-read zero to hide first/second missing vector bytes");
    }
  }
}

void enter(wire::Target &target, const Hcs08FlashImage &image) {
  target.memory.flash = Hcs08Flash(image, wire::chip);
  warm(target.cpu);
  target.cpu.reset(Hcs08ResetEntry::active_background);
  require(!target.cpu.state().pc_known && target.memory.accesses.empty() &&
              target.memory.retired == 0 && !target.memory.resetting &&
              target.memory.flash.image() == image,
          "Active entry retains unknown PC without consuming vector/options/protection cells");
}

void unknown_permissions() {
  for (unsigned operation = 0; operation < 5; ++operation) {
    wire::Target target;
    auto image = preimage(0, false, false);
    image.known.reset(vector_high - hf::first);
    image.known.reset(vector_low - hf::first);
    enter(target, image);
    Hcs08Bdc bdc(target.cpu, target.mapping, wire::chip, held);
    constexpr std::array<std::uint8_t, 1> status{0xe4};
    constexpr std::array<std::uint8_t, 1> read_a{0x68};
    cmds::command(bdc, status);
    require(bdc.read_reply() == active_status, "Active status is independent of unknown SEC");
    cmds::command(bdc, read_a);
    require(bdc.read_reply() == retained_a,
            "Held CPU registers remain inspectable with unknown SEC");
    const auto ram = target.memory.bytes;
    const auto known = target.memory.known;
    const auto failure = rejected([&] {
      switch (operation) {
        case 0: static_cast<void>(cmds::read(bdc, own_entry)); break;
        case 1: cmds::write(bdc, {.address = own_entry, .value = loaded_a}); break;
        case 2: static_cast<void>(cmds::read(bdc, nvm::example)); break;
        case 3: static_cast<void>(cmds::read(bdc, hf::options)); break;
        case 4: static_cast<void>(cmds::read(bdc, hf::protection)); break;
        default: throw Fault("Invalid own permission case");
      }
    });
    require(failure.contains("unknown reset-latched") &&
                failure.contains("pc=unknown pc-known=0") && bdc.faulted() && bdc.state().active &&
                bdc.state().serviced == 2 && !target.cpu.state().pc_known &&
                target.memory.retired == 0 && target.memory.bytes == ram &&
                target.memory.known == known && target.memory.flash.image() == image,
            "Unknown SEC/FPROT never fabricates RAM/NVM data, upload success or a permission bit");
  }
}

void secured_resources() {
  for (unsigned sec = 0; sec < 4; ++sec) {
    wire::Target target;
    auto image = preimage(sec, true, true);
    image.known.reset(vector_high - hf::first);
    image.known.reset(vector_low - hf::first);
    target.memory.install(own_entry, own_code); // Explicit retained warm RAM, not a BDC upload.
    enter(target, image);
    Hcs08Bdc bdc(target.cpu, target.mapping, wire::chip, held);
    require(cmds::read(bdc, own_entry) == (sec == unsecured ? own_code[0] : 0) &&
                cmds::read(bdc, nvm::example) == (sec == unsecured ? nvm::seed_data : 0),
            "All four SEC encodings independently gate actual BDC RAM/FLASH reads");
    cmds::write(bdc, {.address = own_entry, .value = loaded_a});
    cmds::write(bdc, {.address = hf::options, .value = unsecured});
    require(
        target.memory.bytes[own_entry] == (sec == unsecured ? loaded_a : own_code[0]) &&
            target.memory.flash.security_engaged() == (sec != unsecured) &&
            target.memory.flash.image() == image && target.memory.retired == 0 &&
            !target.cpu.state().pc_known,
        "Secured RAM writes and every FOPT write remain ineffective after genuine active entry");
  }
}

void controller_command(wire::Target &target, Hcs08Bdc &bdc, const nvm::Command request) {
  cmds::write(bdc, {.address = request.address, .value = request.value});
  cmds::write(bdc, {.address = hf::command, .value = request.opcode});
  cmds::write(bdc, {.address = hf::status, .value = hf::empty});
  require(target.memory.flash.state().buffer == Hcs08FlashBuffer::launched,
          "Only actual BDC frames reach the controller launch latch");
  target.memory.flash.advance(Hcs08FlashMilestone::transfer_buffer);
  target.memory.flash.advance(Hcs08FlashMilestone::complete_command);
}

void vector_access_scope() {
  Hcs08Flash flash(preimage(0, true, true), wire::chip);
  flash.context({.pc = previous_pc, .ccr = reset_ccr});
  const auto image = flash.image();
  const auto idle = flash.state();
  require(flash.read_reset_vector_byte(vector_high) == 0 &&
              flash.read_reset_vector_byte(vector_low) == own_entry &&
              flash.read_array(nvm::example) == 0,
          "Explicit CPU reset-vector reads do not change ordinary secured-origin access");
  require(rejected([&] {
            static_cast<void>(flash.read_reset_vector_byte(nvm::example));
          }).contains("requires CPU reset fetch") &&
              flash.image() == image && flash.state() == idle,
          "Reset capability cannot read another array address or alter NVM/controller state");
  flash.context({.pc = previous_pc, .ccr = reset_ccr}, Hcs08FlashAgent::background);
  require(rejected([&] {
            static_cast<void>(flash.read_reset_vector_byte(vector_low));
          }).contains("agent=BDC") &&
              flash.read_array(vector_low) == 0,
          "BDC cannot invoke privileged reset-vector reads or see protected vector data");
  nvm::setup(flash);
  nvm::launch(flash, {.opcode = hf::erase_page});
  const auto launched = flash.state();
  require(rejected([&] {
            static_cast<void>(flash.read_reset_vector_byte(vector_high));
          }).contains("executing/pending") &&
              flash.state() == launched && flash.image() == image,
          "Reset-vector access cannot read through a pending NVM command");
  flash.advance(Hcs08FlashMilestone::transfer_buffer);
  const auto executing = flash.state();
  require(rejected([&] {
            static_cast<void>(flash.read_reset_vector_byte(vector_low));
          }).contains("executing/pending") &&
              flash.state() == executing && flash.image() == image,
          "Reset-vector access cannot advance or complete an executing NVM command");
}

void normal_from_background() {
  wire::Target target;
  auto image = preimage(1, true, true);
  constexpr unsigned redirecting_secure_options = 0xbd;
  constexpr unsigned protected_vector_page = 0xfc;
  image.data[hf::nv_options - hf::first] = redirecting_secure_options;
  image.data[hf::nv_protection - hf::first] = protected_vector_page;
  target.memory.install(own_entry, own_code);
  enter(target, image);
  target.cpu.reset();
  require(target.cpu.state().pc_known && target.cpu.state().pc == own_entry &&
              !target.memory.resetting && target.memory.retired == 0 &&
              target.memory.flash.security_engaged() && target.memory.flash.image() == image &&
              target.memory.accesses.size() == 2 &&
              target.memory.accesses.front().address == vector_high &&
              target.memory.accesses.back().address == vector_low,
          "Normal reset from unknown PC uses fixed FFFE/FFFF despite FNORED0 and vector-page "
          "protection");
  target.cpu.step();
  require(target.cpu.state().pc == loaded_pc && target.cpu.state().a == loaded_a &&
              target.memory.retired == 1,
          "Actual normal-reset vector establishes execution after pre-vector background entry");
}

Hcs08FlashImage interrupted_image(const Hcs08FlashAbort abort) {
  Hcs08Flash flash(preimage(unsecured, true, true), wire::chip);
  nvm::setup(flash);
  nvm::launch(flash, {.address = hf::nv_options, .opcode = hf::erase_page});
  flash.advance(Hcs08FlashMilestone::transfer_buffer);
  const auto image = flash.power_off_image(abort);
  require(!image.known[hf::nv_options - hf::first] && !image.known[hf::nv_protection - hf::first] &&
              !image.known[vector_high - hf::first] && !image.known[vector_low - hf::first],
          "Actual interrupted option-page erase leaves configuration and vectors unknown");
  return image;
}

void unknown_image_recovery(const Hcs08FlashAbort abort, const bool persist) {
  wire::Target target;
  const auto image = interrupted_image(abort);
  enter(target, image);
  Hcs08Bdc bdc(target.cpu, target.mapping, wire::chip, held);
  cmds::write(bdc, {.address = hf::protection, .value = hf::erased});
  require(cmds::read(bdc, hf::protection) == hf::erased &&
              !target.memory.flash.image().known[hf::nv_protection - hf::first],
          "Actual BDC FPROT writes establish volatile protection without inventing NVPROT bytes");
  target.memory.flash.observe_bus_clock(
      nvm::hz); // Declared reference input, not original FCDIV timing.
  cmds::write(bdc, {.address = hf::divider, .value = nvm::divisor});
  controller_command(target, bdc, {.opcode = hf::erase_all});
  require(
      target.memory.flash.image().known.all() && target.memory.flash.image().fresh.all() &&
          rejected([&] {
            static_cast<void>(target.memory.flash.security_engaged());
          }).contains("unknown reset-latched"),
      "Completed mass erase establishes actual erased NVM while SEC remains reset-latched unknown");
  controller_command(target, bdc, {.opcode = hf::verify});
  require(!target.memory.flash.security_engaged() &&
              cmds::read(bdc, hf::status) == (idle_status | hf::blank),
          "Only successful complete blank check establishes SEC10 without changing unknown other "
          "FOPT bits");
  require(rejected([&] {
            static_cast<void>(target.memory.flash.read(hf::options));
          }).contains("unknown reset-latched"),
          "Blank-check SEC knowledge cannot manufacture the other reset-latched FOPT bits");
  cmds::write(bdc, {.address = own_entry, .value = loaded_a});
  require(cmds::read(bdc, own_entry) == loaded_a && !target.cpu.state().pc_known,
          "RAM upload succeeds only after actual permission establishment, without an invented PC");
  if (persist) {
    controller_command(
        target,
        bdc,
        {.address = hf::nv_options, .value = nvm::stored_options, .opcode = hf::program});
  }
  const auto saved = target.memory.flash.image();
  for (unsigned n = 0; n < hf::bytes; ++n) {
    const bool programmed_option = persist && n == hf::nv_options - hf::first;
    require(
        saved.data[n] == (programmed_option ? nvm::stored_options : hf::erased) && saved.known[n] &&
            saved.fresh[n] == !programmed_option,
        "Every recovered NVM byte and erase-history bit has an independent literal expectation");
  }
  target.memory.flash.reset();
  target.cpu.reset(Hcs08ResetEntry::active_background);
  Hcs08Bdc restarted(target.cpu, target.mapping, wire::chip, held);
  require(cmds::read(restarted, hf::options) == (persist ? option_value : option_value + 1) &&
              target.memory.flash.security_engaged() == !persist &&
              target.memory.flash.image() == saved && !target.cpu.state().pc_known &&
              target.memory.retired == 0,
          "Warm restart reloads actual NVOPT: blank-only recovery resecures, programmed SEC10 "
          "persists");
}
} // namespace

int main() {
  try {
    normal_vectors();
    missing_vectors();
    unknown_permissions();
    secured_resources();
    vector_access_scope();
    normal_from_background();
    for (const auto abort :
         {Hcs08FlashAbort::unqualified_retention, Hcs08FlashAbort::reference_command_range}) {
      unknown_image_recovery(abort, false);
      unknown_image_recovery(abort, true);
    }
    std::cout << "HCS08 normal reset privilege, active security and BDC recovery verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
