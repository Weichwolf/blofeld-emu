// SPDX-License-Identifier: GPL-3.0-only
#include "board/nand_peer.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "esai_fixture.hpp"
#include "storage/nand.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <memory>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
constexpr unsigned extal = 12000000;
constexpr unsigned timer1 = 0xffff8b;
constexpr unsigned di = 0x1000;

struct Rate {
  unsigned hz, power_ticks;
};

constexpr std::array rates{Rate{.hz = 540000000, .power_ticks = 54000},
                           Rate{.hz = 1080000000, .power_ticks = 108000}};

DspRunClockInputs clocks(const unsigned hz) {
  return {.extal_hz = extal,
          .timebase_hz = hz,
          .switch_timing = DspClockSwitch::reference_cycle_boundary};
}

void power(const Rate rate) {
  OwnDspMachine machine(clocks(rate.hz));
  NandPeer peer(machine, Nand{});
  const auto events = peer.events();
  require(events.next_ticks() == rate.power_ticks && !peer.detailed().base.ready &&
              rejects([&] {
                static_cast<void>(machine.bus().timers().read(timer1, di));
              }).contains("Unknown DSP timer DI read"),
          "NAND power deadline scales to the owner clock while initial Ready remains unknown");
  const auto before = peer.detailed();
  require(rejects([&] { events.observe(rate.power_ticks - 1U); }).contains("exact offered") &&
              peer.detailed() == before,
          "A wrong deadline cannot publish premature Ready or mutate the peer");
  events.observe(rate.power_ticks);
  require(peer.detailed().power_ready && peer.detailed().base.ready == true &&
              peer.detailed().base.ticks == rate.power_ticks && !events.next_ticks() &&
              machine.bus().timers().read(timer1, di) == di,
          "The actual autonomous power event publishes the modeled Ready pin");
  const auto powered = peer.detailed();
  require(rejects([&] { const NandPeer duplicate(machine, Nand{}); }).contains("GPIO") &&
              peer.detailed() == powered && machine.bus().timers().read(timer1, di) == di,
          "A rejected second board owner cannot overwrite the first peer's Ready input");
  peer.before_native(rate.power_ticks);
  const auto snapshot = peer.checkpoint();
  OwnDspMachine fresh(clocks(rate.hz));
  NandPeer copy(snapshot, fresh);
  copy.before_native(rate.power_ticks);
  require(copy.detailed() == peer.detailed() && fresh.bus().timers().read(timer1, di) == di,
          "A fresh same-timebase peer restores actual protocol state and external pin knowledge");
  constexpr unsigned capture = 0x65;
  constexpr unsigned capture_flags = 0x300000;
  fresh.bus().timers().write({.address = timer1 - 1, .value = 7});
  fresh.bus().timers().write({.address = timer1, .value = capture});
  fresh.bus().timers().advance_clk2(3);
  copy.before_native(rate.power_ticks);
  fresh.bus().timers().advance_clk2(1);
  require(fresh.bus().timers().read(timer1, capture_flags) == 0 &&
              fresh.bus().timers().read(timer1 - 3) == 0,
          "A fresh Ready binding establishes capture baseline without inventing a new edge");
  const auto other = rate.hz == rates[0].hz ? rates[1].hz : rates[0].hz;
  OwnDspMachine incompatible(clocks(other));
  require(rejects([&] {
            const NandPeer invalid(snapshot, incompatible);
          }).contains("matching device timebase") &&
              rejects([&] {
                static_cast<void>(incompatible.bus().timers().read(timer1, di));
              }).contains("Unknown DSP timer DI read"),
          "Clock-incompatible clones reject without fabricating target Ready");
}

void binding_lifetime() {
  OwnDspMachine machine(clocks(rates[0].hz));
  auto peer = std::make_unique<NandPeer>(machine, Nand{});
  const auto events = peer->events();
  const auto payload = peer->checkpoint();
  const auto native = machine.checkpoint();
  peer->restore(payload);
  require(rejects([&] {
            static_cast<void>(machine.bus().gpio(DspPort::f).observe_pads(1));
          }).contains("restored native binding"),
          "Staged peer payload cannot leak through an earlier native pad binding");
  machine.restore(native);
  require(rejects([&] {
            static_cast<void>(machine.bus().gpio(DspPort::f).observe_pads(1));
          }).contains("Detached resolved GPIO"),
          "Native restoration preserves the obligation to reconnect the actual board source");
  peer->before_native(0);
  require(machine.bus().gpio(DspPort::f).observe_pads(1).known == 0,
          "Rebinding preserves genuinely unknown startup data");
  peer.reset();
  require(rejects([&] {
            static_cast<void>(machine.bus().gpio(DspPort::f).observe_pads(1));
          }).contains("lost its live board owner") &&
              rejects([&] {
                static_cast<void>(events.next_ticks());
              }).contains("lost its live board owner"),
          "Expired GPIO and event callbacks reject before dereferencing destroyed peer state");
}
}

int main() {
  try {
    for (const auto rate : rates) { power(rate); }
    binding_lifetime();
    std::cout << "NAND peer timebases, real power events and clone guards passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
