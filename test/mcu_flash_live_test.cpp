// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_flash.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu_flash_live_fixture.hpp"
#include "storage/flash_image.hpp"
#include "storage/private_file.hpp"
#include "storage/virtual_device_state.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {
using namespace live;

bool same_image(const blofeld::McuFlashImage &a, const blofeld::McuFlashImage &b) {
  return a.bytes == b.bytes && a.known == b.known && a.fresh == b.fresh;
}

struct Fixture {
  Code algorithm;
  std::shared_ptr<blofeld::VirtualDeviceState> owner;
  blofeld::McuMachine machine;
  unsigned publications = 0;
  unsigned phases = 0;

  Fixture(const std::filesystem::path &path, const Scenario scenario)
      : algorithm(helper(scenario)),
        owner(std::make_shared<blofeld::VirtualDeviceState>(
            path, image(algorithm, scenario), [] { return "EMULIVE00001"; })),
        machine(owner->image(), BLOFELD_DEFAULT_SLA, contract(), {3}) {
    blofeld::write_private_file(path.string() + ".initial",
                                blofeld::encode_flash_image(owner->image()));
    const auto before = machine.checkpoint();
    machine.attach_flash_store(owner);
    blofeld::McuBus second(owner->image());
    require(rejected([&] { second.attach_flash_store(owner); }).contains("already has"),
            "Two independent controllers cannot share one live file writer capability");
    require(rejected([&] { owner->save(owner->image()); }).contains("another live MCU writer"),
            "An outer file save cannot bypass the bound native controller");
    require(rejected([&] { static_cast<void>(machine.checkpoint()); }).contains("cannot be cloned"),
            "External file ownership excludes RAM-only checkpoints");
    require(rejected([&] { machine.restore(before); }).contains("cannot be rewound"),
            "A pre-binding RAM checkpoint cannot drop the owner or rewind its file");
    require(
        rejected([&] { static_cast<void>(machine.bus().clone()); }).contains("cannot be cloned"),
        "Public bus clone cannot branch a live file lease");
    machine.bus().set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
    owner->set_observer([this](const blofeld::PrivateFilePhase phase) {
      if (phase == blofeld::PrivateFilePhase::created) { ++publications; }
      ++phases;
      require(rejected([this] {
                static_cast<void>(machine.bus().read(status_register));
              }).contains("publication is active"),
              "No native ready flag can be read during an unpublished file transition");
      require(rejected([this] { machine.bus().reset(); }).contains("publication is active"),
              "Observer reset cannot partially mutate the device during publication");
      require(rejected([this] { owner->set_observer({}); }).contains("publication is active"),
              "File observers cannot replace themselves during a transaction");
    });
  }

  void boot() {
    require(machine.system_reset_active() && machine.run_clock().elapsed_ticks() == 0,
            "The durable device starts with held POR and no injected CPU/RAM state");
    machine.advance_system_reset(reset_edges, blofeld::McuResetPad::high);
    require(!machine.system_reset_active() && machine.cpu().state().pc == entry,
            "The real reset sequence fetches the persisted own ROM vector");
  }
};

void cold_reopen(const std::filesystem::path &path, const blofeld::McuFlashImage &expected) {
  const blofeld::McuFlashImage unused;
  const blofeld::VirtualDeviceState cold(path, unused, []() -> std::string {
    throw blofeld::Fault("Cold reopen must not generate a serial or repair existing cells");
  });
  require(same_image(cold.image(), expected),
          "Cold reopen preserves every byte, unknown bit and consumed erase history");
}

void run(const std::filesystem::path &path, const Scenario scenario) {
  blofeld::McuFlashImage expected;
  {
    Fixture fixture(path, scenario);
    auto &machine = fixture.machine;
    fixture.boot();
    std::uint64_t launched = 0;
    machine.bus().set_observer([&](const blofeld::BusEvent &event) {
      if (event.write && event.address == status_register && event.value == launch_flag) {
        require(launched == 0, "The guest issues exactly one genuine command");
        launched = machine.run_clock().elapsed_ticks();
      }
    });
    unsigned steps = 0;
    while (launched == 0 && ++steps < instruction_limit) { machine.cpu().step(); }
    require(launched != 0 && machine.bus().next_flash_event_ticks() == launched + scenario.duration,
            "The durable path retains the independently calculated native command deadline");
    const auto elapsed = machine.run_clock().elapsed_ticks();
    for (unsigned i = 0; i < observation_count; ++i) {
      require(machine.bus().read(status_register) == launch_flag,
              "File ownership never makes status observations complete an active command");
    }
    require(machine.run_clock().elapsed_ticks() == elapsed,
            "One hundred status reads cannot advance native clocks");
    const auto end = ram + fixture.algorithm.size() - 2;
    bool woke = false;
    while (machine.cpu().state().pc != end && ++steps < instruction_limit) {
      if (machine.cpu().waiting()) {
        require(machine.advance_wait(instruction_limit) > 0, "Real flash clocks progress in WAI");
      }
      const auto step = machine.cpu().step();
      if (step.interrupt_vector) {
        require(scenario.wait && *step.interrupt_vector == vector && step.stack_reused,
                "Only the documented native CCIF interrupt wakes this WAI frame");
        woke = true;
      }
    }
    require(machine.cpu().state().pc == end && machine.bus().read(marker) == signature &&
                machine.bus().read(status_register) == idle && !machine.timing_faulted() &&
                machine.cpu().state().sp == stack && woke == scenario.wait &&
                !machine.bus().next_flash_event_ticks(),
            "Guest completion follows actual file-backed flash operations and native IRQ/RTI");
    check_array(machine, scenario);
    expected = machine.bus().power_off_flash_image();
    require(same_image(expected, fixture.owner->image()),
            "CCIF exposes exactly the already published cells and erase history");
    const auto count = scenario.command == verify ? 0U : 2U;
    require(fixture.publications == count && fixture.phases == count * 5,
            "Each destructive transfer/completion publishes once; read-only verify publishes none");
    std::cout << "native command=" << scenario.command << " divider=" << scenario.divider
              << " launch=" << launched << " duration=" << scenario.duration
              << " end=" << machine.run_clock().elapsed_ticks() << " WAI=" << scenario.wait
              << " publications=" << fixture.publications << '\n';
  }
  cold_reopen(path, expected);
}

void test_reset(const std::filesystem::path &path) {
  const Scenario scenario{.command = sector, .duration = erase_ticks};
  blofeld::McuFlashImage expected;
  {
    Fixture fixture(path, scenario);
    auto &machine = fixture.machine;
    fixture.boot();
    unsigned steps = 0;
    while (fixture.publications == 0 && ++steps < instruction_limit) { machine.cpu().step(); }
    require(fixture.publications == 1 && machine.bus().read(status_register) == launch_flag,
            "A real native transfer has durably recorded its active-sector cut projection");
    machine.begin_system_reset(blofeld::McuResetSource::external, 3);
    require(machine.system_reset_active() && fixture.publications == 1,
            "Hardware reset keeps the owner and cancels the job without rewriting history");
    require(!fixture.owner->image().known.test(target - entry),
            "The persistent active sector remains unknown after the reset");
    machine.advance_system_reset(reset_edges, blofeld::McuResetPad::high);
    require(!machine.system_reset_active() && machine.cpu().state().pc == entry &&
                machine.run_clock().native_epoch() == 2,
            "Native reset clocks fetch the preserved boot vector in the new epoch");
    const auto end = ram + fixture.algorithm.size() - 2;
    while (machine.cpu().state().pc != end && ++steps < instruction_limit) { machine.cpu().step(); }
    require(machine.cpu().state().pc == end && machine.bus().read(marker) == signature &&
                fixture.publications == 2 && machine.bus().read(status_register) == idle,
            "Own boot firmware re-erases the unknown sector through real native commands");
    check_array(machine, scenario);
    expected = machine.bus().power_off_flash_image();
    require(same_image(expected, fixture.owner->image()),
            "Recovered native cells equal the durable image before completion is acknowledged");
  }
  cold_reopen(path, expected);
  std::cout << "native reset:active cut retained,real reset vector,guest erase recovery passed.\n";
}
}

int main(const int argc, char **argv) {
  try {
    require(argc == 2, "Expected a fresh private test directory");
    const std::filesystem::path path(argv[1]);
    run(path / "program.bfnvm", {});
    run(path / "fractional.bfnvm", {.divider = fractional_divider, .duration = fractional_ticks});
    run(path / "sector.bfnvm", {.command = sector, .duration = erase_ticks});
    run(path / "mass.bfnvm", {.command = mass, .duration = mass_ticks});
    run(path / "verify.bfnvm", {.command = verify, .duration = verify_ticks});
    run(path / "wai.bfnvm", {.wait = true});
    test_reset(path / "reset.bfnvm");
    std::cout
        << "Live native MCU:durable transfer/completion,read-only verify,WAI,reset,cold reopen"
           " and checkpoint/reentry guards passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
