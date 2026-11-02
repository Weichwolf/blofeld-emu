// SPDX-License-Identifier: GPL-3.0-only
#include "board/nand_bus_decode.hpp"
#include "board/nand_peer.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "nand_durable_native_tests.hpp"
#include "nand_persistence_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_write_store.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace {
using namespace blofeld;
constexpr std::uint64_t timebase = 1080000000;
constexpr unsigned extal = 12000000;
constexpr auto part = NandReadContinuation::k9f1208_p;
constexpr auto contract = NandBusContract::ordered_pins;

void admission_guards() {
  const test::PersistenceDirectory directory;
  OwnDspMachine machine{{.extal_hz = extal,
                         .timebase_hz = timebase,
                         .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  test::persistence_reject(
      [&] { const NandPeer peer(machine, std::unique_ptr<NandWriteStore>{}, contract); },
      "fresh healthy owner");
  for (unsigned kind = 0; kind < 3; ++kind) {
    const auto path = directory.file("bad-" + std::to_string(kind) + ".bfnand");
    auto store = std::make_unique<NandWriteStore>(path,
                                                  test::erased_nand_fixture(),
                                                  part,
                                                  timebase + (kind == 0 ? timebase : 0),
                                                  NandCellProfile{});
    if (kind == 1) { store->observe_power_ready(); }
    if (kind == 2) { static_cast<void>(store->advance(1)); }
    test::persistence_reject([&] { const NandPeer peer(machine, std::move(store), contract); },
                             "fresh healthy owner");
  }
}
}

int main() {
  try {
    admission_guards();
    blofeld::test::durable_medium_tests();
    std::cout << "Durable native NAND ownership, actual GPIO writes/status/readback and admission "
                 "guards passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
