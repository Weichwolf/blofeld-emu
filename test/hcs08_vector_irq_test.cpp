// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "board/hcs08_board_serial.hpp"
#include "common/bus.hpp"
#include "hcs08_board_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_vector_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::vectors;
using blofeld::test::hcs08::rejected;

void run(const Configuration config, const Hcs08SpiRunContract order) {
  const auto image = own_image(config);
  std::optional<Hcs08BoardSerial> original(
      std::in_place,
      reset(image),
      BLOFELD_HCS08_SLA,
      Hcs08BoardSerialContract{
          .order = order,
          .launch = Hcs08SpiLaunchContract::reference_one_period,
          .program = Hcs08PrefetchContract::reference_subroutine_program_cycles,
          .foreground = Hcs08SpiForegroundContract::reference_instruction_retirement});
  std::vector<Hcs08BoardSerialCheckpoint> saved;
  for (const unsigned cut : cuts) {
    to(original.value(), cut, config);
    saved.push_back(original.value().checkpoint());
  }
  to(original.value(), endpoint, config);
  endpoint_checks(original.value(), config, image);
  const auto final = original.value().checkpoint();
  original.reset();
  const Hcs08BoardSerial expected(final, BLOFELD_HCS08_SLA);
  for (const auto &snapshot : saved) {
    for (unsigned copy = 0; copy < 2; ++copy) {
      Hcs08BoardSerial fresh(snapshot, BLOFELD_HCS08_SLA);
      to(fresh, endpoint, config);
      endpoint_checks(fresh, config, image);
      require(fresh.cpu_state() == expected.cpu_state() &&
                  fresh.board_state() == expected.board_state() &&
                  fresh.run_state() == expected.run_state() &&
                  fresh.foreground_state() == expected.foreground_state() &&
                  fresh.bdc_state() == expected.bdc_state() &&
                  fresh.clock_state() == expected.clock_state(),
              "Destroyed-owner fresh suffix retains genuine CPU/IRQ/prefetch/clock ownership");
      fixture::same_memory(fresh.memory(), expected.memory());
    }
  }
}

void missing_vectors(const Hcs08SpiRunContract order) {
  const auto config = cases.front();
  for (unsigned byte = 0; byte < 2; ++byte) {
    auto image = own_image(config);
    image.known.reset(config.physical_vector + byte - hcs08_flash::first);
    Hcs08BoardSerial board(
        reset(image),
        BLOFELD_HCS08_SLA,
        {.order = order,
         .launch = Hcs08SpiLaunchContract::reference_one_period,
         .program = Hcs08PrefetchContract::reference_subroutine_program_cycles,
         .foreground = Hcs08SpiForegroundContract::reference_instruction_retirement});
    const auto error = rejected([&] { to(board, endpoint, config); });
    require(error.contains(hex(config.physical_vector + byte)) && error.contains("pc=0xe00f") &&
                error.contains("unknown nonvolatile contents") && board.faulted(),
            "Unknown redirected high/low bytes fail at actual native reads,with source context");
    require(
        rejected([&] { static_cast<void>(board.checkpoint()); }).contains("coordinated recovery"),
        "Failed vector execution cannot create a reusable success checkpoint");
  }
  auto image = own_image(config);
  image.known.reset(vector + 1 - hcs08_flash::first);
  Hcs08BoardSerial board(
      reset(image),
      BLOFELD_HCS08_SLA,
      {.order = order,
       .launch = Hcs08SpiLaunchContract::reference_one_period,
       .program = Hcs08PrefetchContract::reference_subroutine_program_cycles,
       .foreground = Hcs08SpiForegroundContract::reference_instruction_retirement});
  to(board, endpoint, config);
  endpoint_checks(board, config, image);
}
} // namespace

int main() {
  try {
    for (const auto order : {Hcs08SpiRunContract::reference_cpu_before_spi,
                             Hcs08SpiRunContract::reference_spi_before_cpu}) {
      for (const auto config : cases) { run(config, order); }
      missing_vectors(order);
    }
    std::cout << "Verified native routed IRQ/ISR/RTI,ordinary data reads and432fresh suffixes.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
