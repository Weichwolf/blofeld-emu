// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "nand_persistence_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_write_store.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using test::erased_nand_fixture;
using test::persisted_cookie;
using test::persisted_mask;
using test::persisted_row;
using test::persistence_reject;
using test::persistence_require;
using test::PersistenceDirectory;
constexpr auto part = NandReadContinuation::k9f1208_j;
constexpr std::uint64_t timebase = 1000000;
constexpr std::uint64_t program_ticks = 500;
constexpr std::uint64_t erase_ticks = 3000;
constexpr unsigned confirm = 0x10;
constexpr unsigned reset = 0xff;
constexpr unsigned phases = 5;
constexpr std::uint64_t read_idle_ticks = 100;

NandCellProfile profile() {
  return {.contract = NandCellContract::reference_pulse_inhibit, .cells = {}};
}

void disk_cell(const std::filesystem::path &path, const bool uncertain) {
  const auto image = load_nand_image(path);
  const auto cell = image.pages.at(persisted_row).cells.at(0);
  persistence_require(
      cell ==
          (uncertain ? NandByte{} : NandByte{.value = persisted_cookie, .known = persisted_mask}),
      "File must contain the actual interrupted projection or verified result");
  persistence_require(image.pages.at(test::persisted_neighbor).cells.at(0) ==
                          NandByte{.value = persisted_cookie, .known = persisted_mask},
                      "Publication preserves the independent neighboring block");
}

void ordinary_reads() {
  const PersistenceDirectory directory;
  const auto path = directory.file();
  const auto seed = erased_nand_fixture();
  {
    NandWriteStore store(path, seed, part, timebase, profile());
    persistence_require(store.continuation() == part && store.timebase_hz() == timebase,
                        "Part and virtual clock remain explicit");
    persistence_reject([&] { const NandWriteStore second(path, part, timebase, profile()); }, "");
    store.set_observer([](NandFilePhase) { throw Fault("Read attempted host publication"); });
    store.observe_power_ready();
    store.command(reset);
    constexpr std::uint64_t reset_ticks = 5;
    persistence_require(store.advance(reset_ticks).has_value(),
                        "A read-only reset completes without rewriting the NAND image");
    store.command(0);
    for (const auto address : {0U, persisted_row, 0U, 0U}) {
      store.address(static_cast<std::uint8_t>(address));
    }
    constexpr std::uint64_t read_ticks = 20;
    persistence_require(!store.advance(read_ticks - 1) && store.advance(read_ticks).has_value(),
                        "Read completion follows the existing independent fifteen-us expectation");
    persistence_require(store.read() == persisted_mask, "Native controller returns actual cells");
    store.observe_read_end(NandReadEnd::re_high);
    for (std::uint64_t tick = read_ticks + 1; tick < read_ticks + read_idle_ticks; ++tick) {
      persistence_require(!store.advance(tick), "Idle read periods create no completion");
    }
    persistence_require(
        encode_nand_image(load_nand_image(path)) == encode_nand_image(seed),
        "Reads, power observation and elapsed time do not rewrite physical knowledge");
  }
  auto replacement = seed;
  replacement.pages.clear();
  const NandWriteStore reopened(path, replacement, part, timebase, profile());
  persistence_require(reopened.device().cell({.page = persisted_row}) ==
                              NandByte{.value = persisted_mask, .known = persisted_mask} &&
                          !reopened.device().ready(),
                      "Existing image wins over seed and never restores transient Ready");
}

void completed_program() {
  const PersistenceDirectory directory;
  const auto path = directory.file();
  {
    NandWriteStore store(path, erased_nand_fixture(), part, timebase, profile());
    test::prepare_persistent_program(store);
    store.write(persisted_cookie);
    unsigned count = 0;
    store.set_observer([&](const NandFilePhase phase) {
      persistence_require(store.device().pending() == NandPending::none,
                          "Confirm is not visible before file publication");
      if (phase >= NandFilePhase::published) { disk_cell(path, true); }
      ++count;
    });
    store.command(confirm);
    persistence_require(count == phases && store.device().pending() == NandPending::program &&
                            store.device().ready() == false,
                        "Confirm first publishes unknown in-flight cell/history projection");
    persistence_require(!store.advance(program_ticks - 1) && count == phases,
                        "No premature success or repeated filesystem work before deadline");
    store.set_observer([&](const NandFilePhase phase) {
      persistence_require(
          store.device().pending() == NandPending::program && store.device().ready() == false,
          "Old busy state remains visible during autonomous completion publication");
      disk_cell(path, phase < NandFilePhase::published);
      ++count;
    });
    const auto event = store.advance(program_ticks);
    persistence_require(event && event->ticks == program_ticks &&
                            event->operation == NandPending::program && count == phases * 2 &&
                            store.device().ready() == true,
                        "Verified Ready follows durable result at the independent 500-us deadline");
    disk_cell(path, false);
  }
  const NandWriteStore reopened(path, part, timebase, profile());
  const auto persisted = reopened.device().persistent_image().pages.at(persisted_row);
  persistence_require(!reopened.device().ready() && persisted.programs &&
                          persisted.programs->main == 1 && persisted.programs->spare == 0 &&
                          persisted.cells.at(0).value == persisted_cookie,
                      "Cold reopen retains exact cells and consumed partial-program history");
}

void interrupted_and_reset() {
  const PersistenceDirectory directory;
  const auto path = directory.file();
  {
    NandWriteStore store(path, erased_nand_fixture(), part, timebase, profile());
    test::prepare_persistent_program(store);
    store.write(persisted_cookie);
    store.command(confirm);
    disk_cell(path, true);
    store.command(reset);
    constexpr std::uint64_t reset_ticks = 10;
    const auto reset_event = store.advance(reset_ticks);
    persistence_require(reset_event && reset_event->operation == NandPending::reset &&
                            !store.advance(program_ticks),
                        "Actual reset withdraws the old program deadline instead of completing it");
    disk_cell(path, true);
    store.power_loss();
    persistence_require(!store.device().ready(), "Power loss clears live readiness");
  }
  const NandWriteStore reopened(path, part, timebase, profile());
  persistence_require(reopened.device().cell({.page = persisted_row}) == NandByte{},
                      "Interrupted physical knowledge survives cold reconstruction");
}

void erase_and_physical_fail() {
  const PersistenceDirectory directory;
  const auto path = directory.file();
  {
    auto image = erased_nand_fixture();
    image.pages.at(persisted_row).cells.at(0).value = 0;
    image.pages.at(persisted_row).programs = NandProgramCount{.main = 1, .spare = 0};
    NandWriteStore store(path, image, part, timebase, profile());
    store.observe_power_ready();
    store.observe_wp(true);
    constexpr unsigned erase_setup = 0x60;
    constexpr unsigned erase_confirm = 0xd0;
    store.command(erase_setup);
    for (const auto address : {persisted_row, 0U, 0U}) {
      store.address(static_cast<std::uint8_t>(address));
    }
    store.command(erase_confirm);
    disk_cell(path, true);
    persistence_require(!store.advance(erase_ticks - 1) && store.advance(erase_ticks),
                        "Erase uses its actual three-ms cell-model deadline");
    const auto erased = load_nand_image(path).pages.at(persisted_row);
    persistence_require(erased.cells.at(0) ==
                                NandByte{.value = persisted_mask, .known = persisted_mask} &&
                            erased.programs && erased.programs->main == 0,
                        "Verified physical erase publishes ones and renewed history");
  }
  const auto failed_path = directory.file("inhibited.bfnand");
  auto inhibit = profile();
  const auto cell = persisted_row * Nand::page_bytes;
  inhibit.cells.emplace(cell, NandCellInhibit{.program = 1});
  NandWriteStore store(failed_path, erased_nand_fixture(), part, timebase, inhibit);
  test::prepare_persistent_program(store);
  constexpr unsigned requested = 0xfe;
  store.write(requested);
  store.command(confirm);
  static_cast<void>(store.advance(program_ticks));
  persistence_require(
      store.device().ready() == true && (store.device().status().value & 1U) == 1 &&
          store.device().cell({.page = persisted_row}) ==
              NandByte{.value = persisted_mask, .known = persisted_mask},
      "A physically inhibited program retains its actual failed result, never fake pass");
}

void publication_failure(const PersistenceDirectory &directory,
                         const bool completion,
                         const unsigned phase) {
  const std::string prefix = completion ? "completion-" : "confirm-";
  const auto path = directory.file(prefix + std::to_string(phase) + ".bfnand");
  {
    NandWriteStore store(path, erased_nand_fixture(), part, timebase, profile());
    test::prepare_persistent_program(store);
    store.write(persisted_cookie);
    if (completion) { store.command(confirm); }
    store.set_observer([phase](const NandFilePhase current) {
      if (static_cast<unsigned>(current) == phase) { throw Fault("Injected timed-store IO"); }
    });
    persistence_reject(
        [&] {
          if (completion) {
            static_cast<void>(store.advance(program_ticks));
          } else {
            store.command(confirm);
          }
        },
        "Injected timed-store IO");
    persistence_require(store.faulted(), "All persistence failures quarantine live owner");
    persistence_reject([&] { static_cast<void>(store.device()); }, "requires reload");
    persistence_reject([&] { static_cast<void>(store.read()); }, "requires reload");
  }
  const NandWriteStore reopened(path, part, timebase, profile());
  const bool published = phase >= static_cast<unsigned>(NandFilePhase::published);
  NandByte expected;
  if (completion && published) { expected = {.value = persisted_cookie, .known = persisted_mask}; }
  if (!completion && !published) { expected = {.value = persisted_mask, .known = persisted_mask}; }
  persistence_require(reopened.device().cell({.page = persisted_row}) == expected,
                      "Reopen reflects atomic rename and never repairs uncertainty");
}

void publication_failures() {
  const PersistenceDirectory directory;
  for (const bool completion : {false, true}) {
    for (unsigned phase = 0; phase < phases; ++phase) {
      publication_failure(directory, completion, phase);
    }
  }
}

}

int main() {
  try {
    ordinary_reads();
    completed_program();
    interrupted_and_reset();
    erase_and_physical_fail();
    publication_failures();
    std::cout << "Timed durable NAND reads/program/erase/reset/reopen/publication guards passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
