// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/nand.hpp"
#include "blofeld/nand_array.hpp"
#include "blofeld/nand_file.hpp"
#include "blofeld/nand_image.hpp"
#include "blofeld/nand_store.hpp"
#include "nand_persistence_fixture.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>

namespace blofeld::test {
namespace {
constexpr unsigned program_confirm = 0x10;
constexpr unsigned reset = 0xff;
constexpr unsigned file_phases = 5;

void check_disk_target(const std::filesystem::path &path, const bool uncertain) {
  const auto image = load_nand_image(path);
  const auto &page = image.pages.at(persisted_row);
  for (const auto cell : page.cells) {
    persistence_require(
        cell ==
            (uncertain ? NandByte{} : NandByte{.value = persisted_mask, .known = persisted_mask}),
        "File target must retain either pre-command erased data or whole-page uncertainty");
  }
  persistence_require(
      page.programs.has_value() != uncertain &&
          image.blocks.at(persisted_block).valid.has_value() != uncertain &&
          image.pages.at(persisted_neighbor).cells.at(0) ==
              NandByte{.value = persisted_cookie, .known = persisted_mask},
      "Persistent uncertainty must include history/block qualification and preserve neighbors");
}

void test_write_ahead_and_completed_restart() {
  const PersistenceDirectory directory;
  const auto path = directory.file();
  {
    NandStore store(path, erased_nand_fixture());
    prepare_persistent_program(store);
    store.write(persisted_cookie);
    unsigned observations = 0;
    store.set_observer([&](const NandFilePhase phase) {
      persistence_require(
          store.pending() == NandPending::none,
          "A confirm must not become visible before its persistent transaction returns");
      check_disk_target(path, phase >= NandFilePhase::published);
      ++observations;
    });
    store.command(program_confirm);
    persistence_require(observations == file_phases && store.pending() == NandPending::program &&
                            store.ready() == false,
                        "All five host phases precede the visible pending program");
    check_disk_target(path, true);
    store.set_observer([&](const NandFilePhase phase) {
      persistence_require(store.pending() == NandPending::program && store.ready() == false,
                          "Completion must not become visible before durable outcome publication");
      if (phase < NandFilePhase::published) {
        check_disk_target(path, true);
      } else {
        const auto image = load_nand_image(path);
        const auto programs = image.pages.at(persisted_row).programs;
        persistence_require(
            programs.has_value() && programs->main == 1 && programs->spare == 0 &&
                image.pages.at(persisted_row).cells.at(0) ==
                    NandByte{.value = persisted_cookie, .known = persisted_mask},
            "Published successful program must retain exact data and consumed history");
      }
      ++observations;
    });
    store.observe_completion(NandOutcome::pass);
    persistence_require(observations == file_phases * 2 && store.ready() == true,
                        "Ready/pass becomes visible only after the successful file transaction");
  }
  NandStore restored(path);
  persistence_require(
      !restored.ready().has_value() && restored.status().known == 0 &&
          restored.cell({.page = persisted_row}) ==
              NandByte{.value = persisted_cookie, .known = persisted_mask},
      "Process reconstruction retains physical data without restoring WP/ready/result");
  prepare_persistent_program(restored);
  restored.write(0);
  persistence_reject([&] { restored.command(program_confirm); }, "main partial-program limit");
  persistence_require(!restored.faulted(),
                      "Rejected device command before IO must not fault the store");
}

void test_discarded_staging_and_active_power_loss() {
  const PersistenceDirectory directory;
  const auto path = directory.file();
  {
    NandStore store(path, erased_nand_fixture());
    prepare_persistent_program(store);
    store.set_observer([](NandFilePhase) { throw Fault("Unexpected staging publication"); });
    for (unsigned index = 0; index < Nand::page_bytes; ++index) { store.write(0); }
    check_disk_target(path, false);
    store.set_observer({});
    store.command(reset);
    store.observe_completion();
    check_disk_target(path, false);
  }
  {
    NandStore restored(path);
    prepare_persistent_program(restored);
    restored.write(0);
    restored.command(program_confirm);
    restored.power_loss();
    check_disk_target(path, true);
    persistence_require(!restored.ready().has_value(),
                        "Explicit power loss withdraws live readiness");
  }
  NandStore interrupted(path);
  prepare_persistent_program(interrupted);
  interrupted.write(0);
  persistence_reject([&] { interrupted.command(program_confirm); },
                     "unknown/invalid block condition");
  check_disk_target(path, true);
}

void test_fail_stop_at_all_file_phases() {
  const PersistenceDirectory directory;
  for (unsigned requested = 0; requested < file_phases; ++requested) {
    const auto path = directory.file("failed-" + std::to_string(requested) + ".bfnand");
    {
      NandStore store(path, erased_nand_fixture());
      prepare_persistent_program(store);
      store.write(persisted_cookie);
      store.set_observer([requested](const NandFilePhase phase) {
        if (static_cast<unsigned>(phase) == requested) {
          throw Fault("Injected publication failure");
        }
      });
      persistence_reject([&] { store.command(program_confirm); }, "Injected publication failure");
      persistence_require(store.faulted(),
                          "Failed persistence must force reload, including failure after rename");
      persistence_reject([&] { store.command(0); }, "requires reload");
      persistence_reject([&] { static_cast<void>(store.device()); }, "requires reload");
      persistence_reject([&] { static_cast<void>(store.read()); }, "requires reload");
    }
    const NandStore reloaded(path);
    check_disk_target(path, requested >= static_cast<unsigned>(NandFilePhase::published));
    persistence_require(!reloaded.ready().has_value(),
                        "A failed publication cannot restore volatile readiness");
  }
  for (const auto &entry : std::filesystem::directory_iterator(directory.file().parent_path())) {
    persistence_require(entry.path().filename().string().find(".tmp.") == std::string::npos,
                        "Caught host errors must clean every unpublished temporary file");
  }
}

void test_writer_lease_and_reentrancy() {
  const PersistenceDirectory directory;
  const auto path = directory.file();
  {
    NandStore store(path, erased_nand_fixture());
    persistence_reject([&] { const NandStore second(path); }, "writer lease");
    persistence_reject([&] { save_nand_image(path, erased_nand_fixture()); }, "writer lease");
    check_disk_target(path, false);
    prepare_persistent_program(store);
    store.write(0);
    store.set_observer([&](const NandFilePhase phase) {
      if (phase == NandFilePhase::created) { store.command(0); }
    });
    persistence_reject([&] { store.command(program_confirm); }, "Reentrant NAND store mutation");
    persistence_require(store.faulted(),
                        "A reentrant observer cannot resume a partly published transaction");
  }
  persistence_reject([&] { const NandStore overwritten(path, erased_nand_fixture()); },
                     "already exists");
  const NandStore reopened(path);
  check_disk_target(path, false);
}

void test_store_directory_identity_survives_namespace_replacement() {
  const PersistenceDirectory directory;
  const auto parent = directory.file("parent");
  const auto moved = directory.file("moved");
  std::filesystem::create_directory(parent);
  const auto path = parent / "state.bfnand";
  NandStore original(path, erased_nand_fixture());
  prepare_persistent_program(original);
  original.write(0);
  std::filesystem::rename(parent, moved);
  std::filesystem::create_directory(parent);
  const NandStore replacement(path, erased_nand_fixture());
  original.command(program_confirm);
  check_disk_target(moved / "state.bfnand", true);
  check_disk_target(path, false);
  original.observe_completion(NandOutcome::pass);
  const auto completed = load_nand_image(moved / "state.bfnand");
  persistence_require(
      completed.pages.at(persisted_row).cells.at(0) == NandByte{.known = persisted_mask},
      "An existing store must keep its original directory and lease through later transactions");
  check_disk_target(path, false);
}

void test_pinned_directory_and_bounded_loader() {
  const PersistenceDirectory directory;
  const auto parent = directory.file("parent");
  const auto moved = directory.file("moved");
  std::filesystem::create_directory(parent);
  const auto path = parent / "state.bfnand";
  save_nand_image(path, {});
  NandImage following;
  following.pages[0].cells.at(0) = {.value = persisted_cookie, .known = persisted_mask};
  save_nand_image(path, following, [&](const NandFilePhase phase) {
    if (phase != NandFilePhase::synced) { return; }
    std::filesystem::rename(parent, moved);
    std::filesystem::create_directory(parent);
    save_nand_image(path, {});
  });
  persistence_require(
      load_nand_image(path).pages.empty() &&
          load_nand_image(moved / "state.bfnand").pages.at(0).cells.at(0).value == persisted_cookie,
      "Publication must use the pinned directory FD after its original path is renamed/replaced");
  const auto oversized = directory.file("oversized.bfnand");
  std::ofstream(oversized).close();
  std::filesystem::resize_file(oversized, nand_image_max_bytes + 1);
  persistence_reject([&] { static_cast<void>(load_nand_image(oversized)); },
                     "type or length exceeds");
  const auto fifo = directory.file("pipe.bfnand");
  persistence_require(mkfifo(fifo.c_str(), S_IRUSR | S_IWUSR) == 0,
                      "Own FIFO fixture must be created");
  persistence_reject([&] { static_cast<void>(load_nand_image(fifo)); }, "type or length exceeds");
}
}
}

int main() {
  try {
    blofeld::test::test_write_ahead_and_completed_restart();
    blofeld::test::test_discarded_staging_and_active_power_loss();
    blofeld::test::test_fail_stop_at_all_file_phases();
    blofeld::test::test_writer_lease_and_reentrancy();
    blofeld::test::test_store_directory_identity_survives_namespace_replacement();
    blofeld::test::test_pinned_directory_and_bounded_loader();
    std::cout
        << "NAND write-ahead publication, fail-stop, writer lease and restart checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
