// SPDX-License-Identifier: GPL-3.0-only
#include "nand_persistence_fixture.hpp"
#include "storage/image_journal.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace {
using namespace blofeld;
using test::persistence_reject;
using test::persistence_require;
using test::PersistenceDirectory;
constexpr unsigned row = test::persisted_row;
constexpr unsigned neighbor = test::persisted_neighbor;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned first_value = 0x42;
constexpr unsigned second_value = 0x24;
constexpr std::size_t transaction_bytes = 1148;
constexpr std::size_t maximum_file_bytes = 1000000;

NandImage change(const NandLocation location, const std::uint8_t value) {
  NandImage result;
  auto &data = result.pages[location.page];
  data.cells.fill({.value = byte_mask, .known = byte_mask});
  data.cells.front().value = value;
  data.programs = {.main = 1};
  result.blocks[location.page / Nand::block_pages] = {
      .valid = true, .program_protected = false, .erase_protected = false};
  return result;
}

void write_prefix(const std::filesystem::path &path, const std::span<const std::uint8_t> bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output.write(reinterpret_cast<const char *>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  output.close();
  persistence_require(static_cast<bool>(output), "Write the exact owned truncated-file fixture");
}

struct Fixture {
  Fixture() : path(directory.file()) {
    save_nand_image(path, test::erased_nand_fixture());
    base = read_private_file(path, maximum_file_bytes);
    const PrivateDirectory parent(path.parent_path());
    const PrivateFileLease lease(parent, path.filename().string() + ".lock");
    PrivateFile file(parent, path.filename(), PrivateFileAccess::read_write);
    ImageJournalPosition position;
    static_cast<void>(load_image_journal(file, position));
    append_image_journal(file, position, change({.page = row}, first_value));
    append_image_journal(file, position, change({.page = neighbor}, second_value));
    bytes = read_private_file(path, maximum_file_bytes);
    persistence_require(bytes.size() == base.size() + (2 * transaction_bytes) &&
                            std::ranges::equal(base, std::span(bytes).first(base.size())),
                        "A page update appends exactly 1148 bytes and never rewrites the base");
  }

  PersistenceDirectory directory;
  std::filesystem::path path;
  std::vector<std::uint8_t> base;
  std::vector<std::uint8_t> bytes;
};

void every_byte_cut(const Fixture &fixture) {
  const auto cut_path = fixture.directory.file("cut.bfnand");
  for (auto cut = fixture.base.size(); cut <= fixture.bytes.size(); ++cut) {
    write_prefix(cut_path, std::span(fixture.bytes).first(cut));
    const auto restored = load_nand_image(cut_path);
    const auto expected_first =
        cut >= fixture.base.size() + transaction_bytes ? first_value : byte_mask;
    const auto expected_second = cut == fixture.bytes.size() ? second_value : first_value;
    persistence_require(restored.pages.at(row).cells.front().value == expected_first &&
                            restored.pages.at(neighbor).cells.front().value == expected_second,
                        "Every byte cut retains exactly the completed update prefix");
  }
}

void corrupt_complete_records(const Fixture &fixture) {
  const auto bad_path = fixture.directory.file("bad.bfnand");
  constexpr std::array offsets{0U, 8U, 12U, 20U, 24U, 60U, 1128U, 1136U, 1147U};
  for (const auto offset : offsets) {
    auto bad = fixture.bytes;
    bad.at(fixture.base.size() + offset) ^= 1U;
    write_prefix(bad_path, bad);
    persistence_reject([&] { static_cast<void>(load_nand_image(bad_path)); }, "journal");
  }
  auto reordered = fixture.base;
  const auto second = std::span(fixture.bytes).last(transaction_bytes);
  reordered.insert(reordered.end(), second.begin(), second.end());
  write_prefix(bad_path, reordered);
  persistence_reject([&] { static_cast<void>(load_nand_image(bad_path)); }, "sequence");
}

void resume_after_partial_update(const Fixture &fixture) {
  const auto path = fixture.directory.file("resume.bfnand");
  constexpr std::size_t partial = 100;
  write_prefix(path,
               std::span(fixture.bytes).first(fixture.base.size() + transaction_bytes + partial));
  const PrivateDirectory parent(path.parent_path());
  const PrivateFileLease lease(parent, path.filename().string() + ".lock");
  PrivateFile file(parent, path.filename(), PrivateFileAccess::read_write);
  ImageJournalPosition position;
  static_cast<void>(load_image_journal(file, position));
  persistence_require(position.offset == fixture.base.size() + transaction_bytes &&
                          position.sequence == 1,
                      "Reopen locates the actual last commit before the incomplete suffix");
  append_image_journal(file, position, change({.page = neighbor}, second_value));
  const auto restored = load_nand_image(path);
  persistence_require(restored.pages.at(row).cells.front().value == first_value &&
                          restored.pages.at(neighbor).cells.front().value == second_value &&
                          file.size() == fixture.bytes.size(),
                      "A new write replaces only the uncommitted suffix and retains prior updates");
}

void process_cuts() {
  const PersistenceDirectory directory;
  constexpr unsigned phases = 5;
  for (unsigned phase = 0; phase < phases; ++phase) {
    const auto path = directory.file(std::to_string(phase) + ".bfnand");
    save_nand_image(path, test::erased_nand_fixture());
    const auto child = fork();
    persistence_require(child >= 0, "Fork the owned process-cut fixture");
    if (child == 0) {
      const PrivateDirectory parent(path.parent_path());
      const PrivateFileLease lease(parent, path.filename().string() + ".lock");
      PrivateFile file(parent, path.filename(), PrivateFileAccess::read_write);
      ImageJournalPosition position;
      static_cast<void>(load_image_journal(file, position));
      append_image_journal(
          file, position, change({.page = row}, first_value), [phase](const PrivateFilePhase at) {
            if (static_cast<unsigned>(at) == phase) { _exit(0); }
          });
      _exit(1);
    }
    int status = 0;
    persistence_require(waitpid(child, &status, 0) == child && status == 0,
                        "The writer process stops at the selected real publication boundary");
    const auto restored = load_nand_image(path);
    const auto expected =
        phase < static_cast<unsigned>(PrivateFilePhase::published) ? byte_mask : first_value;
    persistence_require(restored.pages.at(row).cells.front().value == expected,
                        "A terminated writer exposes only the previous or fully committed image");
  }
}

void snapshot_isolation() {
  NandArray original;
  constexpr unsigned before_group = 511;
  constexpr unsigned after_group = 512;
  constexpr unsigned last = NandArray::pages - 1;
  for (const auto page : {0U, before_group, after_group, last}) {
    original.observe_cell({.page = page}, {.value = first_value, .known = byte_mask});
  }
  original.observe_block(0, {.valid = true});
  static_cast<void>(original.cell({.page = after_group}));
  auto copy = original;
  copy.observe_cell({.page = after_group}, {.value = second_value, .known = byte_mask});
  copy.observe_block(0, {.valid = false});
  original.observe_cell({.page = before_group}, {.value = second_value, .known = byte_mask});
  persistence_require(
      original.cell({.page = after_group}).value == first_value &&
          copy.cell({.page = before_group}).value == first_value &&
          copy.cell({.page = after_group}).value == second_value &&
          original.block(0).valid == true && copy.block(0).valid == false &&
          copy.cell({.page = last}).value == first_value,
      "Page-group boundaries, metadata and a warm read cache survive independent writes");
  const auto region = copy.image({.first_page = before_group, .page_count = 2});
  persistence_require(region.pages.size() == 2 &&
                          region.pages.at(after_group).cells.front().value == second_value,
                      "Persistent region export contains exactly the requested current pages");
}
}

int main() {
  try {
    const Fixture fixture;
    every_byte_cut(fixture);
    corrupt_complete_records(fixture);
    resume_after_partial_update(fixture);
    process_cuts();
    snapshot_isolation();
    std::cout << "Image commits, every byte cut, process termination, corruption and shared pages "
                 "passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
