// SPDX-License-Identifier: GPL-3.0-only
#include "boot_rom.hpp"
#include "process/configuration.hpp"
#include "profile.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}
}

int main(const int argc, char **argv) try {
  require(argc == 2, "Supply a build artifact directory");
  const auto root = std::filesystem::path(argv[1]) / std::to_string(::getpid());
  require(std::filesystem::create_directories(root), "Create owned unit directory");
  const auto file = root / "default.profile";
  {
    std::ofstream output(file);
    output << "# Local paths\nboot-rom boot.bin\ndevice-state unit.bfnvm\n"
              "nand \"sample image.bfnand\"\n";
  }
  const auto profile = blofeld::app::load_profile(file);
  require(profile.boot_rom == root / "boot.bin" && profile.device_state == root / "unit.bfnvm" &&
              profile.storage == root / "sample image.bfnand" &&
              profile.boot_format == blofeld::app::BootFormat::rom,
          "Relative and quoted paths are resolved against the profile");
  {
    std::ofstream output(file);
    output << "boot-prefix prefix.bin\ndevice-state unit.bfnvm\nnand unit.bfnand\n";
  }
  const auto prefix = blofeld::app::load_profile(file);
  require(prefix.boot_rom == root / "prefix.bin" &&
              prefix.boot_format == blofeld::app::BootFormat::spi_prefix,
          "An explicitly selected document-derived SPI prefix preserves its input kind");
  for (const auto *invalid : {"boot-rom a\nboot-rom b\n",
                              "boot-rom a\nboot-prefix b\ndevice-state c\n",
                              "boot-prefix a\nboot-rom b\ndevice-state c\n",
                              "boot-rom a\ndevice-state c\ndisplay elsewhere\n",
                              "instruction-budget 10\n",
                              ""}) {
    {
      std::ofstream output(file);
      output << invalid;
    }
    bool rejected = false;
    try {
      static_cast<void>(blofeld::app::load_profile(file));
    } catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Duplicate, unknown and empty profiles fail explicitly");
  }
  require(blofeld::app::default_profile().filename() == "default.profile", "Default profile name");
  require(blofeld::host::runtime_home().is_absolute(), "Display runtime root is absolute");
  std::filesystem::remove_all(root);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
