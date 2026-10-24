// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/hcs08_system_options.hpp"
#include "hcs08_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <ostream>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::hcs08_programmer {
namespace hf = hcs08_flash;
constexpr unsigned algorithm = 0xa0, algorithm_bytes = 132, ram_first = 0x80, ram_end = 0x480;
constexpr unsigned buffer = 0x200, marker = 0x82, marker_value = 0x72;
constexpr unsigned record_bytes = 514, page_bytes = 512, maximum_steps = 100000;
constexpr unsigned reference_hz = 6000000, data_canary = 0xa5;

class Memory final : public hcs08::Memory {
public:
  explicit Memory(Hcs08IdleLaunch idle, unsigned delay);
  Hcs08Flash flash;
  Hcs08SystemOptions options;
  const unsigned completion_delay;
  std::optional<unsigned> transfer_due, complete_due;
  bool resetting = false;
  unsigned transfers = 0, completions = 0, cuts = 0;
  unsigned array_writes = 0, command_writes = 0, status_writes = 0, sopt_writes = 0;

  std::uint8_t read(std::uint16_t address) override;
  void write(std::uint16_t address, std::uint8_t value) override;
  void begin_instruction(ExecutionContext value) override;
  void begin_reset(ExecutionContext value) override;
  void end_reset() override;
  void check_execute(InstructionFetch fetch) const override;
  void retire_instruction() override;
  [[nodiscard]] bool cuts_after_retirement() const;
};

struct Saved {
  Hcs08Checkpoint cpu;
  Memory memory;
  std::size_t offset;
  std::string suffix;
};

void step(Hcs08 &cpu, Memory &memory, std::ostream &trace);
void finish_pending(Memory &memory);
[[nodiscard]] std::vector<std::uint8_t> read_input(const char *path, unsigned size);
} // namespace blofeld::test::hcs08_programmer
