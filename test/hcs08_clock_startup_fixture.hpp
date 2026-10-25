// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_system_options.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace blofeld::test::hcs08_clock_startup {
namespace hf = hcs08_flash;
namespace hi = hcs08_icg;
using hcs08::Access;
constexpr unsigned ram_first = 0x80, ram_end = 0x480, records_bytes = 3598;
constexpr unsigned factory_trim = 0xffbe, entry = 0xe000, input_pc = 0xe04e;
constexpr unsigned input_retired = 6176, background_retired = 6181, trim_retired = 6185;
constexpr unsigned trim_pc = 0xe1a8, control1_pc = 0xe1aa, control2_pc = 0xe1ad, status_pc = 0xe1b0;
constexpr unsigned status_retired = 6188, stack = 0x47d, expected_hx = 0x12c;
constexpr unsigned ccr_known = 0xef, prior_live_trim = 0xa7;
constexpr std::array<unsigned, 7> pages{0xe000, 0xe200, 0xe400, 0xe600, 0xe800, 0xea00, 0xfe00};

struct Input {
  unsigned seed;
  std::uint8_t trim;
  int observation;
};

// Untimed, immutable-FLASH startup fixture. No oscillator/NVM clock or command
// completion is supplied; original execution must stop at unqualified ICGS1.
class ApplicationMemory final : public hcs08::MaskedMemory {
public:
  ApplicationMemory(std::span<const std::uint8_t> records, Input input);
  Hcs08Flash flash;
  Hcs08Gpio gpio{Hcs08GpioContract::candidate_gt8a};
  Hcs08Icg icg{Hcs08IcgContract::candidate_gt8a_no_clock_events, Hcs08IcgReset::warm};
  Hcs08Spi spi{Hcs08SpiContract::candidate_gt8a_reference_master_edges};
  Hcs08Adc adc{Hcs08AdcContract::candidate_gt8a_reference_stable_input};
  Hcs08SystemOptions options{Hcs08FlashContract::candidate_gt8a};
  std::vector<Access> peripheral_accesses, flash_reads;
  bool resetting = false;
  unsigned interrupt_entries = 0, interrupt_acknowledgements = 0;

  void bind(int observation);
  void warm_reset();
  void begin_instruction(ExecutionContext value) override;
  void begin_interrupt(InterruptRequest request, bool reuse_stack) override;
  void acknowledge_interrupt(std::uint16_t vector) override;
  Hcs08KnownByte read_known_byte(std::uint16_t address) override;
  void write_known_byte(ByteWrite access, std::uint8_t mask) override;
  [[nodiscard]] std::optional<InterruptRequest> interrupt_request(std::uint8_t ccr) const override;
  void begin_reset(ExecutionContext value) override;
  void end_reset() override;
  std::uint8_t read(std::uint16_t address) override;
  void write(std::uint16_t address, std::uint8_t value) override;
  [[nodiscard]] std::uint8_t peek(std::uint16_t address) const override;
  void check_execute(InstructionFetch fetch) const override;

private:
  void check_reset_access(std::uint16_t address, const std::string &operation) const;
};

[[nodiscard]] Hcs08State warm_state(unsigned seed);
void configured(const ApplicationMemory &memory, const Hcs08 &cpu);
void ram_contents(const ApplicationMemory &memory, bool entered_main);
void same_endpoint(const ApplicationMemory &actual, const ApplicationMemory &expected);
} // namespace blofeld::test::hcs08_clock_startup
