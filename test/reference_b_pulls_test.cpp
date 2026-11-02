// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "esai_fixture.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
constexpr unsigned entry = 0x9000, flash_base = 0x8000, nop = 0xa7, byte_mask = 0xff;
constexpr unsigned protection = 0x7f0d, security = 0x7f0f, unsecured = 0xfe;
constexpr unsigned oscillator = 12000000, timebase = 540000000, budget = 600, suffix = 300;
constexpr unsigned load_a = 0x86, store_a = 0x7a, read_a = 0xb6;
constexpr unsigned pull_up = 0x82, pull_down = 0x7d, strong = 0x55;
constexpr unsigned serial = 0x16, select_pin = 2, clock_pin = 4, mosi_pin = 16;
constexpr unsigned inbound = 0x123456, word_bits = 24, word_mask = 0xffffff;
constexpr unsigned scheduled_release = 450, early_observations = 24;
constexpr unsigned latch_word_writes = (3 * word_bits) + 3;
// DSP56300FM MOVEP immediate/HRX and JCLR17 templates; UM7 SPI24 control.
constexpr unsigned movep_hckr = 0x07f410, movep_hcsr = 0x07f411, spi24 = 9;
constexpr unsigned jclr_hrdf = 0x019191, hrx_x0 = 0x044494, jump_absolute = 0x0af080;
constexpr unsigned boot = 0xff0000, poll = boot + 4, received = boot + 7;
constexpr std::uint64_t external_tick = 9000;
unsigned continuations = 0, refusals = 0;

void store(std::vector<std::uint8_t> &code, const unsigned address, const std::uint8_t value) {
  code.insert(code.end(),
              {load_a,
               value,
               store_a,
               static_cast<std::uint8_t>(address >> 8U),
               static_cast<std::uint8_t>(address & byte_mask)});
}

void check_b(std::vector<std::uint8_t> &code, const std::uint8_t expected) {
  // HC12 LDAA/CMPA/BEQ. A mismatch performs a real unsupported native read.
  constexpr unsigned compare = 0x81;
  constexpr unsigned branch_equal = 0x27;
  constexpr unsigned unmapped = 0x30;
  code.insert(code.end(),
              {read_a, 0, hc12::portb, compare, expected, branch_equal, 3, read_a, unmapped, 0});
}

McuFlashImage image(const std::vector<std::uint8_t> &code) {
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  for (std::size_t index = 0; index < code.size(); ++index) {
    bytes.at(entry - flash_base + index) = code[index];
  }
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

reference::Config config() {
  reference::Config c;
  c.mcu = {.clock = {.oscillator_hz = oscillator,
                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
           .memory_timing = McuRunMemory::unstretched_internal};
  c.dsp_clock = {.extal_hz = oscillator,
                 .timebase_hz = timebase,
                 .switch_timing = DspClockSwitch::reference_cycle_boundary};
  c.dsp = {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
           .maximum_cycles_per_period = 16,
           .writes = McuWriteOrder::before_mcu,
           .external_writes = McuWriteOrder::before_mcu,
           .reset_dsp_first = true,
           .initial_reset = OwnDspResetInputs{
               .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  c.dma_grant = DspDmaGrant::available;
  c.release_ticks = scheduled_release;
  c.operations = budget;
  c.select_n = select_pin;
  c.clock = clock_pin;
  c.mosi = mosi_pin;
  c.pad_order = reference::PadOrder::clock_before_select;
  c.port_b_pulls = {.up = pull_up, .down = pull_down};
  c.initial = {{.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  return c;
}

struct Fixture {
  Fixture(const reference::Config &c,
          const std::vector<std::uint8_t> &code,
          const bool receiving = false)
      : flash(image(code)),
        rom(receiving ? std::vector<std::uint32_t>{movep_hckr,
                                                   0,
                                                   movep_hcsr,
                                                   spi24,
                                                   jclr_hrdf,
                                                   poll,
                                                   hrx_x0,
                                                   jump_absolute,
                                                   received}
                      : std::vector<std::uint32_t>{0, jump_absolute, boot}),
        execution{.image = flash,
                  .rom = rom,
                  .specification = BLOFELD_DEFAULT_SLA,
                  .flash_out = {},
                  .trace = &trace,
                  .instructions = budget},
        board(c, execution) {}

  McuFlashImage flash;
  std::vector<std::uint32_t> rom;
  std::ostringstream trace;
  reference::Execution execution;
  reference::Board board;
};

void steps(reference::Board &board, const unsigned count) {
  for (unsigned index = 0; index < count; ++index) { board.advance(); }
}

std::vector<std::uint8_t> passive_program(const bool timed) {
  std::vector<std::uint8_t> code;
  check_b(code, pull_up);
  store(code, hc12::portb, strong);
  store(code, hc12::ddrb, byte_mask);
  check_b(code, strong);
  store(code, hc12::ddrb, 0);
  constexpr unsigned delay = 128;
  code.insert(code.end(), delay, nop);
  check_b(code, timed ? pull_up | 1U : pull_up);
  return code;
}

void passive_and_restore() {
  for (const bool timed : {false, true}) {
    for (const unsigned cut : {0U, 12U, 50U, 100U}) {
      auto c = config();
      if (timed) {
        c.events.push_back(
            {.ticks = external_tick, .port = McuPort::b, .drive = {.mask = 1, .levels = 1}});
      }
      Fixture f(c, passive_program(timed));
      f.board.initialize();
      steps(f.board, cut);
      const auto saved = f.board.checkpoint();
      const auto before = f.board.status();
      f.trace.str({});
      steps(f.board, suffix);
      const auto expected = f.board.status();
      const auto trace = f.trace.str();
      require(expected.mcu.d == ((timed ? pull_up | 1U : pull_up) << 8U) && expected.frames == 1 &&
                  expected.writes == 1 && !expected.faulted,
              "Literal native reads prove passive idle, output dominance and released inputs");
      for (unsigned replay = 0; replay < 2; ++replay) {
        f.board.restore(saved);
        require(f.board.status() == before, "Passive net restore retains the exact prefix");
        f.trace.str({});
        steps(f.board, suffix);
        require(f.board.status() == expected && f.trace.str() == trace,
                "Fresh native owners restore external B drivers, pulls and SPI observations");
        ++continuations;
      }
    }
  }
}

void store_b(std::vector<std::uint8_t> &code,
             const std::uint8_t value,
             const std::optional<unsigned> word_address) {
  if (!word_address) {
    store(code, hc12::portb, value);
    return;
  }
  constexpr unsigned load_d = 0xcc;
  constexpr unsigned store_d = 0x7c;
  constexpr std::uint8_t other_byte = 0xa5;
  const bool head = *word_address == hc12::portb;
  code.insert(code.end(),
              {load_d,
               head ? value : other_byte,
               head ? other_byte : value,
               store_d,
               0,
               static_cast<std::uint8_t>(*word_address)});
}

std::vector<std::uint8_t> transfer(const bool direction,
                                   const std::optional<unsigned> word_address = std::nullopt) {
  std::vector<std::uint8_t> code;
  store_b(code, direction ? 0 : select_pin, word_address);
  store(code, hc12::ddrb, direction ? clock_pin | mosi_pin : serial);
  constexpr unsigned delay = 64;
  code.insert(code.end(), delay, nop);
  if (direction) {
    store(code, hc12::ddrb, serial);
  } else {
    store_b(code, 0, word_address);
  }
  for (unsigned remaining = word_bits; remaining > 0; --remaining) {
    const auto bit = remaining - 1;
    const bool high = (inbound & (1U << bit)) != 0;
    if (direction) {
      const auto base = static_cast<std::uint8_t>(select_pin | clock_pin | (high ? 0 : mosi_pin));
      store(code, hc12::ddrb, base);
      store(code, hc12::ddrb, static_cast<std::uint8_t>(base & ~clock_pin));
      store(code, hc12::ddrb, base);
    } else {
      const auto data = static_cast<std::uint8_t>(high ? mosi_pin : 0);
      store_b(code, data, word_address);
      store_b(code, data | clock_pin, word_address);
      store_b(code, data, word_address);
    }
  }
  if (direction) {
    store(code, hc12::ddrb, clock_pin | mosi_pin);
  } else {
    store_b(code, select_pin, word_address);
  }
  store(code, hc12::ddrb, 0);
  return code;
}

void word_spi_stores() {
  for (const unsigned cut : {0U, 100U, 180U}) {
    auto c = config();
    c.port_b_pulls = {.up = serial, .down = static_cast<std::uint8_t>(byte_mask & ~serial)};
    Fixture f(c, transfer(false, hc12::porta), true);
    steps(f.board, cut);
    const auto saved = f.board.checkpoint();
    const auto before = f.board.status();
    f.trace.str({});
    steps(f.board, suffix);
    const auto expected = f.board.status();
    const auto trace = f.trace.str();
    require((expected.dsp.x & word_mask) == inbound && expected.frames == 1 &&
                expected.writes == latch_word_writes && !expected.faulted,
            "Actual aligned STD delivers its PORTB tail as the literal native SHI word");
    for (unsigned replay = 0; replay < 2; ++replay) {
      f.board.restore(saved);
      require(f.board.status() == before, "Word-driven SPI restores the exact native prefix");
      f.trace.str({});
      steps(f.board, suffix);
      require(f.board.status() == expected && f.trace.str() == trace,
              "Aligned word observations survive fresh-owner restore");
      ++continuations;
    }
  }
}

void native_receive() {
  for (const bool direction : {false, true}) {
    for (const unsigned cut : {0U, 70U, 100U, 140U, 180U}) {
      auto c = config();
      // Direction-driven rising edges release known-low outputs to explicit pull-ups.
      c.port_b_pulls = {.up = serial, .down = static_cast<std::uint8_t>(byte_mask & ~serial)};
      Fixture f(c, transfer(direction), true);
      steps(f.board, cut);
      const auto saved = f.board.checkpoint();
      const auto before = f.board.status();
      f.trace.str({});
      steps(f.board, suffix);
      const auto expected = f.board.status();
      const auto trace = f.trace.str();
      require((expected.dsp.x & word_mask) == inbound && expected.frames == 1 &&
                  expected.writes == (direction ? 1U : latch_word_writes),
              "Actual native SHI polling/HRX returns the independently chosen 24-bit word");
      for (unsigned replay = 0; replay < 2; ++replay) {
        f.board.restore(saved);
        require(f.board.status() == before, "SPI direction snapshot preserves both native CPUs");
        f.trace.str({});
        steps(f.board, suffix);
        require(f.board.status() == expected && f.trace.str() == trace,
                "In-flight DDR-driven and latch-driven SPI words survive fresh-owner restore");
        ++continuations;
      }
    }
  }
}

void expect_fault(const reference::Config &c,
                  const std::vector<std::uint8_t> &code,
                  const std::string_view message) {
  Fixture f(c, code);
  f.board.initialize();
  const auto saved = f.board.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    f.board.restore(saved);
    const auto error = rejects([&] { steps(f.board, suffix); });
    require(error.contains(message) && error.contains("pc=") && f.board.status().faulted,
            "Unknown/conflicting native pads retain their causal access and execution context");
    ++refusals;
  }
}

void word_direction_store() {
  constexpr unsigned load_d = 0xcc;
  constexpr unsigned store_d = 0x7c;
  std::vector<std::uint8_t> code;
  store(code, hc12::portb, select_pin);
  for (const unsigned direction : {serial, 0U}) {
    // HC12 LDD immediate / STD extended: high byte changes DDRA, low byte DDRB.
    code.insert(code.end(),
                {load_d, byte_mask, static_cast<std::uint8_t>(direction), store_d, 0, hc12::ddra});
  }
  check_b(code, pull_up);
  Fixture f(config(), code);
  steps(f.board, suffix);
  const auto state = f.board.status();
  require(state.mcu.d == (pull_up << 8U) && state.frames == 0 && state.writes == 1,
          "Native word stores deliver only their actual DDRB byte, preserving SPI idle");
}

void electrical_guards() {
  auto c = config();
  c.initial.push_back({.port = McuPort::b, .drive = {.mask = clock_pin}});
  expect_fault(c, passive_program(false), "conflicts=0x04");
  c = config();
  c.events.push_back({.ticks = external_tick, .port = McuPort::b, .drive = {.unknown = clock_pin}});
  expect_fault(c, passive_program(false), "Unknown reference SPI pad read");
  c = config();
  c.port_b_pulls = {.up = select_pin, .down = clock_pin | mosi_pin};
  expect_fault(c, passive_program(false), "Unknown MCU GPIO input");
  c = config();
  std::vector<std::uint8_t> code;
  store(code, hc12::ddrb, serial);
  expect_fault(c, code, "Unknown reference SPI pad read");
  c = config();
  code.clear();
  constexpr unsigned enable_b_pulls = 0x92;
  store(code, hc12::pucr, enable_b_pulls);
  expect_fault(c, code, "conflicts=0x14");
  c = config();
  c.port_b_pulls.reset();
  c.initial.push_back({.port = McuPort::b, .drive = {.mask = serial, .levels = select_pin}});
  expect_fault(c, passive_program(false), "Unknown MCU GPIO input");
  c = config();
  c.port_b_pulls = {.up = serial, .down = static_cast<std::uint8_t>(byte_mask & ~serial)};
  // The MCU reference profile deliberately has no timing for odd register words.
  expect_fault(
      c, transfer(false, hc12::portb), "Unqualified MCU reference W address=0x0001 size=2");
}

void early_lcd_inputs() {
  auto c = config();
  c.lcd_parallel = {.reset_release_ticks = 0};
  c.initial = {{.port = McuPort::e, .drive = {.mask = 2, .levels = 2, .floating = 0x10}}};
  Fixture f(c, {});
  steps(f.board, early_observations);
  require(f.board.status().lcd_cycles == std::array<std::uint64_t, 3>{} &&
              f.board.display_image({.width = 128}).has_value(),
          "Passive LCD deselection survives high input E without inventing display cycles");
  c.port_b_pulls = {.up = select_pin, .down = clock_pin | mosi_pin};
  require(rejects([&] { const Fixture unknown(c, {}); }).contains("Unknown ST7565 chip select"),
          "Unknown chip selects remain strict when the LCD cannot prove deselection");
}

void configuration_guards(const std::filesystem::path &path) {
  constexpr std::string_view base =
      "blofeld-reference-board 2\n"
      "clock 12000000 0 12000000 540000000 reference-boundaries\n"
      "dsp-execution reference-effects\nordering dsp-before-mcu dsp-before-mcu 16\n"
      "spi-b 02 04 10 clock-before-select\ndsp-release 450 low\noperations 32\n";
  const auto write = [&](const std::string_view extra) {
    std::ofstream file(path);
    file << base << extra;
    file.close();
    require(static_cast<bool>(file), "Write the authored parser input");
  };
  write("port-b-pulls 82 7d\n");
  const auto parsed = reference::load_config(path);
  require(parsed.port_b_pulls && parsed.port_b_pulls->up == pull_up &&
              parsed.port_b_pulls->down == pull_down,
          "Actual configuration parser retains both explicit passive masks");
  for (const auto *const bad : {"port-b-pulls 80 80\n",
                                "port-b-pulls 100 00\n",
                                "port-b-pulls 82\n",
                                "port-b-pulls 82 7d extra\n",
                                "port-b-pulls 82 7d\nport-b-pulls 00 ff\n",
                                "port-b-pulls 82 7d\nshi-miso B 08 direct\n",
                                "port-b-pulls 82 7d\nloopback E 04 B 08\n"}) {
    write(bad);
    require(!rejects([&] { static_cast<void>(reference::load_config(path)); }).empty(),
            "Malformed, duplicate and conflicting network configuration is refused");
  }
  for (const auto drive : {PinDrive{.levels = 1},
                           PinDrive{.mask = 1, .floating = 1},
                           PinDrive{.mask = 1, .unknown = 1}}) {
    auto c = config();
    c.initial.push_back({.port = McuPort::b, .drive = drive});
    require(rejects([&] { const Fixture invalid(c, {}); }).contains("Invalid external source"),
            "Direct typed input also preserves all strong/floating/unknown source validation");
  }
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 2) { throw Fault("Expected the private parser artifact path"); }
    configuration_guards(arguments[1]);
    passive_and_restore();
    native_receive();
    word_spi_stores();
    word_direction_store();
    electrical_guards();
    early_lcd_inputs();
    std::cout << "Passive B nets: native reads, SPI HRX, " << continuations
              << " fresh-owner suffixes, " << refusals << " native fault suffixes passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
