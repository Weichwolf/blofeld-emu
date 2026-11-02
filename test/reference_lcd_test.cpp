// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_runtime.hpp"
#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "display/lcd_image.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "esai_fixture.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
constexpr unsigned entry = 0x9000, flash_base = 0x8000, nop = 0xa7, byte_mask = 0xff;
constexpr unsigned protection = 0x7f0d, security = 0x7f0f, unsecured = 0xfe;
constexpr unsigned oscillator = 12000000, timebase = 540000000, budget = 900, suffix = 520;
constexpr unsigned load_a = 0x86, store_a = 0x7a, read_a = 0xb6, read_b = 0xf6;
constexpr unsigned enable = 0x10, idle_b = 0x82, command_b = 0x42, data_b = 0x43;
constexpr unsigned reading_b = 0x63, status_b = 0x62, page0 = 0xb0, display_on = 0xaf;
constexpr unsigned first = 0xa5, second = 0x5a, dummy_result = 0x1900, data_result = 0x1901;
constexpr LcdPanel panel{.width = 128};

enum class Mode : std::uint8_t { normal, unknown_read, status, release_during_read };

void store(std::vector<std::uint8_t> &code, const unsigned address, const std::uint8_t value) {
  code.insert(code.end(),
              {load_a,
               value,
               store_a,
               static_cast<std::uint8_t>(address >> 8U),
               static_cast<std::uint8_t>(address & byte_mask)});
}

void write_lcd(std::vector<std::uint8_t> &code, const unsigned value, const bool data) {
  store(code, hc12::porta, static_cast<std::uint8_t>(value));
  store(code, hc12::portb, data ? data_b : command_b);
  store(code, hc12::porte, enable);
  store(code, hc12::porte, 0);
}

void read_lcd(std::vector<std::uint8_t> &code,
              const unsigned destination,
              const bool cross_reset_release = false) {
  store(code, hc12::porte, enable);
  if (cross_reset_release) {
    constexpr unsigned reset_hold_delay = 128;
    code.insert(code.end(), reset_hold_delay, nop);
  }
  code.insert(code.end(), {read_a, 0, 0});
  code.insert(code.end(),
              {store_a,
               static_cast<std::uint8_t>(destination >> 8U),
               static_cast<std::uint8_t>(destination & byte_mask)});
  store(code, hc12::porte, 0);
}

McuFlashImage image(const Mode mode) {
  std::vector<std::uint8_t> code;
  store(code, hc12::porte, 0);
  store(code, hc12::ddre, enable);
  store(code, hc12::portb, idle_b);
  store(code, hc12::ddrb, byte_mask);
  store(code, hc12::porta, 0);
  store(code, hc12::ddra, byte_mask);
  constexpr unsigned idle_delay = 32;
  code.insert(code.end(), idle_delay, nop);
  if (mode == Mode::normal) {
    for (const auto command : {page0, 0U, enable, display_on}) { write_lcd(code, command, false); }
    write_lcd(code, first, true);
    write_lcd(code, second, true);
    write_lcd(code, 0, false);
  }
  if (mode == Mode::release_during_read) {
    constexpr unsigned pull_enable_a = 0x91;
    store(code, hc12::pucr, pull_enable_a);
  }
  store(code, hc12::ddra, 0);
  store(code, hc12::portb, mode == Mode::status ? status_b : reading_b);
  read_lcd(code, dummy_result, mode == Mode::release_during_read);
  if (mode != Mode::release_during_read) { read_lcd(code, data_result); }
  code.insert(code.end(),
              {read_a,
               dummy_result >> 8U,
               dummy_result & byte_mask,
               read_b,
               data_result >> 8U,
               data_result & byte_mask});
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

reference::Config config(const bool power_on = false) {
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
  constexpr unsigned held_dsp_release = 1000000000;
  constexpr unsigned nominal_lcd_release = 3000;
  constexpr unsigned cold_lcd_release = 12000;
  c.release_ticks = held_dsp_release;
  c.lcd_parallel = {.reset_release_ticks = power_on ? cold_lcd_release : nominal_lcd_release};
  c.operations = budget;
  c.select_n = 2;
  c.clock = 4;
  c.mosi = 16;
  constexpr unsigned serial = 0x16;
  c.initial = {{.port = McuPort::b, .drive = {.mask = serial, .levels = 2}},
               {.port = McuPort::e, .drive = {.mask = 2, .levels = 2}}};
  if (power_on) {
    c.power_on = reference::PowerOnTiming{.inputs = {3}, .reset_pad = McuResetPad::high};
  }
  return c;
}

struct Fixture {
  explicit Fixture(const reference::Config &c, const Mode mode = Mode::normal)
      : flash(image(mode)),
        execution{.image = flash,
                  .rom = rom,
                  .specification = BLOFELD_DEFAULT_SLA,
                  .flash_out = {},
                  .trace = &trace,
                  .instructions = budget},
        board(c, execution) {}

  McuFlashImage flash;
  std::vector<std::uint32_t> rom{0};
  std::ostringstream trace;
  reference::Execution execution;
  reference::Board board;
};

void steps(reference::Board &board, const unsigned count) {
  for (unsigned index = 0; index < count; ++index) { board.advance(); }
}

void verify(const Fixture &f) {
  const auto state = f.board.status();
  require(state.mcu.d == first && state.lcd_cycles == std::array<std::uint64_t, 3>{5, 2, 2},
          "Native MCU stores/read cycles produce the literal dummy/data result and cycle counts");
  const auto image = f.board.display_image(panel);
  if (!image) { throw Fault("The connected controller supplies a read-only image"); }
  for (unsigned y = 0; y < 8; ++y) {
    for (unsigned x = 0; x < 2; ++x) {
      const auto value = x == 0 ? first : second;
      const auto pixel = (value & (1U << y)) != 0 ? LcdPixel::on : LcdPixel::off;
      require(image->pixel(x, y) == pixel, "Native LCD data reaches independently expected pixels");
    }
  }
  require(image->pixel(2, 0) == LcdPixel::unknown && image->pixel(0, 8) == LcdPixel::unknown,
          "Unwritten display RAM remains unknown instead of becoming a fabricated blank frame");
  const auto before = f.board.status();
  const auto trace = f.trace.str();
  require(f.board.display_image(panel) == image && f.board.status() == before &&
              f.trace.str() == trace,
          "Image capture cannot consume a native LCD read or alter either CPU");
}

void checkpoints(const bool passive = false) {
  constexpr std::array cuts{0U, 12U, 43U, 48U, 70U, 90U, 110U, 130U};
  for (const auto power_on : {false, true}) {
    for (const auto cut : cuts) {
      auto c = config(power_on);
      if (passive) {
        c.port_b_pulls = {.up = idle_b, .down = static_cast<std::uint8_t>(byte_mask & ~idle_b)};
        c.initial.erase(c.initial.begin());
      }
      Fixture f(c);
      f.board.initialize();
      constexpr unsigned reset_periods = 195;
      const auto prefix_cut = power_on && cut >= 43 ? reset_periods + cut : cut;
      steps(f.board, prefix_cut);
      const auto prefix = f.board.status();
      const auto saved_image = f.board.display_image(panel);
      const auto saved = f.board.checkpoint();
      f.trace.str({});
      f.trace.clear();
      steps(f.board, suffix);
      verify(f);
      const auto final = f.board.status();
      const auto final_image = f.board.display_image(panel);
      const auto trace = f.trace.str();
      for (unsigned replay = 0; replay < 2; ++replay) {
        f.board.restore(saved);
        require(f.board.status() == prefix && f.board.display_image(panel) == saved_image,
                "Fresh native owners restore the LCD reset, holder, RAM and active overlap");
        f.trace.str({});
        f.trace.clear();
        steps(f.board, suffix);
        require(f.board.status() == final && f.board.display_image(panel) == final_image &&
                    f.trace.str() == trace,
                "Both native CPUs, LCD operations and image reproduce the exact suffix");
      }
    }
  }
}

void faults_and_ownership() {
  for (const auto mode : {Mode::unknown_read, Mode::status, Mode::release_during_read}) {
    auto c = config();
    if (mode == Mode::release_during_read) {
      constexpr unsigned release_deadline = 10000;
      c.lcd_parallel = {.reset_release_ticks = release_deadline};
    }
    Fixture f(c, mode);
    const auto fault = rejects([&] { steps(f.board, suffix); });
    require(f.board.status().faulted &&
                fault.contains(mode == Mode::status ? "Unqualified ST7565 status read"
                                                    : "Unknown MCU GPIO input"),
            "Native status/unknown display reads fail without a ready or RAM substitute");
    require(fault.contains("pc="), "Native display failures retain the executing MCU context");
  }
  for (unsigned kind = 0; kind < 5; ++kind) {
    auto c = config();
    if (kind == 0) { c.initial.push_back({.port = McuPort::a, .drive = {.floating = byte_mask}}); }
    if (kind == 1) { c.events.push_back({.ticks = 1, .port = McuPort::a, .drive = {}}); }
    if (kind == 2) { c.hreq = {.destination = McuPort::a, .mask = 1}; }
    if (kind == 3) { c.miso = {.destination = McuPort::a, .mask = 2}; }
    if (kind == 4) { c.loopback = {.source_mask = 1, .destination_mask = 1}; }
    require(rejects([&] { const Fixture f(c); }).contains("LCD owns MCU A data pads"),
            "Every external return/loopback/data source respects the LCD data owner");
  }
  auto c = config();
  c.lcd_parallel.reset();
  const Fixture absent(c);
  require(!absent.board.display_image(panel), "LCD attachment is explicit and absent by default");
}
}

int main() {
  try {
    checkpoints();
    checkpoints(true);
    faults_and_ownership();
    std::cout
        << "Regular Board native LCD cycles, POR, independent pixels and fresh restore passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
