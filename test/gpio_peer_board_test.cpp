// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_gpio_peer.hpp"
#include "board/mcu_dsp_runtime.hpp"
#include "board/reference_board.hpp"
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
#include <vector>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
constexpr unsigned oscillator = 12000000, timebase = 540000000;
constexpr unsigned entry = 0x9000, flash_base = 0x8000, nop = 0xa7;
constexpr unsigned byte_mask = 0xff, word_mask = 0xffffff, budget = 900, suffix = 450;
constexpr unsigned first = 0xa5, reply = 0x5a, upper = 0xa00, upper_mask = 0xf00;
constexpr unsigned load_a = 0x86, store_a = 0x7a, read_a = 0xb6;
constexpr unsigned gpio_c_data = 0xffffbd, gpio_c_direction = 0xffffbe, gpio_c_control = 0xffffbf;
constexpr unsigned instruction_delay = 200, dsp_first_delay = 120, dsp_reply_delay = 150;
constexpr unsigned scheduled_release = 450, dsp_tick = 45;

void store(std::vector<std::uint8_t> &code, const unsigned address, const std::uint8_t value) {
  code.insert(code.end(),
              {load_a,
               value,
               store_a,
               static_cast<std::uint8_t>(address >> 8U),
               static_cast<std::uint8_t>(address & byte_mask)});
}

McuFlashImage image(const bool tie = false) {
  std::vector<std::uint8_t> code;
  store(code, hc12::ptq, first);
  store(code, hc12::ddrq, byte_mask);
  if (tie) {
    constexpr unsigned change_delay = 80;
    code.insert(code.end(), change_delay, nop);
    store(code, hc12::ptq, reply);
    code.insert(code.end(), instruction_delay - change_delay, nop);
  } else {
    code.insert(code.end(), instruction_delay, nop);
  }
  store(code, hc12::ddrq, 0);
  code.insert(code.end(), instruction_delay, nop);
  if (!tie) { code.insert(code.end(), {read_a, hc12::ptq >> 8U, hc12::ptq & byte_mask}); }
  std::array<std::uint8_t, McuFlashImage::byte_count> bytes{};
  bytes.fill(nop);
  for (std::size_t index = 0; index < code.size(); ++index) {
    bytes.at(entry - flash_base + index) = code[index];
  }
  constexpr unsigned protection = 0x7f0d;
  constexpr unsigned security = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

std::vector<std::uint32_t> program(const bool tie = false) {
  // Authored FM MOVEP #immediate, X:qq and MOVE X:ea,X0 templates.
  constexpr unsigned movep_low_x = 0x07f400;
  constexpr unsigned low_x_base = 0xffff80;
  constexpr unsigned load_x0 = 0x44f000;
  std::vector<std::uint32_t> code;
  const auto write = [&](const unsigned address, const unsigned value) {
    code.insert(code.end(), {movep_low_x | (address - low_x_base), value});
  };
  write(gpio_c_control, byte_mask);
  write(gpio_c_direction, upper_mask);
  write(gpio_c_data, upper);
  if (tie) {
    // An authored three-effect MOVEC places the pad read on an MCU write tick;
    // two-effect NOPs alone keep the read between consecutive MCU boundaries.
    constexpr unsigned movec_sr = 0x05f439;
    constexpr unsigned masked_sr = 0x300;
    code.insert(code.end(), {movec_sr, masked_sr});
  }
  constexpr unsigned tie_read_delay = 60;
  code.insert(code.end(), tie ? tie_read_delay : dsp_first_delay, 0);
  code.insert(code.end(), {load_x0, gpio_c_data});
  constexpr unsigned tie_output_delay = 200;
  code.insert(code.end(), tie ? tie_output_delay : dsp_reply_delay, 0);
  write(gpio_c_control, 0);
  write(gpio_c_direction, upper_mask | byte_mask);
  write(gpio_c_data, upper | reply);
  constexpr unsigned rom_words = 4096;
  code.resize(rom_words);
  return code;
}

reference::Config config(const McuWriteOrder order) {
  reference::Config c;
  c.mcu = {.clock = {.oscillator_hz = oscillator,
                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
           .memory_timing = McuRunMemory::unstretched_internal};
  c.dsp_clock = {.extal_hz = oscillator,
                 .timebase_hz = timebase,
                 .switch_timing = DspClockSwitch::reference_cycle_boundary};
  c.dsp = {.timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
           .maximum_cycles_per_period = 16,
           .writes = order,
           .external_writes = order,
           .initial_reset = OwnDspResetInputs{
               .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  c.gpio_peer = McuDspGpioWiring{};
  c.dma_grant = DspDmaGrant::available;
  c.release_ticks = scheduled_release;
  c.operations = budget;
  c.select_n = 2;
  c.clock = 4;
  c.mosi = 16;
  constexpr unsigned serial_pins = 0x16;
  c.initial = {{.port = McuPort::b, .drive = {.mask = serial_pins, .levels = 2}},
               {.port = McuPort::e, .drive = {.mask = 2, .levels = 2}}};
  return c;
}

struct Fixture {
  explicit Fixture(const reference::Config &c, const bool tie = false)
      : flash(image(tie)),
        rom(program(tie)),
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

void native_exchange_and_restore(const McuWriteOrder order) {
  constexpr std::array cuts{0U, 5U, 100U, 205U, 215U, 300U};
  for (const auto cut : cuts) {
    Fixture f(config(order));
    f.board.initialize();
    steps(f.board, cut);
    const auto saved = f.board.checkpoint();
    const auto prefix = f.board.status();
    f.trace.str({});
    f.trace.clear();
    steps(f.board, suffix);
    const auto expected = f.board.status();
    const auto trace = f.trace.str();
    require((expected.dsp.x & word_mask) == (upper | first) && (expected.mcu.d >> 8U) == reply &&
                !expected.faulted,
            "Real MCU writes reach native DSP MOVE, then native DSP writes reach MCU LDAA");
    require(expected.ties > 0, "Both selected write orders encounter shared-clock ties");
    for (unsigned replay = 0; replay < 2; ++replay) {
      f.board.restore(saved);
      require(f.board.status() == prefix, "Replacement owners restore the exact Board prefix");
      f.trace.str({});
      f.trace.clear();
      steps(f.board, suffix);
      require(f.board.status() == expected && f.trace.str() == trace,
              "Coordinated owner replacement replays actual native exchange and trace exactly");
    }
  }
}

unsigned tie_sample(const unsigned phase, const McuWriteOrder order) {
  auto c = config(order);
  c.release_ticks += static_cast<std::uint64_t>(phase) * dsp_tick;
  Fixture f(c, true);
  steps(f.board, suffix);
  const auto value = static_cast<unsigned>(f.board.status().dsp.x & byte_mask);
  require(value == first || value == reply, "Only the two real MCU output bytes can be sampled");
  return value;
}

void actual_shared_write_tie() {
  constexpr unsigned phases = 128;
  require(tie_sample(0, McuWriteOrder::before_mcu) == first &&
              tie_sample(phases, McuWriteOrder::before_mcu) == reply,
          "The fixed clock-phase interval brackets the authored MCU data change");
  unsigned before = 0;
  unsigned after = phases;
  while (after - before > 1) {
    const auto middle = before + ((after - before) / 2);
    if (tie_sample(middle, McuWriteOrder::before_mcu) == first) {
      before = middle;
    } else {
      after = middle;
    }
  }
  require(
      tie_sample(before, McuWriteOrder::before_mcu) == first &&
          tie_sample(before, McuWriteOrder::after_mcu) == reply,
      "At the same GPIO read/write tick DSP-before-MCU reads old data; MCU-before-DSP reads new");
}

void route_ownership() {
  const auto check = [](const reference::Config &c) {
    require(rejects([&] { const Fixture f(c); }).contains("GPIO peer"),
            "Two independent sources cannot silently own the same connected peer input");
  };
  for (const auto port : {McuPort::q, McuPort::e, McuPort::j}) {
    auto c = config(McuWriteOrder::before_mcu);
    c.initial.push_back({.port = port, .drive = {.mask = 1}});
    check(c);
    c = config(McuWriteOrder::before_mcu);
    c.events.push_back({.port = port, .drive = {.unknown = 1}});
    check(c);
    c = config(McuWriteOrder::before_mcu);
    c.hreq = {.destination = port, .mask = 1};
    check(c);
    c = config(McuWriteOrder::before_mcu);
    c.miso = {.destination = port, .mask = 1};
    check(c);
    c = config(McuWriteOrder::before_mcu);
    c.loopback = {
        .source = McuPort::p, .source_mask = 1, .destination = port, .destination_mask = 1};
    check(c);
  }
  auto c = config(McuWriteOrder::before_mcu);
  c.initial.push_back({.port = McuPort::j, .drive = {.mask = 2, .levels = 2}});
  const Fixture other_j_pins(c);
  require(!other_j_pins.board.status().faulted,
          "Unrelated J input pins remain available beside the owned IRQA strobe");
}

void configuration(const std::filesystem::path &directory) {
  std::filesystem::create_directories(directory);
  const auto path = directory / "gpio.board";
  const std::string prefix =
      "blofeld-reference-board 2\nclock 12000000 0 12000000 540000000 reference-boundaries\n"
      "dsp-execution reference-effects\nordering dsp-before-mcu dsp-before-mcu 32\n"
      "spi-b 02 04 10 reject\noperations 4096\ndsp-release 450 low\n";
  for (const std::string pull : {"none", "up", "down", "invalid"}) {
    {
      std::ofstream output(path);
      output.exceptions(std::ios::failbit | std::ios::badbit);
      output << prefix << "gpio-peer direct ff " << pull << '\n';
    }
    if (pull == "invalid") {
      require(rejects([&] { static_cast<void>(reference::load_config(path)); }).contains("pull"),
              "Unknown notification bias is rejected before the Board exists");
    } else {
      const auto c = reference::load_config(path);
      auto expected = GpioPeerPull::none;
      if (pull == "up") { expected = GpioPeerPull::up; }
      if (pull == "down") { expected = GpioPeerPull::down; }
      require(c.gpio_peer && c.gpio_peer->data_pull_down == byte_mask &&
                  c.gpio_peer->notification_pull == expected,
              "Configuration retains only the explicitly declared topology and pull hypotheses");
    }
  }
}

}

int main(const int count, char **args) {
  try {
    native_exchange_and_restore(McuWriteOrder::before_mcu);
    native_exchange_and_restore(McuWriteOrder::after_mcu);
    actual_shared_write_tie();
    route_ownership();
    require(count == 2, "A private test artifact directory is required");
    configuration(args[1]);
    std::cout << "Common-clock native GPIO exchange and Board restore checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
