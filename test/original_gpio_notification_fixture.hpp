// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "firmware/firmware.hpp"
#include "gpio_notification_wire.hpp"
#include "gpio_parameter_caller.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_gpio.hpp"
#include "original_gpio_pair_fixture.hpp"

#include <array>
#include <cstdint>
#include <ostream>
#include <vector>

namespace blofeld::test::gpio_notification {
using namespace blofeld;
using namespace blofeld::test::gpio_pair;
using blofeld::test::gpio_parameter::caller_entry_bytes;
constexpr unsigned sound_base = 0x936, result_base = 0x2500;
constexpr unsigned policy_address = 0x74c, policy_parameter = 44;
constexpr unsigned cache_begin = 0x1a12, cache_cursor = 0x19fe, cache_bytes = 40;
constexpr unsigned tx_ring = 0x18ae, tx_reader = 0x18ee, tx_writer = 0x18ef;
constexpr unsigned ring_bytes = 64, cursor = 61, ring_mask = 63;
constexpr unsigned write_call_bytes = 18, decision_pc = 0x83b, store_pc = 0x83c;
constexpr unsigned zero_flag = 4, serial_vector = 0xffd6, serial_handler = 0xb5c1;
constexpr unsigned uart_queue = 0xb555, usb_queue = 0xb3ed, midi_sender = 0xb352;
constexpr unsigned maximum_instructions = 100000, baud_divisor = 60;
constexpr unsigned complete_status = 0xc0, irq_enable = 0x80;
constexpr unsigned empty_entries = 0x6d2, signature_address = 0x76c, signature = 127;
constexpr unsigned byte_count = 9, packet_bytes = 15;
constexpr unsigned usb_vector = 0xffca, usb_handler = 0xbe6b;
constexpr unsigned noise_parameter = 69, noise_initial = 42, octave_initial = 64;
constexpr unsigned octave_controller = 27, noise_controller = 62;
constexpr std::array<unsigned, 3> noise_values{0, 127, 42}, octave_values{17, 65, 127};

struct Input {
  unsigned policy;
  bool reject;

  [[nodiscard]] unsigned parameter() const { return reject ? 1U : noise_parameter; }

  [[nodiscard]] unsigned initial() const { return reject ? octave_initial : noise_initial; }

  [[nodiscard]] unsigned controller() const {
    return reject ? octave_controller : noise_controller;
  }

  [[nodiscard]] std::array<unsigned, 3> values() const {
    return reject ? octave_values : noise_values;
  }

  [[nodiscard]] unsigned readback(const unsigned n) const {
    return reject ? initial() : values().at(n);
  }

  [[nodiscard]] Case sound() const {
    return {.kind = 0,
            .location = 0,
            .parameter = parameter(),
            .address = sound_base + parameter(),
            .value = initial()};
  }
};

struct Run {
  Input input;
  OriginalGpioPair pair;
  blofeld::test::gpio_notification::Wire wire;
  unsigned done = 0, decisions = 0, stores = 0, policy_reads = 0;
  unsigned sci_irqs = 0, queue_calls = 0, sender_calls = 0;
  unsigned produced = 0, consumed = 0;
  bool write_returned = false, usb_enabled = false;
  unsigned usb_irqs = 0, usb_calls = 0;

  Run(const Firmware &firmware, const Input config, std::ostream &trace)
      : input(config),
        pair(firmware, config.sound(), trace, DspSetup::secondary_parser),
        wire(trace) {
    prepare();
    pair.additional_completion.observe = [this](const auto &step) { observe(step); };
  }

  unsigned read(const unsigned address) {
    return pair.dsp.bus().read({.space = DspSpace::x, .address = address});
  }

  void prepare() {
    for (const auto [address, value] :
         std::array<std::array<unsigned, 2>, 4>{{{part_table, sound_base},
                                                 {policy_address, input.policy},
                                                 {empty_entries, 0},
                                                 {empty_entries + 1, 0}}}) {
      pair.dsp.bus().write({.space = DspSpace::x, .address = address, .value = value});
    }
    auto &bus = pair.mcu.bus();
    bus.write(cache_cursor, 0);
    for (unsigned n = 0; n < cache_bytes; ++n) {
      bus.write(static_cast<std::uint16_t>(cache_begin + n), 0);
    }
    bus.write(tx_reader, cursor);
    bus.write(tx_writer, cursor);
    for (unsigned n = 0; n < ring_bytes; ++n) {
      bus.write(static_cast<std::uint16_t>(tx_ring + n), ram_canary);
    }
    bus.set_pin_drive(McuPort::r, {.mask = 4, .levels = 4});
    bus.write(hc12::sci_baud_high, 0);
    bus.write(hc12::sci_baud_low, baud_divisor);
    bus.write(hc12::sci_control1, 8);
    bus.write(hc12::sci_control2, 12);
  }

  void observe(const DspScheduledStep &step) {
    const auto producer = read(ring_write);
    const auto consumer = read(ring_read);
    if (producer != input_ring + produced) {
      const auto values = input.values();
      const std::array<unsigned, 5> packet{245, 0, input.parameter(), values.at(produced / 5), 247};
      require(produced < packet_bytes && producer == input_ring + produced + 1 &&
                  read(input_ring + produced) == packet.at(produced % 5) << 16U,
              "Actual original GPIO producer must publish each expected sound byte");
      ++produced;
    }
    if (consumer != input_ring + consumed) {
      require(consumed < produced && consumer == input_ring + consumed + 1,
              "Original secondary parser must consume exactly its published prefix");
      ++consumed;
    }
    if (step.plan.interrupt) { return; }
    const auto &cpu = step.plan.before.cpu;
    if (cpu.pc == decision_pc) {
      require(decisions < input.values().size() && ((cpu.sr & zero_flag) != 0) == !input.reject,
              "Original Sound normalizer must decide independently of notification policy");
      ++decisions;
    }
    if (cpu.pc == store_pc) {
      require(!input.reject && stores < input.values().size() && cpu.r[0] == sound_base &&
                  cpu.n[0] == input.parameter() &&
                  ((cpu.a >> gpio_pair::word_bits) & gpio_pair::word_mask) ==
                      input.values().at(stores) &&
                  read(sound_base + input.parameter()) == input.values().at(stores),
              "Actual original accepted store differs");
      ++stores;
    }
    if (cpu.pc == source_load && cpu.r[6] == policy_address) { ++policy_reads; }
  }

  void install(const std::vector<std::uint8_t> &code, const unsigned order) {
    for (unsigned n = 0; n < code.size(); ++n) {
      pair.mcu.bus().write(static_cast<std::uint16_t>(caller + n), code[n]);
    }
    pair.input = {.kind = 2,
                  .location = 0,
                  .parameter = policy_parameter,
                  .address = policy_address,
                  .value = input.policy};
    pair.start(order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
               order == 0 ? 0 : phase_offset,
               caller + static_cast<unsigned>(code.size() - 2));
    pair.mcu.set_cycle_observer([this](const Hc12BusCycle cycle) {
      pair.on_cycle(cycle);
      wire.sample(pair.mcu.bus(), pair.mcu.run_clock().bus_cycles());
    });
  }

  void before_step() {
    const auto pending = pair.mcu.cpu().pending_interrupt();
    if (pending) {
      if (usb_enabled && pending->vector == usb_vector) {
        ++usb_irqs;
      } else {
        require(pending->vector == serial_vector,
                "Only enabled actual SCI/USB sources may interrupt this fixture");
        ++sci_irqs;
      }
      return;
    }
    const auto pc = pair.mcu.cpu().state().pc;
    if (pc == usb_queue) {
      require(usb_enabled, "Ordinary selector3 must select UART without direct USB duplication");
      ++usb_calls;
    }
    if (pc == uart_queue) { ++queue_calls; }
    if (pc == midi_sender) { ++sender_calls; }
  }

  void readback() {
    if (done == input.values().size()) { return; }
    const auto pc = pair.mcu.cpu().state().pc;
    if (!write_returned && pc == caller + (done * caller_entry_bytes) + write_call_bytes) {
      require(policy_reads == 1, "Original notifier must finish its genuine policy lookup");
      pair.input = input.sound();
      pair.input.value = input.readback(done);
      write_returned = true;
    }
    const auto result = pair.mcu.bus().read(static_cast<std::uint16_t>(result_base + done));
    if (result == ram_canary) { return; }
    require(write_returned && result == input.readback(done) && decisions == done + 1 &&
                pair.link.source_register_checks == done + 2 &&
                pair.link.data_writes == ((done + 1) * 9) + 4 &&
                pc == caller + ((done + 1) * caller_entry_bytes),
            "Original notification must return before native sound readback");
    pair.trace << "RESULT index=" << done << " value=" << static_cast<unsigned>(result) << '\n';
    ++done;
    write_returned = false;
    if (done < input.values().size()) { pair.input.value = input.readback(done); }
  }

  void execute() {
    for (unsigned n = 0; n < maximum_instructions; ++n) {
      const bool entering_irq = pair.mcu.cpu().pending_interrupt().has_value();
      before_step();
      pair.step();
      if (entering_irq) {
        require(pair.mcu.cpu().state().pc == serial_handler,
                "Actual SCI vector must select the unchanged original ISR");
      }
      readback();
      if (done != input.values().size() || !pair.done() ||
          pair.dsp.cpu().state().pc != pair.foreground || pair.dsp.cpu().state().sp != 0 ||
          pair.mcu.cpu().pending_interrupt()) {
        continue;
      }
      const auto status = pair.mcu.bus().read(hc12::sci_status1);
      if ((status & complete_status) != complete_status ||
          (pair.mcu.bus().read(hc12::sci_control2) & irq_enable) != 0) {
        continue;
      }
      require(wire.bytes().size() == byte_count && !wire.active(),
              "Native SCI must finish every independently decoded MIDI frame");
      return;
    }
    throw Fault("Original notification and actual serial drain exceeded native instruction budget");
  }

  void verify(const std::vector<std::uint8_t> &code) {
    std::vector<unsigned> expected;
    for (const auto value : input.values()) {
      for (const auto byte : {0xb0U, input.controller(), value}) { expected.push_back(byte); }
    }
    require(wire.bytes() == expected && decisions == 3 && stores == (input.reject ? 0U : 3U) &&
                policy_reads == 1 && sender_calls == 3 && queue_calls == 3 && sci_irqs > 0 &&
                produced == packet_bytes && consumed == produced &&
                pair.link.unresolved_queries == 0 && read(signature_address) == signature &&
                read(input.sound().address) == input.initial(),
            "Actual Sound effects and original notification/SCI totals differ");
    auto &bus = pair.mcu.bus();
    require(bus.read(tx_reader) == ((cursor + byte_count) & ring_mask) &&
                bus.read(tx_writer) == bus.read(tx_reader) && bus.read(cache_cursor) == 1,
            "Actual SCI IRQ must consume the wrapped original ring and disable TIE");
    for (unsigned n = 0; n < ring_bytes; ++n) {
      const auto relative = (n + ring_bytes - cursor) & ring_mask;
      const auto value = relative < byte_count ? expected.at(relative) : ram_canary;
      require(bus.read(static_cast<std::uint16_t>(tx_ring + n)) == value,
              "Real UART queue must retain exact wrapped bytes and untouched neighbors");
    }
    for (unsigned n = 0; n < cache_bytes; ++n) {
      unsigned value = 0;
      if (n == 5) { value = 2; }
      if (n == 8) { value = policy_parameter; }
      if (n == 9) { value = input.policy; }
      require(bus.read(static_cast<std::uint16_t>(cache_begin + n)) == value,
              "Original policy cache must retain its actual DSP value");
    }
    for (unsigned n = 0; n < code.size(); ++n) {
      require(bus.read(static_cast<std::uint16_t>(caller + n)) == code[n], "Own caller changed");
    }
    bool rejected = false;
    try {
      pair.reject_board_qualification();
    } catch (const Fault &) { rejected = true; }
    require(rejected, "Successful serial notification must not qualify unresolved board voltage");
  }
};

} // namespace blofeld::test::gpio_notification
