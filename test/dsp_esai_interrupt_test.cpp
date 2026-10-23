// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_esai.hpp"
#include "blofeld/dsp_interrupts.hpp"
#include "esai_fixture.hpp"
#include "esai_interrupt_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace blofeld;
using namespace blofeld::test::esai_irq;

void source_matrix() {
  constexpr std::array lane_sets{1U, 2U, 4U, 8U, 16U, 32U, two_lanes, 63U};
  for (const bool second : {false, true}) {
    for (unsigned mode = 0; mode < 8; ++mode) {
      for (const auto lanes : lane_sets) {
        DspEsai serial(instance(second));
        setup(serial, mode, lanes);
        require(!serial.interrupt_vector(), "Primed data cannot assert a transmit interrupt");
        static_cast<void>(blofeld::test::esai::slot(serial));
        require(serial.interrupt_vector() == expected(second, mode, {.even = true, .error = false}),
                "Manual source truth table: first even slot without an underrun");
        fill(serial, lanes);
        require(!serial.interrupt_vector(), "All enabled TX writes clear every data request");
        static_cast<void>(blofeld::test::esai::slot(serial, word_bits, false));
        require(serial.interrupt_vector() ==
                    expected(second, mode, {.even = false, .error = false}),
                "Manual source truth table: serviced odd slot does not assert even IRQ");
        static_cast<void>(blofeld::test::esai::slot(serial));
        const auto pending = expected(second, mode, {.even = true, .error = true});
        require(
            serial.interrupt_vector() == pending && serial.interrupt_vector() == pending,
            "Exception suppresses ordinary/even source when enabled; queries do not service it");
        fill(serial, lanes);
        require(!serial.interrupt_vector() && (serial.read(status_offset) & status_underrun) != 0,
                "IRQ queries do not arm TUE clearing before the actual SAISR read");
        fill(serial, lanes);
        require((serial.read(status_offset) & status_underrun) == 0,
                "Only SAISR read followed by every enabled TX write clears the sticky error");
        serial.reset();
        require(!serial.interrupt_vector(), "Reset clears all transmitter sources and enables");
      }
    }
  }
}

void masks_and_dynamic_enables() {
  DspEsai serial(DspEsaiInstance::primary);
  setup(serial, 0, two_lanes);
  static_cast<void>(blofeld::test::esai::slot(serial));
  serial.write(control_offset, format | network | two_lanes | data_enable);
  require(serial.interrupt_vector() == primary_vector,
          "Enabling TIE services already-empty real TX buffers");
  serial.write(control_offset, format | network | two_lanes);
  require(!serial.interrupt_vector(), "Clearing TIE masks a pending data source");
  serial.write(control_offset, format | network | two_lanes | error_enable);
  static_cast<void>(blofeld::test::esai::slot(serial, word_bits, false));
  require(serial.interrupt_vector() == primary_error,
          "Live TEIE change exposes the actual underrun source");
  static_cast<void>(
      rejects([&] { serial.write(control_offset, format | network | two_lanes | even_enable); }));
  require(serial.interrupt_vector() == primary_error,
          "Unqualified active TEDIE reconfiguration preserves the known control state");
  serial.write(control_offset, format | network | two_lanes | error_enable | personal_reset);
  serial.write(control_offset, format | network | two_lanes | all_enables | personal_reset);
  require(!serial.interrupt_vector(),
          "Personal reset clears requests even with interrupt bits set");

  for (const bool second : {false, true}) {
    DspEsai masked(instance(second));
    setup(masked, 7, two_lanes);
    masked.write(mask_offset, 2); // Only odd slot1 participates in the next frame.
    static_cast<void>(blofeld::test::esai::slot(masked));
    require(!masked.interrupt_vector(), "A masked first slot cannot fabricate data-empty IRQ");
    static_cast<void>(blofeld::test::esai::slot(masked, word_bits, false));
    require(masked.interrupt_vector() == vector_base(second),
            "First unmasked odd slot uses its normal, not even-data, vector");
  }
}

void priority_matrix() {
  DspInterrupts controller;
  constexpr std::array<std::uint32_t, 2> both{0x7c, primary_vector};
  for (unsigned primary = 0; primary < 4; ++primary) {
    for (unsigned secondary = 0; secondary < 4; ++secondary) {
      controller.write(priority_register, primary | (secondary << 10U));
      for (unsigned mask = 0; mask < 4; ++mask) {
        std::optional<DspInterruptRequest> answer;
        if (primary > mask && (secondary <= mask || primary >= secondary)) {
          answer = DspInterruptRequest{.vector = primary_vector, .level = primary - 1};
        } else if (secondary > mask) {
          answer = DspInterruptRequest{.vector = secondary_error, .level = secondary - 1};
        }
        require(controller.select(both, mask) == answer,
                "Independent ESAI group IPL and CPU mask matrix, with primary winning ties");
      }
    }
  }
  constexpr std::array<std::uint32_t, 15> ranked{0x10,
                                                 0x18,
                                                 0x1a,
                                                 0x1c,
                                                 0x1e,
                                                 0x20,
                                                 0x22,
                                                 primary_error,
                                                 primary_even,
                                                 primary_vector,
                                                 0x4c,
                                                 0x54,
                                                 0x7c,
                                                 secondary_even,
                                                 0x78};
  controller.write(core_priority_register, all_core_priorities);
  controller.write(priority_register, all_priorities);
  for (unsigned a = 0; a < ranked.size(); ++a) {
    for (unsigned b = a + 1; b < ranked.size(); ++b) {
      const std::array pair{ranked[b], ranked[a]};
      require(controller.select(pair, 0) == DspInterruptRequest{.vector = ranked[a], .level = 2},
              "Manual table4-5 ordering: DMA, primary ESAI, SHI, timer, secondary ESAI");
    }
  }
  constexpr std::array unsupported_vectors{0x30U, 0x3eU, 0x70U, 0x7eU};
  for (const unsigned unsupported : unsupported_vectors) {
    static_cast<void>(rejects([&] { static_cast<void>(controller.priority(unsupported)); }));
  }
}

void bus_queries_and_service() {
  DspBus bus;
  setup(bus, false, 7);
  setup(bus, true, 7);
  static_cast<void>(word(bus, false));
  static_cast<void>(word(bus, true));
  bus.write({.space = DspSpace::x, .address = priority_register, .value = both_high_priorities});
  const DspInterruptRequest primary{.vector = primary_even, .level = 2};
  const DspInterruptRequest secondary{.vector = secondary_even, .level = 2};
  require(bus.interrupt_request(0) == primary, "Real pending primary ESAI wins equal group IPL");
  for (unsigned attempt = 0; attempt < 3; ++attempt) {
    bus.acknowledge_interrupt(primary);
    require(bus.interrupt_request(0) == primary, "Vector fetch cannot clear a level data request");
  }
  put(bus, false, 0, next_first);
  require(bus.interrupt_request(0) == primary, "A partial multi-lane refill cannot clear TDE");
  put(bus, false, 5, next_second);
  require(bus.interrupt_request(0) == secondary,
          "Real complete refill exposes the secondary source");
  bus.acknowledge_interrupt(secondary);
  put(bus, true, 6, 0);
  require(!bus.interrupt_request(0), "A real secondary TSR write clears its data source");
  static_cast<void>(rejects([&] { bus.acknowledge_interrupt(secondary); }));
}

void strict_clock_and_unimplemented_sources() {
  DspBus bus;
  setup(bus, false, 4, false);
  require(!bus.interrupt_request(0), "Disabled group does not query unknown peripheral clocks");
  bus.write({.space = DspSpace::x, .address = priority_register, .value = 1});
  require(!bus.interrupt_request(1), "CPU mask prevents an unneeded unknown clock observation");
  require(rejects([&] { static_cast<void>(bus.interrupt_request(0)); }).contains("X:0xffffb3"),
          "Eligible enabled source retains its actual unknown-clock status address");
  static_cast<void>(
      rejects([&] { put(bus, false, control_offset, format | two_lanes | last_enable); }));
  static_cast<void>(rejects([&] { put(bus, false, receive_control_offset, 1); }));
  bus.reset_peripherals();
  require(!bus.interrupt_request(0), "Peripheral reset disables priorities and both ESAI sources");
  require(rejects([&] {
            static_cast<void>(bus.read({.space = DspSpace::y, .address = core_priority_register}));
          }).contains("Y:0xffffff"),
          "Reserved Y:ffffff never aliases the X-space priority register");
}
} // namespace

int main() {
  try {
    source_matrix();
    masks_and_dynamic_enables();
    priority_matrix();
    bus_queries_and_service();
    strict_clock_and_unimplemented_sources();
    blofeld::test::esai_irq::native_checks();
    std::cout
        << "ESAI data/even/exception priorities, pure requests and native service verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
