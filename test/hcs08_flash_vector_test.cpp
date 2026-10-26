// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
namespace hf = hcs08_flash;
constexpr unsigned first_vector = 0xffc0, last_vector = 0xfffd, reset_vector = 0xfffe;
constexpr unsigned byte_mask = 0xff, no_redirect = 0x40, spi_vector = 0xffe0;
constexpr unsigned reset_high = 0xf0, reset_low = 0x12, last_page_protection = 0xfc;
constexpr unsigned unredirected_options = 0x42, fclk_divider = 59, bus_hz = 12000000;
constexpr unsigned erase_probe = 0xe100, relocated_spi = 0xfde0;

struct Configuration {
  unsigned protection, options;
};

constexpr ExecutionContext origin{.pc = 0xe010, .ccr = 0x60};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

template <typename Operation> std::string rejected(Operation operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected an explicit vector-fetch refusal");
}

std::uint8_t pattern(const unsigned address) {
  return static_cast<std::uint8_t>((address >> 8U) ^ (address & byte_mask));
}

Hcs08FlashImage own_image(const Configuration config) {
  Hcs08FlashImage image;
  for (unsigned index = 0; index < hf::bytes; ++index) {
    image.data[index] = pattern(hf::first + index);
  }
  image.known.set();
  image.data[hf::nv_protection - hf::first] = static_cast<std::uint8_t>(config.protection);
  image.data[hf::nv_options - hf::first] = static_cast<std::uint8_t>(config.options);
  image.data[reset_vector - hf::first] = reset_high;
  image.data[reset_vector + 1 - hf::first] = reset_low;
  return image;
}

struct Relocation {
  unsigned protection, first;
};

// Independent literal 512-byte page boundaries,GT8A 8192-byte array,GT4.4.6/7.
constexpr std::array<Relocation, 15> relocated{{{.protection = 0xe0, .first = 0xe1c0},
                                                {.protection = 0xe2, .first = 0xe3c0},
                                                {.protection = 0xe4, .first = 0xe5c0},
                                                {.protection = 0xe6, .first = 0xe7c0},
                                                {.protection = 0xe8, .first = 0xe9c0},
                                                {.protection = 0xea, .first = 0xebc0},
                                                {.protection = 0xec, .first = 0xedc0},
                                                {.protection = 0xee, .first = 0xefc0},
                                                {.protection = 0xf0, .first = 0xf1c0},
                                                {.protection = 0xf2, .first = 0xf3c0},
                                                {.protection = 0xf4, .first = 0xf5c0},
                                                {.protection = 0xf6, .first = 0xf7c0},
                                                {.protection = 0xf8, .first = 0xf9c0},
                                                {.protection = 0xfa, .first = 0xfbc0},
                                                {.protection = 0xfc, .first = 0xfdc0}}};

unsigned expected_address(const Configuration config, const unsigned address) {
  if ((config.options & no_redirect) != 0) { return address; }
  for (const auto row : relocated) {
    if (config.protection == row.protection) { return row.first + address - first_vector; }
  }
  return address;
}

void matrix() {
  constexpr unsigned option_combinations = 16;
  constexpr unsigned key_enable = 0x80;
  for (unsigned protection = 0; protection <= byte_mask; ++protection) {
    for (unsigned combination = 0; combination < option_combinations; ++combination) {
      const unsigned options = (combination & 3U) | ((combination & 4U) != 0 ? no_redirect : 0U) |
                               ((combination & 8U) != 0 ? key_enable : 0U);
      const auto image = own_image({.protection = protection, .options = options});
      Hcs08Flash flash(image, Hcs08FlashContract::candidate_gt8a);
      flash.context(origin);
      const auto before = flash.state();
      const auto copied = flash;
      for (unsigned address = first_vector; address <= last_vector; ++address) {
        const auto logical = static_cast<std::uint16_t>(address);
        const auto expected =
            pattern(expected_address({.protection = protection, .options = options}, address));
        require(flash.read_interrupt_vector_byte(logical) == expected &&
                    copied.read_interrupt_vector_byte(logical) == expected &&
                    flash.read_array(logical) == pattern(address),
                "All protection/FNORED/SEC/KEYEN combinations use independent literal vector pages;"
                "ordinary array reads and copies stay direct");
      }
      require(flash.read_reset_vector_byte(reset_vector) == reset_high &&
                  flash.read_reset_vector_byte(reset_vector + 1) == reset_low &&
                  flash.state() == before && flash.image() == image,
              "Fixed reset vector and pure mapping reads preserve all NVM bytes/history/state");
      flash.write({.address = hf::options, .value = byte_mask});
      flash.write({.address = hf::protection, .value = byte_mask});
      require(
          flash.read_interrupt_vector_byte(spi_vector) ==
              pattern(expected_address({.protection = protection, .options = options}, spi_vector)),
          "CPU FOPT/FPROT writes never change the reset-latched relocation selection");
    }
  }
}

void missing_inputs() {
  for (const unsigned protection : {0xfcU, 0xffU, 0xdeU, 0xfeU}) {
    auto image = own_image({.protection = protection, .options = 2});
    image.known.reset(hf::nv_options - hf::first);
    Hcs08Flash flash(image, Hcs08FlashContract::candidate_gt8a);
    flash.context(origin);
    if (protection == last_page_protection) {
      const auto message =
          rejected([&] { static_cast<void>(flash.read_interrupt_vector_byte(spi_vector)); });
      require(message.contains("address=0xffe0") && message.contains("address=0x1821") &&
                  message.contains("pc=0xe010") && message.contains("agent=CPU"),
              "Unknown relevant FNORED includes logical vector,register,PC and agent");
    } else {
      require(flash.read_interrupt_vector_byte(spi_vector) == pattern(spi_vector),
              "Known disabled/full/empty protection excludes unknown irrelevant FNORED");
    }
  }
  for (const unsigned options : {2U, 0x42U}) {
    auto image = own_image({.protection = last_page_protection, .options = options});
    image.known.reset(hf::nv_protection - hf::first);
    Hcs08Flash flash(image, Hcs08FlashContract::candidate_gt8a);
    flash.context(origin);
    if (options == 2) {
      require(rejected([&] {
                static_cast<void>(flash.read_interrupt_vector_byte(spi_vector));
              }).contains("unknown reset-latched bits"),
              "Unknown relevant protection cannot invent a relocation base");
    } else {
      require(flash.read_interrupt_vector_byte(spi_vector) == pattern(spi_vector),
              "Known FNORED1 excludes unknown irrelevant protection");
    }
  }
  for (const unsigned missing : {spi_vector, relocated_spi}) {
    auto image = own_image({.protection = last_page_protection, .options = 2});
    image.known.reset(missing - hf::first);
    Hcs08Flash flash(image, Hcs08FlashContract::candidate_gt8a);
    flash.context(origin);
    if (missing == relocated_spi) {
      require(rejected([&] {
                static_cast<void>(flash.read_interrupt_vector_byte(spi_vector));
              }).contains("address=0xfde0"),
              "Missing actual relocated data cannot fall back to the original vector");
    } else {
      require(flash.read_interrupt_vector_byte(spi_vector) == pattern(relocated_spi),
              "Unknown unused original vector cannot mask known redirected data");
    }
  }
}

void stage(Hcs08Flash &flash, const Hcs08FlashJob job) {
  flash.stage({.address = job.address, .value = job.value});
  flash.write({.address = hf::command, .value = job.command});
  flash.write({.address = hf::status, .value = hf::empty});
  require(flash.state().buffer == Hcs08FlashBuffer::launched && flash.state().errors == 0,
          "Only a real address/data/command/launch sequence can stage NVM work");
}

void scope_and_activity() {
  Hcs08Flash flash(own_image({.protection = last_page_protection, .options = 2}),
                   Hcs08FlashContract::candidate_gt8a);
  flash.context(origin);
  for (const unsigned address : {0U, 0xe000U, 0xffbfU, 0xfffeU, 0xffffU}) {
    require(rejected([&] {
              static_cast<void>(
                  flash.read_interrupt_vector_byte(static_cast<std::uint16_t>(address)));
            }).contains("requires an actual CPU interrupt vector fetch"),
            "An IRQ fetch capability cannot read reset bytes or unrelated memory");
  }
  flash.context(origin, Hcs08FlashAgent::background);
  require(rejected([&] {
            static_cast<void>(flash.read_interrupt_vector_byte(spi_vector));
          }).contains("agent=BDC"),
          "BDC never obtains a CPU vector-read capability");
  flash.write({.address = hf::protection, .value = byte_mask});
  flash.context(origin);
  require(flash.read_interrupt_vector_byte(spi_vector) == pattern(spi_vector),
          "Actual BDC FPROT updates change the current protection latch");
  flash.write({.address = hf::divider, .value = fclk_divider});
  flash.observe_bus_clock(bus_hz);
  stage(flash, {.address = erase_probe, .value = 0, .command = hf::erase_page});
  for (unsigned phase = 0; phase < 2; ++phase) {
    const auto before = flash.state();
    require(rejected([&] {
              static_cast<void>(flash.read_interrupt_vector_byte(spi_vector));
            }).contains("executing/pending command") &&
                flash.state() == before,
            "Pending/active NVM cannot be progressed or bypassed by vector reads");
    if (phase == 0) { flash.advance(Hcs08FlashMilestone::transfer_buffer); }
  }
  flash.advance(Hcs08FlashMilestone::complete_command);
  require(flash.read_interrupt_vector_byte(spi_vector) == pattern(spi_vector),
          "Only an actual command-completion reference restores array availability");
}

void reset_latches() {
  Hcs08Flash flash(own_image({.protection = byte_mask, .options = unredirected_options}),
                   Hcs08FlashContract::candidate_gt8a);
  flash.context(origin);
  flash.write({.address = hf::divider, .value = fclk_divider});
  flash.observe_bus_clock(bus_hz);
  stage(flash, {.address = hf::nv_options, .value = 0, .command = hf::erase_page});
  flash.advance(Hcs08FlashMilestone::transfer_buffer);
  flash.advance(Hcs08FlashMilestone::complete_command);
  for (const auto cell : {std::array<unsigned, 2>{hf::nv_options, 2},
                          std::array<unsigned, 2>{hf::nv_protection, 0xfc}}) {
    stage(flash,
          {.address = static_cast<std::uint16_t>(cell[0]),
           .value = static_cast<std::uint8_t>(cell[1]),
           .command = hf::program});
    flash.advance(Hcs08FlashMilestone::transfer_buffer);
    flash.advance(Hcs08FlashMilestone::complete_command);
  }
  require(flash.read_interrupt_vector_byte(spi_vector) == byte_mask &&
              flash.read(hf::options) == unredirected_options &&
              flash.read(hf::protection) == byte_mask,
          "Programming NVOPT/NVPROT never prematurely reloads FOPT/FPROT");
  const auto final_image = flash.image();
  flash.reset();
  flash.context(origin);
  require(flash.read_interrupt_vector_byte(spi_vector) == pattern(relocated_spi) &&
              flash.read(hf::options) == 2 && flash.read(hf::protection) == last_page_protection &&
              flash.image() == final_image,
          "Actual reset reloads programmed FNORED/protection without changing NVM");
}
} // namespace

int main() {
  try {
    matrix();
    missing_inputs();
    scope_and_activity();
    reset_latches();
    std::cout << "Verified literal HCS08 vector pages,reset latches and strict CPU/NVM scope.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
