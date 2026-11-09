// SPDX-License-Identifier: GPL-3.0-only
#include "io/shi.hpp"
#include "mcu/ports.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::mcu::Port;
using blofeld::vi::mcu::Ports;
constexpr std::uint8_t all_bits = 0xff;
constexpr std::uint16_t port_a = 0, direction_a = 2, port_e = 8, direction_e = 9;
constexpr std::uint16_t assignment = 0xa, mode = 0xb, pull = 0xc, reduced = 0xd;
constexpr std::uint16_t port_t = 0x240, input_t = 0x241, direction_t = 0x242;
constexpr std::uint16_t port_m = 0x248, direction_m = 0x24a, pull_m = 0x24c;
constexpr std::uint16_t routing = 0x257, port_q = 0x260, input_q = 0x261;
constexpr std::uint16_t direction_q = 0x262, pull_q = 0x264, polarity_q = 0x265;
constexpr std::uint8_t reset_pull = 0x90, six_pins = 0x3f, e_outputs = 0xfc;
constexpr std::uint8_t core_control_mask = 0x93, assignment_mask = 0xbc, retained_assignment = 0xac;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void write(Ports &ports, const Ports::Write access) {
  require(ports.write(access), "Write a documented GPIO register");
}

std::uint8_t read(const Ports &ports, const std::uint16_t offset) {
  std::uint8_t value = 0;
  require(ports.read(offset, value), "Read a fully known GPIO register");
  return value;
}

void reset_and_latches() {
  blofeld::vi::Shi shi;
  Ports ports(shi);
  std::uint8_t value = 0;
  require(read(ports, direction_a) == 0 && read(ports, pull) == reset_pull,
          "Normal reset leaves directions input and enables core Port E pulls");
  require(read(ports, port_e) == all_bits, "Normal single-chip Port E has reset pull-ups");
  require(!ports.read(port_a, value), "Floating MEBI inputs are unknown");
  write(ports, {.offset = direction_a, .value = all_bits});
  require(!ports.read(port_a, value), "MEBI output latches are unknown until initialized");
  constexpr std::uint8_t datum = 0xa5;
  write(ports, {.offset = port_a, .value = datum});
  require(read(ports, port_a) == datum, "Initialized GPIO output reads its latch");
  write(ports, {.offset = direction_m, .value = all_bits});
  require(read(ports, port_m) == 0 && read(ports, direction_m) == six_pins,
          "PIM latches reset zero and unimplemented direction bits stay zero");
  write(ports, {.offset = direction_e, .value = all_bits});
  require(read(ports, direction_e) == e_outputs && !ports.read(port_e, value),
          "E0/E1 stay input and uninitialized E outputs remain unknown");
  write(ports, {.offset = port_e, .value = 0});
  require(read(ports, port_e) == 3, "E0/E1 retain pulled input values after a port write");
}

void output_state() {
  blofeld::vi::Shi shi;
  Ports ports(shi);
  constexpr std::uint16_t data_u = 0x278;
  constexpr std::uint16_t direction_u = 0x27a;
  const auto initial = ports.output(Port::u);
  require(initial && initial->direction == 0 && (initial->known & 1) != 0 && initial->value == 0,
          "Port U exposes its reset latch independently of external inputs");
  ports.input(Port::u, {.mask = 1, .known = 1, .value = 1});
  write(ports, {.offset = direction_u, .value = 1});
  const auto driven = ports.output(Port::u);
  require(driven && driven->direction == 1 && driven->value == 0,
          "The reset driver is derived from direction and latch, never from input sampling");
  write(ports, {.offset = data_u, .value = 1});
  const auto high = ports.output(Port::u);
  require(high && high->value == 1, "A new output latch is visible to its device binding");
  require(!ports.output(static_cast<Port>(all_bits)), "An invalid port has no output state");
}

void attached_idle_bias() {
  blofeld::vi::Shi shi;
  Ports ports(shi);
  std::uint8_t value = 0;
  ports.input(Port::q, {.fallback_low = all_bits});
  require(read(ports, port_q) == 0,
          "An attached device can define its otherwise released input level");
  write(ports, {.offset = pull_q, .value = all_bits});
  require(read(ports, port_q) == all_bits,
          "A fallback never overrides configured local input pulls");
  write(ports, {.offset = pull_q, .value = 0});
  ports.input(Port::q, {.mask = 1, .known = 0, .fallback_low = all_bits});
  require(!ports.read(port_q, value) && ports.read(port_q, value, all_bits ^ 1) && value == 0,
          "An active unknown driver remains unknown; only released bits receive the idle bias");
  ports.input(Port::q, {.mask = 1, .known = 1, .value = 1, .fallback_low = all_bits});
  require(read(ports, port_q) == 1, "Active data takes precedence over an attached idle bias");
  ports.input(Port::q, {.fallback_low = all_bits});
  write(ports, {.offset = direction_q, .value = all_bits});
  require(read(ports, port_q) == 0, "PIM output latches retain their known reset value");
  constexpr std::uint8_t datum = 0xa5;
  write(ports, {.offset = port_q, .value = datum});
  require(read(ports, port_q) == datum, "Idle input bias cannot overwrite a local output latch");
  ports.input(Port::a, {.fallback_low = all_bits});
  write(ports, {.offset = direction_a, .value = all_bits});
  require(!ports.read(port_a, value), "Idle bias cannot fill an unknown driven output latch");
}

void pulls_and_inputs() {
  blofeld::vi::Shi shi;
  Ports ports(shi);
  std::uint8_t value = 0;
  require(!ports.read(port_q, value), "Unconnected PIM inputs are not fabricated zeros");
  write(ports, {.offset = pull_q, .value = all_bits});
  require(read(ports, port_q) == all_bits, "Enabled default pulls select high inputs");
  write(ports, {.offset = polarity_q, .value = all_bits});
  require(read(ports, port_q) == 0, "Pull polarity one selects pull-down");
  ports.input(Port::q, {.mask = 1, .known = 0, .value = 0});
  require(!ports.read(port_q, value), "An attached unknown input overrides a pull");
  constexpr std::uint8_t other_bits = 0xfe;
  require(ports.read(port_q, value, other_bits) && value == 0,
          "Masked reads do not require unrequested inputs");
  ports.input(Port::q, {.mask = 1, .known = 1, .value = 1});
  require(read(ports, port_q) == 1, "A known input overrides the enabled pull-down");
  constexpr std::uint8_t latch = 0xa5;
  constexpr std::uint8_t sampled = 0x3c;
  write(ports, {.offset = port_q, .value = latch});
  write(ports, {.offset = direction_q, .value = all_bits});
  ports.input(Port::q, {.mask = all_bits, .known = all_bits, .value = sampled});
  require(read(ports, port_q) == latch && read(ports, input_q) == sampled,
          "The data register reads the output latch, the input register samples the port");
  ports.input(Port::q, {.mask = all_bits, .known = 0, .value = 0});
  require(read(ports, port_q) == latch && !ports.read(input_q, value),
          "Unknown physical input never contaminates a defined output-latch read");
}

void controls_and_routing() {
  blofeld::vi::Shi shi;
  Ports ports(shi);
  require(read(ports, assignment) == 0x10 && read(ports, mode) == 0x80,
          "MEBI reset selects normal single-chip GPIO");
  write(ports, {.offset = assignment, .value = all_bits});
  require(read(ports, assignment) == assignment_mask, "PEAR stores only implemented bits");
  write(ports, {.offset = assignment, .value = 0});
  require(read(ports, assignment) == retained_assignment,
          "Only NECLK changes after the normal-mode first write");
  std::uint8_t value = 0;
  require(!ports.read(port_e, value), "An unsupplied ECLK sample remains unknown");
  write(ports, {.offset = pull, .value = all_bits});
  write(ports, {.offset = reduced, .value = all_bits});
  require(read(ports, pull) == core_control_mask && read(ports, reduced) == core_control_mask,
          "Core drive/pull control reserved bits read zero");
  write(ports, {.offset = pull_m, .value = all_bits});
  require(read(ports, pull_m) == six_pins, "Port M controls match its six implemented pins");
  ports.input(Port::q, {.mask = all_bits, .known = all_bits, .value = all_bits});
  write(ports, {.offset = routing, .value = 3});
  require(read(ports, port_q) == 0, "64-pin routing ties unused Q inputs low");
  write(ports, {.offset = routing, .value = 0});
  require(read(ports, port_q) == all_bits, "100-pin routing restores the supplied Q inputs");
}

void reserved_and_masked() {
  blofeld::vi::Shi shi;
  Ports ports(shi);
  std::uint8_t value = 0;
  constexpr std::uint8_t absent_t_bits = 0xf0;
  require(ports.read(port_t, value, absent_t_bits) && value == 0,
          "Unimplemented pins read zero even while implemented inputs are unknown");
  write(ports, {.offset = input_t, .value = all_bits});
  require(!ports.read(input_t, value), "Writing a read-only input register has no effect");
  write(ports, {.offset = direction_t, .value = all_bits});
  require(read(ports, input_t) == 0, "Unclaimed input samples follow known GPIO outputs");
  constexpr std::uint16_t reserved = 0x246;
  constexpr std::uint16_t memory_control = 0x10;
  constexpr std::uint16_t timer_control = 0x46;
  write(ports, {.offset = reserved, .value = all_bits});
  require(read(ports, reserved) == 0, "Reserved PIM slots ignore writes and read zero");
  require(!Ports::contains(memory_control) && !Ports::contains(timer_control),
          "GPIO ownership does not swallow unrelated register ranges");
}
}

int main() try {
  reset_and_latches();
  output_state();
  attached_idle_bias();
  pulls_and_inputs();
  controls_and_routing();
  reserved_and_masked();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
