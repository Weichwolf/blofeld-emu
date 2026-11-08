// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/ports.hpp"

#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::dsp::Port;
using blofeld::vi::dsp::Ports;
using dsp56300::Space;
constexpr dsp56300::Word byte_mask = 0xff, control_pattern = 0x700;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void registers() {
  Ports ports;
  dsp56300::Word value = 0;
  constexpr dsp56300::Word c_data = 0xffffbd;
  constexpr dsp56300::Word c_direction = 0xffffbe;
  constexpr dsp56300::Word c_function = 0xffffbf;
  constexpr dsp56300::Word d_data = 0xffffd5;
  constexpr dsp56300::Word original_direction = 0xff00;
  constexpr dsp56300::Word implemented_direction = 0xf00;
  require(!ports.read(Space::x, c_data, value) && ports.read(Space::x, d_data, value) && value == 0,
          "C/E/F latches begin unknown while Port D has its documented reset value");
  require(ports.write({.space = Space::x, .address = c_direction, .value = original_direction}) &&
              ports.read(Space::x, c_direction, value) && value == implemented_direction,
          "The original reserved-one direction store retains only implemented fields");
  require(ports.write({.space = Space::x, .address = c_function, .value = byte_mask}) &&
              ports.write({.space = Space::x, .address = c_data, .value = control_pattern}) &&
              ports.read(Space::x, c_data, value) && value == control_pattern,
          "Output readback and input pull-downs share the actual data register");
  const auto output = ports.output(Port::c);
  require(output.mask == implemented_direction && (output.known & output.mask) == output.mask,
          "Only GPIO outputs can publish control transactions");
  ports.reset();
  require(ports.read(Space::x, c_data, value) && value == control_pattern &&
              ports.read(Space::x, c_direction, value) && value == 0,
          "Reset clears direction/function without erasing retained data latches");
}

void receive_data() {
  Ports ports;
  constexpr dsp56300::Word data = 0xfffff5;
  constexpr dsp56300::Word direction = 0xfffff6;
  constexpr dsp56300::Word function = 0xfffff7;
  constexpr dsp56300::Word latch = 0x400;
  constexpr dsp56300::Word inputs = 0x3ff;
  constexpr dsp56300::Word input_value = 0xa5;
  constexpr dsp56300::Word output_direction = 0x4ff;
  constexpr dsp56300::Word input_controls = 0x300;
  dsp56300::Word value = 0;
  require(
      ports.write({.space = Space::y, .address = data, .value = latch}) &&
          ports.write({.space = Space::y, .address = function, .value = input_controls}) &&
          ports.write({.space = Space::y, .address = direction, .value = output_direction}) &&
          ports.write({.space = Space::y, .address = function, .value = inputs}) &&
          !ports.read(Space::y, data, value) &&
          ports.write({.space = Space::y, .address = direction, .value = latch}),
      "Function-before-direction writes retain their fields; only reserved-mode data reads fail");
  ports.input(Port::f, {.mask = byte_mask, .known = byte_mask, .value = input_value});
  require(ports.read(Space::y, data, value) && value == (latch | input_value),
          "Real input data preserves output readback and documented upper input pulls");
  ports.input(Port::f, {.mask = byte_mask, .known = 0, .value = 0});
  require(!ports.read(Space::y, data, value), "An attached unknown byte overrides a pull-down");
  require(!Ports::contains(Space::x, data) && !Ports::contains(Space::program, data),
          "X/Y peripheral maps are distinct from program memory");
}
}

int main() try {
  registers();
  receive_data();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
