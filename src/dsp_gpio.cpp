// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_gpio.hpp"

#include "blofeld/bus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned serial_mask = 0xfff;
constexpr unsigned dedicated_mask = 0x7ff;

struct PortSpec {
  const char *space;
  std::uint32_t base;
};

constexpr std::array<PortSpec, 4> specs{{{.space = "X:", .base = 0xffffbd},
                                         {.space = "Y:", .base = 0xffff9d},
                                         {.space = "Y:", .base = 0xfffff5},
                                         {.space = "X:", .base = 0xffffd5}}};
}

DspGpio::DspGpio(const DspPort port) : port_(port) {
  if (static_cast<std::size_t>(port) >= specs.size()) { throw Fault("Invalid DSP GPIO port"); }
  if (port == DspPort::d) { latch_known_ = pin_mask(); }
}

std::uint32_t DspGpio::pin_mask() const {
  if (port_ == DspPort::d) { return 3; }
  return port_ == DspPort::f ? dedicated_mask : serial_mask;
}

std::string DspGpio::address(const DspGpioRegister reg) const {
  const auto &spec = specs.at(static_cast<std::size_t>(port_));
  return std::string(spec.space) + hex(spec.base + static_cast<unsigned>(reg), 6);
}

void DspGpio::reset() {
  control_ = 0;
  direction_ = 0;
  if (port_ == DspPort::d) {
    latch_ = 0;
    latch_known_ = pin_mask();
  }
  // PDR is not reset. Cold latches start unknown; reset does not invent zeros.
}

DspGpioOutputs DspGpio::outputs() const {
  const auto output_mask = direction_ & ~control_;
  return {.mask = output_mask,
          .known = latch_known_ & output_mask,
          .levels = latch_ & latch_known_ & output_mask,
          .peripheral = direction_ & control_};
}

DspGpioSample DspGpio::resolve(const std::uint32_t mask) const {
  const auto output = outputs();
  const auto conflict = mask & output.known & external_.mask & (output.levels ^ external_.levels);
  if (conflict != 0) {
    throw Fault("DSP GPIO pad contention at " + address(DspGpioRegister::data) +
                " mask=" + hex(conflict, 6));
  }
  const auto external_mask = mask & external_.mask & ~output.mask & ~output.peripheral;
  const auto known = (output.known | external_mask) & mask;
  return {.known = known, .levels = (output.levels | (external_.levels & external_mask)) & known};
}

DspGpioSample DspGpio::observe_pads(const std::uint32_t mask) const {
  if ((mask & ~pin_mask()) != 0) { throw Fault("Invalid DSP GPIO pad observation mask"); }
  return resolve(mask);
}

void DspGpio::validate_peripheral_drive(const DspGpioDrive drive) const {
  if ((drive.mask & ~outputs().peripheral) != 0 || (drive.levels & ~drive.mask) != 0) {
    throw Fault("Invalid DSP peripheral output mask at " + address(DspGpioRegister::data));
  }
  if ((drive.mask & external_.mask & (drive.levels ^ external_.levels)) != 0) {
    throw Fault("DSP peripheral pad contention at " + address(DspGpioRegister::data));
  }
}

std::uint32_t DspGpio::read(const DspGpioRegister reg, const std::uint32_t mask) const {
  if ((mask & ~DspGpioWrite::word_mask) != 0) { throw Fault("DSP GPIO read mask exceeds 24 bits"); }
  switch (reg) {
    case DspGpioRegister::control: return control_ & mask;
    case DspGpioRegister::direction: return direction_ & mask;
    case DspGpioRegister::data: {
      const auto requested = mask & pin_mask();
      const auto gpio = control_ ^ direction_;
      const auto sample = resolve(requested);
      const auto disconnected = port_ == DspPort::d ? requested & ~(control_ | direction_) : 0U;
      const auto retained = disconnected & latch_known_;
      const auto unknown = requested & ~((sample.known & gpio) | retained);
      if (unknown != 0) {
        throw Fault("Unknown DSP GPIO data read at " + address(reg) + " mask=" + hex(unknown, 6));
      }
      return ((sample.levels & gpio) | (latch_ & retained)) & requested;
    }
    default: throw Fault("Invalid DSP GPIO register read");
  }
}

void DspGpio::write_checked(const DspGpioWrite access) {
  if ((access.mask & ~DspGpioWrite::word_mask) != 0 || (access.value & ~access.mask) != 0) {
    throw Fault("Invalid DSP GPIO masked write at " + address(access.reg));
  }
  if ((access.value & ~pin_mask()) != 0) {
    throw Fault("Unqualified DSP GPIO reserved-bit write at " + address(access.reg) +
                " value=" + hex(access.value, 6));
  }
  switch (access.reg) {
    case DspGpioRegister::control: control_ = (control_ & ~access.mask) | access.value; break;
    case DspGpioRegister::direction: direction_ = (direction_ & ~access.mask) | access.value; break;
    case DspGpioRegister::data: {
      const auto outputs = direction_ & ~control_;
      const auto selected = outputs & access.mask;
      latch_ = (latch_ & ~selected) | (access.value & selected);
      // Input/disconnected/peripheral writes do not qualify a later output latch.
      latch_known_ = (latch_known_ | selected) & outputs;
      break;
    }
    default: throw Fault("Invalid DSP GPIO register write");
  }
  if (port_ == DspPort::f && (control_ & direction_) != 0) {
    throw Fault("Reserved DSP Port F pin function selected by write at " + address(access.reg));
  }
  static_cast<void>(resolve(pin_mask()));
}

void DspGpio::write(const DspGpioWrite access) {
  auto next = *this;
  next.write_checked(access);
  *this = next;
}

void DspGpio::set_drive(const DspGpioDrive drive) {
  if ((drive.mask & ~pin_mask()) != 0 || (drive.levels & ~drive.mask) != 0) {
    throw Fault("Invalid DSP GPIO external drive at " + address(DspGpioRegister::data));
  }
  auto next = *this;
  next.external_ = drive;
  static_cast<void>(next.resolve(pin_mask()));
  *this = next;
}

}
