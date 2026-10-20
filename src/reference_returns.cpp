// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "reference_board.hpp"

#include <cstdint>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>

namespace blofeld::reference {
namespace {
bool overlaps(const ReturnRoute route, const McuPort port, const std::uint8_t mask) {
  return route.destination == port && (route.mask & mask) != 0;
}

void validate_route(const Config &config, const ReturnRoute route) {
  McuGpio gpio;
  gpio.set_drive(route.destination, {.mask = route.mask});
  const auto serial = static_cast<std::uint8_t>(config.select_n | config.clock | config.mosi);
  if (overlaps(route, McuPort::b, serial)) {
    throw Fault("SHI return feedback into SPI input pins is unqualified");
  }
  if (config.loopback) {
    const auto &wire = *config.loopback;
    if (overlaps(route, wire.destination, wire.destination_mask)) {
      throw Fault("SHI return destination overlaps the loopback driver");
    }
    if (wire.destination == McuPort::b && (wire.destination_mask & serial) != 0) {
      throw Fault("SPI input loopback with SHI returns is unqualified");
    }
  }
  const auto check = [&](const Input &input) {
    const auto mask = static_cast<std::uint8_t>(input.drive.mask | input.drive.floating);
    if (input.kind == InputKind::drive && overlaps(route, input.port, mask)) {
      throw Fault("SHI return destination also has an explicit external drive/floating input");
    }
  };
  for (const auto &input : config.initial) { check(input); }
  for (const auto &input : config.events) { check(input); }
}

bool same(const ShiOutput first, const ShiOutput second) {
  return first.driven == second.driven && first.level == second.level;
}
}

void validate_returns(const Config &config) {
  if (config.hreq && config.miso &&
      overlaps(*config.hreq, config.miso->destination, config.miso->mask)) {
    throw Fault("SHI return destinations overlap");
  }
  for (const auto &route : {config.hreq, config.miso}) {
    if (route) { validate_route(config, *route); }
  }
}

void Board::update_return(const std::optional<ReturnRoute> &route,
                          const ShiOutput output,
                          std::optional<ShiOutput> &previous,
                          const std::string_view name) {
  if (!route) { return; }
  PinDrive drive;
  if (!output.driven) {
    drive.floating = route->mask;
  } else if (output.level) {
    drive.mask = route->mask;
    drive.levels = (*output.level != route->inverted) ? route->mask : 0;
  }
  try {
    mcu_.bus().update_pin_drive(route->destination, route->mask, drive);
    static_cast<void>(mcu_.bus().observe_pads(route->destination, route->mask));
  } catch (const Fault &fault) {
    throw Fault("SHI " + std::string(name) +
                " return pad observation mcu-pc=" + hex(mcu_.cpu().state().pc) +
                " dsp-pc=" + hex(dsp_.cpu().state().pc, 6) + ": " + fault.what());
  }
  if (execution_.trace != nullptr && (!previous || !same(*previous, output))) {
    constexpr std::string_view ports = "ABETMJPQRSU";
    *execution_.trace << "SHI-RETURN end="
                      << return_boundary_.value_or(schedule_ ? schedule_->end_ticks() : ticks())
                      << " signal=" << name << " driven=" << output.driven << " level=";
    if (output.level) {
      *execution_.trace << (*output.level ? 1 : 0);
    } else {
      *execution_.trace << "unknown";
    }
    *execution_.trace << " port=" << ports.at(static_cast<unsigned>(route->destination))
                      << " mask=" << hex(route->mask, 2) << " inverted=" << route->inverted << '\n';
  }
  previous = output;
}

DspCompletionObserver Board::return_observer() {
  if (!config_.hreq && !config_.miso) { return {}; }
  return {.observe = [this](const DspScheduledStep &step) {
    return_boundary_ = step.completion_ticks;
    update_wiring();
    return_boundary_.reset();
  }};
}

void Board::update_wiring() {
  if (config_.hreq || config_.miso) {
    const auto outputs = dsp_.bus().shi().outputs();
    update_return(config_.hreq, outputs.request, hreq_observed_, "hreq");
    update_return(config_.miso, outputs.miso, miso_observed_, "miso");
  }
  update_loopback();
}
}
