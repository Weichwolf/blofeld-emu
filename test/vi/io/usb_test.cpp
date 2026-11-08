// SPDX-License-Identifier: GPL-3.0-only
#include "io/usb.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::Usb;
constexpr std::uint16_t control_clock = 0x4000, control_enabled = 0xc000;
constexpr std::uint16_t status = 0x302, interrupts = 0x304, timestamp = 0x30a;
constexpr std::uint16_t configuration_status = 0x30c, select = 0x30e, data = 0x310;
constexpr std::uint16_t buffer = 0x380, other_buffer = 0x3c0;
constexpr std::uint16_t reset_status = 0xe040, interrupt_mask = 0x7807, all_bits = 0xffff;
constexpr std::uint16_t counts = 0x0800, alternates = 0x0900, count_mask = 0x0773;
constexpr std::uint16_t lower = 0x5678, upper = 0x1234;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void write(Usb &usb, const Usb::Write access) {
  require(usb.write(access, control_clock), "Write a defined USB register contract");
}

std::uint16_t read(const Usb &usb, const Usb::Read access) {
  std::uint16_t value = 0;
  require(usb.read(access, value), "Read a known USB register or initialized buffer byte");
  return value;
}

void reset_and_status() {
  Usb usb;
  require(read(usb, {.offset = status, .bytes = 2}) == reset_status &&
              read(usb, {.offset = timestamp, .bytes = 2}) == 0 &&
              read(usb, {.offset = configuration_status, .bytes = 2}) == 0 && !usb.pending(),
          "Reset has no transaction, enumeration, SOF or configuration request");
  write(usb, {.offset = status, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = status, .bytes = 2}) == reset_status,
          "W1C cannot fabricate flags or change read-only active endpoint/speed fields");
  write(usb, {.offset = interrupts, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = interrupts, .bytes = 2}) == interrupt_mask &&
              usb.interrupts_enabled(),
          "The interrupt mask retains only implemented enables");
  constexpr std::uint16_t status_vector = 0xffd4;
  require(usb.pending() == status_vector, "RESUMEIE with USD=0 requests the documented level IRQ");
  write(usb, {.offset = interrupts, .value = 0, .bytes = 2});
  write(usb, {.offset = configuration_status, .value = all_bits, .bytes = 2});
  require(!usb.pending() && read(usb, {.offset = configuration_status, .bytes = 2}) == 0,
          "DONEECRU without a host request cannot invent configuration success");
  std::uint16_t value = 0;
  constexpr std::uint16_t setup = 0x338;
  require(!usb.read({.offset = setup}, value) && !usb.write({.offset = setup}, control_clock) &&
              !usb.read({.offset = status - 2}, value),
          "Unknown setup contents and another owner's UMCR are not swallowed");
}

void configuration_commits() {
  Usb usb;
  write(usb, {.offset = select, .value = 0x0100, .bytes = 2});
  write(usb, {.offset = data, .value = upper, .bytes = 2});
  require(read(usb, {.offset = data, .bytes = 2}) == 0 &&
              read(usb, {.offset = data + 2, .bytes = 2}) == 0,
          "The first word stages data without changing the selected 32-bit register");
  write(usb, {.offset = data + 2, .value = lower, .bytes = 2});
  require(read(usb, {.offset = data, .bytes = 2}) == upper &&
              read(usb, {.offset = data + 2, .bytes = 2}) == lower &&
              read(usb, {.offset = data}) == (upper >> 8U),
          "The second word commits both halves with big-endian readback");
  require(!usb.write({.offset = data, .value = 0}, control_clock) &&
              !usb.write({.offset = data + 2, .value = 1, .bytes = 2}, control_clock),
          "Byte writes and a missing staged high word cannot pretend a valid commit");

  write(usb, {.offset = select, .value = counts, .bytes = 2});
  write(usb, {.offset = data, .value = all_bits, .bytes = 2});
  write(usb, {.offset = data + 2, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = data, .bytes = 2}) == 0 &&
              read(usb, {.offset = data + 2, .bytes = 2}) == count_mask,
          "UNCIR has the documented reserved-bit mask");
  require(usb.write({.offset = data, .value = 0, .bytes = 2}, control_enabled) &&
              usb.write({.offset = data + 2, .value = 1, .bytes = 2}, control_enabled) &&
              read(usb, {.offset = data + 2, .bytes = 2}) == count_mask,
          "UNCIR writes while MEN=1 are ignored rather than changing active limits");
  write(usb, {.offset = select, .value = alternates, .bytes = 2});
  write(usb, {.offset = data, .value = upper, .bytes = 2});
  write(usb, {.offset = data + 2, .value = lower, .bytes = 2});
  require(read(usb, {.offset = data, .bytes = 2}) == upper &&
              read(usb, {.offset = data + 2, .bytes = 2}) == lower,
          "UNASR retains all eight alternate-setting nibbles");

  constexpr std::uint16_t endpoint = 0x0200;
  constexpr std::uint16_t noniso_sof = 0xc100;
  constexpr std::uint16_t bulk_in = 0x0052;
  write(usb, {.offset = select, .value = endpoint, .bytes = 2});
  require(usb.write({.offset = data, .value = noniso_sof, .bytes = 2}, control_enabled) &&
              usb.write({.offset = data + 2, .value = bulk_in, .bytes = 2}, control_enabled) &&
              read(usb, {.offset = data, .bytes = 2}) == noniso_sof &&
              read(usb, {.offset = data + 2, .bytes = 2}) == bulk_in,
          "Endpoint configuration stays writable with MEN=1 and preserves non-isochronous NISOF");
}

void direct_buffer_windows() {
  Usb usb;
  auto windows = usb.regions(0);
  require(windows[0].first == buffer && windows[1].first == other_buffer &&
              windows[0].bytes.data() == windows[1].bytes.data() && windows[0].known[0] == 0,
          "Reset windows alias the same borrowed RAM, with unknown initial contents");
  constexpr std::uint8_t first = 0x5a;
  constexpr std::uint8_t second = 0xa5;
  windows[0].bytes[0] = first;
  windows[0].known[0] = 1;
  require(read(usb, {.offset = other_buffer}) == first,
          "Direct memory and register fallback observe the same backing storage");
  constexpr std::uint16_t initial_selection = 0x23;
  constexpr std::uint16_t reversed_selection = 0x32;
  write(usb, {.offset = select + 1, .value = initial_selection});
  require(usb.mapping_pending() && usb.regions(0)[0].bytes.data() == windows[0].bytes.data() &&
              usb.commit_mapping() && !usb.mapping_pending() && !usb.commit_mapping(),
          "Bank changes require exactly one retired-instruction rebind");
  write(usb, {.offset = buffer, .value = first});
  write(usb, {.offset = other_buffer, .value = second});
  write(usb, {.offset = select + 1, .value = reversed_selection});
  require(usb.commit_mapping(), "Commit the reversed buffer mapping");
  windows = usb.regions(0);
  require(windows[0].bytes[0] == second && windows[1].bytes[0] == first &&
              read(usb, {.offset = buffer}) == second &&
              read(usb, {.offset = other_buffer}) == first,
          "Window changes select RAM instead of copying or clearing payloads");
  std::uint16_t value = 0;
  require(!usb.read({.offset = buffer + 1}, value), "Unwritten buffer bytes remain unknown");
  constexpr std::uint16_t reserved_windows = 0x77;
  write(usb, {.offset = select + 1, .value = reserved_windows});
  require(usb.commit_mapping() && usb.regions(0)[0].bytes.empty() &&
              !usb.read({.offset = buffer}, value) && !usb.write({.offset = buffer}, control_clock),
          "Reserved buffer selectors withdraw grants and fail actual accesses");
  write(usb, {.offset = select + 1, .value = initial_selection});
  require(usb.commit_mapping(), "Restore the valid bank selection");
  constexpr std::uint16_t relocated_first = 0x780;
  constexpr std::uint16_t relocated_second = 0x7c0;
  const auto relocated = usb.regions(0x400);
  require(relocated[0].first == relocated_first && relocated[1].first == relocated_second &&
              relocated[0].bytes[0] == first,
          "INITRG relocation moves the views while preserving the selected buffers");
}

void endpoint_controls() {
  Usb usb;
  constexpr std::uint16_t local = 0x314;
  constexpr std::uint16_t auxiliary_out = 0x31c;
  constexpr std::uint16_t out_status = 0x31e;
  constexpr std::uint16_t in_status = 0x322;
  constexpr std::uint16_t local_mask = 0xd07f;
  constexpr std::uint16_t auxiliary_mask = 0xdfff;
  constexpr std::uint16_t out_mask = 0x500f;
  constexpr std::uint16_t in_mask = 0x100f;
  write(usb, {.offset = local, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = local, .bytes = 2}) == local_mask && !usb.pending() &&
              usb.interrupts_enabled(),
          "Priming DVALID and TCIE cannot manufacture a completed transfer or IRQ");
  write(usb, {.offset = auxiliary_out, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = auxiliary_out, .bytes = 2}) == auxiliary_mask,
          "The IQUE control word has its own size and continue-on-error fields");
  write(usb, {.offset = out_status, .value = all_bits, .bytes = 2});
  write(usb, {.offset = in_status, .value = all_bits, .bytes = 2});
  require(read(usb, {.offset = out_status, .bytes = 2}) == out_mask &&
              read(usb, {.offset = in_status, .bytes = 2}) == in_mask && !usb.pending(),
          "Error/short-packet flags are W1C; unavailable IQUE buffers are not invented");
}
}

int main() try {
  reset_and_status();
  configuration_commits();
  direct_buffer_windows();
  endpoint_controls();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
