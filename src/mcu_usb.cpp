// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_usb.hpp"

#include "blofeld/bus.hpp"
#include "mcu_usb_registers.hpp"

#include <cstdint>
#include <limits>
#include <string_view>

namespace blofeld {
namespace {
constexpr unsigned select_high = 0x30e;
constexpr unsigned select_low = 0x30f;
constexpr unsigned data_high = 0x310;
constexpr unsigned data_low = 0x312;
constexpr unsigned data_end = 0x314;
constexpr unsigned selector_mask = 15;
constexpr unsigned buffer_selector_mask = 0x77;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned word_mask = 0xffff;
constexpr unsigned number_configuration_mask = 0x773;
constexpr unsigned number_configurations = 8;
constexpr unsigned alternate_settings = 9;
}

void McuUsb::reset() {
  const UsbHostEvidence next{.link = next_host_revision(host_evidence_.link),
                             .descriptions = next_host_revision(host_evidence_.descriptions)};
  *this = McuUsb{};
  host_evidence_ = next;
}

std::uint64_t McuUsb::next_host_revision(const std::uint64_t revision) {
  if (revision == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("USB host evidence revision exhausted");
  }
  return revision + 1;
}

std::string_view McuUsb::register_name(const std::uint16_t offset) {
  if (offset == usb::status || offset == usb::status + 1) { return "UMSR1"; }
  if (offset == usb::interrupts || offset == usb::interrupts + 1) { return "UIMR"; }
  if (offset == usb::timestamp || offset == usb::timestamp + 1) { return "UTSR"; }
  if (offset == usb::configuration_status || offset == usb::configuration_status + 1) {
    return "UCCSR";
  }
  if (offset >= usb::endpoint_begin && offset < usb::endpoint_end) {
    return "UEPCSR local endpoint";
  }
  if ((offset & ~1U) == usb::ique_out_control) { return "UEPCSR4A unconfigured IQUE endpoint"; }
  if ((offset & ~1U) == usb::ique_in_control) { return "UEPCSR5A unconfigured IQUE endpoint"; }
  if (offset >= usb::setup_begin && offset < usb::setup_end) { return "USTB setup buffer"; }
  if (offset >= usb::buffer_begin && offset < usb::buffer_end) {
    return "UEPLB local endpoint buffer";
  }
  if (offset == select_high) { return "UEPCSELR high"; }
  if (offset == select_low) { return "UEPCSELR low"; }
  if (configuration_data(offset)) { return "UUCFGR configuration window"; }
  return {};
}

bool McuUsb::configuration_data(const std::uint16_t offset) {
  return offset >= data_high && offset < data_end;
}

unsigned McuUsb::selected_register() const {
  if (selector_ >= 1 && selector_ <= 5) { return selector_ - 1U; }
  if (selector_ == number_configurations) { return 5; }
  if (selector_ == alternate_settings) { return 6; }
  throw Fault("Reserved USB configuration selection " + hex(selector_, 2));
}

std::uint8_t McuUsb::read(const std::uint16_t offset) const {
  if (const auto value = read_live(offset)) { return *value; }
  if (offset == select_high) { return selector_; }
  if (offset == select_low) { return buffers_; }
  if (!configuration_data(offset)) {
    throw Fault("Unknown USB configuration read at " + hex(offset));
  }
  const auto shift = (data_end - offset - 1U) * 8;
  return static_cast<std::uint8_t>((configuration_.at(selected_register()) >> shift) & byte_mask);
}

void McuUsb::write(const ByteWrite access) {
  if (write_live(access)) { return; }
  if (access.address == select_high) {
    const auto next = static_cast<std::uint8_t>(access.value & selector_mask);
    if (staged_high_ && next != selector_) {
      throw Fault("Unverified USB configuration selection change between staged words");
    }
    selector_ = next;
    return;
  }
  if (access.address == select_low) {
    buffers_ = static_cast<std::uint8_t>(access.value & buffer_selector_mask);
    return;
  }
  if (configuration_data(access.address)) {
    throw Fault("USB UUCFGR requires ordered high/low 16-bit operand writes");
  }
  throw Fault("Unknown USB configuration write at " + hex(access.address));
}

void McuUsb::write_memory(const MemoryWrite access) {
  if (access.size != 2 || (access.address != data_high && access.address != data_low)) {
    throw Fault("Unsupported USB configuration write width/alignment at " + hex(access.address));
  }
  const auto index = selected_register();
  if (enabled() && index >= 5) { return; } // UNCIR/UNASR writes require MEN=0.
  const auto word = static_cast<std::uint16_t>(access.value & word_mask);
  if (access.address == data_high) {
    staged_high_ = word;
    return;
  }
  if (!staged_high_) {
    throw Fault("Unknown USB configuration staging word before low-word commit");
  }
  if (index < 3 && buffers_data_.at(index == 0 ? 0 : index + 1).transmitted) {
    throw Fault(
        "Unverified USB endpoint description change before transmitted data is acknowledged");
  }
  auto value = (static_cast<std::uint32_t>(*staged_high_) << 16U) | word;
  if (selector_ == number_configurations) {
    value &= number_configuration_mask;
    const auto configurations = value & 3U;
    const auto interfaces1 = (value >> 4U) & 7U;
    const auto interfaces2 = (value >> 8U) & 7U;
    if (configurations > 2 || interfaces1 > 4 || interfaces2 > 4) {
      throw Fault("Reserved USB configuration/interface count");
    }
  }
  if (configuration_.at(index) != value) {
    host_evidence_.descriptions = next_host_revision(host_evidence_.descriptions);
    configuration_.at(index) = value;
  }
  staged_high_.reset();
}

}
