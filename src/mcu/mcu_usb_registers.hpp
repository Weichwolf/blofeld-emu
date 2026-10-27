// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld::usb {
inline constexpr unsigned status = 0x302;
inline constexpr unsigned interrupts = 0x304;
inline constexpr unsigned timestamp = 0x30a;
inline constexpr unsigned configuration_status = 0x30c;
inline constexpr unsigned endpoint_begin = 0x314;
inline constexpr unsigned endpoint_end = 0x31c;
inline constexpr unsigned setup_begin = 0x338;
inline constexpr unsigned setup_end = 0x340;
inline constexpr unsigned buffer_begin = 0x380;
inline constexpr unsigned buffer_second = 0x3c0;
inline constexpr unsigned buffer_end = 0x400;
inline constexpr unsigned enable = 0x8000;
inline constexpr unsigned clock_enable = 0x4000;
inline constexpr unsigned phy_suspend = 0x800;
inline constexpr unsigned disconnect = 0x20;
inline constexpr unsigned descriptor_support = 0x10;
inline constexpr unsigned synch_frame_support = 1;
// USB20D6E2F UMCR3.1.1: capability and power-source selection, not host enable.
inline constexpr unsigned remote_wakeup_capable = 4;
inline constexpr unsigned self_powered = 2;
inline constexpr unsigned suspend_change = 0x1000;
inline constexpr unsigned reset_change = 0x800;
inline constexpr unsigned enumerated = 0x400;
inline constexpr unsigned full_speed = 0x40;
inline constexpr unsigned suspended = 0x10;
inline constexpr unsigned reset_detected = 8;
inline constexpr unsigned sof = 4;
inline constexpr unsigned setup_overrun = 2;
inline constexpr unsigned setup = 1;
inline constexpr unsigned clearable_status = 0x1807;
inline constexpr unsigned interrupt_mask = 0x7807;
inline constexpr unsigned resume_enable = 0x4000;
inline constexpr unsigned configuration_enable = 0x2000;
inline constexpr unsigned configuration_request = 0x2000;
inline constexpr unsigned configuration_done = 0x1000;
inline constexpr unsigned configuration_valid = 0x8000;
inline constexpr unsigned interface_valid = 0x4000;
inline constexpr unsigned frame_mask = 0x7ff;
inline constexpr unsigned stall = 0x8000;
inline constexpr unsigned data_valid = 0x4000;
inline constexpr unsigned transfer_complete = 0x2000;
inline constexpr unsigned transfer_enable = 0x1000;
inline constexpr unsigned packet_size = 0x7f;
inline constexpr unsigned byte_mask = 0xff;
inline constexpr unsigned nibble_mask = 15;
inline constexpr unsigned buffer_bytes = 64;
inline constexpr unsigned maximum_packet_shift = 19;
inline constexpr unsigned input_direction = 0x10;
inline constexpr unsigned ique_out_control = 0x31c;
inline constexpr unsigned ique_in_control = 0x320;
inline constexpr unsigned ique_send_nak = 0x4000;
inline constexpr unsigned ique_continue_error = 0x800;
inline constexpr unsigned ique_writable =
    stall | ique_send_nak | transfer_enable | ique_continue_error | frame_mask;

constexpr bool ique_control(const unsigned offset) {
  return (offset & ~1U) == ique_out_control || (offset & ~1U) == ique_in_control;
}
}
