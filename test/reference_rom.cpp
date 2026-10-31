// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>
#include <vector>

namespace {
constexpr unsigned boot = 0xff0000;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned absolute_jump = 0x0af080;

struct PortWrite {
  bool y;
  unsigned address;
  unsigned value;
};

// Literal authored instructions, independent of an assembler or execution model.
class Program {
public:
  void word(const std::uint32_t value) {
    if (value > word_mask) { throw blofeld::Fault("DSP fixture word exceeds 24 bits"); }
    words_.push_back(value);
  }

  void movep(const PortWrite input) {
    const auto [y, address, value] = input;
    constexpr unsigned io_base = 0xffff80;
    constexpr unsigned high_io_base = 0xffffc0;
    constexpr unsigned group_width = 64;
    // DSP56300FM MOVEP immediate forms, low-X/low-Y/high-X/high-Y groups.
    constexpr std::array opcodes{0x07f400U, 0x07b480U, 0x08f480U, 0x09f480U};
    if (address < io_base || address > word_mask) {
      throw blofeld::Fault("Authored MOVEP destination exceeds the documented I/O groups");
    }
    const auto group = (address >= high_io_base ? 2U : 0U) + static_cast<unsigned>(y);
    word(opcodes.at(group) | ((address - io_base) % group_width));
    word(value);
  }

  [[nodiscard]] unsigned pc() const { return boot + static_cast<unsigned>(words_.size()); }

  [[nodiscard]] const std::vector<std::uint32_t> &words() const { return words_; }

private:
  std::vector<std::uint32_t> words_;
};

void write_rom(const std::filesystem::path &path, const Program &program) {
  std::ofstream stream(path, std::ios::binary);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  for (const auto word : program.words()) {
    constexpr unsigned high_shift = 16;
    const std::array bytes{static_cast<char>(word >> high_shift),
                           static_cast<char>(word >> 8U),
                           static_cast<char>(word)};
    stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }
  stream.close();
}

void echo_loop(Program &result) {
  // Authored polling diagnostic: actual HRX -> RAM -> HTX, no host word injection.
  constexpr unsigned immediate_r4 = 0x64f400;
  constexpr unsigned address = 0x100;
  constexpr unsigned jclr_hrdf = 0x019191;
  constexpr unsigned hrx_x0 = 0x044494;
  constexpr unsigned store_x0_post_r4 = 0x445c00;
  constexpr unsigned x0_htx = 0x04c493;
  result.word(immediate_r4);
  result.word(address);
  const auto loop = result.pc();
  result.word(jclr_hrdf);
  result.word(loop);
  result.word(hrx_x0);
  result.word(store_x0_post_r4);
  result.word(x0_htx);
  result.word(absolute_jump);
  result.word(loop);
}

Program capture_program() {
  constexpr unsigned status = 0xc00000;
  constexpr unsigned vector_offset = 0x54;
  Program result;
  constexpr unsigned immediate_vba = 0x05f430;
  constexpr unsigned immediate_sr = 0x05f439;
  constexpr unsigned shi_control = 0xffff90;
  constexpr unsigned interrupt_priority = 0xfffffe;
  constexpr unsigned timer_load = 0xffff8e;
  constexpr unsigned timer_compare = 0xffff8d;
  constexpr unsigned timer_control = 0xffff8f;
  constexpr unsigned timer_priority = 0x200;
  constexpr unsigned capture_enabled = 0x165;
  result.word(immediate_vba);
  result.word(boot);
  result.word(immediate_sr);
  result.word(status);
  result.movep({.y = false, .address = shi_control, .value = 0});
  result.movep({.y = false, .address = interrupt_priority, .value = timer_priority});
  result.movep({.y = false, .address = timer_load, .value = 0});
  result.movep({.y = false, .address = timer_compare, .value = 1});
  result.movep({.y = false, .address = timer_control, .value = capture_enabled});
  const auto loop = result.pc();
  result.word(0);
  result.word(absolute_jump);
  result.word(loop);
  while (result.pc() < boot + vector_offset) { result.word(0); }
  if (result.pc() != boot + vector_offset) {
    throw blofeld::Fault("Authored capture vector exceeds its declared VBA-relative address");
  }
  // Actual two-word fast vector enables a real SHI request output.
  constexpr unsigned shi_status = 0xffff91;
  constexpr unsigned request_enabled = 0x89;
  result.movep({.y = false, .address = shi_status, .value = request_enabled});
  return result;
}

Program esai_program(const std::string_view mode) {
  const bool second = mode.starts_with("esai1");
  constexpr unsigned primary = 0xffffa0;
  constexpr unsigned secondary = 0xffff80;
  constexpr unsigned legal_mux = 0x838;
  constexpr unsigned missing_hckt_mux = 0x818;
  constexpr unsigned port_control = 31;
  constexpr unsigned port_direction = 30;
  constexpr unsigned transmit_control = 21;
  constexpr unsigned clock_control = 22;
  const unsigned base = second ? secondary : primary;
  Program result;
  const auto put = [&](const unsigned offset, const unsigned value) {
    result.movep({.y = second, .address = base + offset, .value = value});
  };
  const auto mux = mode.ends_with("missing-hckt") ? missing_hckt_mux : legal_mux;
  constexpr unsigned word32 = 0x7d00;
  constexpr unsigned bit24 = 0xf900;
  constexpr unsigned external_clock = 0x200;
  const auto format = mode.ends_with("external") ? bit24 : word32;
  constexpr unsigned hckt_clock = 0x640301;
  constexpr unsigned first = 0xa5c39f;
  put(transmit_control, format);
  if (mode.ends_with("active")) {
    put(port_control, mux);
    put(port_direction, mux);
  }
  const bool external = mode.ends_with("external") || mode.ends_with("external-word");
  constexpr unsigned external_sckt_generated_frame = 0x400200;
  auto selected_clock = external ? external_clock : hckt_clock;
  if (mode.ends_with("external-sckt")) { selected_clock = external_sckt_generated_frame; }
  put(clock_control, selected_clock);
  if (!mode.ends_with("active")) {
    put(port_control, mux);
    put(port_direction, mux);
  }
  put(0, first);
  put(transmit_control, format | 1U);
  if (mode.ends_with("refill") || mode.ends_with("drop-sckt")) {
    constexpr unsigned ordinary_delay = 64;
    constexpr unsigned tied_delay = 154;
    const auto refill_delay = mode.ends_with("tied-refill") ? tied_delay : ordinary_delay;
    for (unsigned index = 0; index < refill_delay; ++index) { result.word(0); }
    constexpr unsigned next = 0x123456;
    if (mode.ends_with("drop-sckt")) {
      constexpr unsigned sckt_mux = 8;
      put(port_control, mux & ~sckt_mux);
    } else {
      put(0, next);
    }
  }
  const auto loop = result.pc();
  result.word(0);
  result.word(absolute_jump);
  result.word(loop);
  return result;
}

Program program(const std::string_view mode) {
  if (mode == "esai0" || mode == "esai1" || mode == "esai0-refill" || mode == "esai1-refill" ||
      mode == "esai1-external" || mode == "esai1-tied-refill" || mode == "esai1-missing-hckt" ||
      mode == "esai1-active" || mode == "esai1-external-word" || mode == "esai1-external-sckt" ||
      mode == "esai1-drop-sckt") {
    return esai_program(mode);
  }
  if (mode == "capture-irq") { return capture_program(); }
  if (mode != "transmit" && mode != "unknown-transmit" && mode != "delayed-request" &&
      mode != "short-request" && mode != "echo") {
    throw blofeld::Fault("Unknown SHI test ROM scenario");
  }
  Program result;
  if (mode == "delayed-request" || mode == "short-request") {
    constexpr unsigned ordinary_delay = 64;
    constexpr unsigned short_delay = 257;
    const unsigned delay = mode == "short-request" ? short_delay : ordinary_delay;
    for (unsigned i = 0; i < delay; ++i) { result.word(0); }
  }
  // Authored mode-0, 24-bit single-word receive request. DSP56371UM chapter 7.
  constexpr unsigned shi_control = 0xffff90;
  constexpr unsigned shi_status = 0xffff91;
  constexpr unsigned shi_transmit = 0xffff93;
  constexpr unsigned receive_request = 0x89;
  constexpr unsigned transmit_receive_request = 0x189;
  constexpr unsigned outbound = 0xa15c37;
  result.movep({.y = false, .address = shi_control, .value = 0});
  result.movep({.y = false,
                .address = shi_status,
                .value = mode == "echo" ? transmit_receive_request : receive_request});
  if (mode == "transmit" || mode == "echo") {
    result.movep({.y = false, .address = shi_transmit, .value = outbound});
  }
  if (mode == "echo") {
    echo_loop(result);
    return result;
  }
  if (mode == "delayed-request" || mode == "short-request") {
    const unsigned pulse = mode == "delayed-request" ? 16 : 0;
    for (unsigned i = 0; i < pulse; ++i) { result.word(0); }
    constexpr unsigned request_disabled = 0x88;
    result.movep({.y = false, .address = shi_status, .value = request_disabled});
  }
  const auto loop = result.pc();
  result.word(0);
  result.word(absolute_jump);
  result.word(loop);
  return result;
}
}

int main(const int argc, char **argv) {
  try {
    if (argc != 3) { throw blofeld::Fault("Usage: reference-rom scenario output.bin"); }
    write_rom(argv[2], program(argv[1]));
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
