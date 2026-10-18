// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_shi.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
// Independent reference: DSP56371UM Rev 2.1, sections 7.4, 7.7.1 and tables 7-1..5.
constexpr unsigned hckr = 0xffff90;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned hsar = 0xffff92;
constexpr unsigned htx = 0xffff93;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned word_bits = 24;
constexpr unsigned initial_slave = 0xb00000;
constexpr unsigned reserved_rx_enable = 0x2001;
constexpr unsigned enabled_master = 0x41;
constexpr unsigned tx_vector = 0x40;
constexpr unsigned tx_error_vector = 0x42;
constexpr unsigned rx_vector = 0x44;
constexpr unsigned full_vector = 0x48;
constexpr unsigned rx_error_vector = 0x4a;
constexpr unsigned idle_empty = 0x8200;
constexpr unsigned underrun = 0x4000;
constexpr unsigned tx_empty = 0x8000;
constexpr unsigned rx_nonempty = 0x20000;
constexpr unsigned rx_full = 0x80000;
constexpr unsigned overrun = 0x100000;
constexpr unsigned busy = 0x400000;
constexpr unsigned spi24 = 9;
constexpr unsigned fifo24 = 0x29;
constexpr unsigned fifo24_request = 0xa9;
constexpr unsigned outbound = 0xa15c37;
constexpr unsigned inbound = 0x123456;

void require(const bool value, const std::string &message) {
  if (!value) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

struct SerialWord {
  unsigned value = 0;
  unsigned bits = word_bits;
};

struct SampledWord {
  unsigned value = 0;
  bool known = true;
};

class Peer {
public:
  explicit Peer(blofeld::DspShi &shi, const unsigned mode = 0) : shi_(shi), mode_(mode) {
    pins_.clock = (mode_ & 2U) != 0;
    pins_.mosi = false;
    shi_.observe_spi(pins_);
    shi_.write({.address = hckr, .value = mode});
  }

  void select() {
    pins_.selected = true;
    shi_.observe_spi(pins_);
  }

  void deselect() {
    pins_.selected = false;
    shi_.observe_spi(pins_);
  }

  std::optional<unsigned> bits(const SerialWord transfer) {
    SampledWord received;
    const auto value = transfer.value;
    const auto count = transfer.bits;
    for (unsigned bit = 0; bit < count; ++bit) {
      pins_.mosi = ((value >> (word_bits - 1 - bit)) & 1U) != 0;
      shi_.observe_spi(pins_);
      pins_.clock = !pins_.clock;
      shi_.observe_spi(pins_);
      if ((mode_ & 1U) == 0) { sample(received); }
      pins_.clock = !pins_.clock;
      shi_.observe_spi(pins_);
      if ((mode_ & 1U) != 0) { sample(received); }
    }
    return received.known ? std::optional<unsigned>(received.value) : std::nullopt;
  }

  std::optional<unsigned> word(const SerialWord transfer) {
    select();
    const auto received = bits(transfer);
    deselect();
    return received;
  }

private:
  void sample(SampledWord &word) const {
    const auto pin = shi_.outputs().miso;
    require(pin.driven, "Selected SPI slave drives MISO");
    word.known = word.known && pin.level.has_value();
    word.value = (word.value << 1U) | (pin.level.value_or(false) ? 1U : 0U);
  }

  blofeld::DspShi &shi_;
  unsigned mode_;
  blofeld::SpiInputs pins_;
};

void test_reset_registers_and_modes() {
  blofeld::DspShi shi;
  require(shi.read(hckr) == 1 && shi.read(hcsr) == idle_empty && shi.read(hsar) == initial_slave,
          "Hardware reset independently sets CPHA, HIDLE, HTDE and I2C slave address");
  require(!shi.outputs().miso.driven && !shi.outputs().request.driven,
          "Reset disables output drivers");
  reject([&shi] { static_cast<void>(shi.read(hrx)); }, "Empty FIFO cannot fabricate zero data");
  reject([&shi] { static_cast<void>(shi.read(htx)); }, "Write-only HTX read is undefined");
  reject([&shi] { shi.write({.address = hrx, .value = 0}); }, "HRX is read-only");
  reject([&shi] { shi.write({.address = hckr, .value = 4}); }, "Illegal HRS/divider combination");
  reject([&shi] { shi.write({.address = hcsr, .value = 0x0d}); }, "Reserved word size fails");
  reject([&shi] { shi.write({.address = hcsr, .value = reserved_rx_enable}); },
         "HRIE=10 is reserved");
  reject([&shi] { shi.write({.address = hcsr, .value = 3}); }, "Unimplemented I2C fails");
  reject([&shi] { shi.write({.address = hcsr, .value = enabled_master}); },
         "Unimplemented master fails");
  require(shi.read(hcsr) == idle_empty, "Rejected configurations leave reset state unchanged");
  for (unsigned mode = 0; mode < 4; ++mode) {
    for (unsigned size = 0; size < 3; ++size) {
      blofeld::DspShi port;
      Peer peer(port, mode);
      const auto control = 0x21U | (size << 2U);
      port.write({.address = hcsr, .value = control});
      port.write({.address = htx, .value = outbound});
      const auto bits = (size + 1) * 8U;
      const auto padding = word_bits - bits;
      const auto observed = peer.word({.value = inbound, .bits = bits});
      require(observed && *observed == (outbound >> padding),
              "All four SPI clock modes transmit MSB-first in 8/16/24-bit formats");
      require(port.read(hrx) == ((inbound >> padding) << padding),
              "Receive data is independently left-aligned with zero low padding");
      require(port.read(hcsr) == (idle_empty | control),
              "Fresh full-duplex transfer leaves only the documented empty/idle state");
    }
  }
}

void test_transmit_pipeline_and_iosr_recycling() {
  for (unsigned mode = 0; mode < 2; ++mode) {
    blofeld::DspShi shi;
    Peer peer(shi, mode);
    shi.write({.address = hcsr, .value = fifo24});
    shi.write({.address = htx, .value = outbound});
    require((shi.read(hcsr) & tx_empty) != 0, "HTX becomes empty when transferred into IOSR");
    constexpr unsigned next = 0x90efab;
    shi.write({.address = htx, .value = next});
    require((shi.read(hcsr) & tx_empty) == 0, "One word may wait behind the prepared IOSR");
    reject([&shi] { shi.write({.address = htx, .value = 1}); }, "Pending HTX is not overwritten");
    require(peer.word({.value = inbound}) == outbound, "First frame uses the prepared IOSR word");
    require(peer.word({.value = outbound}) == next, "Second frame uses the pending HTX word");
    require(peer.word({.value = inbound}) == outbound,
            "Without HTX, the next SPI word shifts out the previous received IOSR contents");
    require((shi.read(hcsr) & underrun) != 0, "No fresh transmit data raises underrun");
  }
  blofeld::DspShi unknown;
  Peer peer(unknown);
  unknown.write({.address = hcsr, .value = spi24});
  require(!peer.word({.value = inbound}),
          "Initial MISO data remains unknown instead of fabricated zero");
  require(unknown.read(hrx) == inbound, "Unknown outgoing data does not invalidate known MOSI");
}

void test_fifo_capacity_order_overrun_and_clear_sequence() {
  for (const unsigned capacity : {1U, 10U}) {
    blofeld::DspShi shi;
    Peer peer(shi);
    const auto control = capacity == 1 ? spi24 : fifo24;
    shi.write({.address = hcsr, .value = control});
    for (unsigned index = 0; index < capacity; ++index) { peer.word({.value = inbound + index}); }
    require((shi.read(hcsr) & (rx_full | rx_nonempty)) == (rx_full | rx_nonempty),
            "HFIFO selects exactly one or ten FIFO words");
    peer.word({.value = outbound}); // Must be discarded, preserving all earlier entries.
    require(shi.read(hrx) == inbound, "Oldest data survives FIFO overrun");
    require((shi.read(hcsr) & overrun) != 0, "HRX without a prior HCSR read cannot clear HROE");
    if (capacity == 1) { peer.word({.value = inbound + 1}); }
    require(shi.read(hrx) == inbound + 1, "HCSR then HRX clears HROE without reordering data");
    require((shi.read(hcsr) & overrun) == 0,
            "Only the documented error-clear sequence retires HROE");
    for (unsigned index = 2; index < capacity; ++index) {
      require(shi.read(hrx) == inbound + index, "All accepted FIFO words preserve arrival order");
    }
    require((shi.read(hcsr) & (rx_nonempty | rx_full)) == 0, "Draining clears HRNE and HRFF");
    constexpr unsigned wrap_count = 23;
    for (unsigned index = 0; index < wrap_count; ++index) {
      peer.word({.value = inbound + index});
      require(shi.read(hrx) == inbound + index, "FIFO head wraps without stale data");
    }
  }
}

void test_underrun_clear_and_host_request() {
  blofeld::DspShi shi;
  Peer peer(shi);
  shi.write({.address = hcsr, .value = fifo24_request});
  require(shi.outputs().request.driven && shi.outputs().request.level == false,
          "Receive HREQ is active low when the FIFO has room");
  peer.select();
  require(shi.outputs().request.level == false, "Selection alone does not deassert receive HREQ");
  peer.bits({.value = inbound, .bits = 1});
  require(shi.outputs().request.level == true, "First serial clock deasserts HREQ");
  peer.deselect();
  shi.write({.address = htx, .value = outbound});
  require((shi.read(hcsr) & underrun) != 0, "HTX without status-read cannot clear underrun");
  shi.write({.address = htx, .value = inbound});
  require((shi.read(hcsr) & underrun) == 0, "HCSR then HTX clears HTUE");
  constexpr unsigned tx_request_control = 0x129;
  shi.write({.address = hcsr, .value = tx_request_control});
  require(shi.outputs().request.level == false, "Fresh IOSR data asserts transmit HREQ");
  peer.word({.value = 0});
  peer.word({.value = 0});
  require(shi.outputs().request.level == true, "Transmit HREQ stays inactive without fresh data");
  constexpr unsigned both_request_control = 0x1a9;
  shi.write({.address = hcsr, .value = both_request_control});
  shi.write({.address = htx, .value = outbound});
  require(shi.outputs().request.level == false,
          "Combined HREQ requires transmit and receive readiness");
  for (unsigned index = 2; index < 10; ++index) { peer.word({.value = 0}); }
  shi.write({.address = htx, .value = inbound});
  require(shi.outputs().request.level == true, "Full receive FIFO prevents combined HREQ");
}

void test_interrupt_encoding_errors_and_mask_latency() {
  blofeld::DspShi shi;
  Peer peer(shi);
  constexpr unsigned rx_tx_control = 0x1829;
  shi.write({.address = hcsr, .value = rx_tx_control});
  require(shi.interrupt_vectors() == std::vector<std::uint32_t>{tx_vector},
          "Enabled empty transmitter requests vector 0040");
  peer.word({.value = inbound});
  require(shi.interrupt_vectors() == std::vector<std::uint32_t>{tx_error_vector, rx_vector},
          "HRIE=01 selects FIFO-not-empty and underrun selects the distinct transmit error vector");
  static_cast<void>(shi.read(hcsr));
  shi.write({.address = htx, .value = outbound});
  require(shi.interrupt_vectors() == std::vector<std::uint32_t>{tx_vector, rx_vector},
          "Table 7-2 places transmit data above receive-not-empty in internal priority");
  constexpr unsigned full_tx_control = 0x3829;
  shi.write({.address = hcsr, .value = full_tx_control});
  require(shi.interrupt_vectors() == std::vector<std::uint32_t>{tx_vector},
          "HRIE=11 waits until the ten-word FIFO is full");
  for (unsigned index = 1; index < 10; ++index) { peer.word({.value = inbound + index}); }
  require(shi.interrupt_vectors() == std::vector<std::uint32_t>{tx_error_vector, full_vector},
          "Full receive interrupt uses vector 0048");
  peer.word({.value = outbound});
  require(shi.interrupt_vectors() == std::vector<std::uint32_t>{rx_error_vector, tx_error_vector},
          "Overrun replaces the regular receive exception with vector 004a");
  shi.write({.address = hcsr, .value = fifo24});
  require(shi.interrupt_vectors() == std::vector<std::uint32_t>{rx_error_vector, tx_error_vector},
          "Clearing interrupt enables preserves requests for one explicit instruction cycle");
  shi.advance_instruction_cycles(0);
  require(!shi.interrupt_vectors().empty(), "Zero elapsed cycles cannot retire an interrupt mask");
  shi.advance_instruction_cycles(1);
  require(shi.interrupt_vectors().empty(),
          "Elapsed instruction cycle retires disabled interrupt requests");
}

void test_abort_reset_and_invalid_pin_observations() {
  blofeld::DspShi shi;
  shi.write({.address = hcsr, .value = spi24});
  reject([&shi] { static_cast<void>(shi.read(hcsr)); }, "Live status cannot invent unknown SS");
  reject([&shi] { shi.observe_spi({.selected = true}); },
         "First baseline cannot invent select edge");
  shi.reset();
  Peer peer(shi);
  shi.write({.address = hcsr, .value = spi24});
  peer.select();
  peer.bits({.value = inbound, .bits = 5});
  peer.deselect();
  require((shi.read(hcsr) & rx_nonempty) == 0, "Deasserting SS discards an incomplete word");
  peer.word({.value = inbound});
  require(shi.read(hrx) == inbound, "A new complete frame does not inherit the aborted prefix");
  peer.select();
  peer.bits({.value = outbound, .bits = 3});
  shi.write({.address = hcsr, .value = spi24 - 1});
  shi.advance_instruction_cycles(0);
  require((shi.read(hcsr) & busy) != 0 && shi.outputs().miso.driven,
          "Individual reset has an explicit one-instruction-cycle delay");
  reject([&shi] { shi.write({.address = hckr, .value = 1}); }, "Clock cannot change before reset");
  shi.advance_instruction_cycles(1);
  require(shi.read(hcsr) == (idle_empty | (spi24 - 1)) && !shi.outputs().miso.driven,
          "Completed individual reset clears paths/errors while preserving control registers");
  reject([&shi] { shi.write({.address = hcsr, .value = spi24}); },
         "Enable inside selected frame fails");
  peer.deselect();
  shi.write({.address = hcsr, .value = spi24});
  reject([&shi] { shi.observe_spi({.selected = true, .clock = true}); },
         "Unordered select/clock transition is not guessed");
  peer.select();
  reject([&shi] { shi.observe_spi({.selected = true, .clock = true}); },
         "Unknown MOSI at a capture edge is an explicit failure");
  peer.bits({.value = inbound});
  peer.deselect();
  require(shi.read(hrx) == inbound, "Rejected unknown edge did not consume a bit");
}

void test_phase_one_continuous_words_and_phase_zero_framing() {
  blofeld::DspShi shi;
  Peer peer(shi, 1);
  shi.write({.address = hcsr, .value = fifo24});
  peer.select();
  peer.bits({.value = inbound});
  peer.bits({.value = outbound});
  peer.deselect();
  require(shi.read(hrx) == inbound && shi.read(hrx) == outbound,
          "CPHA=1 supports consecutive complete words while SS remains asserted");
  shi.reset();
  Peer phase_zero(shi);
  shi.write({.address = hcsr, .value = fifo24});
  phase_zero.select();
  phase_zero.bits({.value = inbound});
  reject([&phase_zero] { phase_zero.bits({.value = outbound, .bits = 1}); },
         "CPHA=0 requires a separate select frame");
  require(shi.read(hrx) == inbound, "Rejected extra clocks cannot enqueue a second word");
}

void test_request_boundary_and_register_rejections() {
  blofeld::DspShi shi;
  Peer peer(shi);
  constexpr unsigned non_word = 0x1000000;
  reject([&shi] { shi.write({.address = htx, .value = non_word}); },
         "A bus word wider than 24 bits cannot be silently truncated");
  constexpr unsigned unknown_register = 0xffff95;
  reject([&shi] { static_cast<void>(shi.read(unknown_register)); },
         "Unknown DSP address remains an explicit read failure");
  reject([&shi] { shi.write({.address = unknown_register, .value = 0}); },
         "Unknown DSP address remains an explicit write failure");
  shi.write({.address = hcsr, .value = fifo24_request});
  peer.select();
  constexpr unsigned tx_request_control = 0x129;
  reject([&shi] { shi.write({.address = hcsr, .value = tx_request_control}); },
         "HRQE cannot change while SS keeps HBUSY asserted");
  peer.bits({.value = inbound});
  const auto boundary = shi.outputs();
  require(boundary.request.driven && !boundary.request.level,
          "Uncalibrated CPHA=0 HREQ reassertion before SS release remains unknown");
  peer.deselect();
  require(shi.outputs().request.level == false,
          "After SS release, available FIFO space gives a known receive request");
}

void test_capture_edge_knowledge_in_all_clock_modes() {
  constexpr unsigned byte = 0xa6;
  constexpr unsigned left_aligned_byte = 0xa60000;
  for (unsigned mode = 0; mode < 4; ++mode) {
    blofeld::DspShi shi;
    const bool idle = (mode & 2U) != 0;
    const bool trailing_capture = (mode & 1U) != 0;
    shi.observe_spi({.clock = idle});
    shi.write({.address = hckr, .value = mode});
    shi.write({.address = hcsr, .value = 1});
    shi.observe_spi({.selected = true, .clock = idle});
    for (unsigned bit = 0; bit < 8; ++bit) {
      const bool data = ((byte >> (7 - bit)) & 1U) != 0;
      shi.observe_spi({.selected = true,
                       .clock = !idle,
                       .mosi = trailing_capture ? std::nullopt : std::optional<bool>(data)});
      shi.observe_spi({.selected = true,
                       .clock = idle,
                       .mosi = trailing_capture ? std::optional<bool>(data) : std::nullopt});
    }
    shi.observe_spi({.clock = idle});
    require(
        shi.read(hrx) == left_aligned_byte,
        "Only the independently specified capture edge needs MOSI; the other edge may be unknown");
  }
}
}

int main() {
  try {
    test_reset_registers_and_modes();
    test_transmit_pipeline_and_iosr_recycling();
    test_fifo_capacity_order_overrun_and_clear_sequence();
    test_underrun_clear_and_host_request();
    test_interrupt_encoding_errors_and_mask_latency();
    test_abort_reset_and_invalid_pin_observations();
    test_phase_one_continuous_words_and_phase_zero_framing();
    test_request_boundary_and_register_rejections();
    test_capture_edge_knowledge_in_all_clock_modes();
    std::cout
        << "DSP56371 SHI modes, FIFO, errors, handshakes, interrupts and reset checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
