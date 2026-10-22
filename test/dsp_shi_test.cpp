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
    constexpr unsigned replacement = 0x654321;
    shi.write({.address = htx, .value = replacement});
    require((shi.read(hcsr) & tx_empty) == 0, "Replacing HTX keeps its pending state");
    require(peer.word({.value = inbound}) == outbound, "First frame uses the prepared IOSR word");
    require(peer.word({.value = outbound}) == replacement,
            "Second frame uses the last HTX write, losing the replaced data");
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

void test_pending_transmit_overwrites() {
  // Independent HTX/IOSR register semantics: DSP56371UM 7.4.1/2 and 7.4.5.1.
  constexpr unsigned replacement = 0x654321;
  constexpr unsigned pending = 0x90efab;
  constexpr unsigned word_mask = 0xffffff;
  for (unsigned mode = 0; mode < 4; ++mode) {
    for (unsigned size = 0; size < 3; ++size) {
      for (unsigned prefix_bits = 0; prefix_bits < 2; ++prefix_bits) {
        blofeld::DspShi shi;
        Peer peer(shi, mode);
        const auto control = 0x21U | (size << 2U);
        const auto width = (size + 1) * 8U;
        const auto padding = word_bits - width;
        shi.write({.address = hcsr, .value = control});
        shi.write({.address = htx, .value = outbound});
        shi.write({.address = htx, .value = pending});
        peer.select();
        const auto prefix = peer.bits({.value = inbound, .bits = prefix_bits});
        shi.write({.address = htx, .value = replacement});
        require((shi.read(hcsr) & tx_empty) == 0, "Overwrite cannot fabricate HTDE");
        const auto suffix =
            peer.bits({.value = (inbound << prefix_bits) & word_mask, .bits = width - prefix_bits});
        peer.deselect();
        if (!prefix || !suffix) { throw blofeld::Fault("Known in-flight IOSR became unknown"); }
        const auto first = (*prefix << (width - prefix_bits)) | *suffix;
        require(first == (outbound >> padding), "Pending overwrite preserves every IOSR bit");
        require(shi.read(hrx) == ((inbound >> padding) << padding),
                "An HTX overwrite leaves receive alignment/data intact");
        require(shi.read(hcsr) == (idle_empty | control),
                "IOSR transfer releases HTX without adding flags or a third buffer");
        shi.write({.address = htx, .value = pending});
        shi.write({.address = htx, .value = 0});
        require((shi.read(hcsr) & tx_empty) == 0, "Pending zero is data, not an empty register");
        require(peer.word({.value = outbound, .bits = width}) == (replacement >> padding),
                "Next frame contains the replacement, not the lost pending word");
        require(shi.read(hrx) == ((outbound >> padding) << padding),
                "Second receive survives the next pending overwrite");
        require(peer.word({.value = inbound, .bits = width}) == 0,
                "A zero replacement transfers as a complete known word");
        require(shi.read(hrx) == ((inbound >> padding) << padding) &&
                    shi.read(hcsr) == (idle_empty | control),
                "A fresh zero word must not raise transmit underrun");
        require(peer.word({.value = outbound, .bits = width}) == (inbound >> padding),
                "After the two real buffers drain, IOSR reuses its last received data");
        require(shi.read(hrx) == ((outbound >> padding) << padding) &&
                    shi.read(hcsr) == (idle_empty | control | underrun),
                "Only the later unprimed word raises underrun");
      }
    }
  }
}

void test_overwrite_underrun_acknowledgement() {
  blofeld::DspShi shi;
  Peer peer(shi, 1);
  shi.write({.address = hcsr, .value = fifo24});
  static_cast<void>(peer.word({.value = inbound}));
  require(shi.read(hrx) == inbound, "Known receive without a transmit preload");
  constexpr unsigned pending = 0x90efab;
  constexpr unsigned replacement = 0x654321;
  shi.write({.address = htx, .value = outbound});
  shi.write({.address = htx, .value = pending});
  shi.write({.address = htx, .value = replacement});
  require((shi.read(hcsr) & underrun) != 0,
          "Unarmed pending overwrites cannot acknowledge transmit underrun");
  shi.write({.address = htx, .value = inbound});
  require(shi.read(hcsr) == (0x200U | fifo24),
          "HCSR-read then pending HTX-write clears only the armed underrun");
  require(peer.word({.value = inbound}) == outbound,
          "Underrun acknowledgement retains the prepared IOSR");
  require(peer.word({.value = outbound}) == inbound,
          "The acknowledging write also replaces the pending data");
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

void test_inactive_shift_bits() {
  for (unsigned mode = 0; mode < 4; ++mode) {
    for (unsigned size = 0; size < 2; ++size) {
      for (const unsigned padding_bit : {0U, 1U}) {
        blofeld::DspShi shi;
        Peer peer(shi, mode);
        const auto width = (size + 1) * 8U;
        const auto padding = word_bits - width;
        const auto inactive = padding_bit << (padding - 1);
        shi.write({.address = hcsr, .value = 1U | (size << 2U)});
        shi.write({.address = htx, .value = inactive});
        require(peer.word({.value = 0, .bits = width}) == 0,
                "Inactive HTX bits never enter the transmitted upper byte/word");
        require(shi.read(hrx) == 0,
                "Inactive HTX bits never contaminate a completely zero receive word");
        require(peer.word({.value = 0, .bits = width}) == 0 && shi.read(hrx) == 0,
                "The next unprimed IOSR echo also retains only the active word width");
      }
    }
  }
}

struct ResetFormat {
  unsigned mode;
  unsigned size;
  bool fifo;
};

void verify_reconfigured_receive(blofeld::DspShi &shi, Peer &peer, const ResetFormat format) {
  const auto size = format.size;
  const auto capacity = format.fifo ? 10U : 1U;
  constexpr unsigned fresh_outbound = 0x5ea3c8;
  constexpr unsigned fresh_inbound = 0x765432;
  const auto width = (size + 1) * 8U;
  const auto padding = word_bits - width;
  shi.write({.address = htx, .value = fresh_outbound});
  require(peer.word({.value = fresh_inbound, .bits = width}) == (fresh_outbound >> padding),
          "Re-enable uses fresh HTX, not either discarded transmit buffer");
  const auto received = shi.read(hrx);
  require(received == ((fresh_inbound >> padding) << padding),
          "Re-enabled receive mode=" + std::to_string(format.mode) +
              " size=" + std::to_string(size) + " fifo=" + (format.fifo ? "ten" : "one") +
              " actual=" + blofeld::hex(received, 6) +
              " expected=" + blofeld::hex((fresh_inbound >> padding) << padding, 6));
  reject([&shi] { static_cast<void>(shi.read(hrx)); }, "Only the new word survived reset");
  for (unsigned index = 0; index <= capacity; ++index) {
    peer.word({.value = (index + 1) << (word_bits - 8), .bits = width});
  }
  require((shi.read(hcsr) & (rx_full | overrun)) == (rx_full | overrun),
          "The new receive capacity overflows at its independent first excess word");
  for (unsigned index = 0; index < capacity; ++index) {
    require(shi.read(hrx) == (index + 1) << (word_bits - 8),
            "The new FIFO preserves its exact independent first words");
  }
  reject([&shi] { static_cast<void>(shi.read(hrx)); }, "The new excess word was discarded");
}

void verify_reset_transition(const ResetFormat format) {
  const auto mode = format.mode;
  const auto size = format.size;
  const bool fifo = format.fifo;
  blofeld::DspShi shi;
  Peer peer(shi, mode);
  constexpr unsigned interrupts = 0x1800;
  constexpr unsigned receive_request = 0x80;
  constexpr unsigned fifo_bit = 0x20;
  constexpr unsigned programmed_slave = 0xa40000;
  const auto control = 1U | (size << 2U) | (fifo ? fifo_bit : 0U) | interrupts;
  const auto next_size = (size + 1) % 3;
  const auto reset_control =
      (next_size << 2U) | (!fifo ? fifo_bit : 0U) | receive_request | interrupts;
  shi.write({.address = hsar, .value = programmed_slave});
  shi.write({.address = hcsr, .value = control});
  const auto old_capacity = fifo ? 10U : 1U;
  for (unsigned index = 0; index <= old_capacity; ++index) {
    peer.word({.value = inbound, .bits = (size + 1) * 8U});
  }
  shi.write({.address = htx, .value = outbound});
  shi.write({.address = htx, .value = inbound});
  require(shi.read(hcsr) == (0x200U | control | rx_full | rx_nonempty | underrun | overrun),
          "Reset starts with full RX, both errors armed and both real transmit buffers occupied");
  shi.write({.address = hcsr, .value = reset_control});
  const auto checkpoint = shi;
  for (unsigned replay = 0; replay < 2; ++replay) {
    shi = checkpoint;
    shi.advance_instruction_cycles(0);
    reject([&shi] { static_cast<void>(shi.read(hcsr)); },
           "Changed FIFO/word-size flags during the reset delay are not fabricated");
    reject([&shi] { static_cast<void>(shi.read(hrx)); },
           "Unqualified receive geometry cannot consume a pending word");
    reject([&shi] { shi.write({.address = htx, .value = 0}); },
           "Unqualified reset-transition HTX writes cannot fabricate successful transmission");
    reject([&shi] { static_cast<void>(shi.interrupt_vectors()); },
           "Unqualified transitional flags cannot fabricate an interrupt request");
    reject([&shi, reset_control] { shi.write({.address = hcsr, .value = reset_control | 1U}); },
           "Re-enable requires the actual reset cycle");
    reject([&shi, mode] { shi.write({.address = hckr, .value = mode ^ 1U}); },
           "A changed HCKR still requires completed reset");
    const auto clock = (mode & 2U) != 0;
    shi.observe_spi({.clock = clock});
    reject([&shi, clock] { shi.observe_spi({.selected = true, .clock = clock, .mosi = false}); },
           "A selected transfer during reconfiguration remains an explicit failure");
    const auto pins = shi.outputs();
    require(!pins.miso.driven && pins.request.driven && !pins.request.level,
            "Idle MISO is tri-stated; transitional HREQ ownership cannot imply known readiness");
    require(shi.read(hckr) == mode && shi.read(hsar) == programmed_slave,
            "The independent clock and slave-address controls remain readable and retained");
    shi.advance_instruction_cycles(replay == 0 ? 1 : 7);
    require(shi.read(hcsr) == (idle_empty | reset_control) && shi.read(hckr) == mode &&
                shi.read(hsar) == programmed_slave && shi.interrupt_vectors().empty() &&
                !shi.outputs().miso.driven && !shi.outputs().request.driven,
            "One actual instruction cycle clears paths/errors, disables pins and retains controls");
    reject([&shi] { static_cast<void>(shi.read(hrx)); }, "Reset leaves an actually empty FIFO");
    shi.write({.address = hcsr, .value = reset_control | 1U});
    verify_reconfigured_receive(shi, peer, {.mode = mode, .size = next_size, .fifo = !fifo});
  }
}

void test_idle_reset_reconfiguration() {
  for (unsigned mode = 0; mode < 4; ++mode) {
    for (unsigned size = 0; size < 3; ++size) {
      for (unsigned fifo = 0; fifo < 2; ++fifo) {
        verify_reset_transition({.mode = mode, .size = size, .fifo = fifo != 0});
      }
    }
  }
  blofeld::DspShi shi;
  const Peer peer(shi, 1);
  shi.write({.address = hcsr, .value = fifo24});
  shi.write({.address = hcsr, .value = 0});
  shi.reset();
  shi.advance_instruction_cycles(0);
  require(shi.read(hcsr) == idle_empty && shi.read(hckr) == 1,
          "Hardware reset cancels a pending reconfiguration and restores hardware controls");
}

void test_reset_reconfiguration_rejections() {
  for (unsigned mode = 0; mode < 4; ++mode) {
    blofeld::DspShi shi;
    Peer peer(shi, mode);
    shi.write({.address = hcsr, .value = fifo24});
    peer.select();
    peer.bits({.value = inbound, .bits = 3});
    const auto before = shi.read(hcsr);
    reject([&shi] { shi.write({.address = hcsr, .value = 0}); },
           "HM changes require known HBUSY=0 even when HEN is cleared");
    require(shi.read(hcsr) == before, "Rejected busy reset reconfiguration leaves flags intact");
    constexpr unsigned word_mask = 0xffffff;
    peer.bits({.value = (inbound << 3U) & word_mask, .bits = word_bits - 3});
    peer.deselect();
    require(shi.read(hrx) == inbound, "Rejected reconfiguration does not change a live word");
    reject([&shi] { shi.write({.address = hcsr, .value = 1}); },
           "Configuration changes with HEN still set remain unqualified");
  }
  blofeld::DspShi unknown;
  unknown.write({.address = hcsr, .value = fifo24});
  reject([&unknown] { unknown.write({.address = hcsr, .value = 0}); },
         "A missing pin baseline cannot establish idle reset reconfiguration");
  unknown.observe_spi({.mosi = false});
  require(unknown.read(hcsr) == (idle_empty | fifo24),
          "Rejected unknown-idle reconfiguration preserves the actual control word");
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
    test_pending_transmit_overwrites();
    test_overwrite_underrun_acknowledgement();
    test_fifo_capacity_order_overrun_and_clear_sequence();
    test_underrun_clear_and_host_request();
    test_interrupt_encoding_errors_and_mask_latency();
    test_abort_reset_and_invalid_pin_observations();
    test_inactive_shift_bits();
    test_idle_reset_reconfiguration();
    test_reset_reconfiguration_rejections();
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
