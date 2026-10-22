// SPDX-License-Identifier: GPL-3.0-only
#include "original_usb_peer_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_usb.hpp"
#include "original_midi_forward_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace blofeld::test::usb_peer {
namespace mf = midi_forward;
using namespace usb_host;

namespace {
constexpr unsigned q_initial = 0x5a;
constexpr unsigned q_all = 0xff;
constexpr unsigned j_all = 7;
constexpr unsigned t_all = 0x0f;
constexpr unsigned ddrs = 0x272;
constexpr unsigned high_byte = 8;
constexpr unsigned unknown_byte = 256;
constexpr unsigned stack_bytes = 48;
constexpr unsigned maximum_instructions = 20000;
constexpr unsigned terminal_byte = 0xfb;
constexpr unsigned initial_crg_flags = 0xd0;
constexpr unsigned qram = 0x21;
constexpr unsigned high_word_mask = 0xff00;
constexpr unsigned usb_register_base = 0x300;
constexpr unsigned idle = 0x1000;
constexpr unsigned busy = 0x7000;
constexpr unsigned irq_masks = 0xd0;
constexpr unsigned preserved_flags = 0xc0;
constexpr CpuState initial{
    .pc = foreground, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = stack, .ccr = 0x40};

std::array<unsigned, 6> registers(const CpuState state) {
  return {state.pc, state.d, state.x, state.y, state.sp, state.ccr};
}

std::array<unsigned, 8> pads(const McuBus &bus) {
  const auto j = bus.observe_pads(McuPort::j, j_all);
  const auto q = bus.observe_pads(McuPort::q, q_all);
  const auto t = bus.observe_pads(McuPort::t, t_all);
  const auto s = bus.observe_pads(McuPort::s, q_all);
  return {j.known, j.levels, q.known, q.levels, t.known, t.levels, s.known, s.levels};
}

bool original(const unsigned pc) {
  constexpr std::array<std::pair<unsigned, unsigned>, 5> spans{{{forward_entry, 136},
                                                                {frame_entry, 64},
                                                                {dispatch_entry, 155},
                                                                {strobe, 12},
                                                                {out_handler, 33}}};
  return std::ranges::any_of(
      spans, [pc](const auto pair) { return pc >= pair.first && pc - pair.first < pair.second; });
}

bool forwarding(const Mode mode) {
  return mode == Mode::usb || mode == Mode::forward;
}
}

Rig::Rig(const Firmware &firmware, Case input)
    : machine_(McuFlashImage::from_known(firmware.mcu), BLOFELD_DEFAULT_SLA),
      input_(std::move(input)),
      initial_(initial) {
  prepare();
  for (unsigned index = 0; index < mf::ram_bytes; ++index) {
    before_.push_back(machine_.bus().read(static_cast<std::uint16_t>(mf::ram_begin + index)));
  }
  flash_ = machine_.bus().power_off_flash_image();
  evidence_ = machine_.bus().usb_host_evidence();
  configuration_ = machine_.bus().usb_host_configuration();
  observe();
}

void Rig::prepare() {
  auto &bus = machine_.bus();
  bus.write(hc12::initee, qram);
  bus.retire_instruction();
  usb_host::prepare(bus, mf::endpoint_bytes);
  for (unsigned index = 0; index < mf::ram_bytes; ++index) {
    bus.write(static_cast<std::uint16_t>(mf::ram_begin + index), mf::canary);
  }
  for (unsigned index = 0; index < input_.packet.size(); ++index) {
    bus.write(static_cast<std::uint16_t>(source + index), input_.packet[index]);
  }
  bus.write(activity, initial_activity);
  bus.write(bus_flags, initial_bus_flags);
  bus.write(transfer_flags, static_cast<std::uint8_t>(input_.phase));
  bus.write(hc12::ddrj, static_cast<std::uint8_t>(input_.missing_j ? 0 : j_all));
  bus.write(hc12::ddrt, t_all);
  bus.write(ddrs, q_all);
  bus.write(hc12::ptq, q_initial);
  bus.write(hc12::crg_flags, initial_crg_flags);
  bus.write(hc12::crg_interrupts, static_cast<std::uint8_t>(input_.rtie ? 0x80 : 0));
  initial_.ccr = static_cast<std::uint8_t>(input_.ccr);
  if (input_.mode != Mode::usb) {
    if (input_.mode == Mode::forward) {
      initial_.pc = forward_entry;
    } else if (input_.mode == Mode::frame) {
      initial_.pc = frame_entry;
    } else {
      initial_.pc = dispatch_entry;
    }
    initial_.d = static_cast<std::uint16_t>((initial_.d & high_word_mask) |
                                            (input_.mode == Mode::dispatch ? 8 : input_.count));
    bus.write_memory({.address = stack, .size = 2, .value = foreground});
    bus.write_memory({.address = stack + 2, .size = 2, .value = source});
  }
  machine_.cpu().set_state(initial_);
  if (input_.mode == Mode::usb) { receive(false); }
}

void Rig::observe() {
  machine_.bus().set_observer([this](const BusEvent &event) {
    const bool gpio = event.address == hc12::ptj || event.address == hc12::ptq ||
                      event.address == hc12::ddrq || event.address == hc12::pts ||
                      event.address == hc12::ptt;
    const bool usb =
        input_.mode == Mode::usb &&
        (event.address == buffer_select || event.address == output_control ||
         event.address == output_control + 1 ||
         (event.address >= out_window && event.address < out_window + mf::endpoint_bytes));
    require(event.write && (gpio || usb),
            "Original peer paths write only their declared GPIO/USB registers");
    const auto observed = pads(machine_.bus());
    const auto direction = machine_.bus().read(hc12::ddrq);
    const Write write{.address = event.address,
                      .value = event.value,
                      .pc = active_pc_,
                      .pads = observed,
                      .q_direction = direction};
    stats_.writes.push_back(write);
    check_write(write);
  });
}

void Rig::check_write(const Write &write) {
  const auto &observed = write.pads;
  if (write.address == hc12::ddrq) {
    const auto expected_known = write.value == 0 ? 0U : q_all;
    require(observed[2] == expected_known, "Actual DDRQ writes drive or release pad knowledge");
    if (write.value != 0) {
      require(observed[3] == expected_preload(),
              "Original preload ordering determines the first driven Q value");
    }
  }
  if (write.address == hc12::ptj && write.pc == strobe + 3) {
    require(!input_.expected.empty() &&
                stats_.strobes < input_.expected.size() * (input_.repeat ? 2U : 1U),
            "Only independently declared peer bytes may be strobed");
    require(write.value == 1 && observed[0] == j_all && observed[1] == 1 && observed[2] == q_all &&
                write.q_direction == q_all &&
                observed[3] == input_.expected[stats_.strobes % input_.expected.size()],
            "Each real settling/strobe call drives the independent expected peer data byte");
    check_consumed_header();
    ++stats_.strobes;
  }
}

unsigned Rig::expected_preload() const {
  if (!forwarding(input_.mode)) { return input_.expected.front(); }
  if (!input_.repeat || stats_.irqs != 2) { return q_initial; }
  if (!input_.rtie && ((input_.phase ^ 2U) & 2U) != 0) { return terminal_byte; }
  return input_.expected.back();
}

void Rig::check_consumed_header() {
  if (!forwarding(input_.mode)) { return; }
  const auto position = stats_.strobes % input_.expected.size();
  unsigned end = 0;
  for (unsigned index = 0; index < input_.lengths.size(); ++index) {
    end += input_.lengths[index];
    if (position < end) {
      const auto base = input_.mode == Mode::usb ? out_window : source;
      // RAM/local-buffer reads are side-effect free; cloning inside an observer is prohibited.
      require(machine_.bus().read(static_cast<std::uint16_t>(base + input_.headers[index])) == 0,
              "The original event header is already cleared before its first actual peer strobe");
      return;
    }
  }
  throw Fault("Peer strobe has no independently expected source event");
}

void Rig::receive(const bool data1) {
  require(machine_.bus().receive_usb_out(
              {.buffer = UsbBuffer::endpoint3, .data = input_.packet, .data1 = data1}) ==
              UsbResponse::ack,
          "Only an actual decoded own OUT packet establishes received bytes and completion");
  const auto pending = machine_.cpu().pending_interrupt();
  require(pending && pending->vector == out_vector,
          "The real physical EP3 completion selects FFC8");
}

void Rig::check_frame() const {
  auto copy = machine_.bus().clone();
  const std::array<unsigned, 9> frame{input_.ccr,
                                      initial_.d & byte_mask,
                                      static_cast<unsigned>(initial_.d) >> high_byte,
                                      static_cast<unsigned>(initial_.x) >> high_byte,
                                      initial_.x & byte_mask,
                                      static_cast<unsigned>(initial_.y) >> high_byte,
                                      initial_.y & byte_mask,
                                      foreground >> high_byte,
                                      0};
  for (unsigned index = 0; index < frame.size(); ++index) {
    require(copy->read(static_cast<std::uint16_t>(stack - frame.size() + index)) == frame[index],
            "Every literal NXP hardware frame byte retains the interrupted foreground context");
  }
}

void Rig::step() {
  const auto before = machine_.cpu().state();
  active_pc_ = before.pc;
  require(stats_.instructions < maximum_instructions && before.sp >= stack - stack_bytes &&
              before.sp <= stack + 2,
          "Original peer/IRQ paths retain declared instruction and stack bounds");
  const auto result = machine_.cpu().step();
  if (result.instruction_executed) {
    require(original(before.pc), "Only pinned original routines execute");
    ++stats_.instructions;
  }
  if (result.interrupt_vector) {
    require(input_.mode == Mode::usb && *result.interrupt_vector == out_vector &&
                machine_.cpu().state().pc == out_handler,
            "Only the genuine original OUT ISR receives the real native frame");
    ++stats_.irqs;
    check_frame();
  }
}

void Rig::until(const unsigned pc) {
  while (machine_.cpu().state().pc != pc || machine_.cpu().pending_interrupt()) { step(); }
}

void Rig::capture_boundary() {
  if (input_.truncated) {
    while (machine_.cpu().state().pc != payload_load ||
           machine_.cpu().state().x != out_window + input_.packet.size()) {
      step();
    }
  } else if (!input_.expected.empty()) {
    until(strobe);
    if (input_.missing_j) {
      for (unsigned index = 0; index < 3; ++index) { step(); }
    }
  }
}

void Rig::duplicate(const bool data1) {
  const auto before = outcome();
  require(machine_.bus().receive_usb_out(
              {.buffer = UsbBuffer::endpoint3, .data = input_.packet, .data1 = data1}) ==
              UsbResponse::ack,
          "A duplicate old DATA PID is acknowledged without redelivery");
  require(outcome() == before,
          "Duplicate OUT cannot replace already cleared headers, flags or native progress");
  if ((word(machine_.bus(), output_control) & 0x4000U) != 0) {
    require(machine_.bus().receive_usb_out(
                {.buffer = UsbBuffer::endpoint3, .data = input_.packet, .data1 = !data1}) ==
                    UsbResponse::nak &&
                outcome() == before,
            "A new DATA PID cannot overwrite the occupied original receive buffer");
  }
}

void Rig::check_failure() {
  const auto before = registers(machine_.cpu().state());
  auto expected = before;
  if (input_.truncated) { ++expected[2]; } // Native indexed-load address update precedes the read.
  const auto address = input_.missing_j ? hc12::ptj : out_window + input_.packet.size();
  const auto pc = input_.missing_j ? strobe + 3 : payload_load;
  try {
    step();
  } catch (const Fault &fault) {
    const std::string_view text(fault.what());
    require(text.contains("MCU read at " + hex(address)) && text.contains("pc=" + hex(pc)) &&
                text.contains("ccr=") &&
                text.contains(input_.missing_j ? "Unknown MCU GPIO input"
                                               : "Unknown USB endpoint-buffer byte") &&
                registers(machine_.cpu().state()) == expected,
            std::string("Unexpected failure prefix: ") + fault.what() +
                " before-X=" + hex(before[2]) + " after-X=" + hex(machine_.cpu().state().x));
    const auto partial = outcome();
    bool stopped = false;
    try {
      step();
    } catch (const Fault &retry) {
      stopped = std::string_view(retry.what()).contains("coordinated recovery");
    }
    require(
        stopped && outcome() == partial,
        "A failed original indexed/GPIO access cannot retry past its partial instruction prefix");
    if (input_.mode == Mode::usb) {
      require(word(machine_.bus(), output_control) == (busy | input_.packet.size()),
              "A failed original OUT handler cannot acknowledge completion or release its packet");
      duplicate(false);
      check_frame();
    }
    require(machine_.bus().read(activity) == (initial_activity | (input_.multi ? 4U : 0U)) &&
                machine_.bus().read(bus_flags) ==
                    (initial_bus_flags | (input_.activity ? 0x80U : 0U)) &&
                machine_.bus().read(transfer_flags) ==
                    (input_.phase | (input_.multi ? 0x10U : 0U)) &&
                stats_.strobes == (input_.missing_j ? 0U : input_.expected.size()),
            "Expected failure retains the actual partial activity/strobe prefix without a terminal "
            "action");
    guard();
    return;
  }
  throw Fault("An unqualified original peer access reported success");
}

void Rig::complete() {
  if (input_.missing_j || input_.truncated) {
    check_failure();
    return;
  }
  if (input_.mode == Mode::usb) { duplicate(false); }
  until(foreground);
  if (input_.mode == Mode::usb) {
    duplicate(false);
    check_frame();
    if (input_.repeat) {
      receive(true);
      until(foreground);
      duplicate(true);
      check_frame();
    }
  }
  check_result();
  guard();
}

std::vector<std::pair<unsigned, unsigned>> Rig::expected_writes() const {
  std::vector<std::pair<unsigned, unsigned>> result;
  auto phase = input_.phase;
  const auto exchanges = input_.repeat ? 2U : 1U;
  for (unsigned exchange = 0; exchange < exchanges; ++exchange) {
    const bool setup = !input_.packet.empty() && input_.packet[0] != 0;
    if (!forwarding(input_.mode)) {
      result = {{hc12::ptj, 1},
                {hc12::pts, 0},
                {hc12::ptt, 1},
                {hc12::ptq, input_.expected.front()},
                {hc12::ddrq, q_all}};
    } else if (setup) {
      result.insert(result.end(),
                    {{hc12::ptj, 1}, {hc12::ddrq, q_all}, {hc12::pts, 0}, {hc12::ptt, 1}});
    }
    for (const auto byte : input_.expected) {
      result.insert(result.end(), {{hc12::ptq, byte}, {hc12::ptj, 0}, {hc12::ptj, 1}});
    }
    if (!forwarding(input_.mode)) {
      result.emplace_back(hc12::ddrq, 0);
    } else if (setup && !input_.rtie) {
      phase ^= 2U;
      if ((phase & 2U) != 0) {
        result.insert(result.end(), {{hc12::ptt, 3}, {hc12::ptq, terminal_byte}});
      }
    }
  }
  return result;
}

void Rig::check_result() const {
  const auto state = machine_.cpu().state();
  auto copy = machine_.bus().clone();
  if (input_.mode == Mode::usb) {
    require(registers(state) == registers(initial_),
            "Real original OUT/forwarder/strobe/RTI restore the complete foreground");
  } else {
    const auto masks =
        forwarding(input_.mode) ? input_.ccr & irq_masks : input_.ccr & preserved_flags;
    require(state.pc == foreground && state.sp == stack + 2 && (state.ccr & irq_masks) == masks,
            "Real near return preserves original flag policy, including unconditional CLI in the "
            "frame writer");
    const auto advance = input_.mode == Mode::forward ? 4 * input_.headers.size() : 0;
    require(word(*copy, stack + 2) == source + advance,
            "Only the original forwarder publishes its actual advanced caller pointer");
  }
  const bool setup = !input_.packet.empty() && input_.packet[0] != 0;
  auto phase = input_.phase | (input_.multi ? 0x10U : 0U);
  if (forwarding(input_.mode) && setup && !input_.rtie && !input_.repeat) { phase ^= 2U; }
  require(copy->read(activity) == (initial_activity | (input_.multi ? 4U : 0U)) &&
              copy->read(bus_flags) == (initial_bus_flags | (input_.activity ? 0x80U : 0U)) &&
              copy->read(transfer_flags) == phase,
          "Independent activity, multibyte and raw post-loop phase flags match");
  check_pads(phase);
  std::vector<std::pair<unsigned, unsigned>> writes;
  for (const auto &event : stats_.writes) {
    if (event.address < usb_register_base) { writes.emplace_back(event.address, event.value); }
  }
  require(writes == expected_writes() &&
              stats_.strobes == input_.expected.size() * (input_.repeat ? 2U : 1U),
          "The full GPIO write order distinguishes preload, actual settling strobes, release and "
          "unstrobbed terminal latches");
  if (input_.mode == Mode::usb) {
    require(stats_.irqs == (input_.repeat ? 2U : 1U) &&
                word(*copy, output_control) == (idle | input_.packet.size()) &&
                machine_.bus().usb_host_endpoint(output_address).data1 == !input_.repeat,
            "Only the real final whole-word OUT acknowledgment releases completion and preserves "
            "size/PID");
  }
}

void Rig::check_pads(const unsigned phase) const {
  const auto observed = pads(machine_.bus());
  const bool setup = !input_.packet.empty() && input_.packet[0] != 0;
  const bool output = forwarding(input_.mode) && setup;
  auto copy = machine_.bus().clone();
  require(copy->read(hc12::ddrq) == (output ? q_all : 0) && observed[2] == (output ? q_all : 0) &&
              observed[0] == j_all && observed[1] == (setup ? 1U : 0U) && observed[4] == t_all &&
              observed[6] == q_all && observed[7] == 0,
          "Original direction release/retention and final J/S/T/Q pad knowledge remain explicit");
  unsigned expected_t = setup ? 1 : 0;
  if (output) {
    auto expected_q = input_.expected.empty() ? q_initial : input_.expected.back();
    if (!input_.rtie && (phase & 2U) != 0) {
      expected_t = 3;
      expected_q = terminal_byte;
    }
    require(observed[3] == expected_q,
            "Final original Q retains data or its unstrobbed terminal latch");
  }
  require(observed[5] == expected_t, "The original whole T latch retains its actual terminal mode");
}

void Rig::guard() const {
  const auto flash = machine_.bus().power_off_flash_image();
  require(
      flash.bytes == flash_.bytes && flash.known == flash_.known && flash.fresh == flash_.fresh &&
          machine_.bus().usb_host_evidence() == evidence_ &&
          machine_.bus().usb_host_configuration() == configuration_,
      "All NVM bytes/knowledge/erase history and USB link/configuration identity remain unchanged");
  auto copy = machine_.bus().clone();
  for (unsigned index = 0; index < mf::ram_bytes; ++index) {
    const auto address = mf::ram_begin + index;
    unsigned expected = before_[index];
    if (input_.mode != Mode::usb && address >= source && address < source + input_.packet.size()) {
      const auto offset = address - source;
      if ((input_.clear_tail && offset == 2) ||
          std::ranges::find(input_.headers, offset) != input_.headers.end()) {
        expected = 0;
      }
    }
    const bool caller_pointer =
        input_.mode == Mode::forward && address >= stack + 2 && address < stack + 4;
    const bool stack_region = address >= stack - stack_bytes && address < stack + 2;
    const bool permitted = caller_pointer || address == activity || address == bus_flags ||
                           address == transfer_flags || stack_region;
    if (!permitted) {
      require(
          copy->read(static_cast<std::uint16_t>(address)) == expected,
          "Caller/source bytes and all unrelated mapped RAM retain independent expected values");
    }
  }
  guard_buffers();
}

void Rig::guard_buffers() const {
  const auto actual = outcome().buffers;
  for (unsigned buffer = 0; buffer < 4; ++buffer) {
    for (unsigned byte = 0; byte < mf::endpoint_bytes; ++byte) {
      unsigned expected = unknown_byte;
      if (input_.mode == Mode::usb && buffer == 3 && byte < input_.packet.size()) {
        expected = std::ranges::find(input_.headers, byte) != input_.headers.end()
                       ? 0
                       : input_.packet[byte];
      }
      require(actual[(buffer * mf::endpoint_bytes) + byte] == expected,
              "Every local USB byte retains independently expected payload/unknown knowledge");
    }
  }
}

Saved Rig::save() const {
  return {.board = machine_.checkpoint(), .stats = stats_};
}

void Rig::restore(const Saved &saved) {
  machine_.restore(saved.board);
  stats_ = saved.stats;
  observe();
}

Outcome Rig::outcome() const {
  auto copy = machine_.bus().clone();
  Outcome result{.cpu = registers(machine_.cpu().state()),
                 .ram = {},
                 .buffers = {},
                 .pads = pads(machine_.bus()),
                 .control = word(*copy, output_control),
                 .q_direction = copy->read(hc12::ddrq),
                 .data1 = machine_.bus().usb_host_endpoint(output_address).data1,
                 .stats = stats_};
  for (unsigned index = 0; index < mf::ram_bytes; ++index) {
    result.ram.push_back(copy->read(static_cast<std::uint16_t>(mf::ram_begin + index)));
  }
  for (unsigned buffer = 0; buffer < 4; ++buffer) {
    copy->write(buffer_select, static_cast<std::uint8_t>(buffer));
    for (unsigned byte = 0; byte < mf::endpoint_bytes; ++byte) {
      unsigned value = unknown_byte;
      try {
        value = copy->read(static_cast<std::uint16_t>(local + byte));
      } catch (const Fault &fault) {
        require(std::string_view(fault.what()).contains("Unknown USB endpoint-buffer byte"),
                "Only genuinely unknown USB payload bytes use the inspection sentinel");
      }
      result.buffers.push_back(value);
    }
  }
  return result;
}
}
