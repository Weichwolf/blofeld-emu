// SPDX-License-Identifier: GPL-3.0-only
#include "application_fixture.hpp"
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_usb.hpp"
#include "storage/flash_image.hpp"
#include "usb/usb_device.hpp"
#include "usb/usb_host.hpp"
#include "usb/usb_transfer.hpp"
#include "usb/usbip.hpp"
#include "usb/usbip_runtime.hpp"
#include "usbip_fixture.hpp"
#include "usbip_tcp_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <sys/poll.h>
#include <utility>
#include <variant>
#include <vector>

namespace {
constexpr unsigned reset_vector = 0xffd4;
constexpr unsigned configuration_vector = 0xffd0;
constexpr unsigned control_in_vector = 0xffce;
constexpr unsigned control_in_return = 0xbe5e;
constexpr unsigned reset_poll = 0xbdf9;
constexpr unsigned reset_delay = 0xbdf1;
constexpr unsigned reset_return = 0xbe10;
constexpr unsigned suspend_return = 0xbddc;
constexpr unsigned configuration_return = 0xbe4f;
constexpr unsigned split_controls = 0xbf0c;
constexpr unsigned main_wait = 0x8314;
constexpr unsigned main_resume = 0x8315;
constexpr unsigned main_stack = 0x25f3;
constexpr unsigned wait_stack = 0x25ea;
constexpr unsigned ram_begin = 0x1200;
constexpr unsigned ram_bytes = 5120;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned instruction_limit = 100000;
constexpr unsigned out_control = 0x31c;
constexpr unsigned in_control = 0x320;
constexpr unsigned retained_control = 0x1800;
constexpr unsigned select = 0x30e;
constexpr unsigned config_data = 0x310;
constexpr unsigned configuration_status = 0x30c;
constexpr unsigned in_description = 0x820008d1;
constexpr unsigned out_description = 0x820008c1;
using Registers = std::array<unsigned, 6>;
using Writes = std::vector<std::pair<unsigned, unsigned>>;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw blofeld::Fault(std::string(message)); }
}

Registers registers(const blofeld::CpuState s) {
  return {s.pc, s.d, s.x, s.y, s.sp, s.ccr};
}

struct Profile {
  unsigned id = 0;
  unsigned release_poll = 0; // Zero retains reset through the actual counter timeout.
};

struct Counters {
  unsigned polls = 0;
  unsigned delays = 0;
  unsigned entries = 0;
  unsigned returns = 0;
  unsigned native = 0;
  unsigned control_entries = 0;
  unsigned control_returns = 0;
  unsigned out_entries = 0;
  unsigned out_returns = 0;
  Writes writes;
  bool operator==(const Counters &) const = default;
};

struct Saved {
  blofeld::McuCheckpoint machine;
  blofeld::test::LcdFixture display;
  Profile profile;
  Counters counters;
  Registers interrupted;
};

struct Outcome {
  Registers cpu;
  std::vector<unsigned> ram;
  std::vector<unsigned> usb;
  Counters counters;
  bool operator==(const Outcome &) const = default;
};

struct Session {
  Session(const blofeld::Firmware &firmware, std::ostream &trace)
      : machine(blofeld::McuFlashImage::from_known(firmware.mcu), BLOFELD_DEFAULT_SLA),
        display(std::make_unique<blofeld::test::LcdFixture>(machine.bus())),
        trace_(trace),
        initial_(machine.bus().power_off_flash_image()) {
    static_cast<void>(blofeld::test::start_application(machine, display, trace, false));
    bind();
  }

  [[nodiscard]] Saved save() const {
    return {.machine = machine.checkpoint(),
            .display = *display,
            .profile = profile,
            .counters = counters,
            .interrupted = interrupted};
  }

  void restore(const Saved &saved) {
    machine.restore(saved.machine);
    display = std::make_unique<blofeld::test::LcdFixture>(machine.bus(), saved.display);
    profile = saved.profile;
    counters = saved.counters;
    interrupted = saved.interrupted;
    bind();
  }

  void bind() {
    machine.bus().set_observer([this](const blofeld::BusEvent &event) {
      require(event.write, "The USB scenario observer accepts actual MCU writes");
      counters.writes.emplace_back(event.address, event.value);
      display->observe(event);
    });
  }

  void start(const Profile input) {
    profile = input;
    counters = {};
    interrupted = registers(machine.cpu().state());
    machine.bus().observe_usb_decoder({.reset = true});
  }

  void step(unsigned vector);

  void check_flash() const {
    const auto now = machine.bus().power_off_flash_image();
    require(now.bytes == initial_.bytes && now.known == initial_.known &&
                now.fresh == initial_.fresh,
            "Original USB execution preserves every MCU NVM byte, knowledge and erase history");
  }

  Outcome outcome();

  unsigned word(unsigned address) {
    return static_cast<unsigned>(
        machine.bus().read_memory({.address = static_cast<std::uint16_t>(address), .size = 2}));
  }

  void finish_configuration();
  void check_bulk_admission();

  void report(std::string_view kind) const {
    trace_ << kind << ' ' << profile.id << ' ' << counters.polls << ' ' << counters.delays << ' '
           << counters.entries << ' ' << counters.returns << ' ' << counters.native << '\n';
  }

  blofeld::McuMachine machine;
  std::unique_ptr<blofeld::test::LcdFixture> display;
  Profile profile;
  Counters counters;
  Registers interrupted{};
  std::uint64_t actual_retirements = 0; // Host evidence; deliberately not checkpointed.

  std::ostream &trace_;
  blofeld::McuFlashImage initial_;
};

void Session::step(const unsigned vector) {
  const auto before = machine.cpu().state();
  if (vector == reset_vector && before.pc == reset_poll) {
    ++counters.polls;
    require((before.d & byte_mask) == ((counters.delays + 1) & byte_mask),
            "The original reset counter increments once per genuine byte-delay call");
    if (profile.release_poll == counters.polls) { machine.bus().observe_usb_decoder({}); }
  }
  if (vector == reset_vector && before.pc == reset_delay) { ++counters.delays; }
  const auto result = machine.cpu().step();
  if (result.interrupt_vector) {
    require(*result.interrupt_vector == vector && !result.instruction_executed &&
                result.stack_reused && counters.entries == 0 && before.pc == main_resume &&
                before.sp == wait_stack,
            "The original USB interrupt must enter through its real vector and existing WAI frame");
    ++counters.entries;
  }
  if (result.instruction_executed) {
    ++counters.native;
    ++actual_retirements;
  }
  if (before.pc == main_wait) {
    const auto after = machine.cpu().state();
    constexpr unsigned interrupt_mask = 0x10;
    require(result.instruction_executed && before.sp == main_stack && after.pc == main_resume &&
                after.sp == wait_stack && machine.cpu().waiting() &&
                (after.ccr & interrupt_mask) == 0,
            "Actual original WAI saves a fresh enabled-interrupt foreground frame");
    const std::array<unsigned, 9> frame{before.ccr,
                                        before.d & byte_mask,
                                        static_cast<unsigned>(before.d) >> 8U,
                                        static_cast<unsigned>(before.x) >> 8U,
                                        before.x & byte_mask,
                                        static_cast<unsigned>(before.y) >> 8U,
                                        before.y & byte_mask,
                                        main_resume >> 8U,
                                        main_resume & byte_mask};
    for (unsigned i = 0; i < frame.size(); ++i) {
      require(machine.bus().read(static_cast<std::uint16_t>(wait_stack + i)) == frame[i],
              "Every real WAI frame byte matches the independent NXP register order");
    }
  }
  if ((vector == reset_vector && (before.pc == reset_return || before.pc == suspend_return)) ||
      (vector == configuration_vector && before.pc == configuration_return) ||
      (vector == control_in_vector && before.pc == control_in_return)) {
    auto expected = interrupted;
    expected[4] = main_stack;
    require(result.instruction_executed && registers(machine.cpu().state()) == expected &&
                !machine.cpu().waiting(),
            "The actual original RTI restores every interrupted CPU register before foreground "
            "resumes");
    ++counters.returns;
  }
}

Outcome Session::outcome() {
  Outcome result{
      .cpu = registers(machine.cpu().state()), .ram = {}, .usb = {}, .counters = counters};
  for (unsigned i = 0; i < ram_bytes; ++i) {
    result.ram.push_back(machine.bus().read(static_cast<std::uint16_t>(ram_begin + i)));
  }
  // Inspect a detached board copy: no host selector/control write reaches native execution.
  auto copy = machine.bus().clone();
  constexpr std::array<unsigned, 11> controls{
      0x300, 0x302, 0x304, 0x30c, 0x30e, 0x314, 0x316, 0x318, 0x31a, out_control, in_control};
  for (const auto at : controls) {
    result.usb.push_back(static_cast<unsigned>(
        copy->read_memory({.address = static_cast<std::uint16_t>(at), .size = 2})));
  }
  for (const auto bank : {1U, 2U, 3U, 4U, 5U, 8U, 9U}) {
    copy->write(select, static_cast<std::uint8_t>(bank));
    result.usb.push_back(
        static_cast<unsigned>(copy->read_memory({.address = config_data, .size = 2})));
    result.usb.push_back(
        static_cast<unsigned>(copy->read_memory({.address = config_data + 2, .size = 2})));
  }
  check_flash();
  return result;
}

std::optional<Saved> finish_reset(Session &session, const bool capture) {
  std::optional<Saved> middle;
  do {
    require(session.counters.native < instruction_limit,
            "Original reset processing must return or fail visibly");
    if (capture && session.machine.cpu().state().pc == split_controls) {
      require(!middle && session.word(out_control) == retained_control &&
                  session.word(in_control) == 0,
              "Snapshot follows actual UEPCSR4A acknowledgement before original UEPCSR5A write");
      middle.emplace(session.save());
    }
    session.step(reset_vector);
  } while (!session.machine.cpu().waiting() || session.machine.cpu().pending_interrupt());
  const unsigned polls = session.profile.release_poll == 0 ? 256 : session.profile.release_poll;
  require(session.counters.polls == polls && session.counters.delays == polls - 1 &&
              session.counters.entries == 1 && session.counters.returns == 1 &&
              session.machine.cpu().state().pc == main_resume &&
              session.word(out_control) == retained_control &&
              session.word(in_control) == retained_control,
          "Actual reset polling/reconfiguration/RTI and foreground must reach original main WAI");
  Writes actual;
  for (const auto write : session.counters.writes) {
    if ((write.first >= out_control && write.first <= out_control + 1) ||
        (write.first >= in_control && write.first <= in_control + 1)) {
      actual.push_back(write);
    }
  }
  const Writes expected{
      {out_control, 0x38}, {out_control + 1, 0}, {in_control, 0x38}, {in_control + 1, 0}};
  require(actual == expected,
          "The genuine original STD instructions write both exact control words in order");
  session.check_flash();
  return middle;
}

void Session::finish_configuration() {
  counters = {};
  interrupted = registers(machine.cpu().state());
  machine.bus().observe_usb_decoder({.enumerated = true});
  constexpr std::array<std::uint8_t, 8> request{0, 9, 1, 0, 0, 0, 0, 0};
  const blofeld::UsbTransferRequest transfer{.sequence = 7,
                                             .address = 0,
                                             .length = 0,
                                             .setup = request,
                                             .output = {},
                                             .short_not_ok = false,
                                             .zero_packet = false};
  blofeld::UsbipDecoder decoder(blofeld::test::usbip_fixture::device);
  decoder.feed(blofeld::test::usbip_fixture::submit(transfer));
  const auto command = decoder.next();
  if (!command || !std::holds_alternative<blofeld::UsbTransferRequest>(*command)) {
    throw blofeld::Fault("Complete own USB/IP configuration submission must decode");
  }
  blofeld::UsbHostQueue queue;
  queue.submit(std::get<blofeld::UsbTransferRequest>(*command));
  const auto context = [this] {
    const auto state = machine.cpu().state();
    return blofeld::ExecutionContext{.pc = state.pc, .ccr = state.ccr};
  };
  constexpr unsigned pending_configuration = 0xa100;
  require(queue.poll(machine.bus(), context()).empty() &&
              queue.poll(machine.bus(), context()).empty() && queue.pending() == 1 &&
              !queue.faulted() && word(configuration_status) == pending_configuration,
          "USB/IP configuration SUBMIT must reach SETECR and pending status NAK before the MCU");
  decoder.finish();
  trace_ << "CONFIGURATION_SETUP ACK NAK\n";
  const auto pending = save();
  const auto pending_queue = queue;
  std::optional<Outcome> expected;
  std::optional<Outcome> expected_status;
  constexpr unsigned requested = 0x2000;
  constexpr unsigned valid_configuration = 0x8100;
  for (unsigned replay = 0; replay < 3; ++replay) {
    if (replay != 0) {
      restore(pending);
      queue = pending_queue;
    }
    do {
      require(counters.native < instruction_limit,
              "Original configuration acknowledgement must return");
      step(configuration_vector);
    } while (!machine.cpu().waiting() || machine.cpu().pending_interrupt());
    require(counters.entries == 1 && counters.returns == 1 &&
                (word(configuration_status) & (requested | valid_configuration)) ==
                    valid_configuration,
            "Original configuration IRQ acknowledges SETECR and retains CFGVALID/configuration1");
    const auto actual = outcome();
    if (!expected) { expected = actual; }
    require(actual == *expected,
            "Pending configuration snapshot replays CPU/RAM/USB/write prefix through native RTI");
    report("CONFIGURATION");
    const auto completed = queue.poll(machine.bus(), context());
    const blofeld::UsbTransferCompletion expected_completion{
        .sequence = 7,
        .status = blofeld::UsbTransferStatus::success,
        .transferred = 0,
        .input = {},
        .input_direction = false};
    require(completed.size() == 1 && completed.front() == expected_completion &&
                queue.pending() == 0 && !queue.faulted(),
            "Original DONEECRU plus actual DATA1/ZLP/ACK completes exactly one real host transfer");
    constexpr unsigned endpoint_control = 0x314;
    constexpr unsigned acknowledged_status = 0x3000;
    require(word(endpoint_control) == acknowledged_status && machine.cpu().pending_interrupt(),
            "Host ACK produces literal TFRC/TCIE with DVALID clear and a real pending CPU IRQ");
    trace_ << "CONFIGURATION_HOST_STATUS 7 0 0 " << word(endpoint_control) << '\n';
    counters = {};
    interrupted = registers(machine.cpu().state());
    do {
      require(counters.native < instruction_limit, "Original control-IN IRQ must return to WAI");
      step(control_in_vector);
    } while (!machine.cpu().waiting() || machine.cpu().pending_interrupt());
    constexpr unsigned ready_control = 0x1000;
    require(counters.entries == 1 && counters.returns == 1 &&
                word(endpoint_control) == ready_control &&
                queue.poll(machine.bus(), context()).empty(),
            "Original control-IN ISR acknowledges actual TFRC and returns through native RTI/WAI");
    const auto status_outcome = outcome();
    if (!expected_status) { expected_status = status_outcome; }
    require(status_outcome == *expected_status,
            "Original status IRQ replays all CPU/RAM/USB state and actual write ordering");
    check_flash();
    report("CONFIGURATION_STATUS");
  }
}

void Session::check_bulk_admission() {
  const auto pristine = save();
  const auto before = outcome();
  constexpr std::array<std::uint8_t, 4> note_on{9, 0x90, 64, 127};
  constexpr unsigned midi_in_vector = 0xffca;
  constexpr unsigned midi_out_vector = 0xffc8;
  constexpr unsigned input_control = 0x318;
  constexpr unsigned output_control = 0x31a;
  constexpr unsigned local_window = 0x380;
  constexpr unsigned local_select = 0x30f;
  constexpr unsigned completed_input = 0x3000;
  constexpr unsigned received_output = 0x7004;
  for (const bool input : {false, true}) {
    restore(pristine);
    const auto endpoint = machine.bus().usb_host_endpoint(input ? 0x81 : 1);
    require(endpoint.type == 2 && endpoint.maximum_packet == 64 && endpoint.data1 == false,
            "Original full-speed bulk descriptors retain their actual initial DATA0 phase");
    if (input) {
      const auto packet = machine.bus().request_usb_in(blofeld::UsbBuffer::endpoint2);
      require(packet.response == blofeld::UsbResponse::data && packet.size == 0 && !packet.data1,
              "Original initialization primes its genuine MIDI-IN zero-length DATA0 packet");
      machine.bus().finish_usb_in(true);
      require(word(input_control) == completed_input,
              "Only actual host ACK clears original DVALID and sets TFRC while retaining TCIE");
    } else {
      require(machine.bus().receive_usb_out(
                  {.buffer = blofeld::UsbBuffer::endpoint3, .data = std::span{note_on}}) ==
                      blofeld::UsbResponse::ack &&
                  word(output_control) == received_output,
              "Actual DATA0 copies four USB-MIDI bytes and sets literal DVALID/TFRC/TCIE/size");
      auto copy = machine.bus().clone();
      copy->write(local_select, 3);
      for (unsigned byte = 0; byte < note_on.size(); ++byte) {
        require(copy->read(static_cast<std::uint16_t>(local_window + byte)) == note_on[byte],
                "Every admitted byte matches the independent actual host packet");
      }
    }
    const auto pending = machine.cpu().pending_interrupt();
    if (!pending) {
      throw blofeld::Fault("Original bulk admission requires its real completion IRQ");
    }
    require(pending->vector == (input ? midi_in_vector : midi_out_vector) &&
                registers(machine.cpu().state()) == before.cpu && machine.cpu().waiting(),
            "Real bulk completion asserts its original IRQ without fabricating CPU execution");
    auto copy = machine.bus().clone();
    copy->write(select, static_cast<std::uint8_t>(input ? 2 : 3));
    const auto description =
        (static_cast<std::uint32_t>(copy->read_memory({.address = config_data, .size = 2}))
         << 16U) |
        static_cast<std::uint32_t>(copy->read_memory({.address = config_data + 2, .size = 2}));
    require(description == (input ? in_description : out_description),
            "Actual transfers cannot rewrite the original raw NISOF descriptor bits");
    check_flash();
    trace_ << "MIDI_ADMISSION " << (input ? "IN " : "OUT ") << (input ? 0 : note_on.size()) << ' '
           << pending->vector << ' ' << blofeld::hex(description, 8) << '\n';
  }
  restore(pristine);
  require(outcome() == before, "Packet admission restores the complete pristine fixture state");
}

constexpr unsigned setup_vector = 0xffd2;
constexpr unsigned setup_entry = 0xbe21;
constexpr unsigned setup_return = 0xbe3b;
constexpr unsigned setup_fault_pc = 0xbc3d;
constexpr unsigned status_word = 0x302;
constexpr unsigned control_in_word = 0x314;
constexpr unsigned control_out_vector = 0xffcc;
constexpr unsigned control_in_entry = 0xbe50;
constexpr unsigned control_out_entry = 0xbe5f;
constexpr unsigned control_out_return = 0xbe6a;
constexpr unsigned setup_source = 0x338;
constexpr unsigned descriptor_source = 0xfe49;
constexpr unsigned device_descriptor_size = 18;
constexpr unsigned feature_flags = 0x81e0;
constexpr unsigned flash_begin = 0x8000;
constexpr unsigned local_window = 0x380;
constexpr unsigned local_select = 0x30f;
constexpr unsigned idle_usb_status = 0xe440;
constexpr unsigned setup_usb_status = idle_usb_status | 1U;
constexpr unsigned overrun_usb_status = idle_usb_status | 3U;
constexpr unsigned queued_control = 0x5000;    // DVALID | TCIE, TFRC clear.
constexpr unsigned completed_control = 0x3000; // TFRC | TCIE, DVALID clear.
constexpr unsigned callback_frame_bytes = 20;  // JSR return plus 18-byte local frame.
constexpr unsigned fault_ccr = 0x90;           // Retained S/X fields and masked I; NZVC clear.

struct SetupProfile {
  unsigned length = 0;
  bool overrun = false;
};

std::array<std::uint8_t, 8> setup_request(const unsigned length) {
  return {0x80,
          6,
          0,
          1,
          0,
          0,
          static_cast<std::uint8_t>(length & byte_mask),
          static_cast<std::uint8_t>(length >> 8U)};
}

void start_setup(Session &session, const SetupProfile profile) {
  session.counters = {};
  session.interrupted = registers(session.machine.cpu().state());
  if (profile.overrun) {
    require(session.machine.bus().receive_usb_setup(setup_request(device_descriptor_size)) ==
                blofeld::UsbResponse::ack,
            "The superseded own GET_DESCRIPTOR is a genuine decoded SETUP");
    require(session.machine.bus().receive_usb_setup(setup_request(profile.length)) ==
                blofeld::UsbResponse::ack,
            "The last own SETUP supplies the actual requested word length");
  } else {
    require(session.machine.bus().receive_usb_setup(setup_request(profile.length)) ==
                blofeld::UsbResponse::ack,
            "Ordinary device GET_DESCRIPTOR is an actual decoded SETUP");
    const auto expected = setup_request(profile.length);
    for (unsigned byte = 0; byte < expected.size(); ++byte) {
      require(session.machine.bus().read(static_cast<std::uint16_t>(setup_source + byte)) ==
                  expected[byte],
              "Probe-generated SETUP bytes match independent original device-request fields");
    }
  }
  require(session.word(status_word) == (profile.overrun ? overrun_usb_status : setup_usb_status),
          "One/two actual SETUP events establish their independently expected flags");
}

void inspect_setup_payload(Session &session, Outcome &outcome) {
  auto copy = session.machine.bus().clone();
  constexpr unsigned unknown_byte = 0x100;
  constexpr unsigned buffer_bytes = 64;
  constexpr unsigned buffers = 4;
  constexpr auto inspected_bytes = static_cast<std::size_t>(buffers) * buffer_bytes;
  outcome.usb.reserve(outcome.usb.size() + inspected_bytes);
  for (unsigned buffer = 0; buffer < buffers; ++buffer) {
    copy->write(local_select, static_cast<std::uint8_t>(buffer));
    for (unsigned i = 0; i < buffer_bytes; ++i) {
      unsigned value = unknown_byte;
      try {
        value = copy->read(static_cast<std::uint16_t>(local_window + i));
      } catch (const blofeld::Fault &fault) {
        require(std::string_view(fault.what()).contains("Unknown USB endpoint-buffer byte"),
                "Only unknown buffer-byte knowledge can produce an inspection sentinel");
      }
      outcome.usb.push_back(value);
    }
  }
}

void check_descriptor_footprint(Session &session,
                                const blofeld::Firmware &firmware,
                                const unsigned size) {
  auto copy = session.machine.bus().clone();
  constexpr unsigned serial_number_index = 16;
  constexpr unsigned serial_suppression = 0x80;
  constexpr unsigned buffer_bytes = 64;
  const bool omit_serial = (firmware.mcu.at(feature_flags - flash_begin) & serial_suppression) != 0;
  const auto copied = (size + 1U) & ~1U;
  for (unsigned buffer = 0; buffer < 4; ++buffer) {
    copy->write(local_select, static_cast<std::uint8_t>(buffer));
    for (unsigned byte = 0; byte < buffer_bytes; ++byte) {
      const bool serial = byte == serial_number_index && omit_serial;
      const bool expected_known = buffer == 0 && (byte < copied || serial);
      try {
        const auto value = copy->read(static_cast<std::uint16_t>(local_window + byte));
        require(expected_known,
                "Original paired-byte copy cannot make unrelated buffer bytes known");
        const auto expected = serial ? 0U : firmware.mcu.at(descriptor_source - flash_begin + byte);
        require(value == expected,
                "Every copied byte and conditional serial field matches its ROM source");
      } catch (const blofeld::Fault &fault) {
        require(!expected_known &&
                    std::string_view(fault.what()).contains("Unknown USB endpoint-buffer byte"),
                "Only independently expected unknown local-buffer bytes may remain unreadable");
      }
    }
  }
}

void check_device_reply(Session &session,
                        const blofeld::Firmware &firmware,
                        const SetupProfile profile) {
  const auto size = std::min(profile.length, device_descriptor_size);
  require(session.word(control_in_word) == (queued_control | size) &&
              session.word(status_word) == (profile.overrun ? setup_usb_status : idle_usb_status),
          "Original device callback queues the clamped payload with the actual SETUP flag state");
  check_descriptor_footprint(session, firmware, size);
  const auto packet = session.machine.bus().request_usb_in(blofeld::UsbBuffer::control_in);
  require(packet.response == blofeld::UsbResponse::data && packet.data1 && packet.size == size,
          "Actual original firmware bytes supply a DATA1 control IN packet of the expected size");
  constexpr unsigned serial_number_index = 16;
  constexpr unsigned serial_suppression = 0x80;
  const bool omit_serial = (firmware.mcu.at(feature_flags - flash_begin) & serial_suppression) != 0;
  for (unsigned byte = 0; byte < size; ++byte) {
    const auto expected = byte == serial_number_index && omit_serial
                              ? 0U
                              : firmware.mcu.at(descriptor_source - flash_begin + byte);
    require(packet.data[byte] == expected,
            "Every returned byte matches the independently pinned ROM descriptor/feature rule");
  }
  session.machine.bus().finish_usb_in(true);
  require(session.word(control_in_word) == (completed_control | size),
          "Only the real decoded host ACK sets TFRC, clears DVALID and retains TCIE/size");
  const auto pending = session.machine.cpu().pending_interrupt();
  require(!pending, "The original ISR still masks interrupt delivery before its RTI");
  constexpr unsigned enabled_ccr = 0x40;
  const auto selected = session.machine.bus().interrupt_request(enabled_ccr);
  require(selected && selected->vector == (profile.overrun ? setup_vector : control_in_vector),
          "Pending original SETUP has priority; otherwise the genuine control-IN IRQ is selected");
}

void check_setup_step(Session &session,
                      const blofeld::CpuState before,
                      const blofeld::CpuStep result) {
  if (result.instruction_executed) {
    ++session.counters.native;
    ++session.actual_retirements;
  }
  const bool setup_rti = result.instruction_executed && before.pc == setup_return;
  const bool input_rti = result.instruction_executed && before.pc == control_in_return;
  const bool output_rti = result.instruction_executed && before.pc == control_out_return;
  if (setup_rti) { ++session.counters.returns; }
  if (input_rti) { ++session.counters.control_returns; }
  if (output_rti) { ++session.counters.out_returns; }
  if (result.interrupt_vector) {
    unsigned entry = 0;
    switch (*result.interrupt_vector) {
      case setup_vector:
        require(session.counters.entries < 2,
                "Only the independently expected ordinary/overrun SETUP IRQs may enter");
        ++session.counters.entries;
        entry = setup_entry;
        break;
      case control_in_vector:
        require(session.counters.control_entries++ == 0 && setup_rti,
                "Real IN ACK chains its single completion IRQ through original SETUP RTI");
        entry = control_in_entry;
        break;
      case control_out_vector:
        require(session.counters.out_entries++ == 0 && !result.instruction_executed,
                "Actual control status OUT wakes the original waiting foreground once");
        entry = control_out_entry;
        break;
      default: throw blofeld::Fault("Unexpected IRQ during original SETUP completion");
    }
    require(result.stack_reused,
            "All original USB entries reuse the actual native interrupt frame");
    auto expected = session.interrupted;
    expected[0] = entry;
    expected[5] |= 0x10U;
    require(registers(session.machine.cpu().state()) == expected,
            "Native entry preserves the independently expected interrupted registers and sets I");
  } else if (setup_rti || input_rti || output_rti) {
    auto expected = session.interrupted;
    expected[4] = main_stack;
    require(registers(session.machine.cpu().state()) == expected &&
                !session.machine.cpu().waiting(),
            "Actual original RTI restores every interrupted register before foreground resumes");
  }
}

void drain_setup(Session &session) {
  while (!session.machine.cpu().waiting() || session.machine.cpu().pending_interrupt()) {
    require(session.counters.native < instruction_limit, "Original USB completion must be bounded");
    const auto before = session.machine.cpu().state();
    check_setup_step(session, before, session.machine.cpu().step());
  }
  require(session.machine.cpu().state().pc == main_resume,
          "Original completion and foreground must reach their genuine main WAI");
}

void verify_setup_read(Session &session, const blofeld::CpuState before, const unsigned length) {
  // Original BC3D is LDAB 7,X: independently check the wire byte and S12 NZVC effect.
  const auto expected_byte = setup_request(length).back();
  constexpr unsigned nzv = 0x0e;
  constexpr unsigned negative = 0x08;
  constexpr unsigned zero = 0x04;
  constexpr unsigned sign = 0x80;
  const auto flags = (before.ccr & ~nzv) | ((expected_byte & sign) != 0 ? negative : 0U) |
                     (expected_byte == 0 ? zero : 0U);
  const auto after = session.machine.cpu().state();
  require(
      before.x == setup_source && before.d == setup_source && after.pc == setup_fault_pc + 2 &&
          after.d == ((before.d & ~byte_mask) | expected_byte) && after.ccr == flags,
      "Actual post-W1C LDAB retires once using the retained accepted byte and independent flags");
}

std::optional<Saved> finish_setup(Session &session,
                                  const blofeld::Firmware &firmware,
                                  const SetupProfile profile,
                                  const bool capture) {
  std::optional<Saved> middle;
  bool packet = false;
  do {
    require(session.counters.native < instruction_limit,
            "Original SETUP completion must be bounded");
    const auto before = session.machine.cpu().state();
    if (capture && !middle && before.pc == setup_fault_pc) {
      require(before.ccr == fault_ccr && before.sp == wait_stack - callback_frame_bytes,
              "Pre-read snapshot retains the original frame and masked original CCR");
      middle.emplace(session.save());
    }
    if (!packet && before.pc == setup_return) {
      check_device_reply(session, firmware, profile);
      packet = true;
    }
    check_setup_step(session, before, session.machine.cpu().step());
    if (before.pc == setup_fault_pc) { verify_setup_read(session, before, profile.length); }
  } while (!session.machine.cpu().waiting() || session.machine.cpu().pending_interrupt());
  require(packet && session.counters.entries == (profile.overrun ? 2U : 1U) &&
              session.counters.returns == session.counters.entries &&
              session.counters.control_entries == 1 && session.counters.control_returns == 1,
          "Original SETUP/overrun and real IN completion each return through actual native RTI");
  if (profile.length != 0) {
    session.interrupted = registers(session.machine.cpu().state());
    require(session.machine.bus().receive_usb_out(
                {.buffer = blofeld::UsbBuffer::control_out, .data = {}, .data1 = true}) ==
                blofeld::UsbResponse::ack,
            "Only a real DATA1/ZLP supplies the control read's OUT status stage");
    drain_setup(session);
  }
  require(session.machine.cpu().state().pc == main_resume &&
              session.word(status_word) == idle_usb_status,
          "Original control request completes without a fabricated flag or skipped instruction");
  for (unsigned i = 0; i < 8; ++i) {
    require(session.machine.bus().read(static_cast<std::uint16_t>(setup_source + i)) ==
                setup_request(profile.length)[i],
            "All actual accepted SETUP bytes remain known after both firmware acknowledgements");
  }
  session.check_flash();
  return middle;
}

std::optional<Saved> finish_suspend(Session &session, const bool capture) {
  constexpr unsigned suspend_mask_instruction = 0xbdd8;
  std::optional<Saved> middle;
  while (session.counters.returns == 0) {
    require(session.counters.native < 7,
            "Original suspend-change handler must retire exactly seven instructions");
    if (capture && session.counters.native == 5) {
      require(session.machine.cpu().state().pc == suspend_mask_instruction &&
                  (session.word(status_word) & 0x1000U) == 0,
              "Suspend snapshot follows actual USSC acknowledgement before original mask BCLR");
      middle.emplace(session.save());
    }
    session.step(reset_vector);
  }
  require(session.counters.entries == 1 && session.counters.native == 7 &&
              session.counters.polls == 0 && session.counters.delays == 0,
          "Suspend and resume use the actual status vector, short handler and native RTI");
  return middle;
}

void check_suspend_paths(Session &session) {
  const auto configured = session.save();
  constexpr unsigned suspend_change = 0x1000;
  constexpr unsigned suspend_detected = 0x10;
  constexpr unsigned status_index = 1;
  constexpr unsigned masks_index = 2;
  constexpr unsigned interrupt_mask_word = 0x304;
  constexpr unsigned original_mask_high = 0x38;
  for (unsigned profile = 0; profile < 4; ++profile) {
    const bool suspended = profile % 2 == 0;
    session.counters = {};
    session.interrupted = registers(session.machine.cpu().state());
    auto expected = session.outcome();
    inspect_setup_payload(session, expected);
    const auto evidence = session.machine.bus().usb_host_evidence();
    const auto configuration = session.machine.bus().usb_host_configuration();
    const blofeld::UsbDecoderState observation{.suspended = suspended, .enumerated = true};
    session.machine.bus().observe_usb_decoder(observation);
    const unsigned retained_status = idle_usb_status | (suspended ? suspend_detected : 0U);
    const auto pending = session.machine.cpu().pending_interrupt();
    require(session.word(status_word) == (retained_status | suspend_change) &&
                pending.has_value() && pending->vector == reset_vector &&
                session.machine.bus().usb_host_evidence() == evidence &&
                session.machine.bus().usb_host_configuration() == configuration,
            "Decoded changes assert the real enabled status IRQ and retain configuration/link");
    const auto middle = finish_suspend(session, true);
    if (!middle) {
      throw blofeld::Fault("Every suspend/resume run requires its actual mid-handler snapshot");
    }
    expected.cpu[4] = main_stack;
    expected.usb[status_index] = retained_status;
    expected.usb[masks_index] &= ~0x4000U;
    const Writes writes{
        {status_word, 0x10}, {status_word + 1, 0}, {interrupt_mask_word, original_mask_high}};
    const auto check = [&] {
      auto actual = session.outcome();
      inspect_setup_payload(session, actual);
      require(
          actual.cpu == expected.cpu && actual.ram == expected.ram && actual.usb == expected.usb &&
              actual.counters.writes == writes && !session.machine.cpu().pending_interrupt() &&
              session.machine.bus().usb_host_suspended() == suspended &&
              session.machine.bus().usb_host_evidence() == evidence &&
              session.machine.bus().usb_host_configuration() == configuration,
          "Original RTI preserves all registers/RAM/USB payload knowledge/NVM except USSC/mask");
      return actual;
    };
    const auto completed = check();
    session.trace_ << "SUSPEND_CASE " << profile << ' ' << suspended << " 7 1 1 3\n";
    for (unsigned replay = 0; replay < 2; ++replay) {
      session.restore(*middle);
      static_cast<void>(finish_suspend(session, false));
      require(check() == completed, "Both mid-handler restores reproduce complete native state");
      session.trace_ << "SUSPEND_REPLAY " << profile << ' ' << suspended << " 7 1 1 3\n";
    }
    session.machine.bus().observe_usb_decoder(observation);
    require(!session.machine.cpu().pending_interrupt() &&
                session.word(status_word) == retained_status && check() == completed,
            "Repeated identical decoder input cannot manufacture a second change interrupt");
    do {
      require(session.counters.native < instruction_limit,
              "Original resumed foreground must reach its own next WAI");
      session.step(reset_vector);
    } while (!session.machine.cpu().waiting() || session.machine.cpu().pending_interrupt());
    session.trace_ << "SUSPEND_FOREGROUND " << profile << ' ' << session.counters.native << '\n';
  }
  session.restore(configured);
}

void check_setup_paths(Session &session, const blofeld::Firmware &firmware) {
  const auto configured = session.save();
  constexpr unsigned maximum_length = 0xffff;
  constexpr std::array<unsigned, 7> lengths{0, 1, 8, 9, 17, 18, maximum_length};
  for (unsigned index = 0; index <= lengths.size(); ++index) {
    const SetupProfile profile{.length = index == 0 ? device_descriptor_size : lengths[index - 1],
                               .overrun = index != 0};
    session.restore(configured);
    start_setup(session, profile);
    const auto middle = finish_setup(session, firmware, profile, true);
    if (!middle) {
      throw blofeld::Fault("Each original SETUP branch requires its real pre-read snapshot");
    }
    auto expected = session.outcome();
    inspect_setup_payload(session, expected);
    const auto report = [&](const std::string_view kind) {
      session.trace_ << kind << ' ' << profile.overrun << ' ' << profile.length << ' '
                     << session.counters.native << ' ' << session.counters.entries << ' '
                     << session.counters.returns << ' '
                     << std::min(profile.length, device_descriptor_size) << '\n';
      session.trace_ << "SETUP_COMPLETION " << profile.overrun << ' ' << profile.length << ' '
                     << session.counters.control_entries << ' ' << session.counters.control_returns
                     << ' ' << session.counters.out_entries << ' ' << session.counters.out_returns
                     << '\n';
    };
    report("SETUP_CASE");
    session.trace_ << "SETUP_PREFIX " << profile.overrun << ' ' << profile.length << ' '
                   << middle->counters.native << '\n';
    for (unsigned replay = 0; replay < 2; ++replay) {
      session.restore(*middle);
      static_cast<void>(finish_setup(session, firmware, profile, false));
      auto actual = session.outcome();
      inspect_setup_payload(session, actual);
      require(actual == expected,
              "Both pre-read joint restores retain CPU, all RAM, USB controls/payload knowledge "
              "and actual write prefix through the same qualified frontier");
      report("SETUP_REPLAY");
    }
  }
}

using Bytes = std::vector<std::uint8_t>;

Bytes expected_list(const blofeld::Firmware &firmware) {
  constexpr unsigned configuration_bytes = 101;
  constexpr unsigned configuration_source = descriptor_source + device_descriptor_size;
  // Independent USB/IP OP_REP_DEVLIST layout: 12-byte prefix, 312-byte device,
  // then four bytes per active interface. Identity fields come only from ROM.
  constexpr unsigned reply_prefix = 12;
  constexpr unsigned record_bytes = 312;
  constexpr unsigned interface_bytes = 4;
  constexpr unsigned path_bytes = 256;
  constexpr unsigned location_bytes = 288;
  constexpr unsigned identity_offset = 300;
  const std::span<const std::uint8_t> rom(firmware.mcu);
  const auto device = rom.subspan(descriptor_source - flash_begin, device_descriptor_size);
  const auto configuration = rom.subspan(configuration_source - flash_begin, configuration_bytes);
  const auto count = configuration[4];
  Bytes expected(reply_prefix + record_bytes + (count * interface_bytes), 0);
  expected[0] = 1;
  constexpr unsigned usbip_version_low = 0x11;
  expected[1] = usbip_version_low; // USB/IP version 0x0111.
  expected[3] = 5;                 // OP_REP_DEVLIST.
  expected[11] = 1;                // One device, big-endian uint32.
  constexpr std::string_view path = "/blofeld-emu/usb/1-1";
  constexpr std::string_view busid = "1-1";
  std::ranges::copy(path, expected.begin() + reply_prefix);
  std::ranges::copy(busid, expected.begin() + reply_prefix + path_bytes);
  expected[reply_prefix + location_bytes + 3] = 1;  // Bus 1.
  expected[reply_prefix + location_bytes + 7] = 1;  // Device 1.
  expected[reply_prefix + location_bytes + 11] = 2; // USB_SPEED_FULL.
  unsigned output = reply_prefix + identity_offset;
  for (const unsigned input : {8U, 10U, 12U}) {
    expected.at(output++) = device[input + 1];
    expected.at(output++) = device[input];
  }
  for (const unsigned input : {4U, 5U, 6U}) { expected.at(output++) = device[input]; }
  expected.at(output++) = configuration[5];
  constexpr unsigned configuration_count_offset = 17;
  expected.at(output++) = device[configuration_count_offset];
  expected.at(output++) = count;
  for (unsigned offset = 0; offset < configuration.size();) {
    const auto length = configuration[offset];
    require(length >= 2 && length <= configuration.size() - offset,
            "Independently traversed original descriptor spans must fit the ROM extent");
    if (configuration[offset + 1] == 4) {
      require(length >= 9, "Original interface descriptor must contain class metadata");
      if (configuration[offset + 3] == 0) {
        for (unsigned field = 5; field < 8; ++field) {
          expected.at(output++) = configuration[offset + field];
        }
        ++output;
      }
    }
    offset += length;
  }
  require(output == expected.size(), "Wire interface count must equal the independent ROM count");
  return expected;
}

Bytes list_device(Session &instrument,
                  blofeld::UsbipRuntime &runtime,
                  blofeld::UsbipRuntimeBoundary &boundary) {
  namespace tcp = blofeld::test::usbip_tcp;
  auto client = tcp::connect(runtime.port());
  constexpr unsigned usbip_version_low = 0x11;
  constexpr std::array<std::uint8_t, 8> request{1, usbip_version_low, 0x80, 5, 0, 0, 0, 0};
  tcp::send(client, request);
  tcp::Received received;
  constexpr unsigned maximum_polls = 64;
  for (unsigned attempt = 0; !received.eof && attempt < maximum_polls; ++attempt) {
    ++boundary.ticks;
    static_cast<void>(
        runtime.poll(instrument.machine.bus(),
                     blofeld::ExecutionContext{.pc = instrument.machine.cpu().state().pc,
                                               .ccr = instrument.machine.cpu().state().ccr},
                     boundary));
    tcp::read(client, received);
    if (!received.eof) {
      pollfd ready{.fd = client.get(), .events = POLLIN, .revents = 0};
      static_cast<void>(::poll(&ready, 1, 1));
    }
  }
  require(received.eof && runtime.published() && !runtime.faulted(),
          "Real loopback DEVLIST must finish without retiring its native descriptor publication");
  return received.bytes;
}

void drain_discovery(Session &session, std::optional<Saved> &middle) {
  while (!session.machine.cpu().waiting() || session.machine.cpu().pending_interrupt()) {
    require(session.counters.native < instruction_limit,
            "Original descriptor discovery must finish within its native instruction bound");
    const auto before = session.machine.cpu().state();
    if (!middle && before.pc == setup_fault_pc) { middle.emplace(session.save()); }
    const auto step = session.machine.cpu().step();
    if (step.instruction_executed) {
      ++session.counters.native;
      ++session.actual_retirements;
    }
    if (step.interrupt_vector) {
      require(*step.interrupt_vector == setup_vector ||
                  *step.interrupt_vector == control_in_vector ||
                  *step.interrupt_vector == control_out_vector,
              "Only original descriptor control-transfer vectors may execute during discovery");
      ++session.counters.entries;
    }
    if (step.instruction_executed && (before.pc == setup_return || before.pc == control_in_return ||
                                      before.pc == control_out_return)) {
      ++session.counters.returns;
    }
  }
  require(session.machine.cpu().state().pc == main_resume,
          "Every native descriptor callback/completion must return to actual foreground WAI");
}

void check_descriptors(const blofeld::UsbDeviceDescriptors &descriptors,
                       const blofeld::Firmware &firmware) {
  constexpr unsigned configuration_size = 101;
  constexpr unsigned serial_number_index = 16;
  const auto rom = std::span(firmware.mcu);
  const auto original = rom.subspan(descriptor_source - flash_begin, device_descriptor_size);
  for (unsigned byte = 0; byte < original.size(); ++byte) {
    const auto expected = byte == serial_number_index ? 0U : original[byte];
    require(descriptors.device()[byte] == expected,
            "Captured device bytes match original ROM and its executed serial-suppression rule");
  }
  const auto configuration =
      rom.subspan(descriptor_source + device_descriptor_size - flash_begin, configuration_size);
  require(descriptors.device().size() == device_descriptor_size &&
              std::ranges::equal(descriptors.configuration(), configuration) &&
              descriptors.interface_count() == configuration[4],
          "Only complete original ROM configuration bytes and interfaces qualify discovery");
}

void finish_probe(Session &session,
                  blofeld::UsbDeviceProbe &probe,
                  std::optional<Saved> &middle,
                  std::optional<blofeld::UsbDeviceProbe> &saved_probe) {
  constexpr unsigned maximum_polls = 1000;
  for (unsigned poll = 0; poll < maximum_polls; ++poll) {
    const auto cpu = session.machine.cpu().state();
    const bool ready = probe.poll(session.machine.bus(), {.pc = cpu.pc, .ccr = cpu.ccr});
    const bool capturing = !middle;
    drain_discovery(session, middle);
    if (capturing && middle) { saved_probe = probe; }
    if (ready) { return; }
  }
  throw blofeld::Fault("Original descriptor probe exceeded its fixed host-poll bound");
}

void check_probe_paths(Session &session,
                       const Saved &configured,
                       const blofeld::Firmware &firmware) {
  session.restore(configured);
  session.counters = {};
  blofeld::UsbDeviceProbe probe;
  std::optional<Saved> middle;
  std::optional<blofeld::UsbDeviceProbe> saved_probe;
  finish_probe(session, probe, middle, saved_probe);
  if (!middle || !saved_probe) {
    throw blofeld::Fault(
        "Successful discovery requires its actual pre-read native/host checkpoint");
  }
  check_descriptors(probe.descriptors(), firmware);
  auto expected = session.outcome();
  inspect_setup_payload(session, expected);
  session.trace_ << "PROBE_COMPLETE 0 " << session.counters.native << ' '
                 << session.counters.entries << ' ' << session.counters.returns << " 18 101\n";
  for (unsigned replay = 1; replay < 3; ++replay) {
    session.restore(*middle);
    probe = *saved_probe;
    auto retained = middle;
    auto host = saved_probe;
    finish_probe(session, probe, retained, host);
    check_descriptors(probe.descriptors(), firmware);
    auto actual = session.outcome();
    inspect_setup_payload(session, actual);
    require(actual == expected,
            "Both joint pre-read restores reproduce all CPU/RAM/USB/payload/write state");
    const auto cpu = session.machine.cpu().state();
    require(probe.poll(session.machine.bus(), {.pc = cpu.pc, .ccr = cpu.ccr}),
            "A completed descriptor probe remains ready without another transaction");
    auto unchanged = session.outcome();
    inspect_setup_payload(session, unchanged);
    require(unchanged == actual, "Ready probe queries cannot mutate the completed original MCU");
    session.trace_ << "PROBE_COMPLETE " << replay << ' ' << session.counters.native << ' '
                   << session.counters.entries << ' ' << session.counters.returns << " 18 101\n";
  }
}

void poll_discovery(Session &session,
                    blofeld::UsbipRuntime &runtime,
                    blofeld::UsbipRuntimeBoundary &boundary) {
  ++boundary.ticks; // Host-operation reference; not emulated USB clock ticks.
  const auto cpu = session.machine.cpu().state();
  static_cast<void>(runtime.poll(session.machine.bus(), {.pc = cpu.pc, .ccr = cpu.ccr}, boundary));
}

void check_runtime_retirement(Session &session,
                              blofeld::UsbipRuntime &runtime,
                              blofeld::UsbipRuntimeBoundary &boundary,
                              const Saved &middle) {
  namespace tcp = blofeld::test::usbip_tcp;
  auto pending = tcp::connect(runtime.port());
  poll_discovery(session, runtime, boundary);
  session.restore(middle);
  auto expected = session.outcome();
  inspect_setup_payload(session, expected);
  const auto evidence = session.machine.bus().usb_host_evidence();
  ++boundary.restore_revision;
  bool retired = false;
  try {
    poll_discovery(session, runtime, boundary);
  } catch (const blofeld::Fault &fault) {
    const std::string_view reason(fault.what());
    require(reason.contains("board owner/restore/time changed") &&
                reason.contains("mcu-pc=0xbc3d ccr=0x90"),
            "Restoring a real pre-read checkpoint retires the old transport with actual context");
    retired = true;
  }
  tcp::Received closed;
  constexpr unsigned maximum_close_polls = 64;
  for (unsigned attempt = 0; !closed.eof && attempt < maximum_close_polls; ++attempt) {
    tcp::read(pending, closed);
    if (!closed.eof) {
      pollfd ready{.fd = pending.get(), .events = POLLIN, .revents = 0};
      static_cast<void>(::poll(&ready, 1, 1));
    }
  }
  auto actual = session.outcome();
  inspect_setup_payload(session, actual);
  require(retired && runtime.faulted() && !runtime.published() && closed.eof &&
              closed.bytes.empty() && actual == expected &&
              session.machine.bus().usb_host_evidence() == evidence,
          "Retirement closes the accepted idle TCP peer and preserves every restored guest bit");
}

void check_runtime_paths(Session &session,
                         const Saved &configured,
                         const blofeld::Firmware &firmware) {
  std::optional<Outcome> expected;
  for (unsigned replay = 0; replay < 3; ++replay) {
    session.restore(configured);
    session.counters = {};
    const blofeld::UsbipRuntimeOptions options{
        .tcp = {},
        .location = {.path = "/blofeld-emu/usb/1-1", .busid = "1-1", .bus = 1, .device = 1},
        .after_ticks = 0,
        .period_ticks = 1,
        .probe_timeout_ticks = instruction_limit};
    blofeld::UsbipRuntime runtime(options);
    blofeld::UsbipRuntimeBoundary boundary{.owner = std::make_shared<const int>(0),
                                           .restore_revision = replay};
    std::optional<Saved> middle;
    constexpr unsigned maximum_polls = 1000;
    while (!runtime.published() && boundary.ticks < maximum_polls) {
      poll_discovery(session, runtime, boundary);
      drain_discovery(session, middle);
    }
    require(runtime.published() && !runtime.faulted(),
            "Only three completed original descriptor transfers may publish USB/IP metadata");
    if (!middle) {
      throw blofeld::Fault("Runtime discovery requires its actual pre-read checkpoint");
    }
    auto before = session.outcome();
    inspect_setup_payload(session, before);
    const auto reply = list_device(session, runtime, boundary);
    require(reply == expected_list(firmware),
            "Every real TCP DEVLIST byte matches independent USB/IP layout and original ROM");
    auto after = session.outcome();
    inspect_setup_payload(session, after);
    require(after == before, "TCP discovery cannot mutate native CPU/RAM/USB/payload/write state");
    if (expected) {
      require(after == *expected, "Three fresh runtime discoveries reproduce full original state");
    } else {
      expected = after;
    }
    session.trace_ << "USBIP_RUNTIME_COMPLETE " << replay << ' ' << session.counters.native << ' '
                   << session.counters.entries << ' ' << session.counters.returns << ' '
                   << reply.size() << " 1\n";
    check_runtime_retirement(session, runtime, boundary, *middle);
  }
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path trace;
};

void run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  std::ofstream trace(inputs.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  Session session(firmware, trace);
  const auto cold = session.save();
  std::optional<Saved> delayed;
  for (const auto profile : {Profile{.id = 0, .release_poll = 1},
                             Profile{.id = 1, .release_poll = 3},
                             Profile{.id = 2}}) {
    session.restore(cold);
    session.start(profile);
    auto middle = finish_reset(session, profile.id == 1);
    const auto expected = session.outcome();
    session.report("RESET");
    if (profile.id == 1) { delayed.emplace(session.save()); }
    if (middle) {
      for (unsigned replay = 0; replay < 2; ++replay) {
        session.restore(*middle);
        static_cast<void>(finish_reset(session, false));
        require(session.outcome() == expected,
                "Mid-ISR native/RAM/register/write-prefix replay must match completely");
        session.report("REPLAY");
      }
    }
  }
  if (!delayed) { throw blofeld::Fault("Released-reset checkpoint must be present"); }
  session.restore(*delayed);
  session.finish_configuration();
  check_suspend_paths(session);
  session.check_bulk_admission();
  const auto configured = session.save();
  check_setup_paths(session, firmware);
  check_probe_paths(session, configured, firmware);
  check_runtime_paths(session, configured, firmware);
  trace << "NATIVE_RETIREMENTS " << session.actual_retirements << '\n';
  std::cout
      << "Original cold MCU -> USB reset IRQ/configuration -> native RTI/main WAI verified; "
         "three real poll profiles, two mid-ISR replays and unchanged NVM. Original bulk MIDI "
         "packet admission retains raw NISOF and raises real completion IRQs. All ordinary/overrun "
         "SETUP profiles execute retained-byte reads, real data/status ACK and native "
         "completion IRQs/RTI/WAI. No full board, "
         "physical USB timing, host USB claim.\n";
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-usb-reset original.zip private.trace");
    }
    run({.firmware = arguments[1], .trace = arguments[2]});
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
