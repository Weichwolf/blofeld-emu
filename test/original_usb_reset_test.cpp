// SPDX-License-Identifier: GPL-3.0-only
#include "application_fixture.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_usb.hpp"
#include "blofeld/usb_device.hpp"
#include "blofeld/usb_host.hpp"
#include "blofeld/usb_transfer.hpp"
#include "blofeld/usbip.hpp"
#include "blofeld/usbip_runtime.hpp"
#include "lcd_fixture.hpp"
#include "usbip_fixture.hpp"
#include "usbip_session_fixture.hpp"
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
constexpr unsigned reset_poll = 0xbdf9;
constexpr unsigned reset_delay = 0xbdf1;
constexpr unsigned reset_return = 0xbe10;
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
    static_cast<void>(
        blofeld::test::start_application(machine.bus(), machine.cpu(), *display, trace, false));
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
  void reject_midi();

  void report(std::string_view kind) const {
    trace_ << kind << ' ' << profile.id << ' ' << counters.polls << ' ' << counters.delays << ' '
           << counters.entries << ' ' << counters.returns << ' ' << counters.native << '\n';
  }

  blofeld::McuMachine machine;
  std::unique_ptr<blofeld::test::LcdFixture> display;
  Profile profile;
  Counters counters;
  Registers interrupted{};

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
  if (result.instruction_executed) { ++counters.native; }
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
  if (before.pc == (vector == reset_vector ? reset_return : configuration_return)) {
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
    bool rejected = false;
    try {
      static_cast<void>(queue.poll(machine.bus(), context()));
    } catch (const blofeld::Fault &fault) {
      require(std::string_view(fault.what())
                  .contains("automatic configuration status completion after DONEECRU"),
              "Automatic status completion retains its specific unresolved boundary");
      const std::string_view reason = fault.what();
      require(reason.contains("seq=7 endpoint=0x00 phase=status") &&
                  reason.contains("mcu-pc=" + blofeld::hex(machine.cpu().state().pc)) &&
                  queue.faulted() && queue.pending() == 1,
              "Host transfer preserves sequence/phase/actual MCU context and poisons on fault");
      trace_ << "CONFIGURATION_HOST_FRONTIER " << reason << '\n';
      trace_ << "CONFIGURATION_STATUS_FRONTIER "
             << reason.substr(reason.find("Unqualified USB automatic configuration")) << '\n';
      rejected = true;
    }
    require(rejected && outcome() == actual,
            "Unknown status completion cannot mutate CPU/RAM/USB/NVM or synthesize host success");
    check_flash();
    report("CONFIGURATION");
  }
}

void Session::reject_midi() {
  const auto before = outcome();
  const std::array<std::uint8_t, 4> note_on{9, 0x90, 64, 127};
  for (const bool input : {false, true}) {
    bool rejected = false;
    try {
      if (input) {
        static_cast<void>(machine.bus().request_usb_in(blofeld::UsbBuffer::endpoint2));
      } else {
        static_cast<void>(machine.bus().receive_usb_out(
            {.buffer = blofeld::UsbBuffer::endpoint3, .data = std::span{note_on}}));
      }
    } catch (const blofeld::Fault &fault) {
      const std::string_view text(fault.what());
      require(text.contains("Reserved USB local endpoint NISOF field") &&
                  text.contains(input ? "physical-endpoint=2" : "physical-endpoint=3") &&
                  text.contains(blofeld::hex(input ? in_description : out_description, 8)),
              "Original MIDI endpoint descriptors retain the documented NISOF failure");
      trace_ << "MIDI_FRONTIER " << (input ? "IN " : "OUT ") << text << '\n';
      rejected = true;
    }
    require(
        rejected && outcome() == before && !machine.cpu().pending_interrupt(),
        "Rejected decoded MIDI transactions cannot mutate CPU/RAM/USB/NVM or invent completion");
  }
}

constexpr unsigned setup_vector = 0xffd2;
constexpr unsigned setup_entry = 0xbe21;
constexpr unsigned setup_return = 0xbe3b;
constexpr unsigned setup_callback = 0xbc38;
constexpr unsigned setup_fault_pc = 0xbc3d;
constexpr unsigned status_word = 0x302;
constexpr unsigned control_in_word = 0x314;
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

void start_setup(Session &session,
                 const SetupProfile profile,
                 std::optional<blofeld::UsbDeviceProbe> &probe) {
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
    const auto state = session.machine.cpu().state();
    auto &capture = probe.emplace();
    require(!capture.poll(session.machine.bus(), {.pc = state.pc, .ccr = state.ccr}) &&
                !capture.ready(),
            "The production descriptor probe submits its real first GET_DESCRIPTOR");
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
              session.word(status_word) == setup_usb_status,
          "The original overrun callback queues the clamped device descriptor and retains SETUP");
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
  require(selected && selected->vector == setup_vector,
          "Retained SETUP wins over the genuine lower-priority control-IN completion");
}

void check_setup_step(Session &session,
                      const blofeld::CpuState before,
                      const blofeld::CpuStep result) {
  if (result.instruction_executed) { ++session.counters.native; }
  if (result.interrupt_vector) {
    require(*result.interrupt_vector == setup_vector && result.stack_reused &&
                session.counters.entries < 2,
            "Original SETUP entry and chained RTI reuse the actual interrupted frame");
    if (session.counters.entries == 0) {
      require(!result.instruction_executed && before.pc == main_resume && before.sp == wait_stack,
              "The first SETUP interrupt enters directly from original main WAI");
    } else {
      require(result.instruction_executed && before.pc == setup_return,
              "The second SETUP source chains only through the actual original RTI");
      ++session.counters.returns;
    }
    auto expected = session.interrupted;
    expected[0] = setup_entry;
    expected[5] |= 0x10U;
    require(registers(session.machine.cpu().state()) == expected,
            "Native interrupt entry restores the original interrupted registers and sets I");
    ++session.counters.entries;
  }
}

std::optional<Saved> finish_setup(Session &session,
                                  const blofeld::Firmware &firmware,
                                  const SetupProfile profile,
                                  const bool capture) {
  std::optional<Saved> middle;
  bool packet = false;
  while (session.counters.native < instruction_limit) {
    const auto before = session.machine.cpu().state();
    if (capture && !middle && before.pc == setup_callback) {
      require(session.word(status_word) == (profile.overrun ? setup_usb_status : idle_usb_status) &&
                  before.d == setup_source && before.sp == wait_stack - 2,
              "The snapshot follows the real flag acknowledgement and JSR, before callback reads");
      middle.emplace(session.save());
    }
    if (before.pc == setup_return) {
      require(profile.overrun && !packet, "Only the overrun path reaches its first original RTI");
      check_device_reply(session, firmware, profile);
      packet = true;
    }
    try {
      check_setup_step(session, before, session.machine.cpu().step());
    } catch (const blofeld::Fault &fault) {
      const std::string_view reason(fault.what());
      require(before.pc == setup_fault_pc && reason.contains("MCU read at 0x033f") &&
                  reason.contains("pc=0xbc3d ccr=0x90") &&
                  reason.contains("USB setup-buffer contents are not valid"),
              "Only the actual post-acknowledgement original USTB read is the expected frontier");
      require(registers(session.machine.cpu().state()) == registers(before) &&
                  before.d == setup_source && before.x == setup_source &&
                  before.y == session.interrupted[3] &&
                  before.sp == wait_stack - callback_frame_bytes && before.ccr == fault_ccr &&
                  session.word(status_word) == idle_usb_status &&
                  session.counters.entries == (profile.overrun ? 2U : 1U) &&
                  session.counters.returns == (profile.overrun ? 1U : 0U) &&
                  packet == profile.overrun,
              "Unknown setup data cannot commit a fabricated CPU byte or skip the real IRQ path");
      if (!profile.overrun) {
        require(session.counters.native == 9,
                "The ordinary path retires exactly its nine original pre-read instructions");
      }
      session.check_flash();
      return middle;
    }
  }
  throw blofeld::Fault("Original SETUP must return or reach its explicit invalid-data boundary");
}

void check_incomplete_probe(Session &session, std::optional<blofeld::UsbDeviceProbe> &probe) {
  if (!probe) { return; }
  auto before = session.outcome();
  inspect_setup_payload(session, before);
  const auto state = session.machine.cpu().state();
  require(!probe->poll(session.machine.bus(), {.pc = state.pc, .ccr = state.ccr}) &&
              !probe->ready(),
          "Original USTB failure leaves descriptor capture pending without inventing bytes");
  bool unavailable = false;
  try {
    static_cast<void>(probe->descriptors());
  } catch (const blofeld::Fault &fault) {
    require(std::string_view(fault.what()).contains("USB descriptor capture is unavailable"),
            "No original descriptor metadata can escape the incomplete transfer");
    unavailable = true;
  }
  auto after = session.outcome();
  inspect_setup_payload(session, after);
  require(unavailable && before == after,
          "Incomplete metadata queries preserve every native register, RAM and USB payload bit");
  session.check_flash();
  session.trace_ << "DESCRIPTOR_PROBE_UNAVAILABLE " << session.counters.native << ' ' << state.pc
                 << ' ' << static_cast<unsigned>(state.ccr) << '\n';
}

void poll_runtime(Session &session,
                  blofeld::UsbipRuntime &runtime,
                  blofeld::UsbipRuntimeBoundary &boundary) {
  const auto cpu = session.machine.cpu().state();
  static_cast<void>(runtime.poll(session.machine.bus(), {.pc = cpu.pc, .ccr = cpu.ccr}, boundary));
  // Explicit operation-count reference, not a clocked board/USB timing claim.
  ++boundary.ticks;
}

Saved runtime_setup_fault(Session &session,
                          blofeld::UsbipRuntime &runtime,
                          blofeld::UsbipRuntimeBoundary &boundary) {
  std::optional<Saved> before_fault;
  constexpr unsigned maximum_operations = 16;
  for (unsigned operation = 0; operation < maximum_operations; ++operation) {
    poll_runtime(session, runtime, boundary);
    if (operation == 0) {
      const auto expected = setup_request(device_descriptor_size);
      for (unsigned i = 0; i < expected.size(); ++i) {
        require(session.machine.bus().read(static_cast<std::uint16_t>(setup_source + i)) ==
                    expected[i],
                "The runtime submits every independently specified ordinary SETUP byte");
      }
      require(session.word(status_word) == setup_usb_status,
              "Runtime admission establishes the real original SETUP flag");
    }
    const auto before = session.machine.cpu().state();
    if (before.pc == setup_fault_pc) { before_fault.emplace(session.save()); }
    try {
      check_setup_step(session, before, session.machine.cpu().step());
    } catch (const blofeld::Fault &fault) {
      const std::string_view reason(fault.what());
      if (!before_fault) {
        throw blofeld::Fault("Original runtime fault lacks its pre-read state");
      }
      require(before.pc == setup_fault_pc && session.counters.native == 9 &&
                  session.counters.entries == 1 && before.ccr == fault_ccr &&
                  registers(session.machine.cpu().state()) == registers(before) &&
                  reason.contains("MCU read at 0x033f") && reason.contains("pc=0xbc3d ccr=0x90") &&
                  reason.contains("USB setup-buffer contents are not valid"),
              "Only the unchanged original nine-instruction USTB frontier qualifies this runtime");
      return *before_fault;
    }
  }
  throw blofeld::Fault("Original runtime SETUP did not reach its independently expected failure");
}

void runtime_empty_list(Session &session,
                        blofeld::UsbipRuntime &runtime,
                        blofeld::UsbipRuntimeBoundary &boundary) {
  namespace tcp = blofeld::test::usbip_tcp;
  auto client = tcp::connect(runtime.port());
  tcp::send(client, blofeld::test::usbip_session::list);
  tcp::Received received;
  constexpr unsigned maximum_polls = 64;
  for (unsigned attempt = 0; !received.eof && attempt < maximum_polls; ++attempt) {
    poll_runtime(session, runtime, boundary);
    tcp::read(client, received);
    if (!received.eof) {
      pollfd ready{.fd = client.get(), .events = POLLIN, .revents = 0};
      static_cast<void>(::poll(&ready, 1, 1));
    }
  }
  constexpr std::array<std::uint8_t, 12> empty_list{1, 0x11, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0};
  require(
      received.eof && std::ranges::equal(received.bytes, empty_list) && runtime.started() &&
          !runtime.published() && !runtime.faulted(),
      "A real TCP DEVLIST after the original fault returns exactly zero devices and no identity");
}

void check_runtime_paths(Session &session, const Saved &configured) {
  namespace tcp = blofeld::test::usbip_tcp;
  std::optional<Outcome> expected;
  for (unsigned replay = 0; replay < 3; ++replay) {
    session.restore(configured);
    session.counters = {};
    session.interrupted = registers(session.machine.cpu().state());
    const blofeld::UsbipRuntimeOptions options{
        .tcp = {},
        .location = {.path = "/blofeld-emu/usb/1-1", .busid = "1-1", .bus = 1, .device = 1},
        .after_ticks = 0,
        .period_ticks = 1,
        .probe_timeout_ticks = instruction_limit};
    blofeld::UsbipRuntime runtime(options);
    blofeld::UsbipRuntimeBoundary boundary{.owner = std::make_shared<const int>(0),
                                           .restore_revision = replay};
    const auto middle = runtime_setup_fault(session, runtime, boundary);
    auto before = session.outcome();
    inspect_setup_payload(session, before);
    const auto evidence = session.machine.bus().usb_host_evidence();
    runtime_empty_list(session, runtime, boundary);
    auto after = session.outcome();
    inspect_setup_payload(session, after);
    require(
        before == after && session.machine.bus().usb_host_evidence() == evidence,
        "Post-fault TCP/probe polling preserves CPU, all RAM, USB payload knowledge and evidence");
    if (expected) {
      require(after == *expected,
              "Three fresh runtime attempts reproduce the complete original failure state");
    } else {
      expected = after;
    }
    auto pending = tcp::connect(runtime.port());
    poll_runtime(session, runtime, boundary);
    session.restore(middle);
    ++boundary.restore_revision;
    bool retired = false;
    try {
      poll_runtime(session, runtime, boundary);
    } catch (const blofeld::Fault &fault) {
      const std::string_view reason(fault.what());
      require(
          reason.contains("board owner/restore/time changed") &&
              reason.contains("mcu-pc=0xbc3d ccr=0x90"),
          "A trusted native restore invalidates its old live transport with actual MCU context");
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
    require(retired && runtime.faulted() && !runtime.published() && closed.eof &&
                closed.bytes.empty(),
            "Restore retirement closes an accepted idle TCP peer without publishing any bytes");
    auto restored = session.outcome();
    inspect_setup_payload(session, restored);
    require(restored == after && session.machine.bus().usb_host_evidence() == evidence,
            "Transport retirement cannot mutate the restored native/peripheral state");
    session.check_flash();
    session.trace_ << "USBIP_RUNTIME_REFUSAL " << replay << ' ' << session.counters.native << ' '
                   << session.machine.cpu().state().pc << ' '
                   << static_cast<unsigned>(session.machine.cpu().state().ccr) << " 12 1\n";
  }
}

void check_setup_paths(Session &session, const blofeld::Firmware &firmware) {
  const auto configured = session.save();
  constexpr unsigned maximum_length = 0xffff;
  constexpr std::array<unsigned, 7> lengths{0, 1, 8, 9, 17, 18, maximum_length};
  for (unsigned index = 0; index <= lengths.size(); ++index) {
    const SetupProfile profile{.length = index == 0 ? device_descriptor_size : lengths[index - 1],
                               .overrun = index != 0};
    session.restore(configured);
    std::optional<blofeld::UsbDeviceProbe> probe;
    start_setup(session, profile, probe);
    const auto pending_probe = probe;
    const auto middle = finish_setup(session, firmware, profile, true);
    if (!middle) {
      throw blofeld::Fault("Each original SETUP branch requires its real pre-read snapshot");
    }
    check_incomplete_probe(session, probe);
    auto expected = session.outcome();
    inspect_setup_payload(session, expected);
    const auto report = [&](const std::string_view kind) {
      session.trace_ << kind << ' ' << profile.overrun << ' ' << profile.length << ' '
                     << session.counters.native << ' ' << session.counters.entries << ' '
                     << session.counters.returns << ' '
                     << (profile.overrun ? std::min(profile.length, device_descriptor_size) : 0)
                     << '\n';
    };
    report("SETUP_CASE");
    for (unsigned replay = 0; replay < 2; ++replay) {
      session.restore(*middle);
      probe = pending_probe;
      static_cast<void>(finish_setup(session, firmware, profile, false));
      check_incomplete_probe(session, probe);
      auto actual = session.outcome();
      inspect_setup_payload(session, actual);
      require(actual == expected,
              "Both pre-read joint restores retain CPU, all RAM, USB controls/payload knowledge "
              "and actual write prefix through the same qualified frontier");
      report("SETUP_REPLAY");
    }
  }
  check_runtime_paths(session, configured);
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
  session.reject_midi();
  check_setup_paths(session, firmware);
  std::cout << "Original cold MCU -> USB reset IRQ/configuration -> native RTI/main WAI verified; "
               "three real poll profiles, two mid-ISR replays and unchanged NVM. Original MIDI "
               "transactions still reject their reserved NISOF descriptor fields. Ordinary SETUP "
               "keeps its post-ack invalid-USTB boundary; overrun paths first queue genuine device "
               "descriptors and chain back to the same boundary. No full board, "
               "physical USB timing, host USB or fuzzing claim.\n";
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
