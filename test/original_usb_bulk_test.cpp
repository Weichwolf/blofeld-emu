// SPDX-License-Identifier: GPL-3.0-only
#include "application_fixture.hpp"
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "storage/flash_image.hpp"
#include "usb/usb_host.hpp"
#include "usb/usb_transfer.hpp"
#include "usb/usbip_runtime.hpp"
#include "usbip_tcp_fixture.hpp"

#include <algorithm>
#include <array>
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
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned instruction_limit = 100000;
constexpr unsigned main_resume = 0x8315;
constexpr unsigned reset_poll = 0xbdf9;
constexpr unsigned flash_begin = 0x8000;
constexpr unsigned device_source = 0xfe49;
constexpr unsigned device_bytes = 18;
constexpr unsigned configuration_source = device_source + device_bytes;
constexpr unsigned configuration_bytes = 101;
using Bytes = std::vector<std::uint8_t>;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

struct Instrument {
  Instrument(const Firmware &firmware, std::ostream &trace)
      : machine(McuFlashImage::from_known(firmware.mcu), BLOFELD_DEFAULT_SLA),
        display(std::make_unique<test::LcdFixture>(machine.bus())),
        initial(machine.bus().power_off_flash_image()) {
    static_cast<void>(test::start_application(machine, display, trace, false));
    machine.bus().set_observer([this](const BusEvent &event) {
      display->observe(event);
      observe_peer(event);
    });
  }

  void begin_peer_capture() {
    const auto pin = machine.bus().observe_pads(McuPort::j, 1);
    require(pin.known == 1, "Native peer strobe must be known before passive capture");
    last_strobe = pin.levels != 0;
  }

  void observe_peer(const BusEvent &event) {
    if (!last_strobe || !event.write || event.address != hc12::ptj) { return; }
    const auto pin = machine.bus().observe_pads(McuPort::j, 1);
    require(pin.known == 1, "Actual native strobe writes retain known GPIO levels");
    const bool high = pin.levels != 0;
    if (high && !*last_strobe) {
      constexpr unsigned all_pins = 255;
      const auto data = machine.bus().observe_pads(McuPort::q, all_pins);
      require(data.known == all_pins && machine.bus().read(hc12::ddrq) == all_pins,
              "Native USB forwarding must drive every captured Port Q payload bit");
      constexpr unsigned observation_limit = 16;
      require(peer_bytes.size() < observation_limit, "Passive byte capture has a fixed bound");
      peer_bytes.push_back(data.levels);
    }
    last_strobe = high;
  }

  [[nodiscard]] ExecutionContext context() const {
    const auto state = machine.cpu().state();
    return {.pc = state.pc, .ccr = state.ccr};
  }

  void drain(const bool release_reset = false) {
    unsigned attempts = 0;
    while (!machine.cpu().waiting() || machine.cpu().pending_interrupt()) {
      require(++attempts < instruction_limit, "Original ISR/foreground exceeded its step bound");
      if (release_reset && machine.cpu().state().pc == reset_poll) {
        machine.bus().observe_usb_decoder({});
      }
      const auto result = machine.cpu().step();
      if (result.instruction_executed) { ++retired; }
      if (result.interrupt_vector) { ++interrupts; }
    }
    require(machine.cpu().state().pc == main_resume,
            "Original IRQ/RTI must return to actual foreground WAI");
  }

  void configure() {
    machine.bus().observe_usb_decoder({.reset = true});
    drain(true);
    machine.bus().observe_usb_decoder({.enumerated = true});
    constexpr std::array<std::uint8_t, 8> setup{0, 9, 1, 0, 0, 0, 0, 0};
    UsbHostQueue queue;
    queue.submit({.sequence = 1,
                  .address = 0,
                  .length = 0,
                  .setup = setup,
                  .output = {},
                  .short_not_ok = false,
                  .zero_packet = false});
    require(queue.poll(machine.bus(), context()).empty() &&
                queue.poll(machine.bus(), context()).empty(),
            "SET_CONFIGURATION status must NAK until the original configuration ISR");
    drain();
    const auto complete = queue.poll(machine.bus(), context());
    require(complete.size() == 1 && complete.front().status == UsbTransferStatus::success &&
                complete.front().transferred == 0 && queue.pending() == 0,
            "Only the original configuration acknowledgement completes the actual status packet");
    drain();
  }

  void check_flash() const {
    const auto now = machine.bus().power_off_flash_image();
    require(now.bytes == initial.bytes && now.known == initial.known && now.fresh == initial.fresh,
            "All original MCU flash bytes, knowledge and erase history must remain unchanged");
  }

  McuMachine machine;
  std::unique_ptr<test::LcdFixture> display;
  McuFlashImage initial;
  unsigned retired = 0;
  unsigned interrupts = 0;
  std::optional<bool> last_strobe;
  Bytes peer_bytes;
};

Bytes expected_list(const Firmware &firmware) {
  // Independent USB/IP OP_REP_DEVLIST layout: 12-byte prefix, 312-byte device,
  // then four bytes per active interface. Identity fields come only from ROM.
  constexpr unsigned reply_prefix = 12;
  constexpr unsigned record_bytes = 312;
  constexpr unsigned interface_bytes = 4;
  constexpr unsigned path_bytes = 256;
  constexpr unsigned location_bytes = 288;
  constexpr unsigned identity_offset = 300;
  const std::span<const std::uint8_t> rom(firmware.mcu);
  const auto device = rom.subspan(device_source - flash_begin, device_bytes);
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

Bytes list_device(Instrument &instrument, UsbipRuntime &runtime, UsbipRuntimeBoundary &boundary) {
  namespace tcp = test::usbip_tcp;
  auto client = tcp::connect(runtime.port());
  constexpr unsigned usbip_version_low = 0x11;
  constexpr std::array<std::uint8_t, 8> request{1, usbip_version_low, 0x80, 5, 0, 0, 0, 0};
  tcp::send(client, request);
  tcp::Received received;
  constexpr unsigned maximum_polls = 64;
  for (unsigned attempt = 0; !received.eof && attempt < maximum_polls; ++attempt) {
    ++boundary.ticks;
    static_cast<void>(runtime.poll(instrument.machine.bus(), instrument.context(), boundary));
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

struct WireLink {
  Instrument &instrument;
  UsbipRuntime &runtime;
  UsbipRuntimeBoundary &boundary;
};

struct WireWord {
  unsigned offset;
  std::uint32_t value;
};

void put_word(Bytes &bytes, const WireWord word) {
  constexpr unsigned byte_bits = 8;
  constexpr unsigned byte_mask = 255;
  for (unsigned i = 0; i < 4; ++i) {
    bytes.at(word.offset + i) =
        static_cast<std::uint8_t>((word.value >> ((3U - i) * byte_bits)) & byte_mask);
  }
}

Bytes receive_wire(WireLink &link, const test::usbip_tcp::Socket &client, const unsigned length) {
  test::usbip_tcp::Received received;
  constexpr unsigned maximum_polls = 64;
  for (unsigned attempt = 0; received.bytes.size() < length && attempt < maximum_polls; ++attempt) {
    ++link.boundary.ticks;
    static_cast<void>(
        link.runtime.poll(link.instrument.machine.bus(), link.instrument.context(), link.boundary));
    link.instrument.drain();
    test::usbip_tcp::read(client, received);
    require(!received.eof, "Imported native USB session cannot close before its expected reply");
    if (received.bytes.size() < length) {
      pollfd ready{.fd = client.get(), .events = POLLIN, .revents = 0};
      static_cast<void>(::poll(&ready, 1, 1));
    }
  }
  require(received.bytes.size() == length, "Actual TCP reply must match the exact wire extent");
  return received.bytes;
}

test::usbip_tcp::Socket import_device(WireLink &link, const Firmware &firmware) {
  auto client = test::usbip_tcp::connect(link.runtime.port());
  constexpr unsigned version_low = 0x11;
  constexpr unsigned import_request_bytes = 40;
  constexpr unsigned import_reply_bytes = 320;
  Bytes request(import_request_bytes, 0);
  request[0] = 1;
  request[1] = version_low;
  request[2] = 0x80;
  request[3] = 3;
  constexpr std::string_view busid = "1-1";
  std::ranges::copy(busid, request.begin() + 8);
  test::usbip_tcp::send(client, request);
  const auto actual = receive_wire(link, client, import_reply_bytes);
  Bytes expected{1, version_low, 0, 3, 0, 0, 0, 0};
  const auto list = expected_list(firmware);
  constexpr unsigned list_prefix_bytes = 12;
  constexpr unsigned device_record_bytes = 312;
  for (unsigned i = 0; i < device_record_bytes; ++i) {
    expected.push_back(list.at(list_prefix_bytes + i));
  }
  require(actual == expected, "Every real IMPORT reply byte must match protocol and original ROM");
  return client;
}

struct BulkRequest {
  unsigned sequence;
  bool input;
  unsigned length;
  Bytes payload;
};

void bulk_request(WireLink &link,
                  const test::usbip_tcp::Socket &client,
                  const BulkRequest &request) {
  // Independent Linux USB/IP CMD_SUBMIT/RET_SUBMIT header fields.
  constexpr unsigned header_bytes = 48;
  constexpr unsigned device_id = 0x00010001;
  constexpr unsigned input_direction_offset = 12;
  constexpr unsigned endpoint_offset = 16;
  constexpr unsigned length_offset = 24;
  Bytes submit(header_bytes, 0);
  put_word(submit, {.offset = 0, .value = 1});
  put_word(submit, {.offset = 4, .value = request.sequence});
  put_word(submit, {.offset = 8, .value = device_id});
  put_word(submit, {.offset = input_direction_offset, .value = request.input ? 1U : 0U});
  put_word(submit, {.offset = endpoint_offset, .value = 1});
  put_word(submit, {.offset = length_offset, .value = request.length});
  submit.insert(submit.end(), request.payload.begin(), request.payload.end());
  test::usbip_tcp::send(client, submit);
  const auto actual = receive_wire(link, client, header_bytes);
  Bytes expected(header_bytes, 0);
  put_word(expected, {.offset = 0, .value = 3});
  put_word(expected, {.offset = 4, .value = request.sequence});
  const auto transferred = request.input ? 0U : request.length;
  put_word(expected, {.offset = length_offset, .value = transferred});
  require(actual == expected, "Actual bulk completion must have exact sequence, status and extent");
}

void check_bulk(WireLink &link, const Firmware &firmware, std::ostream &trace) {
  auto client = import_device(link, firmware);
  link.instrument.begin_peer_capture();
  const auto before = link.instrument.retired;
  const auto interrupts = link.instrument.interrupts;
  constexpr std::uint8_t note_on = 0x90;
  constexpr std::uint8_t note_off = 0x80;
  constexpr std::uint8_t note = 0x45;
  constexpr std::uint8_t velocity = 100;
  constexpr unsigned first_sequence = 42;
  constexpr unsigned maximum_packet = 64;
  bulk_request(link,
               client,
               {.sequence = first_sequence,
                .input = false,
                .length = 4,
                .payload = {9, note_on, note, velocity}});
  bulk_request(link,
               client,
               {.sequence = first_sequence + 1,
                .input = false,
                .length = 4,
                .payload = {8, note_off, note, 0}});
  bulk_request(
      link,
      client,
      {.sequence = first_sequence + 2, .input = true, .length = maximum_packet, .payload = {}});
  const std::array<std::uint8_t, 6> expected{note_on, note, velocity, note_off, note, 0};
  require(std::ranges::equal(link.instrument.peer_bytes, expected),
          "Original OUT ISR must forward precisely the six actual MIDI bytes on native GPIO");
  require(link.instrument.interrupts - interrupts == 3,
          "Two real OUT completions and one initial IN ZLP must execute three original IRQs");
  trace << "USBIP_BULK " << link.instrument.retired - before << ' '
        << link.instrument.interrupts - interrupts << ' ' << link.instrument.peer_bytes.size()
        << " 2 1\n";
  link.runtime.retire();
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path trace;
};

void run(const Inputs &inputs) {
  const auto firmware = Firmware::load(inputs.firmware);
  std::ofstream trace(inputs.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  Instrument instrument(firmware, trace);
  instrument.configure();
  const UsbipRuntimeOptions options{
      .tcp = {},
      .location = {.path = "/blofeld-emu/usb/1-1", .busid = "1-1", .bus = 1, .device = 1},
      .after_ticks = 0,
      .period_ticks = 1,
      .probe_timeout_ticks = instruction_limit};
  UsbipRuntime runtime(options);
  UsbipRuntimeBoundary boundary{.owner = std::make_shared<const int>(0)};
  constexpr unsigned maximum_host_polls = 1000;
  const auto before = instrument.retired;
  const auto interrupts = instrument.interrupts;
  while (!runtime.published() && boundary.ticks < maximum_host_polls) {
    ++boundary.ticks;
    static_cast<void>(runtime.poll(instrument.machine.bus(), instrument.context(), boundary));
    instrument.drain();
  }
  require(runtime.published() && !runtime.faulted(),
          "Only complete native descriptor transfers may publish the original device");
  const auto polls = boundary.ticks;
  const auto reply = list_device(instrument, runtime, boundary);
  require(reply == expected_list(firmware),
          "Every real DEVLIST byte must match independent protocol layout and original ROM");
  const auto discovery_retired = instrument.retired - before;
  const auto discovery_interrupts = instrument.interrupts - interrupts;
  WireLink link{.instrument = instrument, .runtime = runtime, .boundary = boundary};
  check_bulk(link, firmware, trace);
  instrument.check_flash();
  const std::filesystem::path reply_path(inputs.trace.string() + ".devlist");
  std::ofstream output(reply_path, std::ios::binary);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output.write(reinterpret_cast<const char *>(reply.data()),
               static_cast<std::streamsize>(reply.size()));
  trace << "RUNTIME_DISCOVERY " << discovery_retired << ' ' << discovery_interrupts << ' ' << polls
        << ' ' << reply.size() << '\n';
  std::cout << "Private bulk hypothesis: original MCU USB discovery/import, two OUT notes, "
               "six native GPIO payload bytes and initial IN ZLP verified. "
               "Conditional untimed MCU/peer/LCD fixture; poll ticks are host operation counts. "
               "No coupled boot, kernel attachment, DSP voice/audio or physical timing claim.\n";
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) { throw Fault("Expected original firmware ZIP and private trace path"); }
    run({.firmware = arguments[1], .trace = arguments[2]});
    return 0;
  } catch (const std::exception &failure) {
    std::cerr << failure.what() << '\n';
    return 1;
  }
}
