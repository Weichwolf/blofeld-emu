// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
// Independent USB2.0 9.4.2/9.4.4 and USB20D6E2F 3.1.9/3.1.10/4.2.5.1.
constexpr unsigned selector = 0x30e, config_data = 0x310, word_mask = 0xffff;
constexpr unsigned irq_mask = 0x304, configuration_status = 0x30c, status_low = 0x303;
constexpr unsigned endpoint_in = 0x314, endpoint_out = 0x316, setup_buffer = 0x338;
constexpr unsigned local_buffer = 0x380, byte_mask = 0xff;
constexpr unsigned valid = 0x4000, complete = 0x2000, interrupt = 0x1000;
constexpr unsigned payload_sentinel = 0x91;
constexpr unsigned decoded_address = 42;
constexpr unsigned module_control = 0xc000, setup_configuration_enable = 0x2003;
constexpr unsigned configuration_counts = 0x442;
constexpr std::array<std::uint8_t, 8> get_configuration{0x80, 8, 0, 0, 0, 0, 1, 0};
constexpr std::array<std::uint8_t, 8> get_interface{0x81, 10, 0, 0, 0, 0, 1, 0};
constexpr std::array<std::uint8_t, 8> vendor{0x40, 0x1f, 0, 0, 0, 0, 0, 0};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

void rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &) { return; }
  throw Fault("Expected visible automatic USB read refusal");
}

unsigned word(const McuUsb &usb, const unsigned address) {
  return (static_cast<unsigned>(usb.read(static_cast<std::uint16_t>(address))) << 8U) |
         usb.read(static_cast<std::uint16_t>(address + 1));
}

void write_word(McuUsb &usb, const std::uint16_t address, const unsigned value) {
  usb.write({.address = address, .value = static_cast<std::uint8_t>(value >> 8U)});
  usb.write({.address = static_cast<std::uint16_t>(address + 1),
             .value = static_cast<std::uint8_t>(value & byte_mask)});
}

struct Configuration {
  std::uint8_t index = 0;
  std::uint32_t value = 0;
};

void config(McuUsb &usb, const Configuration config) {
  usb.write({.address = selector, .value = config.index});
  usb.write_memory({.address = config_data, .size = 2, .value = config.value >> 16U});
  usb.write_memory({.address = config_data + 2, .size = 2, .value = config.value & word_mask});
}

void address(McuUsb &usb, const std::uint8_t value) {
  const std::array<std::uint8_t, 8> setup{0, 5, value, 0, 0, 0, 0, 0};
  require(usb.receive_setup(setup) == UsbResponse::ack, "Actual SET_ADDRESS ACK");
  const auto packet = usb.request_in(UsbBuffer::control_in);
  require(packet.response == UsbResponse::data && packet.size == 0 && packet.data1,
          "Address status is actual DATA1/ZLP");
  usb.finish_in(true);
  usb.write({.address = endpoint_in, .value = 0x20});
}

McuUsb start(const std::uint32_t counts = configuration_counts) {
  McuUsb usb;
  constexpr unsigned default_endpoint = 0x02000000;
  constexpr unsigned alternate_counts = 0x33333333;
  config(usb, {.index = 1, .value = default_endpoint});
  config(usb, {.index = 8, .value = counts});
  config(usb, {.index = 9, .value = alternate_counts});
  usb.observe_decoder({});
  usb.set_control(module_control);
  usb.observe_decoder({.enumerated = true});
  write_word(usb, irq_mask, setup_configuration_enable);
  usb.write({.address = local_buffer, .value = payload_sentinel});
  address(usb, decoded_address);
  return usb;
}

void select(McuUsb &usb, const unsigned configuration) {
  const std::array<std::uint8_t, 8> setup{
      0, 9, static_cast<std::uint8_t>(configuration), 0, 0, 0, 0, 0};
  require(usb.receive_setup(setup) == UsbResponse::ack, "Actual SET_CONFIGURATION ACK");
  usb.write({.address = configuration_status, .value = 0x10});
  const auto packet = usb.request_in(UsbBuffer::control_in);
  require(packet.response == UsbResponse::data && packet.size == 0 && packet.data1,
          "Real configuration status is DATA1/ZLP");
  usb.finish_in(true);
  usb.write({.address = endpoint_in, .value = 0x20});
}

void alternate(McuUsb &usb, const unsigned interface, const unsigned setting) {
  const std::array<std::uint8_t, 8> setup{
      1, 11, static_cast<std::uint8_t>(setting), 0, static_cast<std::uint8_t>(interface), 0, 0, 0};
  require(usb.receive_setup(setup) == UsbResponse::ack, "Actual SET_INTERFACE ACK");
  usb.write({.address = configuration_status, .value = 0x10});
  const auto packet = usb.request_in(UsbBuffer::control_in);
  require(packet.response == UsbResponse::data && packet.size == 0 && packet.data1,
          "Real interface status is DATA1/ZLP");
  usb.finish_in(true);
  usb.write({.address = endpoint_in, .value = 0x20});
}

void data(McuUsb &usb, const unsigned expected) {
  const auto result = usb.request_in(UsbBuffer::control_in);
  require(result.response == UsbResponse::data && result.size == 1 && result.data1 &&
              result.data[0] == expected,
          "Hardware read returns independently expected one-byte DATA1,without FIFO contents");
}

struct Case {
  unsigned configuration = 0;
  unsigned interface = 0;
  unsigned setting = 0;
  unsigned size = 0;
  unsigned enables = 0;
  bool read_interface = false;
  bool data_valid = false;
};

void complete_read(McuUsb &usb, const Case test, const unsigned before) {
  const auto in_enable = (test.enables & 1U) != 0;
  const auto out_enable = (test.enables & 2U) != 0;
  const auto expected = test.read_interface ? test.setting : test.configuration;
  for (unsigned retry = 0; retry < 3; ++retry) {
    data(usb, expected);
    data(usb, expected);
    require(word(usb, endpoint_in) == before && usb.interrupt_sources() == 0,
            "No ACK means no endpoint completion or IRQ");
    rejects([&] { usb.write({.address = local_buffer, .value = 0}); });
    rejects([&] { write_word(usb, endpoint_in, 0); });
    usb.finish_in(false);
    usb.observe_sof(static_cast<std::uint16_t>(retry));
  }
  data(usb, expected);
  usb.finish_in(true);
  const auto completed = (before & ~valid) | complete;
  require(word(usb, endpoint_in) == completed &&
              usb.interrupt_sources() == (in_enable ? 0x10U : 0U),
          "Only data ACK clears DVALID,sets TFRC and raises enabled IN completion");
  require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak,
          "Acknowledged one-byte read cannot replay its data phase");
  rejects([&] { usb.finish_in(true); });
  const auto after_data = usb;
  for (unsigned replay = 0; replay < 2; ++replay) {
    usb = after_data;
    const UsbOutPacket status{.data = {}, .data1 = true};
    require(usb.receive_out(status) == UsbResponse::ack, "Actual DATA1/ZLP completes status OUT");
    const auto out = valid | complete | (out_enable ? interrupt : 0U);
    require(word(usb, endpoint_out) == out && word(usb, endpoint_in) == completed &&
                usb.interrupt_sources() == ((in_enable ? 0x10U : 0U) | (out_enable ? 0x20U : 0U)),
            "General OUT completion preserves exact size/valid/IRQ rules");
    require(usb.receive_out(status) == UsbResponse::ack && word(usb, endpoint_out) == out,
            "A lost status ACK produces only duplicate acknowledgement");
    require(usb.read(local_buffer) == payload_sentinel,
            "Automatic data/status never manufactures or overwrites FIFO payload bytes");
    rejects([&] { static_cast<void>(usb.read(setup_buffer)); });
  }
}

void matrix(const Case test) {
  auto usb = start();
  select(usb, test.configuration);
  if (test.read_interface) { alternate(usb, test.interface, test.setting); }
  auto request = test.read_interface ? get_interface : get_configuration;
  if (test.read_interface) { request[4] = static_cast<std::uint8_t>(test.interface); }
  const auto configuration = word(usb, configuration_status);
  require(usb.receive_setup(request) == UsbResponse::ack && usb.interrupt_sources() == 0 &&
              word(usb, configuration_status) == configuration && (usb.read(status_low) & 3U) == 0,
          "Automatic reads leave USTB/SETUP/SETOVR/SETECR and selections alone");
  const auto before =
      ((test.enables & 1U) != 0 ? interrupt : 0U) | (test.data_valid ? valid : 0U) | test.size;
  write_word(usb, endpoint_in, before);
  write_word(usb, endpoint_out, (test.enables & 2U) != 0 ? interrupt : 0U);
  rejects([&] { static_cast<void>(usb.receive_out({.data = {}, .data1 = true})); });
  data(usb, test.read_interface ? test.setting : test.configuration);
  const auto pending = usb;
  for (unsigned replay = 0; replay < 2; ++replay) {
    usb = pending;
    complete_read(usb, test, before);
  }
}

unsigned malformed() {
  unsigned count = 0;
  for (const auto request : {get_configuration, get_interface}) {
    auto usb = start();
    select(usb, 1);
    for (const unsigned index : {2U, 3U, 5U, 6U, 7U}) {
      for (unsigned value = 0; value <= byte_mask; ++value) {
        if (value == request[index]) { continue; }
        auto changed = request;
        changed[index] = static_cast<std::uint8_t>(value);
        auto candidate = usb;
        rejects([&] { static_cast<void>(candidate.receive_setup(changed)); });
        require(word(candidate, configuration_status) == word(usb, configuration_status) &&
                    candidate.interrupt_sources() == 0,
                "Malformed fields cannot fabricate configuration or IRQ effects");
        ++count;
      }
    }
  }
  return count;
}

void cancellation() {
  auto usb = start();
  select(usb, 2);
  require(usb.receive_setup(get_configuration) == UsbResponse::ack, "Begin automatic read");
  data(usb, 2);
  require(usb.receive_setup(vendor) == UsbResponse::ack && usb.read(setup_buffer) == vendor[0],
          "New vendor SETUP aborts the old response and supplies genuine new USTB bytes");
  rejects([&] { usb.finish_in(true); });
  require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak,
          "Replacement request cannot inherit automatic read data");
  rejects([&] { static_cast<void>(usb.receive_setup(get_configuration)); });
  usb.write({.address = status_low, .value = 1});
  require(usb.receive_setup(get_configuration) == UsbResponse::ack, "Resume after real W1C");
  data(usb, 2);
  usb.observe_decoder({.reset = true});
  rejects([&] { usb.finish_in(true); });
  usb.observe_decoder({.enumerated = true});
  require(word(usb, configuration_status) == 0 && word(usb, endpoint_in) == 0 &&
              usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak &&
              usb.host_address() == 0,
          "Functional reset cancels automatic read without packet completion");
}

unsigned addresses() {
  constexpr unsigned maximum_address = 127;
  auto initial = start();
  initial.observe_decoder({.reset = true});
  initial.observe_decoder({.enumerated = true});
  require(initial.host_address() == 0, "Bus reset starts genuinely unaddressed");
  rejects([&] { static_cast<void>(initial.receive_setup(get_configuration)); });
  rejects([&] { static_cast<void>(initial.receive_setup(get_interface)); });
  unsigned cases = 0;
  for (unsigned value = 1; value <= maximum_address; ++value) {
    for (const bool enabled : {false, true}) {
      auto usb = initial;
      const std::array<std::uint8_t, 8> setup{
          0, 5, static_cast<std::uint8_t>(value), 0, 0, 0, 0, 0};
      require(usb.receive_setup(setup) == UsbResponse::ack && usb.host_address() == 0 &&
                  usb.interrupt_sources() == 0,
              "SET_ADDRESS changes neither actual address nor firmware SETUP IRQ");
      write_word(usb, endpoint_in, enabled ? interrupt : 0U);
      const auto pending = usb;
      for (unsigned replay = 0; replay < 2; ++replay) {
        usb = pending;
        for (unsigned retry = 0; retry < 3; ++retry) {
          const auto packet = usb.request_in(UsbBuffer::control_in);
          require(packet.response == UsbResponse::data && packet.size == 0 && packet.data1,
                  "Every unacknowledged address status retains DATA1/ZLP");
          usb.finish_in(false);
          require(usb.host_address() == 0 && usb.interrupt_sources() == 0,
                  "Missing host ACK cannot change the selected device address");
        }
        static_cast<void>(usb.request_in(UsbBuffer::control_in));
        usb.finish_in(true);
        require(usb.host_address() == value &&
                    word(usb, endpoint_in) == (complete | (enabled ? interrupt : 0U)) &&
                    usb.interrupt_sources() == (enabled ? 0x10U : 0U),
                "Successful status ACK alone commits the actual decoded address");
        require(usb.receive_setup(get_configuration) == UsbResponse::ack,
                "Addressed state now permits GET_CONFIGURATION zero");
        data(usb, 0);
      }
      ++cases;
    }
  }
  return cases;
}

unsigned request_errors() {
  unsigned cases = 0;
  for (unsigned configuration = 0; configuration < 3; ++configuration) {
    auto initial = start();
    select(initial, configuration);
    for (unsigned interface = 0; interface <= byte_mask; ++interface) {
      if (configuration != 0 && interface < 4) { continue; }
      auto usb = initial;
      auto request = get_interface;
      request[4] = static_cast<std::uint8_t>(interface);
      require(usb.receive_setup(request) == UsbResponse::ack && usb.interrupt_sources() == 0,
              "Invalid interface is hardware-decoded without a firmware SETUP IRQ");
      require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::stall &&
                  usb.receive_out({.data = {}, .data1 = true}) == UsbResponse::stall,
              "USB2.0 invalid/unconfigured interface produces Request Error in data/status");
      rejects([&] { usb.finish_in(true); });
      require(usb.receive_setup(get_configuration) == UsbResponse::ack,
              "New SETUP retires automatic request-error STALL");
      data(usb, configuration);
      ++cases;
    }
  }
  return cases;
}

unsigned address_refusals() {
  const auto initial = start();
  constexpr std::array<std::uint8_t, 8> valid_setup{0, 5, 1, 0, 0, 0, 0, 0};
  unsigned cases = 0;
  for (const unsigned index : {2U, 3U, 4U, 5U, 6U, 7U}) {
    for (unsigned value = 0; value <= byte_mask; ++value) {
      if ((index == 2 && value != 0 && value < 128) || (index != 2 && value == 0)) { continue; }
      auto usb = initial;
      auto setup = valid_setup;
      setup[index] = static_cast<std::uint8_t>(value);
      rejects([&] { static_cast<void>(usb.receive_setup(setup)); });
      require(usb.host_address() == decoded_address && usb.interrupt_sources() == 0,
              "Unspecified/malformed address requests never change the actual address");
      ++cases;
    }
  }
  auto configured = initial;
  select(configured, 1);
  rejects([&] { static_cast<void>(configured.receive_setup(valid_setup)); });
  return cases;
}

unsigned variants(Case test) {
  unsigned count = 0;
  for (unsigned enables = 0; enables < 4; ++enables) {
    for (const bool data_valid : {false, true}) {
      for (const unsigned size : {0U, 7U, 63U}) {
        test.enables = enables;
        test.data_valid = data_valid;
        test.size = size;
        matrix(test);
        ++count;
      }
    }
  }
  return count;
}

unsigned interface_counts() {
  unsigned cases = 0;
  for (unsigned count = 0; count < 4; ++count) {
    const auto counts = 2U | (count << 4U) | (count << 8U);
    for (unsigned configuration = 1; configuration <= 2; ++configuration) {
      auto initial = start(counts);
      select(initial, configuration);
      for (unsigned interface = 0; interface <= 4; ++interface) {
        auto usb = initial;
        auto request = get_interface;
        request[4] = static_cast<std::uint8_t>(interface);
        if (count == 0 || interface == count) {
          rejects([&] { static_cast<void>(usb.receive_setup(request)); });
          require(usb.interrupt_sources() == 0,
                  "Ambiguous interface counts cannot manufacture data,STALL or firmware IRQ");
        } else {
          require(usb.receive_setup(request) == UsbResponse::ack, "Qualified interface request");
          if (interface < count) {
            data(usb, 0);
          } else {
            require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::stall,
                    "Indices above either interpretation of the count are invalid interfaces");
          }
        }
        ++cases;
      }
    }
  }
  return cases;
}

enum class Abort : std::uint8_t { setup, bus_reset, endpoint_reset, module_reset, replacement };

void abort_address(McuUsb &usb, const Abort operation) {
  constexpr unsigned replacement_address = 89;
  switch (operation) {
    case Abort::setup:
      require(usb.receive_setup(vendor) == UsbResponse::ack, "Actual replacement SETUP");
      break;
    case Abort::bus_reset:
      usb.observe_decoder({.reset = true});
      usb.observe_decoder({.enumerated = true});
      break;
    case Abort::endpoint_reset: usb.reset_endpoints(); break;
    case Abort::module_reset: {
      usb.reset();
      rejects([&] { static_cast<void>(usb.host_address()); });
      constexpr unsigned default_endpoint = 0x02000000;
      config(usb, {.index = 1, .value = default_endpoint});
      usb.observe_decoder({});
      usb.set_control(module_control);
      usb.observe_decoder({.enumerated = true});
      break;
    }
    case Abort::replacement: {
      constexpr std::array<std::uint8_t, 8> replacement{0, 5, replacement_address, 0, 0, 0, 0, 0};
      require(usb.receive_setup(replacement) == UsbResponse::ack, "Replace pending address");
      break;
    }
    default: throw Fault("Unknown own address-cut case");
  }
}

unsigned address_cuts() {
  constexpr unsigned pending_address = 73;
  constexpr unsigned replacement_address = 89;
  constexpr std::array<std::uint8_t, 8> setup{0, 5, pending_address, 0, 0, 0, 0, 0};
  unsigned cases = 0;
  for (unsigned stage = 0; stage < 4; ++stage) {
    auto initial = start();
    require(initial.receive_setup(setup) == UsbResponse::ack, "Address stage under test");
    if (stage > 0) { static_cast<void>(initial.request_in(UsbBuffer::control_in)); }
    if (stage == 2) { initial.finish_in(false); }
    if (stage == 3) { initial.finish_in(true); }
    const auto committed = stage == 3 ? pending_address : decoded_address;
    for (const auto operation : {Abort::setup,
                                 Abort::bus_reset,
                                 Abort::endpoint_reset,
                                 Abort::module_reset,
                                 Abort::replacement}) {
      auto usb = initial;
      abort_address(usb, operation);
      rejects([&] { usb.finish_in(true); });
      const auto reset = operation == Abort::bus_reset || operation == Abort::module_reset;
      require(usb.host_address() == (reset ? 0U : committed),
              "Reset domains and new SETUP preserve only actually committed address state");
      if (operation == Abort::replacement) {
        const auto packet = usb.request_in(UsbBuffer::control_in);
        require(packet.response == UsbResponse::data && packet.size == 0 && packet.data1,
                "Only replacement status can commit its replacement address");
        usb.finish_in(true);
        require(usb.host_address() == replacement_address, "Replacement actual ACK commits once");
      }
      ++cases;
    }
    if (stage == 3) { continue; }
    auto usb = initial;
    usb.observe_decoder({.suspended = true, .enumerated = true});
    require(usb.host_address() == decoded_address, "Suspend retains the prior committed address");
    rejects([&] { static_cast<void>(usb.request_in(UsbBuffer::control_in)); });
    usb.observe_decoder({.enumerated = true});
    const auto packet = usb.request_in(UsbBuffer::control_in);
    require(packet.response == UsbResponse::data && packet.size == 0 && packet.data1,
            "Resume retains the actual pending status packet");
    usb.finish_in(true);
    require(usb.host_address() == pending_address, "Only resumed status ACK changes address");
    ++cases;
  }
  return cases;
}

unsigned class_vendor() {
  unsigned cases = 0;
  constexpr std::array<unsigned, 4> request_types{0x20, 0x40, 0xa1, 0xc1};
  for (const auto initial : {get_configuration, get_interface}) {
    for (const unsigned type : request_types) {
      auto usb = start();
      auto request = initial;
      request[0] = static_cast<std::uint8_t>(type);
      require(usb.receive_setup(request) == UsbResponse::ack && usb.interrupt_sources() == 2,
              "Class/vendor request numbers eight/ten remain actual firmware SETUP events");
      for (unsigned byte = 0; byte < request.size(); ++byte) {
        require(usb.read(static_cast<std::uint16_t>(setup_buffer + byte)) == request[byte],
                "Every actual class/vendor SETUP byte reaches USTB unchanged");
      }
      require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak,
              "No automatic selection byte substitutes for a class/vendor firmware reply");
      ++cases;
    }
  }
  return cases;
}
}

int main() {
  try {
    unsigned cases = 0;
    for (unsigned configuration = 0; configuration < 3; ++configuration) {
      const auto selections = configuration == 0 ? 1U : 13U;
      for (unsigned selection = 0; selection < selections; ++selection) {
        cases += variants({.configuration = configuration,
                           .interface = selection == 0 ? 0 : (selection - 1) / 3,
                           .setting = selection == 0 ? 0 : (selection - 1) % 3,
                           .read_interface = selection != 0});
      }
    }
    const auto refusals = malformed();
    const auto address_cases = addresses();
    const auto error_cases = request_errors();
    const auto address_errors = address_refusals();
    const auto count_cases = interface_counts();
    const auto cut_cases = address_cuts();
    const auto class_cases = class_vendor();
    cancellation();
    std::cout << cases << " automatic USB read cases, " << refusals << " malformed fields, "
              << address_cases << " address cases, " << error_cases << " request errors, "
              << address_errors << " address refusals, " << count_cases
              << " interface-count cases, " << cut_cases << " address cuts, " << class_cases
              << " class/vendor requests passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
