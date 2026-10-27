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
// Independent USB2.0 9.4.1/9.4.5/9.4.9 and USB20D6E2F UMCR3.1.1.
constexpr unsigned selector = 0x30e, config_data = 0x310, endpoint = 0x314;
constexpr unsigned out_endpoint = 0x316, irq_mask = 0x304, uccsr = 0x30c;
constexpr unsigned status_low = 0x303, setup_buffer = 0x338, fifo = 0x380;
constexpr unsigned byte_mask = 0xff, word_mask = 0xffff, base_control = 0xc008;
constexpr unsigned valid = 0x4000, completed = 0x2000, enabled = 0x1000;
constexpr unsigned sentinel = 0x91, device_address = 42;
constexpr unsigned read_variants = 24;
constexpr unsigned configuration_setup_enable = 0x2003;
constexpr std::array<std::uint8_t, 8> status_device{0x80, 0, 0, 0, 0, 0, 2, 0};
constexpr std::array<std::uint8_t, 8> status_interface{0x81, 0, 0, 0, 0, 0, 2, 0};
constexpr std::array<std::uint8_t, 8> set_wake{0, 3, 1, 0, 0, 0, 0, 0};
constexpr std::array<std::uint8_t, 8> clear_wake{0, 1, 1, 0, 0, 0, 0, 0};
constexpr std::array<std::uint8_t, 8> vendor{0x40, 0x1f, 0, 0, 0, 0, 0, 0};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

void rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &) { return; }
  throw Fault("Expected visible USB status/feature refusal");
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
  std::uint8_t bank = 0;
  std::uint32_t value = 0;
};

void config(McuUsb &usb, const Configuration config) {
  usb.write({.address = selector, .value = config.bank});
  usb.write_memory({.address = config_data, .size = 2, .value = config.value >> 16U});
  usb.write_memory({.address = config_data + 2, .size = 2, .value = config.value & word_mask});
}

void status_in(McuUsb &usb) {
  const auto packet = usb.request_in(UsbBuffer::control_in);
  require(packet.response == UsbResponse::data && packet.size == 0 && packet.data1,
          "No-data feature/address/configuration status is exactly DATA1/ZLP");
}

struct Profile {
  unsigned configuration = 0;
  bool power = false;
  bool capable = false;
};

McuUsb start(const Profile profile) {
  McuUsb usb;
  constexpr unsigned default_endpoint = 0x02000000;
  constexpr unsigned counts = 0x442;
  constexpr unsigned alternates = 0x33333333;
  config(usb, {.bank = 1, .value = default_endpoint});
  config(usb, {.bank = 8, .value = counts});
  config(usb, {.bank = 9, .value = alternates});
  usb.observe_decoder({});
  usb.set_control(static_cast<std::uint16_t>(base_control | (profile.power ? 2U : 0U) |
                                             (profile.capable ? 4U : 0U)));
  usb.observe_decoder({.enumerated = true});
  write_word(usb, irq_mask, configuration_setup_enable);
  constexpr std::array<std::uint8_t, 8> address{0, 5, device_address, 0, 0, 0, 0, 0};
  require(usb.receive_setup(address) == UsbResponse::ack, "Actual address SETUP");
  status_in(usb);
  usb.finish_in(true);
  if (profile.configuration != 0) {
    const std::array<std::uint8_t, 8> setup{
        0, 9, static_cast<std::uint8_t>(profile.configuration), 0, 0, 0, 0, 0};
    require(usb.receive_setup(setup) == UsbResponse::ack, "Actual configuration SETUP");
    usb.write({.address = uccsr, .value = 0x10});
    status_in(usb);
    usb.finish_in(true);
  }
  usb.write({.address = endpoint, .value = 0x20});
  usb.write({.address = fifo, .value = sentinel});
  return usb;
}

void status_data(McuUsb &usb, const unsigned expected) {
  const auto packet = usb.request_in(UsbBuffer::control_in);
  require(packet.response == UsbResponse::data && packet.size == 2 && packet.data1 &&
              packet.data[0] == expected && packet.data[1] == 0,
          "GET_STATUS returns exactly two little-endian DATA1 bytes,including reserved zeros");
}

void query(McuUsb &usb, const unsigned expected) {
  require(usb.receive_setup(status_device) == UsbResponse::ack, "Actual GET_STATUS SETUP");
  status_data(usb, expected);
  usb.finish_in(true);
  require(usb.receive_out({.data = {}, .data1 = true}) == UsbResponse::ack,
          "Actual GET_STATUS opposite-direction status");
}

struct Case {
  Profile profile;
  unsigned interrupts = 0;
  unsigned size = 0;
  bool data_valid = false;
  bool interface = false;
};

void read_case(const Case test) {
  auto usb = start(test.profile);
  if (test.interface) {
    constexpr std::array<std::uint8_t, 8> alternate{1, 11, 1, 0, 0, 0, 0, 0};
    require(usb.receive_setup(alternate) == UsbResponse::ack, "Select actual nonzero alternate");
    usb.write({.address = uccsr, .value = 0x10});
    status_in(usb);
    usb.finish_in(true);
    usb.write({.address = endpoint, .value = 0x20});
  }
  const auto before_configuration = word(usb, uccsr);
  const auto before_link = usb.host_evidence();
  require(usb.receive_setup(test.interface ? status_interface : status_device) == UsbResponse::ack,
          "Hardware accepts valid status SETUP");
  const auto before =
      ((test.interrupts & 1U) != 0 ? enabled : 0U) | (test.data_valid ? valid : 0U) | test.size;
  write_word(usb, endpoint, before);
  write_word(usb, out_endpoint, (test.interrupts & 2U) != 0 ? enabled : 0U);
  const auto expected = static_cast<unsigned>(test.profile.power && !test.interface);
  const auto pending = usb;
  for (unsigned replay = 0; replay < 2; ++replay) {
    usb = pending;
    for (unsigned retry = 0; retry < 3; ++retry) {
      status_data(usb, expected);
      require(word(usb, endpoint) == before && usb.interrupt_sources() == 0,
              "Capability alone cannot enable wakeup; no ACK cannot complete status data");
      usb.finish_in(false);
      usb.observe_sof(static_cast<std::uint16_t>(retry));
    }
    status_data(usb, expected);
    usb.finish_in(true);
    require(word(usb, endpoint) == ((before & ~valid) | completed) &&
                usb.interrupt_sources() == ((test.interrupts & 1U) != 0 ? 0x10U : 0U),
            "Only genuine data ACK supplies exact DVALID/TFRC/TCIE effects");
    require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak,
            "Completed status data does not repeat");
    require(usb.receive_out({.data = {}, .data1 = true}) == UsbResponse::ack,
            "Actual DATA1/ZLP status OUT completes this read");
    require(word(usb, out_endpoint) ==
                    (valid | completed | ((test.interrupts & 2U) != 0 ? enabled : 0U)) &&
                usb.host_evidence() == before_link && word(usb, uccsr) == before_configuration &&
                usb.read(fifo) == sentinel,
            "Read retains FIFO bytes,configuration and host evidence while completing OUT");
    rejects([&] { static_cast<void>(usb.read(setup_buffer)); });
  }
}

unsigned reads() {
  unsigned cases = 0;
  for (unsigned properties = 0; properties < 4; ++properties) {
    for (unsigned configuration = 0; configuration < 3; ++configuration) {
      const Profile profile{.configuration = configuration,
                            .power = (properties & 1U) != 0,
                            .capable = (properties & 2U) != 0};
      for (const bool interface : {false, true}) {
        if (interface && configuration == 0) { continue; }
        for (unsigned variant = 0; variant < read_variants; ++variant) {
          constexpr std::array<unsigned, 3> sizes{0, 7, 63};
          read_case({.profile = profile,
                     .interrupts = variant / 6,
                     .size = sizes[variant % 3],
                     .data_valid = (variant % 6) >= 3,
                     .interface = interface});
          ++cases;
        }
      }
    }
  }
  return cases;
}

void feature_case(const Profile profile, const std::array<std::uint8_t, 8> &operation) {
  auto usb = start(profile);
  if (profile.capable && operation == clear_wake) {
    require(usb.receive_setup(set_wake) == UsbResponse::ack, "Prime actual enabled feature");
    status_in(usb);
    usb.finish_in(true);
  }
  const auto link = usb.host_evidence();
  require(usb.receive_setup(operation) == UsbResponse::ack && (usb.read(status_low) & 3U) == 0,
          "Feature command is decoded without firmware SETUP/SETOVR");
  if (!profile.capable) {
    require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::stall,
            "An unsupported wake capability produces Request Error,not a successful status");
    query(usb, profile.power ? 1U : 0U);
    return;
  }
  const auto pending = usb;
  for (unsigned stage = 0; stage < 3; ++stage) {
    usb = pending;
    if (stage != 0) {
      status_in(usb);
      usb.finish_in(stage == 2);
    }
    const auto expected = (profile.power ? 1U : 0U) | (operation == set_wake ? 2U : 0U);
    query(usb, expected);
    require(usb.host_evidence() == link && usb.host_address() == device_address &&
                !usb.host_suspended(),
            "Feature acts before status; aborted/lost status cannot roll it back or emit resume");
  }
}

unsigned features() {
  unsigned cases = 0;
  for (unsigned properties = 0; properties < 4; ++properties) {
    for (unsigned configuration = 0; configuration < 3; ++configuration) {
      const Profile profile{.configuration = configuration,
                            .power = (properties & 1U) != 0,
                            .capable = (properties & 2U) != 0};
      for (const auto operation : {set_wake, clear_wake}) {
        feature_case(profile, operation);
        ++cases;
      }
    }
  }
  return cases;
}

unsigned malformed() {
  unsigned cases = 0;
  for (const auto request : {status_device, status_interface, set_wake, clear_wake}) {
    const auto initial = start({.configuration = 1, .power = true, .capable = true});
    for (const unsigned index : {2U, 3U, 5U, 6U, 7U}) {
      for (unsigned value = 0; value <= byte_mask; ++value) {
        if (request[index] == value) { continue; }
        auto usb = initial;
        auto input = request;
        input[index] = static_cast<std::uint8_t>(value);
        rejects([&] { static_cast<void>(usb.receive_setup(input)); });
        require(usb.host_address() == device_address && usb.interrupt_sources() == 0,
                "Malformed fields never manufacture an address or firmware interrupt");
        ++cases;
      }
    }
  }
  for (const auto request : {status_device, set_wake, clear_wake}) {
    for (unsigned value = 1; value <= byte_mask; ++value) {
      auto usb = start({.configuration = 1, .capable = true});
      auto input = request;
      input[4] = static_cast<std::uint8_t>(value);
      rejects([&] { static_cast<void>(usb.receive_setup(input)); });
      ++cases;
    }
  }
  return cases;
}

unsigned cuts() {
  unsigned cases = 0;
  const Profile profile{.configuration = 1, .power = true, .capable = true};
  auto initial = start(profile);
  require(initial.receive_setup(set_wake) == UsbResponse::ack, "Enable actual wake feature");
  for (unsigned stage = 0; stage < 3; ++stage) {
    auto pending = initial;
    if (stage != 0) {
      status_in(pending);
      pending.finish_in(stage == 2);
    }
    auto endpoint_reset = pending;
    endpoint_reset.reset_endpoints();
    query(endpoint_reset, 3);
    auto suspended = pending;
    suspended.observe_decoder({.suspended = true, .enumerated = true});
    rejects([&] { static_cast<void>(suspended.request_in(UsbBuffer::control_in)); });
    suspended.observe_decoder({.enumerated = true});
    query(suspended, 3);
    auto reset = pending;
    reset.observe_decoder({.reset = true});
    reset.observe_decoder({.enumerated = true});
    constexpr std::array<std::uint8_t, 8> address{0, 5, device_address, 0, 0, 0, 0, 0};
    require(reset.receive_setup(address) == UsbResponse::ack, "Fresh address after bus reset");
    status_in(reset);
    reset.finish_in(true);
    query(reset, 1);
    auto replaced = pending;
    require(replaced.receive_setup(vendor) == UsbResponse::ack, "Actual vendor replacement");
    replaced.write({.address = status_low, .value = 1});
    query(replaced, 3);
    auto changed = pending;
    rejects([&] { changed.set_control(base_control | 2U); });
    query(changed, 3);
    auto module = pending;
    module.reset();
    rejects([&] { static_cast<void>(module.host_address()); });
    constexpr unsigned default_endpoint = 0x02000000;
    config(module, {.bank = 1, .value = default_endpoint});
    module.observe_decoder({});
    module.set_control(base_control | 6U);
    module.observe_decoder({.enumerated = true});
    require(module.receive_setup(address) == UsbResponse::ack, "Address after actual module reset");
    status_in(module);
    module.finish_in(true);
    query(module, 1);
    cases += 6;
  }
  return cases;
}

unsigned guards() {
  unsigned cases = 0;
  auto reset = start({.configuration = 1, .capable = true});
  reset.observe_decoder({.reset = true});
  reset.observe_decoder({.enumerated = true});
  for (const auto operation : {status_device, set_wake, clear_wake}) {
    auto usb = reset;
    rejects([&] { static_cast<void>(usb.receive_setup(operation)); });
    ++cases;
  }
  for (const auto request : {status_device, set_wake, clear_wake}) {
    auto usb = start({.configuration = 1, .capable = true});
    require(usb.receive_setup(vendor) == UsbResponse::ack, "Genuine pending firmware SETUP");
    rejects([&] { static_cast<void>(usb.receive_setup(request)); });
    require(usb.read(setup_buffer) == vendor[0],
            "Refused automatic request preserves firmware USTB");
    ++cases;
  }
  constexpr std::array<unsigned, 3> request_types{0x20, 0x40, 0xc0};
  for (const auto operation : {status_device, set_wake, clear_wake}) {
    for (const auto type : request_types) {
      auto usb = start({.configuration = 1, .capable = true});
      auto request = operation;
      request[0] = static_cast<std::uint8_t>(type);
      require(usb.receive_setup(request) == UsbResponse::ack && usb.interrupt_sources() == 2,
              "Class/vendor requests retain firmware ownership of zero/one/three numbers");
      for (unsigned index = 0; index < request.size(); ++index) {
        require(usb.read(static_cast<std::uint16_t>(setup_buffer + index)) == request[index],
                "No hardware status reply replaces actual class/vendor SETUP bytes");
      }
      ++cases;
    }
  }
  auto addressed = start({.capable = true});
  require(addressed.receive_setup(status_interface) == UsbResponse::ack &&
              addressed.request_in(UsbBuffer::control_in).response == UsbResponse::stall,
          "Interface status in addressed state returns Request Error");
  ++cases;
  return cases;
}
}

int main() {
  try {
    const auto read_cases = reads();
    const auto feature_cases = features();
    const auto malformed_cases = malformed();
    const auto cut_cases = cuts();
    const auto guard_cases = guards();
    std::cout << read_cases << " status reads, " << feature_cases << " feature cases, "
              << malformed_cases << " malformed fields, " << cut_cases << " cuts, " << guard_cases
              << " guards passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
