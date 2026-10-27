// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "usb_functional_halt_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace usb_halt_fixture;
constexpr unsigned setup_status = 0x303;
constexpr unsigned setup_buffer = 0x338;
constexpr std::uint8_t endpoint_input_request = 0x82;
constexpr std::array<std::uint8_t, 8> vendor{0x40, 0x1f, 0, 0, 0, 0, 0, 0};

void rejects(const auto &operation, const char *reason) {
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains(reason), "An exact uncertainty must be visible");
    return;
  }
  throw Fault("Unknown hardware state cannot become a successful status response");
}

std::array<std::uint8_t, 8> request(const std::uint8_t address) {
  return {endpoint_input_request, 0, 0, 0, address, 0, 2, 0};
}

void data(McuUsb &usb, const bool halted) {
  const auto packet = usb.request_in(UsbBuffer::control_in);
  require(packet.response == UsbResponse::data && packet.size == 2 && packet.data1 &&
              packet.data[0] == static_cast<unsigned>(halted) && packet.data[1] == 0,
          "Endpoint status is exactly DATA1 with Halt and fifteen reserved zeros");
}

void query(McuUsb &usb, const std::uint8_t address, const bool halted) {
  require(usb.receive_setup(request(address)) == UsbResponse::ack, "Actual status SETUP");
  data(usb, halted);
  usb.finish_in(true);
  require(usb.receive_out({.data = {}, .data1 = true}) == UsbResponse::ack,
          "Actual opposite-direction status OUT");
}

McuUsb primed(const Case test, const bool interrupt, const unsigned halt_kind) {
  auto usb = armed_case(test, {.interrupt = interrupt});
  if (halt_kind == 0) {
    const auto high = irq_enable | (test.occupied ? valid : 0U);
    usb.write({.address = static_cast<std::uint16_t>(test.input ? ep2 : ep3),
               .value = static_cast<std::uint8_t>(high >> 8U)});
  } else if (halt_kind == 2) {
    require(transaction(usb, test) == UsbResponse::stall, "Actually consumed STALL command");
  }
  return usb;
}

void bulk_state(McuUsb &usb, const Case test, const unsigned halt_kind) {
  const auto expected =
      irq_enable | (test.occupied ? valid | payload.size() : 0U) | (halt_kind == 1 ? stall : 0U);
  require(word(usb, static_cast<std::uint16_t>(test.input ? ep2 : ep3)) == expected &&
              usb.interrupt_sources() == 0,
          "Status read cannot modify data ownership, size, command or completion flags");
  require(usb.host_endpoint(test.input ? input_address : output_address).data1 ==
              (!test.input && test.occupied),
          "Status read cannot advance independently expected data endpoint PID");
  usb.write({.address = buffer_select, .value = static_cast<std::uint8_t>(test.input ? 2 : 3)});
  if (test.occupied) {
    for (unsigned index = 0; index < payload.size(); ++index) {
      require(usb.read(static_cast<std::uint16_t>(fifo + index)) == payload[index],
              "All original data endpoint bytes survive status reads");
    }
  } else {
    rejects([&usb] { static_cast<void>(usb.read(fifo)); }, "Unknown USB endpoint-buffer byte");
  }
}

void read_case(const Case test, const bool interrupt, const unsigned halt_kind) {
  const auto initial = primed(test, interrupt, halt_kind);
  const auto address = test.input ? input_address : output_address;
  const auto evidence = initial.host_evidence();
  for (unsigned cut = 0; cut < 4; ++cut) {
    auto saved = initial;
    if (cut != 0) {
      require(saved.receive_setup(request(address)) == UsbResponse::ack, "Before saved SETUP");
    }
    if (cut >= 2) {
      data(saved, halt_kind != 0);
      saved.finish_in(cut == 3);
    }
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto usb = saved;
      if (cut == 0) {
        require(usb.receive_setup(request(address)) == UsbResponse::ack, "Restored status SETUP");
      }
      if (cut < 3) {
        for (unsigned retry = 0; retry < 2; ++retry) {
          data(usb, halt_kind != 0);
          usb.finish_in(false);
          usb.observe_sof(static_cast<std::uint16_t>(retry));
        }
        data(usb, halt_kind != 0);
        usb.finish_in(true);
      }
      require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::nak,
              "Acknowledged automatic data cannot be repeated");
      require(usb.receive_out({.data = {}, .data1 = true}) == UsbResponse::ack,
              "A genuine DATA1 status packet ends every restored read");
      bulk_state(usb, test, halt_kind);
      require(usb.host_evidence() == evidence,
              "Status reads do not retire the current transport identity or descriptions");
      if (halt_kind != 0) {
        require(transaction(usb, test) == UsbResponse::stall, "Query never clears functional halt");
      }
    }
  }
}

void select(McuUsb &usb, const std::uint8_t interface) {
  const std::array<std::uint8_t, 8> setup{1, 11, 0, 0, interface, 0, 0, 0};
  require(usb.receive_setup(setup) == UsbResponse::ack, "Actual interface selection");
  usb.write({.address = config_status, .value = 0x10});
  const auto status = usb.request_in(UsbBuffer::control_in);
  require(status.response == UsbResponse::data && status.size == 0 && status.data1,
          "Actual selection DATA1 status");
  usb.finish_in(true);
}

void reset_case(const bool input, const bool interrupt, const bool halted) {
  const Case test{.input = input, .occupied = true};
  auto usb = primed(test, interrupt, halted ? 2U : 0U);
  usb.set_control(0x8000);
  usb.reset_endpoints();
  usb.set_control(module_enable);
  if (halted) {
    const auto saved = usb;
    for (unsigned replay = 0; replay < 2; ++replay) {
      usb = saved;
      rejects([&] { query(usb, input ? input_address : output_address, false); },
              "functional-halt status after endpoint reset");
    }
    select(usb, 0);
  }
  query(usb, input ? input_address : output_address, false);
  require(usb.host_endpoint(input ? input_address : output_address).data1 ==
              (halted ? std::optional<bool>(false) : std::nullopt),
          "Query cannot invent the unknown PID; selection independently establishes DATA0");
}

void request_error(McuUsb &usb, const std::uint8_t address) {
  require(usb.receive_setup(request(address)) == UsbResponse::ack,
          "Missing endpoints still ACK a valid control SETUP");
  for (unsigned retry = 0; retry < 2; ++retry) {
    require(usb.request_in(UsbBuffer::control_in).response == UsbResponse::stall,
            "USB2.0 requires Request Error for a missing endpoint");
  }
  require((usb.read(setup_status) & 3U) == 0,
          "Hardware rejects a standard request without forwarding it to firmware");
}

unsigned errors() {
  unsigned cases = 0;
  constexpr std::array<std::uint8_t, 6> missing{1, 0x82, 3, 0x83, 15, 0x8f};
  for (const auto address : missing) {
    auto usb = configured();
    request_error(usb, address);
    query(usb, input_address, false);
    ++cases;
  }
  auto addressed = configured();
  addressed.observe_decoder({.reset = true});
  addressed.observe_decoder({.enumerated = true});
  constexpr std::array<std::uint8_t, 8> setup{0, 5, device_address, 0, 0, 0, 0, 0};
  require(addressed.receive_setup(setup) == UsbResponse::ack, "Actual post-reset address");
  require(addressed.request_in(UsbBuffer::control_in).response == UsbResponse::data,
          "Actual address status");
  addressed.finish_in(true);
  request_error(addressed, input_address);
  ++cases;
  for (const auto inactive : {0x02000151U, 0x020080d1U}) {
    auto usb = configured();
    describe(usb, {.bank = 2, .value = inactive});
    request_error(usb, input_address);
    query(usb, output_address, false);
    ++cases;
  }
  return cases;
}

unsigned guards() {
  unsigned cases = 0;
  constexpr std::array<std::uint8_t, 4> addresses{0, 0x80, 0x71, 0xf1};
  for (const auto address : addresses) {
    auto usb = configured();
    rejects([&] { query(usb, address, false); },
            address == 0 || address == 0x80 ? "control endpoint functional-halt"
                                            : "Reserved USB endpoint GET_STATUS");
    ++cases;
  }
  constexpr unsigned physical4_description = 0x020000c3;
  constexpr unsigned physical5_description = 0x020000d3;
  constexpr std::uint8_t input3_address = 0x83;
  for (const unsigned bank : {4U, 5U}) {
    auto usb = configured();
    describe(usb,
             {.bank = bank, .value = bank == 4 ? physical4_description : physical5_description});
    rejects([&] { query(usb, static_cast<std::uint8_t>(bank == 4 ? 3 : input3_address), false); },
            "Unimplemented USB endpoint halt status physical-endpoint=");
    ++cases;
  }
  constexpr unsigned reserved_description = 0x820000d1;
  constexpr unsigned duplicate_description = 0x020000d1;
  auto reserved = configured();
  describe(reserved, {.bank = 2, .value = reserved_description});
  rejects([&] { query(reserved, input_address, false); }, "Reserved USB local endpoint NISOF");
  ++cases;
  auto ambiguous = configured();
  describe(ambiguous, {.bank = 3, .value = duplicate_description});
  rejects([&] { query(ambiguous, input_address, false); }, "Ambiguous USB endpoint status");
  ++cases;
  auto pending = configured();
  require(pending.receive_setup(vendor) == UsbResponse::ack, "Genuine pending firmware SETUP");
  rejects([&] { query(pending, input_address, false); }, "overlapping firmware SETUP");
  require(pending.read(setup_buffer) == vendor[0], "Refusal retains firmware SETUP data");
  ++cases;
  return cases;
}

void selective(const bool interrupt, const std::uint8_t interface) {
  auto usb = configured({.interrupt = interrupt, .separate_interfaces = true});
  usb.write({.address = ep2, .value = 0x80});
  usb.write({.address = ep3, .value = 0x80});
  require(usb.request_in(UsbBuffer::endpoint2).response == UsbResponse::stall &&
              usb.receive_out({.buffer = UsbBuffer::endpoint3, .data = payload}) ==
                  UsbResponse::stall,
          "Both actual data endpoints enter functional halt");
  select(usb, interface);
  query(usb, input_address, interface != 0);
  query(usb, output_address, interface != 1);
}
}

int main() {
  try {
    unsigned reads = 0;
    unsigned resets = 0;
    for (const bool interrupt : {false, true}) {
      for (const bool input : {false, true}) {
        for (const bool occupied : {false, true}) {
          for (unsigned kind = 0; kind < 3; ++kind) {
            read_case({.input = input, .occupied = occupied}, interrupt, kind);
            ++reads;
          }
        }
        for (const bool halted : {false, true}) {
          reset_case(input, interrupt, halted);
          ++resets;
        }
      }
      selective(interrupt, 0);
      selective(interrupt, 1);
    }
    const auto error_cases = errors();
    const auto strict_cases = guards();
    std::cout << "endpoint-status patterns=" << reads << " snapshot-replays=" << reads * 8
              << " reset-patterns=" << resets << " request-errors=" << error_cases
              << " strict-guards=" << strict_cases << " selective-interfaces=4\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
