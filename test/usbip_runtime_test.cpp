// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_usb.hpp"
#include "blofeld/usbip_runtime.hpp"
#include "blofeld/usbip_tcp.hpp"
#include "usb_device_native_fixture.hpp"
#include "usb_endpoint_reset_fixture.hpp"
#include "usb_host_fixture.hpp"
#include "usbip_runtime_fixture.hpp"
#include "usbip_session_fixture.hpp"
#include "usbip_tcp_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <sys/poll.h>
#include <sys/socket.h>

namespace {
using namespace blofeld;
using namespace blofeld::test::usb_host;
using blofeld::test::usbip_runtime::Fixture;
using blofeld::test::usbip_runtime::options;
namespace native = blofeld::test::usb_device_native;
namespace reset = blofeld::test::endpoint_reset;
namespace tcp = blofeld::test::usbip_tcp;
namespace session = blofeld::test::usbip_session;
constexpr unsigned maximum_socket_polls = 1000;
constexpr unsigned first_management_tick = 20;
constexpr unsigned slow_poll_period = 100;
constexpr unsigned alternate_bulk_in = 0x020000d2;

void test_capture_and_reset_pulse() {
  Fixture f;
  f.capture();
  auto client = tcp::connect(f.runtime.port());
  tcp::send(client, session::list);
  tcp::Received received;
  for (unsigned i = 0; i < maximum_socket_polls && !received.eof; ++i) {
    f.poll(first_management_tick + i);
    tcp::read(client, received);
    if (!received.eof) {
      pollfd ready{.fd = client.get(), .events = POLLIN, .revents = 0};
      static_cast<void>(::poll(&ready, 1, 1));
    }
  }
  // The native fixture previously selected configuration1; UNCIR declares two interfaces.
  constexpr unsigned configured_list_bytes = 332;
  constexpr unsigned configuration_offset = 321;
  constexpr unsigned interface_count_offset = 323;
  require(received.eof && received.bytes.size() == configured_list_bytes &&
              received.bytes[11] == 1 && received.bytes[configuration_offset] == 1 &&
              received.bytes[interface_count_offset] == 2 && received.bytes.back() == 0,
          "Actual TCP discovery exports native captured descriptors and the active two interfaces");
  const auto before = f.bus.usb_host_evidence();
  f.bus.observe_usb_decoder({.reset = true});
  f.bus.observe_usb_decoder({.enumerated = true});
  const auto after = f.bus.usb_host_evidence();
  require(after.link == before.link + 2 && after.descriptions == before.descriptions,
          "A complete reset pulse invalidates the link even after its original level returns");
  reject([&] { f.poll(f.boundary.ticks + 1); }, "link/reset evidence expired");
  require(f.runtime.faulted() && !f.runtime.published(),
          "A real reset retires the publication and listener without automatic recovery");
  const UsbipTcpServer replacement({.port = f.runtime.port()});
}

void test_preimport_description_and_configuration_changes() {
  for (const bool description : {false, true}) {
    Fixture f;
    f.capture();
    reset::configure(f.bus, {.bank = 2, .value = reset::bulk_in});
    f.bus.observe_usb_decoder({.enumerated = true});
    f.poll(first_management_tick - 1);
    require(f.runtime.published(), "Identical descriptions/decoder observations retain evidence");
    if (description) {
      reset::configure(f.bus, {.bank = 2, .value = alternate_bulk_in});
    } else {
      f.bus.observe_usb_configuration({.configuration = 1});
    }
    reject([&] { f.poll(first_management_tick); },
           description ? "description evidence" : "pending firmware");
    require(f.runtime.faulted(), "Changed preimport hardware evidence cannot serve stale metadata");
  }
}

void test_imported_configuration_and_retirement() {
  Fixture f;
  f.capture();
  auto client = tcp::connect(f.runtime.port());
  tcp::send(client, session::import("1-1"));
  tcp::Received received;
  constexpr unsigned import_bytes = 320;
  for (unsigned i = 0; i < maximum_socket_polls && received.bytes.size() < import_bytes; ++i) {
    f.poll(first_management_tick + i);
    tcp::read(client, received);
    if (received.bytes.size() < import_bytes) {
      pollfd ready{.fd = client.get(), .events = POLLIN, .revents = 0};
      static_cast<void>(::poll(&ready, 1, 1));
    }
  }
  require(received.bytes.size() == import_bytes && !received.eof,
          "A real import owns the exact publication until its host stream ends");
  // Own controller stimulus, not a guest configuration/status-completion proof.
  f.bus.observe_usb_configuration({.configuration = 1});
  reset::configure(f.bus, {.bank = 2, .value = alternate_bulk_in});
  f.poll(f.boundary.ticks + 1);
  require(f.runtime.published() && !f.runtime.faulted(),
          "Imported ownership permits actual pending configuration/description changes");
  require(::shutdown(client.get(), SHUT_WR) == 0, "Own imported peer half-closes its input");
  reject(
      [&] {
        for (unsigned i = 0; i < maximum_socket_polls; ++i) { f.poll(f.boundary.ticks + 1); }
      },
      "board-owned recovery required");
  reject([&] { static_cast<void>(f.bus.usb_host_configuration()); }, "pending firmware");
  require(f.runtime.faulted() && !f.runtime.published(),
          "Imported closure retires transport without clearing actual pending guest configuration");
}

void test_budget_and_between_poll_invalidation() {
  auto config = options();
  config.period_ticks = slow_poll_period;
  config.probe_timeout_ticks = 5;
  Fixture timeout(config);
  timeout.poll(10);
  timeout.poll(14);
  reject([&] { timeout.poll(15); }, "reference tick budget");
  require((word(timeout.bus, status) & 1U) != 0,
          "Probe timeout preserves the real already accepted SETUP for guest recovery");
  Fixture reset_between(config);
  reset_between.poll(10);
  reset_between.bus.observe_usb_decoder({.reset = true});
  reject([&] { reset_between.poll(11); }, "link/reset evidence");
  Fixture overflow;
  reject([&] { overflow.poll(std::numeric_limits<std::uint64_t>::max()); }, "timestamp exhausted");
  Fixture inactive;
  inactive.bus.write(reset::control, reset::mce_high);
  reject([&] { inactive.poll(10); }, "mcu-pc=0x8000");
  require(
      inactive.runtime.faulted() && inactive.bus.read(reset::control) == reset::mce_high &&
          (word(inactive.bus, status) & 1U) == 0 && inactive.cpu.state().pc == native::initial.pc,
      "Unready hardware at admission fails without enabling the module or executing guest code");
}

void test_foreign_owner_restore_and_time_rewind() {
  for (unsigned variant = 0; variant < 3; ++variant) {
    Fixture f;
    f.poll(5);
    if (variant == 0) {
      f.boundary.owner = std::make_shared<const int>(1);
    } else if (variant == 1) {
      ++f.boundary.restore_revision;
    }
    reject([&] { f.poll(variant == 2 ? 4 : 6); }, "board owner/restore/time changed");
    require(!f.runtime.started() && f.runtime.faulted() && (word(f.bus, status) & 1U) == 0,
            "An invalid external boundary fails before guest SETUP or capture state is supplied");
  }
  auto invalid = options();
  invalid.period_ticks = 0;
  reject([&] { const UsbipRuntime missing_period(invalid); }, "positive poll period");
}
}

int main() {
  try {
    test_capture_and_reset_pulse();
    test_preimport_description_and_configuration_changes();
    test_imported_configuration_and_retirement();
    test_budget_and_between_poll_invalidation();
    test_foreign_owner_restore_and_time_rewind();
    std::cout << "Own native USB runtime capture, real TCP metadata, reset/state invalidation "
                 "and explicit reference admission/budgets passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
