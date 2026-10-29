// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "nand_native_tests.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned ghz = 1000000000;
constexpr unsigned changed_at = 100;
constexpr unsigned read_at = 100;
constexpr unsigned valid_at = 130;
constexpr unsigned refreshed_at = 200;
constexpr unsigned released_at = 210;
constexpr unsigned hold_end = 225;
constexpr unsigned floating_at = 240;
constexpr unsigned byte_mask = 0xff;
constexpr NandByte busy{.value = 0x80, .known = 0xc0};
constexpr NandByte passed{.value = 0xc0, .known = 0xc1};
constexpr NandByte failed{.value = 0xc1, .known = 0xc1};
constexpr auto contract = NandStatusTiming::reference_controller_boundary;

void require(const bool value, const std::string &why) {
  if (!value) { throw Fault(why); }
}

void reject(const auto &action, const std::string &part) {
  try {
    action();
  } catch (const Fault &fault) {
    require(std::string(fault.what()).contains(part),
            "Wrong refusal: " + std::string(fault.what()));
    return;
  }
  throw Fault("Missing refusal: " + part);
}

NandReadSetupPins pins() {
  return {.ce_n = false, .re_n = true, .we_n = true, .ale = false, .cle = false, .ready = false};
}

void change(NandReadSetupPins &value, const unsigned index, const bool active) {
  if (index == 0) { value.we_n = !active; }
  if (index == 1) { value.ale = active; }
  if (index == 2) { value.cle = active; }
}

void setup_bounds() {
  constexpr std::array<std::array<unsigned, 4>, 3> vectors{{
      {{1000000000, 60, 10, 10}},
      {{1080000000, 65, 11, 11}},
      {{216000000, 13, 3, 3}},
  }};
  constexpr std::array<const char *, 3> names{"tWHR", "tAR", "tCLR"};
  for (const auto &row : vectors) {
    for (unsigned signal = 0; signal < names.size(); ++signal) {
      NandReadSetup model(row[0]);
      auto value = pins();
      change(value, signal, true);
      require(!model.observe(0, value, NandReadKind::status70), "Initial status controls observed");
      change(value, signal, false);
      require(!model.observe(changed_at, value, NandReadKind::status70), "Actual control edge");
      value.re_n = false;
      const auto before = model;
      reject(
          [&] {
            static_cast<void>(
                model.observe(changed_at + row[signal + 1] - 1, value, NandReadKind::status70));
          },
          names[signal]);
      require(model == before, "Rejected status setup keeps the complete old state");
      require(model.observe(changed_at + row[signal + 1], value, NandReadKind::status70),
              "Exact primary status setup boundary passes with actual Ready low");
    }
  }
  std::cout << "PASS STATUS_SETUP_BOUNDS independent-pairs=9\n";
}

void setup_modes() {
  for (const auto ready :
       {std::optional<bool>{false}, std::optional<bool>{true}, std::optional<bool>{}}) {
    auto value = pins();
    value.ready = ready;
    NandReadSetup model(ghz);
    require(!model.observe(0, value), "No read at the first high observation");
    auto array = model;
    value.re_n = false;
    require(model.observe(read_at, value, NandReadKind::status70), "Status never invents Ready");
    if (ready == true) {
      require(array.observe(read_at, value), "Known Ready retains normal array admission");
    } else {
      reject([&] { static_cast<void>(array.observe(read_at, value)); }, "observed Ready high");
    }
  }
  auto value = pins();
  NandReadSetup model(ghz);
  require(!model.observe(0, value), "Start with real Busy");
  value.ready = true;
  require(!model.observe(read_at - 1, value), "Observe real completion just before RE");
  auto array = model;
  value.re_n = false;
  require(model.observe(read_at, value, NandReadKind::status70), "Status has no array tRR");
  reject([&] { static_cast<void>(array.observe(read_at, value)); }, "tRR");
  auto invalid = array;
  reject([&] { static_cast<void>(invalid.observe(read_at, value, NandReadKind::unqualified)); },
         "invalid read kind");
  require(invalid == array, "Invalid kind cannot mutate timing state");
  std::cout << "PASS STATUS_SETUP_MODES ready-domains=3 array-tRR-preserved=1 invalid-kind=1\n";
}

NandReadTiming active() {
  NandReadTiming output(ghz);
  output.observe(0, {.ce_n = false, .re_n = true});
  output.observe(read_at, {.ce_n = false, .re_n = false}, [] { return busy; });
  return output;
}

void held_refresh(const NandByte completion) {
  auto output = active();
  output.refresh_status(read_at + 1, busy, contract);
  require(output.next_transition() == valid_at && !output.output(valid_at - 1).driven,
          "Refresh never shortens the actual tREA access envelope");
  require(output.output(valid_at) == NandReadOutput{.driven = true, .data = busy},
          "Busy status becomes readable at its independent original bound");
  auto pending = output;
  unsigned sources = 0;
  output.refresh_status(refreshed_at, completion, contract);
  output.observe(refreshed_at, {.ce_n = false, .re_n = false}, [&] {
    ++sources;
    return busy;
  });
  require(sources == 0 && !output.next_transition() &&
              output.output(refreshed_at) == NandReadOutput{.driven = true, .data = completion},
          "Held status changes without a source resample,new pin edge or new access delay");
  require(pending.output(refreshed_at).data == busy,
          "An isolated copied busy read cannot acquire another owner's completion");
  pending.refresh_status(refreshed_at, completion, contract);
  require(pending == output, "Copied held-read state retains exact access/phase knowledge");
  output.observe(released_at, {.ce_n = false, .re_n = true});
  require(output.output(hold_end - 1) == NandReadOutput{.driven = true, .data = completion},
          "Actual last refreshed value remains held after RE rises");
  require(!output.output(hold_end).driven && output.output(floating_at).driven == false,
          "Original15ns hold and30ns release boundaries remain exact");
  std::cout << "PASS HELD_STATUS_REFRESH value=" << static_cast<unsigned>(completion.value) << '\n';
}

void refresh_guards() {
  auto output = active();
  const auto before = output;
  reject([&] { output.refresh_status(read_at, passed, NandStatusTiming::unqualified); },
         "explicit controller-boundary");
  reject(
      [&] { output.refresh_status(read_at, {.value = byte_mask, .known = byte_mask}, contract); },
      "undefined status-pin");
  reject([&] { output.refresh_status(read_at, {.value = 1, .known = 0}, contract); },
         "noncanonical");
  reject([&] { output.refresh_status(read_at - 1, busy, contract); }, "backwards");
  require(output == before, "All rejected status refreshes retain the complete value state");
  NandReadTiming cold(ghz);
  const auto initial = cold;
  reject([&] { cold.refresh_status(0, busy, contract); }, "held RE-low");
  require(cold == initial, "No initial held read is invented");
  output.observe(released_at, {.ce_n = false, .re_n = true});
  const auto released = output;
  reject([&] { output.refresh_status(released_at, passed, contract); }, "held RE-low");
  require(output == released, "Refresh cannot rewrite a released hold value");
  std::cout << "PASS STATUS_REFRESH_GUARDS strong-refusals=6\n";
}
}

int main() {
  try {
    setup_bounds();
    setup_modes();
    held_refresh(passed);
    held_refresh(failed);
    refresh_guards();
    blofeld::test::native_medium_tests();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL BUSY_STATUS " << error.what() << '\n';
    return 1;
  }
}
