// SPDX-License-Identifier: GPL-3.0-only
#include "parallel_update_fixture.hpp"

#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <ostream>
#include <span>
#include <string_view>
#include <vector>

namespace blofeld::test {
namespace {
constexpr unsigned application = 0x8200;
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned strap_first = 0x8029;
constexpr unsigned strap_second = 0x8036;
constexpr unsigned boot_limit = 200000;
constexpr unsigned call_limit = 100000;
constexpr unsigned caller = 0x1700;
constexpr unsigned receiver = 0xbad9;
constexpr unsigned sample_pc = 0xbad3;
constexpr unsigned helper = 0x1200;
constexpr unsigned helper_return = 0xbb56;
constexpr unsigned service_table = 0xf81b;
constexpr unsigned first_service = 0x8d70;
constexpr unsigned helper_callback_return = 0x1209;
constexpr unsigned stack = 0x2600;
constexpr unsigned helper_stack = stack - 12;
constexpr unsigned scratch_bytes = 1024;
constexpr unsigned sentinel = 0xcd;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned data_mask = 0x3f;
constexpr unsigned external_mask = 0x7f;
constexpr unsigned strobe = 0x80;
constexpr unsigned update_mode = 12;
constexpr unsigned interrupt_masked = 0xd0;
constexpr unsigned pe_mask = 0xfb;
constexpr unsigned packed_word_bits = 24;
constexpr unsigned application_wait = 0x8314;
constexpr unsigned application_dispatch = 0x8315;
constexpr unsigned request_flags = 0x1815;
constexpr unsigned request_handler_return = 0xba62;
constexpr unsigned xirq_vector = 0xfff4;
constexpr unsigned unmasked_flags = 0x80;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

}

void boot(blofeld::McuBus &bus, blofeld::Hc12 &cpu, std::ostream &trace) {
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = pe_mask, .levels = 3});
  bus.set_pin_drive(blofeld::McuPort::q, {.mask = byte_mask});
  cpu.reset();
  bool phy = false;
  unsigned instructions = 0;
  while (cpu.state().pc != application) {
    require(instructions < boot_limit, "Original boot must reach its application entry");
    const auto pc = cpu.state().pc;
    if (pc == phy_poll && !phy) {
      bus.observe_phy({.edges = 15});
      phy = true;
    }
    if (pc == strap_first || pc == strap_second) {
      const auto output = bus.observe_pads(blofeld::McuPort::e, 4);
      require(output.known == 4, "Controlled E2/E3 loopback requires a known original output");
      const auto levels = static_cast<std::uint8_t>(3U | ((output.levels & 4U) != 0 ? 8U : 0U));
      bus.set_pin_drive(blofeld::McuPort::e, {.mask = pe_mask, .levels = levels});
    }
    cpu.step();
    ++instructions;
  }
  require(bus.read(blofeld::hc12::modrr) == 0 && cpu.state().sp == stack,
          "Original strap test retains 100-pin routing and completes its own stack/checksum");
  trace << "conditional-boot instructions=" << instructions << " modrr=0\n";
}

namespace {
std::vector<std::uint8_t> encode(const std::span<const std::uint8_t> payload) {
  require(payload.size() < scratch_bytes, "Fixture payload must fit its isolated target window");
  std::vector<std::uint8_t> bytes{static_cast<std::uint8_t>(payload.size() >> 8U),
                                  static_cast<std::uint8_t>(payload.size() & byte_mask)};
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  while (bytes.size() % 3 != 0) { bytes.push_back(0); }
  std::vector<std::uint8_t> samples;
  for (std::size_t index = 0; index < bytes.size(); index += 3) {
    const auto word = (static_cast<unsigned>(bytes[index]) << 16U) |
                      (static_cast<unsigned>(bytes[index + 1]) << 8U) | bytes[index + 2];
    for (unsigned remaining = packed_word_bits; remaining != 0; remaining -= 6) {
      samples.push_back(static_cast<std::uint8_t>((word >> (remaining - 6)) & data_mask));
    }
  }
  return samples;
}

void prepare(blofeld::McuBus &bus, blofeld::Hc12 &cpu, const bool dispatch) {
  for (unsigned offset = 0; offset < scratch_bytes; ++offset) {
    bus.write(static_cast<std::uint16_t>(helper + offset), sentinel);
  }
  constexpr unsigned jsr_extended = 0x16;
  bus.write(caller, jsr_extended);
  bus.write(caller + 1, receiver >> 8U);
  bus.write(caller + 2, receiver & byte_mask);
  if (dispatch) {
    require((bus.read(request_flags) & 9U) == 0, "Native boot leaves the request unasserted");
    bus.set_pin_drive(blofeld::McuPort::q, {.mask = byte_mask, .levels = update_mode});
    cpu.set_state({.pc = application_wait, .sp = stack, .ccr = unmasked_flags});
  } else {
    bus.set_pin_drive(blofeld::McuPort::q, {.mask = external_mask, .levels = update_mode});
    cpu.set_state({.pc = caller, .sp = stack, .ccr = interrupt_masked});
  }
}

void check_target(blofeld::McuBus &bus, const std::span<const std::uint8_t> expected) {
  for (unsigned offset = 0; offset < scratch_bytes; ++offset) {
    const auto value = offset < expected.size() ? expected[offset] : sentinel;
    require(bus.read(static_cast<std::uint16_t>(helper + offset)) == value,
            "Native copy writes exactly the independently expected prefix and preserves its tail");
  }
}

void check_entry(blofeld::McuBus &bus, const blofeld::Hc12 &cpu) {
  require(cpu.state().pc == helper && cpu.state().d == service_table &&
              cpu.state().sp == helper_stack &&
              bus.read_memory({.address = helper_stack, .size = 2}) == helper_return,
          "Original LDD/JSR supplies its real service-table argument and return frame");
}

class RequestDelivery {
public:
  explicit RequestDelivery(const bool enabled) : enabled_(enabled) {}

  void before_step(blofeld::McuBus &bus, blofeld::Hc12 &cpu) {
    if (!enabled_ || !cpu.waiting() || injected_) { return; }
    const auto state = cpu.state();
    require(state.pc == application_dispatch && state.sp == stack - 9 &&
                (bus.read(request_flags) & 9U) == 0,
            "Original application WAI waits with its own frame before request delivery");
    for (unsigned idle = 0; idle < 2; ++idle) {
      require(!cpu.step().instruction_executed && cpu.waiting(),
              "Original wait does not fabricate a request while clocks/inputs are unchanged");
    }
    bus.set_pin_drive(blofeld::McuPort::e, {.mask = pe_mask, .levels = 2});
    injected_ = true;
  }

  void after_step(blofeld::McuBus &bus,
                  const blofeld::Hc12 &cpu,
                  const blofeld::CpuState &before,
                  const blofeld::CpuStep &result) {
    if (result.interrupt_vector) {
      require(enabled_ && *result.interrupt_vector == xirq_vector && !result.instruction_executed &&
                  result.stack_reused,
              "Original request enters through native XIRQ using the WAI frame");
      ++entries_;
      bus.set_pin_drive(blofeld::McuPort::e, {.mask = pe_mask, .levels = 3});
    }
    if (enabled_ && before.pc == request_handler_return) {
      require(cpu.state().pc == application_dispatch && cpu.state().sp == stack &&
                  (bus.read(request_flags) & 9U) == 9,
              "Original XIRQ instructions set the request and RTI restores the main wait context");
      ++returns_;
      bus.set_pin_drive(blofeld::McuPort::q, {.mask = external_mask, .levels = update_mode});
    }
  }

  void verify(blofeld::McuBus &bus, std::ostream &trace) const {
    if (!enabled_) { return; }
    require(injected_ && entries_ == 1 && returns_ == 1 && (bus.read(request_flags) & 8U) == 0,
            "Original main dispatch consumes the request without host flag writes or redirects");
    trace << "native-request xirq-entries=1 rti-returns=1 main-dispatch=1\n";
  }

private:
  bool enabled_ = false;
  bool injected_ = false;
  unsigned entries_ = 0;
  unsigned returns_ = 0;
};

}

void transfer(blofeld::McuBus &bus,
              blofeld::Hc12 &cpu,
              const Transfer &request,
              std::ostream &trace) {
  prepare(bus, cpu, request.dispatch);
  const auto samples = encode(request.payload);
  std::size_t consumed = 0;
  unsigned instructions = 0;
  unsigned steps = 0;
  RequestDelivery delivery(request.dispatch);
  while (cpu.state().pc != helper) {
    require(steps < call_limit, "Original receiver must reach an explicit bounded outcome");
    const auto state = cpu.state();
    delivery.before_step(bus, cpu);
    if (state.pc == sample_pc) {
      const auto output = bus.observe_pads(blofeld::McuPort::q, strobe);
      require(output.known == strobe && output.levels == (consumed % 2 == 0 ? strobe : 0),
              "Original receiver alternates its own Q7 strobe between six-bit observations");
      if (request.loss_at && consumed == *request.loss_at) {
        bus.set_pin_drive(blofeld::McuPort::q, {.mask = 0x40});
      } else {
        require(consumed < samples.size(), "Receiver requested data beyond the complete fixture");
        bus.set_pin_drive(blofeld::McuPort::q,
                          {.mask = external_mask, .levels = samples[consumed++]});
      }
    }
    trace << "pc=" << blofeld::hex(state.pc) << " d=" << blofeld::hex(state.d)
          << " sp=" << blofeld::hex(state.sp) << '\n';
    try {
      const auto result = cpu.step();
      if (result.instruction_executed) { ++instructions; }
      delivery.after_step(bus, cpu, state, result);
    } catch (const blofeld::Fault &fault) {
      trace << "observed-fault " << fault.what() << '\n';
      const std::string_view message(fault.what());
      require(request.loss_at && consumed == *request.loss_at && state.pc == sample_pc &&
                  message.contains("Unknown MCU GPIO input at 0x0260 mask=0x3f") &&
                  message.contains("pc=0xbad3") && message.contains("ccr="),
              "Only the deliberately withdrawn parallel input may fail this transfer");
      const auto decoded = (consumed / 4) * 3;
      const auto copied = decoded > 2 ? decoded - 2 : 0;
      check_target(bus, request.payload.first(copied));
      trace << "input-loss samples=" << consumed << " copied=" << copied << '\n';
      return;
    }
    ++steps;
  }
  require(!request.loss_at && consumed == samples.size(),
          "Complete input reaches the helper using the exact expected sample count");
  check_target(bus, request.payload);
  check_entry(bus, cpu);
  delivery.verify(bus, trace);
  trace << "helper-entry bytes=" << request.payload.size() << " samples=" << consumed
        << " instructions=" << instructions << " return=" << blofeld::hex(helper_return) << '\n';
}

void check_first_callback(blofeld::McuBus &bus, blofeld::Hc12 &cpu, std::ostream &trace) {
  unsigned instructions = 0;
  while (cpu.state().pc != first_service) {
    require(instructions < 12, "First helper callback must resolve through the supplied table");
    trace << "callback-pc=" << blofeld::hex(cpu.state().pc) << '\n';
    cpu.step();
    ++instructions;
  }
  require(cpu.state().y == service_table && cpu.state().sp == helper_stack - 4 &&
              bus.read_memory({.address = helper_stack - 4, .size = 2}) == helper_callback_return,
          "Unchanged helper TFR and indirect JSR use the original function table and stack");
}

std::span<const std::uint8_t> original_helper(const std::vector<std::uint8_t> &block,
                                              const unsigned size) {
  require(block.size() >= size + 2 &&
              ((static_cast<unsigned>(block.at(0)) << 8U) | block.at(1)) == size,
          "Supplied original object retains its independently checked helper size");
  return std::span<const std::uint8_t>(block).subspan(2, size);
}

}
