// SPDX-License-Identifier: GPL-3.0-only
#include "application_update_fixture.hpp"

#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "parallel_update_fixture.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <ostream>
#include <span>
#include <string_view>
#include <vector>

namespace blofeld::test {
namespace {
constexpr unsigned byte_mask = 255;
constexpr unsigned data_mask = 0x3f;
constexpr unsigned strobe = 0x80;
constexpr unsigned external_mask = 0x7f;
constexpr unsigned e_mask = 0xeb;
constexpr unsigned enable_pad = 0x10;
constexpr unsigned released_e_input = 4;
constexpr unsigned released_e_pads = enable_pad | released_e_input;
constexpr unsigned mode = 12;
constexpr unsigned main_resume = 0x8315;
constexpr unsigned main_stack = 0x25f3;
constexpr unsigned wai_stack = main_stack - 9;
constexpr unsigned helper = 0x1200;
constexpr unsigned helper_stack = main_stack - 12;
constexpr unsigned helper_return = 0xbb56;
constexpr unsigned helper_bytes = 761;
constexpr unsigned main_helper_bytes = 632;
constexpr unsigned first_object_bytes = 768;
constexpr unsigned main_prefix_bytes = 636;
constexpr unsigned first_entry_samples = 1020;
constexpr unsigned first_complete_samples = 1032;
constexpr unsigned main_entry_samples = 848;
constexpr unsigned sample_pc = 0xbad3;
constexpr unsigned q_output_enabled = 0xbaf7;
constexpr unsigned request_return = 0xba62;
constexpr unsigned request_flags = 0x1815;
constexpr unsigned xirq_vector = 0xfff4;
constexpr unsigned service_table = 0xf81b;
constexpr unsigned drain_call = 0xbb69;
constexpr unsigned release_poll = 0xbb83;
constexpr unsigned flash_frontier = 0x13bb;
constexpr unsigned loaded_divider = 0xc8;
constexpr unsigned idle_flash = 0xc0;
constexpr unsigned flash_status = 0x105;
constexpr unsigned flash_command = 0x106;
constexpr unsigned phase_limit = 300000;

enum class Object : std::uint8_t { prepare, main };

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

bool same_state(const CpuState &left, const CpuState &right) {
  return left.pc == right.pc && left.sp == right.sp && left.d == right.d && left.x == right.x &&
         left.y == right.y && left.ccr == right.ccr;
}

void check_nvm(const McuBus &bus, const McuFlashImage &expected) {
  const auto current = bus.power_off_flash_image();
  require(current.bytes == expected.bytes && current.known == expected.known &&
              current.fresh == expected.fresh,
          "Native request, helpers and withdrawn reads preserve all MCU NVM and history");
}

std::vector<std::uint8_t> samples(const Firmware &firmware, const Object object) {
  const auto &block = firmware.update_blocks.at(object == Object::prepare ? 0 : 4);
  const auto count = object == Object::prepare ? first_object_bytes : main_prefix_bytes;
  require(block.size() >= count, "Verified update contains the full original helper prefix");
  std::vector<std::uint8_t> bytes(block.begin(), block.begin() + count);
  if (object == Object::prepare) {
    // Explicit own peer drain bytes after the complete original block/checksum.
    constexpr unsigned drain_bytes = 6;
    bytes.insert(bytes.end(), drain_bytes, 0);
  }
  require(bytes.size() % 3 == 0, "The source consists of complete 24-bit channel words");
  std::vector<std::uint8_t> result;
  for (std::size_t index = 0; index < bytes.size(); index += 3) {
    const auto word = (static_cast<unsigned>(bytes[index]) << 16U) |
                      (static_cast<unsigned>(bytes[index + 1]) << 8U) | bytes[index + 2];
    constexpr std::array<unsigned, 4> shifts{18, 12, 6, 0};
    for (const auto shift : shifts) {
      result.push_back(static_cast<std::uint8_t>((word >> shift) & data_mask));
    }
  }
  return result;
}

void begin_request(McuBus &bus, Hc12 &cpu) {
  require(cpu.waiting() && cpu.state().pc == main_resume && cpu.state().sp == wai_stack &&
              !cpu.pending_interrupt() && (bus.read(request_flags) & 9U) == 0,
          "Update request starts at the application's own idle WAI with no request flag");
  require((bus.read(hc12::ddre) & released_e_input) == 0 &&
              (bus.read(hc12::pucr) & enable_pad) != 0,
          "Explicitly released E2 is a native input with its actual pull-up enabled");
  const auto before = cpu.state();
  for (unsigned idle = 0; idle < 2; ++idle) {
    const auto result = cpu.step();
    require(!result.instruction_executed && !result.interrupt_vector && cpu.waiting() &&
                same_state(before, cpu.state()),
            "Idle application cannot fabricate an update request or leave its native wait");
  }
  bus.set_pin_drive(McuPort::q, {.mask = byte_mask, .levels = mode});
  bus.set_pin_drive(McuPort::e, {.mask = e_mask, .levels = 2, .floating = released_e_pads});
}

void check_unknown_sample(McuMachine &machine,
                          std::unique_ptr<LcdFixture> &display,
                          const McuFlashImage &nvm,
                          std::ostream &trace) {
  const auto checkpoint = machine.checkpoint();
  const LcdFixture saved_display(machine.bus(), *display);
  auto &bus = machine.bus();
  auto &cpu = machine.cpu();
  bus.set_pin_drive(McuPort::q, {.mask = 0x40});
  const auto before = cpu.state();
  const auto revision = bus.revision();
  bool rejected = false;
  try {
    cpu.step();
  } catch (const Fault &error) {
    const std::string_view message(error.what());
    require(message.contains("MCU read at 0x0260") && message.contains("pc=0xbad3") &&
                message.contains("ccr=") && message.contains("mask=0x3f"),
            "Withdrawn request data fails at the actual original six-bit GPIO read");
    trace << "missing-update-data " << message << '\n';
    rejected = true;
  }
  require(rejected && same_state(before, cpu.state()) && bus.revision() == revision,
          "Rejected plain data read preserves visible CPU and bus state");
  bool stopped = false;
  try {
    cpu.step();
  } catch (const Fault &error) {
    stopped = std::string_view(error.what()).contains("requires coordinated recovery");
  }
  require(stopped && same_state(before, cpu.state()) && bus.revision() == revision,
          "A failed update data read requires explicit recovery before another CPU step");
  check_nvm(bus, nvm);
  machine.restore(checkpoint);
  display = std::make_unique<LcdFixture>(machine.bus(), saved_display);
  machine.bus().set_observer([&display](const BusEvent &event) { display->observe(event); });
  require(same_state(before, machine.cpu().state()),
          "Joint recovery restores the pre-withdrawal CPU boundary before fresh peer data");
}

class Peer {
public:
  Peer(const Firmware &firmware, const Object object)
      : object_(object),
        samples_(samples(firmware, object)),
        payload_(original_helper(firmware.update_blocks.at(object == Object::prepare ? 0 : 4),
                                 object == Object::prepare ? helper_bytes : main_helper_bytes)) {}

  void before(McuMachine &machine,
              std::unique_ptr<LcdFixture> &display,
              const McuFlashImage &nvm,
              std::ostream &trace) {
    const auto state = machine.cpu().state();
    if (state.pc == q_output_enabled) {
      machine.bus().set_pin_drive(McuPort::q, {.mask = external_mask, .levels = mode});
    }
    if (state.pc == sample_pc) {
      const auto pins = machine.bus().observe_pads(McuPort::q, strobe);
      require(pins.known == strobe && pins.levels == (consumed_ % 2 == 0 ? strobe : 0),
              "Every six-bit observation follows the original alternating Q7 strobe");
      require(consumed_ < samples_.size(), "Original code cannot read past the supplied channel");
      if (consumed_ == 0) {
        check_unknown_sample(machine, display, nvm, trace);
        ++retries_;
      }
      machine.bus().set_pin_drive(McuPort::q,
                                  {.mask = external_mask, .levels = samples_[consumed_++]});
    }
    auto &bus = machine.bus();
    const auto &cpu = machine.cpu();
    if (state.pc == helper) { check_helper(bus, cpu, trace); }
    if (state.pc == helper_return) {
      require(object_ == Object::prepare && returns_ == 0 && state.sp == main_stack - 10 &&
                  (bus.read(request_flags) & 0x40U) != 0,
              "Complete first helper returns natively after setting its own prepared flag");
      ++returns_;
    }
    if (state.pc == drain_call) { ++drains_; }
    if (state.pc == release_poll) {
      require(bus.read(hc12::ddrq) == 0 && bus.read(hc12::perq) == byte_mask &&
                  bus.read(hc12::ppsq) == byte_mask,
              "Original loader restores Q inputs and enables its actual pull-downs");
      bus.set_pin_drive(McuPort::q, {.floating = byte_mask});
      const auto pads = bus.observe_pads(McuPort::q, byte_mask);
      require(pads.known == byte_mask && pads.levels == 0,
              "Native Q pull configuration resolves the released peer to idle");
      ++releases_;
    }
  }

  void after(McuBus &bus, const Hc12 &cpu, const CpuState &before, const CpuStep &result) {
    if (result.interrupt_vector) {
      require(*result.interrupt_vector == xirq_vector && !result.instruction_executed &&
                  result.stack_reused && cpu.state().sp == wai_stack && entries_ == 0,
              "Original XIRQ reuses the natural application WAI frame exactly once");
      ++entries_;
      bus.set_pin_drive(McuPort::e, {.mask = e_mask, .levels = 3, .floating = released_e_pads});
    }
    if (before.pc == request_return) {
      require(cpu.state().pc == main_resume && cpu.state().sp == main_stack &&
                  (bus.read(request_flags) & 9U) == 9 && irq_returns_ == 0,
              "Original request ISR and RTI restore foreground and assert the request themselves");
      ++irq_returns_;
    }
  }

  void verify(const Hc12 &cpu, std::ostream &trace) const {
    require(entries_ == 1 && irq_returns_ == 1 && helpers_ == 1 && retries_ == 1 &&
                consumed_ == samples_.size(),
            "Each native request loads one unchanged helper with one coordinated read recovery");
    if (object_ == Object::prepare) {
      require(cpu.waiting() && cpu.state().pc == main_resume && cpu.state().sp == wai_stack &&
                  returns_ == 1 && drains_ == 9 && releases_ == 1 &&
                  consumed_ == first_complete_samples && !cpu.pending_interrupt(),
              "Complete first helper and nine-byte drain return to natural main WAI");
    } else {
      require(cpu.state().pc == flash_frontier && returns_ == 0 && drains_ == 0 && releases_ == 0 &&
                  consumed_ == main_entry_samples,
              "Original main helper executes UI and initialization up to the FSTAT frontier");
    }
    trace << "native-update object=" << (object_ == Object::prepare ? "prepare" : "main")
          << " samples=" << consumed_ << " xirq=" << entries_ << " rti=" << irq_returns_
          << " retries=" << retries_ << " drains=" << drains_ << " pc=" << hex(cpu.state().pc)
          << " sp=" << hex(cpu.state().sp) << '\n';
  }

private:
  void check_helper(McuBus &bus, const Hc12 &cpu, std::ostream &trace) {
    require(helpers_ == 0 && cpu.state().d == service_table && cpu.state().sp == helper_stack &&
                bus.read_memory({.address = helper_stack, .size = 2}) == helper_return,
            "Native caller supplies its own service-table argument and exact return frame");
    const auto expected_samples =
        object_ == Object::prepare ? first_entry_samples : main_entry_samples;
    require(consumed_ == expected_samples,
            "Original helper entry follows its exact encoded prefix");
    for (std::size_t index = 0; index < payload_.size(); ++index) {
      require(bus.read(static_cast<std::uint16_t>(helper + index)) == payload_[index],
              "Every loaded helper byte equals the unchanged verified firmware object");
    }
    ++helpers_;
    trace << "native-update-entry bytes=" << payload_.size() << " samples=" << consumed_ << '\n';
  }

  Object object_;
  std::vector<std::uint8_t> samples_;
  std::span<const std::uint8_t> payload_;
  std::size_t consumed_ = 0;
  unsigned entries_ = 0;
  unsigned irq_returns_ = 0;
  unsigned helpers_ = 0;
  unsigned returns_ = 0;
  unsigned retries_ = 0;
  unsigned drains_ = 0;
  unsigned releases_ = 0;
};

void run_object(McuMachine &machine,
                std::unique_ptr<LcdFixture> &display,
                const Firmware &firmware,
                const Object object,
                std::ostream &trace) {
  const auto initial = machine.bus().power_off_flash_image();
  Peer peer(firmware, object);
  begin_request(machine.bus(), machine.cpu());
  unsigned instructions = 0;
  unsigned steps = 0;
  while (true) {
    require(steps < phase_limit, "Native update entry must reach a bounded completion or frontier");
    const auto before = machine.cpu().state();
    if (object == Object::main && before.pc == flash_frontier) { break; }
    peer.before(machine, display, initial, trace);
    auto &bus = machine.bus();
    auto &cpu = machine.cpu();
    const auto result = cpu.step();
    peer.after(bus, cpu, before, result);
    if (result.instruction_executed) { ++instructions; }
    ++steps;
    if (object == Object::prepare && cpu.waiting() && !cpu.pending_interrupt()) { break; }
  }
  peer.verify(machine.cpu(), trace);
  check_nvm(machine.bus(), initial);
  trace << "native-update instructions=" << instructions << " steps=" << steps << '\n';
}
}

void verify_application_update(McuMachine &machine,
                               std::unique_ptr<LcdFixture> &display,
                               const Firmware &firmware,
                               std::ostream &trace) {
  run_object(machine, display, firmware, Object::prepare, trace);
  run_object(machine, display, firmware, Object::main, trace);
  auto &bus = machine.bus();
  require(bus.read(hc12::flash_clock_divider) == loaded_divider &&
              bus.read(flash_status) == idle_flash && bus.read(flash_command) == 0,
          "Original main updater configures its divider but has launched no flash command");
  trace << "native-update-frontier FCLKDIV=c8 FSTAT=c0 FCMD=0; idle CBEIF RMW unqualified\n";
}
}
