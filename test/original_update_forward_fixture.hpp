// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/gpio_pads.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <span>
#include <string_view>
#include <tuple>
#include <vector>

namespace blofeld::test::update_forward {
using namespace blofeld;
using namespace blofeld::test::gpio_pair;
using blofeld::test::require;
using blofeld::test::gpio_pair::word_bits;
using blofeld::test::gpio_pair::word_mask;
constexpr unsigned update_entry = 0x2e00;
constexpr unsigned helper_words = 400;
constexpr unsigned marker = 0x14477;
constexpr unsigned source = marker + 1;
constexpr unsigned payload_base = 0xd000;
constexpr unsigned payload_block = 4;
constexpr unsigned helper_block = 5;
constexpr unsigned receiver = 0xbad9;
constexpr unsigned receiver_sample = 0xbad3;
constexpr unsigned target = 0x1200;
constexpr unsigned expected_bytes = 188;
constexpr unsigned expected_samples = 256;
constexpr unsigned service_table = 0xf81b;
constexpr unsigned helper_return = 0xbb56;
constexpr unsigned call_limit = 100000;
constexpr unsigned masked_sr = 0x300;
constexpr unsigned port_e_strobe = 0x200;
constexpr unsigned chunks_per_word = 4;
constexpr unsigned chunk_bits = 6;
constexpr unsigned first_chunk_shift = 18;
constexpr unsigned object_stream_bytes = 195;
constexpr unsigned stream_samples = (object_stream_bytes / 3) * chunks_per_word;
constexpr unsigned accumulator_extension_shift = 48;
constexpr unsigned data_outputs = 0x7f;
constexpr std::array<unsigned, 4> chunk_pcs{0x2e63, 0x2e68, 0x2e6d, 0x2e72};
constexpr unsigned stream_entry = 0x2e5e;
constexpr unsigned stream_read = stream_entry;
constexpr unsigned word_bytes = 3;
constexpr unsigned size_bytes = 2;
constexpr unsigned data_mask = 0x3f;
constexpr unsigned branch_bytes = 2;
constexpr unsigned absolute_call = 0x0bf080;
constexpr unsigned guard_bytes = 512;
constexpr unsigned entry_stack_bytes = 12;
constexpr unsigned handshake_strobe = 0x80;
constexpr unsigned prefetched_words = 65;
constexpr unsigned request_flag = 8;
constexpr unsigned request_irq = 0xb9c6;
constexpr unsigned request_rti = 0xba62;
constexpr unsigned foreground_ccr = 0x80;
constexpr unsigned load_b_extended = 0xf6;
constexpr unsigned and_b_immediate = 0xc4;
constexpr unsigned branch_equal = 0x27;
constexpr unsigned repeat_request_loop = 0xf9;
constexpr unsigned irq_input = 1;
constexpr unsigned idle_irq_input = 2;
constexpr unsigned caller_size = 12;
constexpr unsigned xirq_vector = 0xfff4;
constexpr unsigned interrupt_frame_bytes = 9;
constexpr unsigned request_loop_bytes = 7;
constexpr unsigned hold_flags = 9;
constexpr std::array<std::uint8_t, caller_size> request_caller{
    load_b_extended,
    static_cast<std::uint8_t>(bus_flags >> 8U),
    static_cast<std::uint8_t>(bus_flags &mask),
    and_b_immediate,
    request_flag,
    branch_equal,
    repeat_request_loop,
    0x16,
    static_cast<std::uint8_t>(receiver >> 8U),
    static_cast<std::uint8_t>(receiver &mask),
    branch_opcode,
    branch_self};

inline unsigned word(const std::span<const std::uint8_t> data, const unsigned index) {
  const auto begin = static_cast<std::size_t>(index) * word_bytes;
  require(begin + word_bytes <= data.size(), "Original forwarding word must be present");
  return (static_cast<unsigned>(data[begin]) << 16U) |
         (static_cast<unsigned>(data[begin + 1]) << 8U) | data[begin + 2];
}

inline std::vector<std::uint8_t> caller_program(const bool diagnostic) {
  std::vector<std::uint8_t> code;
  if (diagnostic) {
    // Own native GPIO/display setup, followed by the unchanged request loop.
    constexpr std::array<std::uint8_t, 23> prefix{0x86, 0xf7, 0x7a, 0, 3,    0x86, 0x80, 0x7a,
                                                  0,    1,    0x86, 0, 0x7a, 0,    8,    0x86,
                                                  0x90, 0x7a, 0,    9, 0x16, 0x8b, 0x94};
    code.insert(code.end(), prefix.begin(), prefix.end());
  }
  code.insert(code.end(), request_caller.begin(), request_caller.end());
  return code;
}

inline auto registers(const CpuState &state) {
  return std::tuple(state.pc, state.d, state.x, state.y, state.sp, state.ccr);
}

using PayloadPreparation = std::function<void(OriginalGpioPair &)>;

struct Forward {
  OriginalGpioPair pair;
  const Firmware &firmware;
  const std::vector<std::uint8_t> own_caller;
  std::unique_ptr<blofeld::test::LcdFixture> display;
  std::optional<CpuState> interrupted;
  std::vector<unsigned> samples;
  const std::vector<unsigned> transport_tail;
  unsigned sent = 0, received = 0, loaded = 0, irq_entries = 0, irq_returns = 0;
  unsigned irq_queries = 0, unknown_irq_queries = 0, last_irq_known = 0, last_irq_levels = 0;

  Forward(const Firmware &image,
          std::ostream &trace,
          const bool diagnostic = false,
          const PayloadPreparation &prepare = {},
          const std::vector<unsigned> &tail = {})
      : pair(image, cases.back(), trace),
        firmware(image),
        own_caller(caller_program(diagnostic)),
        transport_tail(tail) {
    if (diagnostic) {
      pair.mcu.bus().set_pin_drive(McuPort::b, {.mask = 8}); // Explicit idle B3 input.
      display = std::make_unique<blofeld::test::LcdFixture>(pair.mcu.bus());
      pair.additional_bus_write = [this](const BusEvent &event) { display->observe(event); };
      pair.mcu.bus().write(bus_flags, 1); // Declared direct LCD I/O context before live execution.
    }
    const auto &bytes = firmware.update_blocks.at(payload_block);
    const auto offset = (source - payload_base) * word_bytes;
    require(bytes.size() == offset + object_stream_bytes && bytes[offset] == 0 &&
                bytes[offset + 1] == expected_bytes,
            "Original error object retains its encoded size and full stream");
    for (unsigned index = 0; index < stream_samples; ++index) {
      const auto packed = word(std::span(bytes).subspan(offset), index / chunks_per_word);
      samples.push_back((packed >> (first_chunk_shift - ((index % chunks_per_word) * chunk_bits))) &
                        data_mask);
    }
    for (const auto value : transport_tail) {
      for (const unsigned shift : {18U, 12U, 6U, 0U}) {
        samples.push_back((value >> shift) & data_mask);
      }
    }
    install(prepare);
    pair.additional_completion.observe = [this](const DspScheduledStep &step) { observe(step); };
  }

  void install(const PayloadPreparation &prepare) {
    if (prepare) {
      prepare(pair);
    } else {
      install_source();
    }
    install_connections();
  }

  void install_source() {
    const auto &helper = firmware.update_blocks.at(helper_block);
    const auto &payload = firmware.update_blocks.at(payload_block);
    require(helper.size() == static_cast<std::size_t>(helper_words) * word_bytes,
            "Original main helper retains all words");
    for (unsigned n = 0; n < helper_words; ++n) {
      pair.dsp.bus().write(
          {.space = DspSpace::p, .address = update_entry + n, .value = word(helper, n)});
    }
    for (unsigned n = 0; n < payload.size() / word_bytes; ++n) {
      pair.dsp.bus().write(
          {.space = DspSpace::y, .address = payload_base + n, .value = word(payload, n)});
    }
    pair.dsp.bus().write({.space = DspSpace::y, .address = marker, .value = 0});
    blofeld::test::DspProgram caller_code;
    caller_code.immediate("sr", masked_sr);
    caller_code.word(absolute_call);
    caller_code.word(update_entry);
    caller_code.word(absolute_jump);
    caller_code.word(blofeld::test::boot);
    caller_code.install(pair.dsp.bus());
    const std::array<std::uint32_t, branch_bytes> jump{absolute_jump, blofeld::test::boot};
    pair.dsp.bus().install_rom({.start = pair.foreground, .words = jump});
  }

  void install_connections() {
    // Declared pre-existing GPIO output state; input writes do not qualify latches.
    pair.dsp.bus()
        .gpio(DspPort::e)
        .write({.reg = DspGpioRegister::direction, .value = port_e_strobe, .mask = port_e_strobe});
    pair.dsp.bus()
        .gpio(DspPort::e)
        .write({.reg = DspGpioRegister::data, .value = port_e_strobe, .mask = port_e_strobe});
    // Own native busy loop waits for the original IRQ handler's request flag.
    for (unsigned n = 0; n < own_caller.size(); ++n) {
      pair.mcu.bus().write(static_cast<std::uint16_t>(caller + n), own_caller[n]);
    }
    pair.mcu.bus().write(hc12::ddrq, handshake_strobe);
    pair.mcu.bus().write(hc12::ptq, handshake_strobe);
    auto state = pair.mcu.cpu().state();
    state.ccr = foreground_ccr;
    pair.mcu.cpu().set_state(state);
    pair.mcu.bus().set_pin_drive(McuPort::e, {});
    attach_interrupt_pads();
    pair.sync();
  }

  void attach_interrupt_pads() {
    pair.mcu.bus().attach_gpio_pad_source(McuPort::e, [this](const std::uint32_t bits) {
      const auto drive = pair.dsp.bus().gpio(DspPort::e).outputs();
      const auto local = pair.mcu.bus().gpio_outputs(McuPort::e);
      require((local.mask & bits & 3U) == 0,
              "The declared IRQ route requires MCU inputs, not opposed local outputs");
      const bool known = (drive.mask & drive.known & port_e_strobe) != 0;
      const auto irq_known = idle_irq_input | (known ? irq_input : 0U);
      const auto irq_levels =
          idle_irq_input | ((drive.levels & port_e_strobe) != 0 ? irq_input : 0U);
      const auto known_bits = (irq_known | (local.known & ~3U)) & bits;
      const auto resolved = (irq_levels | (local.levels & ~3U)) & known_bits;
      if ((bits & 3U) != 0) {
        ++irq_queries;
        unknown_irq_queries += static_cast<unsigned>((irq_known & bits) != (bits & 3U));
        if (irq_known != last_irq_known || irq_levels != last_irq_levels) {
          pair.trace << "IRQ_PAD known=" << irq_known << " levels=" << irq_levels << '\n';
          last_irq_known = irq_known;
          last_irq_levels = irq_levels;
        }
      }
      return GpioPadSample{.known = known_bits, .levels = resolved};
    });
  }

  void observe(const DspScheduledStep &step) {
    pair.mcu.bus().observe_interrupt_pads();
    const auto pc = step.plan.before.cpu.pc;
    if (pc == stream_read) {
      const auto address = step.plan.before.cpu.r[1];
      const auto value = (pair.dsp.cpu().state().a >> word_bits) & word_mask;
      const auto &bytes = firmware.update_blocks.at(payload_block);
      const auto index = source - payload_base + loaded;
      const auto original_words = static_cast<unsigned>(bytes.size() / word_bytes);
      const auto expected =
          index < original_words ? word(bytes, index) : transport_tail.at(index - original_words);
      require(address == source + loaded && value == expected &&
                  pair.dsp.cpu().state().r[1] == address + 1 &&
                  pair.dsp.cpu().state().pc == stream_read + 1,
              "Actual original source read/register/cursor must match the next original word");
      pair.trace << "SOURCE index=" << loaded << " address=" << address << " value=" << value
                 << '\n';
      ++loaded;
    }
    if (std::ranges::find(chunk_pcs, pc) == chunk_pcs.end()) { return; }
    const auto value = (step.plan.before.cpu.a >> accumulator_extension_shift) & mask;
    const auto outputs = pair.dsp.bus().gpio(DspPort::c).outputs();
    require(sent < samples.size() && value == samples.at(sent) &&
                (outputs.mask & outputs.known & data_outputs) == data_outputs &&
                (outputs.levels & data_outputs) == value && pair.dsp.cpu().state().pc == pc + 1,
            "Actual original MOVEP source/register effect must match the independent group");
    pair.trace << "CHUNK index=" << sent << " value=" << value << " pc=" << pc << '\n';
    ++sent;
  }

  void verify(const McuFlashImage &nvm) {
    const auto state = pair.mcu.cpu().state();
    require(sent == expected_samples && received == sent && loaded == prefetched_words &&
                state.pc == target && state.d == service_table &&
                state.sp == stack - entry_stack_bytes && irq_entries == 1 && irq_returns == 1 &&
                pair.mcu.bus().read_memory({.address = state.sp, .size = 2}) == helper_return,
            "Real original copy/JSR must deliver the helper with its actual service-table frame");
    const auto &bytes = firmware.update_blocks.at(payload_block);
    const auto offset = (source - payload_base) * word_bytes;
    for (unsigned n = 0; n < guard_bytes; ++n) {
      const auto expected = n < expected_bytes ? bytes[offset + size_bytes + n] : ram_canary;
      require(pair.mcu.bus().read(static_cast<std::uint16_t>(target + n)) == expected,
              "Original native receiver copies exactly the helper and preserves its RAM tail");
    }
    const auto &helper = firmware.update_blocks.at(helper_block);
    for (unsigned n = 0; n < helper_words; ++n) {
      require(pair.dsp.bus().read({.space = DspSpace::p, .address = update_entry + n}) ==
                  word(helper, n),
              "Original forwarding program must remain unchanged");
    }
    for (unsigned n = 0; n < bytes.size() / word_bytes; ++n) {
      const auto address = payload_base + n;
      require(pair.dsp.bus().read({.space = DspSpace::y, .address = address}) ==
                  (address == marker ? 0 : word(bytes, n)),
              "Forwarding preserves every explicitly prepared payload word");
    }
    const auto output = pair.mcu.bus().gpio_outputs(McuPort::q);
    require(output.mask == handshake_strobe && output.levels == 0 &&
                pair.link.unresolved_queries == 0 && unknown_irq_queries == 0,
            "Native final Q7 phase and every consumed pad observation remain known");
    for (unsigned n = 0; n < own_caller.size(); ++n) {
      require(pair.mcu.bus().read(static_cast<std::uint16_t>(caller + n)) == own_caller[n],
              "Own native request caller must remain unchanged");
    }
    bool rejected = false;
    try {
      pair.reject_board_qualification();
    } catch (const Fault &error) {
      require(std::string_view(error.what()).contains("Board qualification failed"),
              "Unexpected board audit diagnostic");
      rejected = true;
    }
    require(rejected, "A logical helper transfer must not qualify unresolved board intervals");
    pair.verify_retained(firmware, nvm);
    pair.trace << "RESULT bytes=" << expected_bytes << " chunks=" << sent << " samples=" << received
               << " source-loads=" << loaded << " irq-entries=" << irq_entries
               << " irq-returns=" << irq_returns << " irq-queries=" << irq_queries
               << " irq-unknown=" << unknown_irq_queries << " queries=" << pair.link.pad_queries
               << " unknown-queries=" << pair.link.unresolved_queries
               << " unresolved-events=" << pair.link.unresolved_events << " mcu-pc=" << state.pc
               << " mcu-d=" << state.d << " mcu-sp=" << state.sp
               << " nvm-unchanged=1 program-unchanged=1 payload-unchanged=1 caller-unchanged=1"
                  " board-rejected=1\n";
  }

  void native_step() {
    pair.trace << "M " << pair.mcu.run_clock().elapsed_ticks() << ' '
               << hex(pair.mcu.cpu().state().pc) << '\n';
    const auto before = pair.mcu.cpu().state();
    const auto result = pair.mcu.cpu().step();
    ++pair.link.msteps;
    pair.schedule->end_instruction();
    if (result.interrupt_vector) {
      require(result.interrupt_vector == xirq_vector && !result.instruction_executed &&
                  !result.stack_reused && pair.mcu.cpu().state().pc == request_irq &&
                  pair.mcu.cpu().state().sp == before.sp - interrupt_frame_bytes,
              "Native XIRQ entry must fetch the original vector and save the real CPU frame");
      interrupted = before;
      ++irq_entries;
      pair.trace << "IRQ_ENTRY vector=" << *result.interrupt_vector
                 << " sp=" << pair.mcu.cpu().state().sp << '\n';
    }
  }

  void execute(const unsigned order, const unsigned phase) {
    pair.start(order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
               phase,
               target);
    for (unsigned steps = 0; steps < call_limit; ++steps) {
      const auto pc = pair.mcu.cpu().state().pc;
      if (pc == target) { return; }
      native_step();
      if (pc == request_rti) {
        require(interrupted && registers(pair.mcu.cpu().state()) == registers(*interrupted) &&
                    (pair.mcu.bus().read(bus_flags) & hold_flags) == hold_flags,
                "Actual original RTI must restore every interrupted foreground register");
        ++irq_returns;
        pair.trace << "IRQ_RETURN pc=" << pair.mcu.cpu().state().pc
                   << " sp=" << pair.mcu.cpu().state().sp << '\n';
      }
      if (pc == receiver_sample) {
        const auto value = pair.mcu.cpu().state().d & data_mask;
        require(received < samples.size() && value == samples.at(received),
                "Actual native MCU input instruction must consume the emitted DSP group");
        pair.trace << "CONSUMER index=" << received << " value=" << value << '\n';
        ++received;
      }
    }
    throw Fault("Original forwarding did not reach the native MCU helper entry");
  }
};
} // namespace blofeld::test::update_forward
