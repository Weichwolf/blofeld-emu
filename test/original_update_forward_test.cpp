// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/gpio_pads.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "dsp_norm_fixture.hpp"
#include "original_gpio_pair_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <span>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::gpio_pair;
using blofeld::test::require;
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

unsigned word(const std::span<const std::uint8_t> data, const unsigned index) {
  const auto begin = static_cast<std::size_t>(index) * word_bytes;
  require(begin + word_bytes <= data.size(), "Original forwarding word must be present");
  return (static_cast<unsigned>(data[begin]) << 16U) |
         (static_cast<unsigned>(data[begin + 1]) << 8U) | data[begin + 2];
}

struct Forward {
  OriginalGpioPair pair;
  const Firmware &firmware;
  std::vector<unsigned> samples;
  unsigned sent = 0, received = 0, loaded = 0, irq_entries = 0, irq_returns = 0;
  unsigned irq_queries = 0, unknown_irq_queries = 0, last_irq_known = 0, last_irq_levels = 0;

  Forward(const Firmware &image, std::ostream &trace)
      : pair(image, cases.back(), trace), firmware(image) {
    const auto &bytes = firmware.update_blocks.at(payload_block);
    const auto offset = (source - payload_base) * word_bytes;
    require(bytes.size() == offset + object_stream_bytes && bytes[offset] == 0 &&
                bytes[offset + 1] == expected_bytes,
            "Original error object retains its encoded size and full stream");
    for (unsigned index = 0; index < expected_samples; ++index) {
      const auto packed = word(std::span(bytes).subspan(offset), index / chunks_per_word);
      samples.push_back((packed >> (first_chunk_shift - ((index % chunks_per_word) * chunk_bits))) &
                        data_mask);
    }
    install();
    pair.additional_completion.observe = [this](const DspScheduledStep &step) { observe(step); };
  }

  void install() {
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
    // Declared pre-existing GPIO output state; input writes do not qualify latches.
    pair.dsp.bus()
        .gpio(DspPort::e)
        .write({.reg = DspGpioRegister::direction, .value = port_e_strobe, .mask = port_e_strobe});
    pair.dsp.bus()
        .gpio(DspPort::e)
        .write({.reg = DspGpioRegister::data, .value = port_e_strobe, .mask = port_e_strobe});
    blofeld::test::DspProgram caller_code;
    caller_code.immediate("sr", masked_sr);
    caller_code.word(absolute_call);
    caller_code.word(update_entry);
    caller_code.word(absolute_jump);
    caller_code.word(blofeld::test::boot);
    caller_code.install(pair.dsp.bus());
    const std::array<std::uint32_t, branch_bytes> jump{absolute_jump, blofeld::test::boot};
    pair.dsp.bus().install_rom({.start = pair.foreground, .words = jump});
    // Own native busy loop waits for the original IRQ handler's request flag.
    for (unsigned n = 0; n < request_caller.size(); ++n) {
      pair.mcu.bus().write(static_cast<std::uint16_t>(caller + n), request_caller[n]);
    }
    pair.mcu.bus().write(hc12::ddrq, handshake_strobe);
    pair.mcu.bus().write(hc12::ptq, handshake_strobe);
    auto state = pair.mcu.cpu().state();
    state.ccr = foreground_ccr;
    pair.mcu.cpu().set_state(state);
    pair.mcu.bus().set_pin_drive(McuPort::e, {});
    pair.mcu.bus().attach_gpio_pad_source(McuPort::e, [this](const std::uint32_t bits) {
      const auto drive = pair.dsp.bus().gpio(DspPort::e).outputs();
      require((pair.mcu.bus().gpio_outputs(McuPort::e).mask & bits & 3U) == 0,
              "The declared IRQ route requires MCU inputs, not opposed local outputs");
      const bool known = (drive.mask & drive.known & port_e_strobe) != 0;
      const auto known_bits = (idle_irq_input | (known ? irq_input : 0U)) & bits;
      const auto levels = idle_irq_input | ((drive.levels & port_e_strobe) != 0 ? irq_input : 0U);
      ++irq_queries;
      unknown_irq_queries += static_cast<unsigned>(known_bits != bits);
      const auto resolved = levels & known_bits;
      if (known_bits != last_irq_known || resolved != last_irq_levels) {
        pair.trace << "IRQ_PAD known=" << known_bits << " levels=" << resolved << '\n';
        last_irq_known = known_bits;
        last_irq_levels = resolved;
      }
      return GpioPadSample{.known = known_bits, .levels = resolved};
    });
    pair.sync();
  }

  void observe(const DspScheduledStep &step) {
    pair.mcu.bus().observe_interrupt_pads();
    const auto pc = step.plan.before.cpu.pc;
    if (pc == stream_read) {
      const auto address = step.plan.before.cpu.r[1];
      const auto value = (pair.dsp.cpu().state().a >> word_bits) & word_mask;
      const auto &bytes = firmware.update_blocks.at(payload_block);
      require(address == source + loaded && value == word(bytes, source - payload_base + loaded) &&
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
    for (unsigned n = 0; n < request_caller.size(); ++n) {
      require(pair.mcu.bus().read(static_cast<std::uint16_t>(caller + n)) == request_caller[n],
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
    const auto result = pair.mcu.cpu().step();
    ++pair.link.msteps;
    pair.schedule->end_instruction();
    if (result.interrupt_vector) {
      require(result.interrupt_vector == xirq_vector && !result.instruction_executed &&
                  !result.stack_reused && pair.mcu.cpu().state().pc == request_irq &&
                  pair.mcu.cpu().state().sp == stack - interrupt_frame_bytes,
              "Native XIRQ entry must fetch the original vector and save the real CPU frame");
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
        require(pair.mcu.cpu().state().pc >= caller &&
                    pair.mcu.cpu().state().pc < caller + request_loop_bytes &&
                    (pair.mcu.bus().read(bus_flags) & hold_flags) == hold_flags,
                "Actual original RTI must return to the native request-flag loop");
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
}

int main(const int argc, char **argv) {
  try {
    require(argc == 3, "Original update forwarding needs firmware and private trace");
    const auto firmware = Firmware::load(argv[1]);
    std::ofstream trace(argv[2]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    for (const unsigned order : {0U, 1U}) {
      for (const unsigned phase : {0U, phase_offset}) {
        trace << "CASE order=" << order << " phase=" << phase << '\n';
        Forward run(firmware, trace);
        const auto nvm = run.pair.mcu.bus().power_off_flash_image();
        run.execute(order, phase);
        run.verify(nvm);
      }
    }
    std::cout << "Original DSP/MCU error helper: four native IRQ/transfer/call contexts.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
