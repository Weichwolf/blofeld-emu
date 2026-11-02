// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "board/mcu_dsp_runtime.hpp"
#include "board/nand_peer.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_boot_reference.hpp"
#include "dsp_document_boot_reference.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "original_owned_clock_fixture.hpp"
#include "storage/flash_image.hpp"
#include "storage/nand.hpp"
#include "storage/nand_file.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

namespace blofeld::test::reset_source {
namespace {
inline constexpr unsigned common_hz = 540000000, oscillator_hz = 12000000;

namespace wire {
inline constexpr unsigned ready_poll = 0xb98d, port_b = 1;
inline constexpr unsigned select_n = 2, clock_pin = 4, mosi_pin = 16;
inline constexpr unsigned serial_pins = select_n | clock_pin | mosi_pin;
inline constexpr unsigned other_b_inputs = 0xe8, bootstrap_base = 0xff0000;
inline constexpr unsigned frame_offset = 0x7875, stage_words = 94, stage_base = 0x6a;
inline constexpr unsigned long_ram_end = 0x2000, hcsr = 0xffff91, hrx = 0xffff94;
inline constexpr unsigned expected_port_writes = 7275;
}

struct Progress {
  owned_clock::Census census;
  owned_clock::FlashPhases phases;
  owned_clock::Interrupts interrupts;
};

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

std::uint64_t common_ticks(const McuMachine &machine, const std::uint64_t timebase = common_hz) {
  const auto &clock = machine.run_clock();
  require(timebase % clock.timebase_hz() == 0, "MCU/common timebase ratio must be exact");
  const auto factor = timebase / clock.timebase_hz();
  require(clock.elapsed_ticks() <= std::numeric_limits<std::uint64_t>::max() / factor,
          "MCU/common clock conversion must not wrap");
  return clock.elapsed_ticks() * factor;
}

void record_clock_write(const McuMachine &machine, const BusEvent &event, std::ostream &trace) {
  constexpr unsigned register_end = 0x70;
  if (event.write && event.address < register_end) {
    trace << "cycle=" << machine.run_clock().bus_cycles() << " write=" << hex(event.address) << ':'
          << hex(event.value, 2) << '\n';
  }
}

void step(McuMachine &machine, Progress &progress) {
  const auto before = machine.cpu().state();
  const auto opcode = machine.bus().peek(before.pc);
  progress.census.before(machine);
  const auto result = machine.cpu().step();
  require(result.instruction_executed || result.interrupt_vector.has_value(), "Native progress");
  progress.census.retire(before, result);
  progress.interrupts.observe(before, result, machine.cpu(), opcode);
}
}
}

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace source = blofeld::test::reset_source;
namespace coupled = source::wire;
using source::require;
constexpr unsigned u_direction = 0x27a, u_data = 0x278, release_pc = 0xb98a;
constexpr unsigned other_u = 0x3e, spi_pins = coupled::serial_pins;
constexpr unsigned all_modes = 15, spi_mode = 5;
constexpr unsigned limit = 2500000, release_boot_inputs = 0x820e;
constexpr unsigned initial_e = 0xfb, byte_mask = 0xff, receive_pad = 4;
constexpr unsigned reset_status = 0xc00300, stage_mode = 0x600389, rx_not_empty = 0x020000;
constexpr unsigned rx_overrun = 0x100000, ram_begin = 0x1200, ram_size = 5120;
constexpr unsigned recovery_periods = 2, word_bytes = 3, clear_pc = 0x72;
constexpr std::uint64_t common = source::common_hz;
constexpr unsigned no_pending_effect = 0xff;
constexpr unsigned nand_sample_pc = 0x98, accumulator_word_shift = 24;

struct Lab {
  Lab(McuMachine &native_mcu,
      source::Progress &native_progress,
      std::ostream &output,
      const std::uint64_t timebase = common)
      : mcu(native_mcu), progress(native_progress), trace(output), timebase_hz(timebase) {}

  [[nodiscard]] std::uint64_t common_ticks() const {
    return source::common_ticks(mcu, timebase_hz);
  }

  void inputs() {
    const auto pc = mcu.cpu().state().pc;
    // Retain existing explicit PHY/loopback/TIO observations, never its ready input.
    if (pc != coupled::ready_poll) { observed.observe(mcu); }
    if (pc == release_boot_inputs) {
      mcu.bus().set_pin_drive(McuPort::b, {.mask = coupled::other_b_inputs});
    }
  }

  [[nodiscard]] GpioPadSample net(const std::uint32_t mask) const {
    const auto drive = mcu.bus().gpio_outputs(McuPort::u);
    const auto bias = mcu.bus().gpio_biases(McuPort::u);
    return resolve_gpio_net(
        mask,
        {.active = drive.mask & mask,
         .known = drive.known & mask,
         .levels = drive.levels & mask,
         .reduced = drive.reduced_drive & mask,
         .pull_up = bias.up & mask,
         .pull_down = bias.down & mask},
        {.pull_up = 1U & mask, .pull_down = unused_pulldowns ? (other_u & mask) : 0});
  }

  void bind_u() {
    mcu.bus().attach_gpio_pad_source(McuPort::u, [this](const auto mask) { return net(mask); });
  }

  [[nodiscard]] std::uint64_t assert_tick() const {
    if (!asserted) { throw Fault("Missing actual native U0 assertion boundary"); }
    return *asserted;
  }

  McuMachine &mcu;
  source::Progress &progress;
  std::ostream &trace;
  const std::uint64_t timebase_hz;
  owned_clock::Inputs observed;
  ExecutionContext context;
  std::optional<std::uint64_t> asserted;
  bool unused_pulldowns = false;
};

struct LinkCounts {
  bool released = false;
  std::uint64_t release_tick = 0;
  std::uint64_t dsp_steps = 0;
  unsigned writes = 0;
  unsigned frames = 0;
  unsigned ordered_pairs = 0;
  bool runtime_entered = false;
};

class Link {
public:
  Link(Lab &lab, OwnDspMachine &dsp, const Firmware &firmware, NandPeer *nand = nullptr)
      : lab_(lab), dsp_(dsp), firmware_(firmware), nand_(nand) {
    scheduler_ = std::make_unique<McuDspRuntime>(
        McuDspRuntimeInputs{
            .timing = OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
            .maximum_cycles_per_period = 32,
            .writes = McuWriteOrder::before_mcu,
            .reset_dsp_first = true,
            .initial_reset = OwnDspResetInputs{.mode_known = all_modes,
                                               .mode_levels = spi_mode,
                                               .pinit = false,
                                               .recovery_cycles = recovery_periods}},
        lab.mcu,
        dsp,
        DspRuntimeEvents{},
        peripheral());
  }

  [[nodiscard]] DspRuntimePeripheralEvents peripheral() const {
    if (nand_ == nullptr) { return {}; }
    const auto events = nand_->events();
    return {.identity = events.identity,
            .next_ticks = events.next_ticks,
            .observe = events.observe,
            .order = DspPeerOrder::peer_first,
            .maximum_events_per_period = 8};
  }

  [[nodiscard]] SpiInputs sample() const {
    const auto pins = lab_.mcu.bus().observe_pads(McuPort::b, spi_pins);
    if (pins.known != spi_pins) {
      throw Fault("Unknown actual SPI outputs known=" + hex(pins.known, 2) +
                  " levels=" + hex(pins.levels, 2));
    }
    return {.selected = (pins.levels & coupled::select_n) == 0,
            .clock = (pins.levels & coupled::clock_pin) != 0,
            .mosi = (pins.levels & coupled::mosi_pin) != 0};
  }

  DspDmaInputs cycle_inputs() {
    if (nand_ != nullptr) { nand_->before_native(dsp_.clock().elapsed_ticks()); }
    return {.known_requests = 15,
            .timing = DspDmaTiming::reference_sampled_cycles,
            .grant = DspDmaGrant::available};
  }

  [[nodiscard]] std::size_t verify_runtime_records() const {
    constexpr unsigned maximum_address = 0xffffff;
    std::map<std::pair<unsigned, unsigned>, unsigned> expected;
    for (const auto &record : firmware_.dsp) {
      require(record.space <= 2 && record.address <= maximum_address,
              "Known original DSP record space and address");
      for (std::size_t index = 0; index < record.words.size(); ++index) {
        require(index <= maximum_address - record.address, "Original DSP record address range");
        const auto address = record.address + static_cast<unsigned>(index);
        expected.insert_or_assign({record.space, address}, record.words[index]);
      }
    }
    for (const auto &[key, value] : expected) {
      const DspAddress address{.space = static_cast<DspSpace>(key.first), .address = key.second};
      if (dsp_.bus().read(address) != value) {
        throw Fault("Original loader record mismatch at space=" + std::to_string(key.first) +
                    " address=" + hex(key.second, 6) + " pc=" + hex(dsp_.cpu().registers().pc, 6));
      }
    }
    return expected.size();
  }

  void before(const DspRuntimeCycle &event) {
    ++counts_.dsp_steps;
    const auto state = dsp_.cpu().registers();
    const auto pending = dsp_.cpu().pending_instruction();
    const bool retired = event.result.native && event.result.native->retired;
    if (nand_ == nullptr || retired) {
      lab_.trace << "DSP cycle=" << event.cycle + 1 << " end=" << event.completion_ticks
                 << " period=" << event.completion_ticks - event.start_ticks
                 << " pc=" << hex(state.pc, 6) << " pctl=" << hex(dsp_.bus().pll().read(), 6)
                 << " retired=" << (event.result.native && event.result.native->retired)
                 << " native=" << event.result.native.has_value()
                 << " native_pc=" << hex(event.result.native_pc.value_or(0), 6) << " pending="
                 << (pending ? static_cast<unsigned>(pending->effect.kind) : no_pending_effect)
                 << " r4=" << state.r[4] << " lc=" << state.lc << '\n';
    }
    constexpr unsigned runtime_pc = 0x100;
    if (nand_ != nullptr) {
      if (retired && event.result.native_pc == nand_sample_pc) {
        lab_.trace << "CPU_BYTE tick=" << event.completion_ticks
                   << " value=" << hex(state.a >> accumulator_word_shift, 6) << " r5=" << state.r[5]
                   << '\n';
      }
      if (retired && state.pc == runtime_pc && !counts_.runtime_entered) {
        const auto resident = verify_runtime_records();
        counts_.runtime_entered = true;
        lab_.trace << "HANDOFF tick=" << event.completion_ticks << " r4=" << state.r[4]
                   << " r5=" << state.r[5] << " resident=" << resident << '\n';
      }
      nand_->after_native(event.completion_ticks, event.result.native_pc);
    }
  }

  void attach() {
    lab_.mcu.set_cycle_observer([this](const auto cycle) {
      lab_.context = cycle.context;
      lab_.progress.phases.observe(cycle, lab_.mcu.run_clock().bus_cycles());
      const bool shared = cycle.address && (cycle.kind == 'w' || cycle.kind == 'W') &&
                          (*cycle.address == coupled::port_b || *cycle.address == u_data ||
                           *cycle.address == u_direction);
      scheduler_->begin_cycle(
          cycle,
          shared,
          [this] { return cycle_inputs(); },
          [this](const auto &event) { before(event); });
    });
    lab_.mcu.bus().set_observer([this](const auto &event) {
      if (!event.write) { return; }
      source::record_clock_write(lab_.mcu, event, lab_.trace);
      if (event.address != coupled::port_b && event.address != u_data &&
          event.address != u_direction) {
        return;
      }
      scheduler_->observe_write(
          event,
          [&] { observe(event); },
          [this] { return cycle_inputs(); },
          [this](const auto &native) { before(native); });
    });
  }

  void observe(const BusEvent &event) {
    if (event.address != coupled::port_b) {
      if (!lab_.asserted) {
        const auto output = lab_.mcu.bus().gpio_outputs(McuPort::u);
        lab_.trace << "U-PREFIX address=" << hex(event.address) << " value=" << hex(event.value, 2)
                   << " pc=" << hex(lab_.context.pc)
                   << " active=" << static_cast<unsigned>(output.mask)
                   << " known=" << static_cast<unsigned>(output.known)
                   << " levels=" << static_cast<unsigned>(output.levels) << '\n';
        if ((output.mask & output.known & 1U) != 0 && (output.levels & 1U) == 0) {
          lab_.asserted = lab_.common_ticks();
        }
        return;
      }
      const auto level = lab_.net(1);
      require_known_gpio_pads(level, 1);
      const bool low = (level.levels & 1U) == 0;
      lab_.trace << "U " << scheduler_->end_ticks() << ' ' << hex(lab_.context.pc) << ' '
                 << hex(event.address) << ' ' << hex(event.value, 2) << " low=" << low << '\n';
      if (!low && !counts_.released) {
        require(scheduler_->completed_cycles() == 0,
                "No DSP instruction during the actual low hold");
        previous_ = sample();
        dsp_.bus().shi().observe_spi(*previous_);
        scheduler_->release_dsp_reset();
        counts_.released = true;
        counts_.release_tick = scheduler_->end_ticks();
      } else if (low && counts_.released) {
        throw Fault("Unexpected native reassertion");
      }
      return;
    }
    if (!counts_.released) { return; }
    const auto next = sample();
    if (!previous_) { throw Fault("Released SPI link lost its actual sampled inputs"); }
    if (previous_->selected != next.selected && previous_->clock != next.clock) {
      auto intermediate = next;
      intermediate.selected = previous_->selected;
      dsp_.bus().shi().observe_spi(intermediate);
      ++counts_.ordered_pairs;
    }
    dsp_.bus().shi().observe_spi(next);
    ++counts_.writes;
    if (previous_->selected && !next.selected) { ++counts_.frames; }
    previous_ = next;
    lab_.trace << "B " << scheduler_->end_ticks() << ' ' << hex(lab_.context.pc) << ' '
               << hex(event.value, 2) << " frames=" << counts_.frames << '\n';
  }

  void step() {
    lab_.inputs();
    source::step(lab_.mcu, lab_.progress);
    scheduler_->end_instruction();
  }

  McuDspRuntime &schedule() { return *scheduler_; }

  [[nodiscard]] std::optional<SpiInputs> previous() const { return previous_; }

  void restore(const McuDspRuntimeCheckpoint &snapshot) {
    if (nand_ != nullptr) { throw Fault("NAND replay requires its joint peripheral payload"); }
    scheduler_.reset();
    scheduler_ = std::make_unique<McuDspRuntime>(snapshot, lab_.mcu, dsp_);
  }

  void restore_wires(const std::optional<SpiInputs> pins) {
    previous_ = pins;
    counts_.released = false;
    counts_.release_tick = 0;
    counts_.writes = 0;
    counts_.frames = 0;
    counts_.ordered_pairs = 0;
    counts_.dsp_steps = 0;
    attach();
  }

  [[nodiscard]] const LinkCounts &counts() const { return counts_; }

private:
  LinkCounts counts_;
  Lab &lab_;
  OwnDspMachine &dsp_;
  const Firmware &firmware_;
  NandPeer *nand_;
  std::unique_ptr<McuDspRuntime> scheduler_;
  std::optional<SpiInputs> previous_;
};

void prefix(Lab &lab, Link &link) {
  auto &mcu = lab.mcu;
  mcu.bus().set_pin_drive(McuPort::e, {.mask = initial_e, .levels = 3});
  mcu.bus().set_pin_drive(McuPort::q, {.mask = byte_mask});
  mcu.bus().set_pin_drive(McuPort::r, {.mask = receive_pad, .levels = receive_pad});
  for (unsigned count = 0; count < limit && !lab.asserted; ++count) { link.step(); }
  if (!lab.asserted) { throw Fault("No actual native U0 low output assertion"); }
  lab.trace << "ASSERT tick=" << lab.assert_tick() << " completed=" << lab.common_ticks()
            << " pc=" << hex(mcu.cpu().state().pc) << " retired=" << lab.progress.census.retired
            << '\n';
}

void write_text(const std::filesystem::path &path, const std::string &text) {
  std::ofstream file(path);
  file.exceptions(std::ios::failbit | std::ios::badbit);
  file << text;
}

std::array<std::uint8_t, ram_size> ram(const McuMachine &mcu) {
  std::array<std::uint8_t, ram_size> result{};
  for (unsigned n = 0; n < ram_size; ++n) {
    result[n] = mcu.bus().peek(static_cast<std::uint16_t>(ram_begin + n));
  }
  return result;
}

void verify_stage(OwnDspMachine &dsp, const Firmware &firmware, const Link &link) {
  require(link.counts().frames == coupled::stage_words + 3 && link.counts().ordered_pairs == 1,
          "Original native stores retain97 frames and the declared pad-order transition");
  const std::span<const std::uint8_t> image(firmware.mcu);
  for (unsigned n = 0; n < coupled::stage_words; ++n) {
    const auto offset = coupled::frame_offset + ((static_cast<std::size_t>(n) + 2) * word_bytes);
    require(dsp.bus().read({.address = coupled::stage_base + n}) ==
                dsp_boot_word(
                    std::span<const std::uint8_t, word_bytes>(image.subspan(offset, word_bytes))),
            "Actual ROM execution receives every original stage word from native SPI stores");
  }
  auto copy = dsp.bus();
  const auto trailing =
      coupled::frame_offset + ((static_cast<std::size_t>(coupled::stage_words) + 2) * word_bytes);
  require((copy.shi().read(coupled::hcsr) & (rx_not_empty | rx_overrun)) == rx_not_empty &&
              copy.shi().read(coupled::hrx) ==
                  dsp_boot_word(std::span<const std::uint8_t, word_bytes>(
                      image.subspan(trailing, word_bytes))) &&
              (copy.shi().read(coupled::hcsr) & rx_not_empty) == 0,
          "Only the genuine trailing UI word remains in the copied native FIFO, without overrun");
}

void verify_payload(OwnDspMachine &dsp, const Firmware &firmware, const Link &link) {
  require(link.counts().writes == coupled::expected_port_writes,
          "Strict reset frontier retains the original7275 Port-B writes");
  verify_stage(dsp, firmware, link);
  require(dsp.bus().read({.space = DspSpace::x, .address = coupled::long_ram_end}) == 0,
          "The genuine X write preceding the reserved Y rejection remains published");
  const auto pending = dsp.cpu().pending_instruction();
  require(pending && pending->effect.kind == dsp56300::EffectKind::write &&
              pending->effect.space == dsp56300::Space::y &&
              pending->effect.address == coupled::long_ram_end && pending->effect.value == 0 &&
              dsp.cpu().registers().a == 0 &&
              dsp.cpu().registers().r[4] == coupled::long_ram_end + 1 &&
              dsp.cpu().registers().lc == 0x20000U - coupled::long_ram_end,
          "Actual REP/AGU/X-prefix/Y-failure state differs from the independent long-clear oracle");
  for (unsigned n = 0; n < coupled::long_ram_end; ++n) {
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      require(dsp.bus().read({.space = space, .address = n}) == 0,
              "Every preceding mapped clear pair is zero");
    }
  }
}

void record(const unsigned replay,
            Lab &lab,
            OwnDspMachine &dsp,
            Link &link,
            std::ostream &summary,
            const std::string &error,
            const bool nvm_verified = true) {
  const auto state = dsp.cpu().registers();
  const auto &clock = lab.mcu.run_clock();
  summary << "STOP replay=" << replay << " unused_pulldowns=" << lab.unused_pulldowns
          << " release=" << link.counts().release_tick << " retired=" << lab.progress.census.retired
          << " cycles=" << clock.bus_cycles() << " ticks=" << clock.elapsed_ticks()
          << " OSC=" << clock.oscillator_cycles() << " entries=" << lab.progress.interrupts.entries
          << " common=" << lab.common_ticks() << " mcu=" << hex(lab.mcu.cpu().state().pc)
          << " dsp=" << hex(state.pc, 6) << " r4=" << state.r[4] << " lc=" << state.lc
          << " omr=" << hex(state.omr, 6) << " units=" << link.schedule().completed_cycles()
          << " dsp_ticks=" << dsp.clock().elapsed_ticks()
          << " reset_ticks=" << dsp.clock().hardware_reset_ticks()
          << " dsp_retired=" << dsp.cpu().progress().instructions
          << " clk2=" << dsp.clock().timer_clk2_edges()
          << " dsp_attempts=" << link.counts().dsp_steps << " frames=" << link.counts().frames
          << " writes=" << link.counts().writes << " pairs=" << link.counts().ordered_pairs;
  if (nvm_verified) { summary << " nvm=1"; }
  summary << " fault=" << error << '\n';
}

void require_quarantined(OwnDspMachine &dsp, const bool clock_fault = true) {
  const auto before = dsp.cpu().registers();
  try {
    static_cast<void>(dsp.begin_fetch());
  } catch (const Fault &error) {
    require(std::string(error.what()).contains("clocked timeline owns operation=begin-fetch") &&
                dsp.cpu().registers() == before && dsp.clock().faulted() == clock_fault,
            "The failed native clock owner rejects execution before any state mutation");
    return;
  }
  throw Fault("Failed native DSP unexpectedly continued executing");
}

void exercise(Lab &lab,
              OwnDspMachine &dsp,
              Link &link,
              const Firmware &firmware,
              std::ostringstream &trace,
              const std::string &path,
              std::ostream &summary) {
  auto &mcu = lab.mcu;
  auto &progress = lab.progress;
  const auto cs = link.schedule().checkpoint();
  const auto cp = progress;
  const auto ci = lab.observed;
  const auto wires = link.previous();
  std::optional<CpuState> expected_mcu;
  std::optional<dsp56300::Registers> expected_dsp;
  std::optional<dsp56300::Progress> expected_progress;
  std::optional<std::array<std::uint8_t, ram_size>> expected_ram;
  std::string expected_error;
  std::string expected_trace;
  for (const unsigned replay : {0U, 1U, 2U}) {
    link.restore(cs);
    progress = cp;
    lab.observed = ci;
    lab.unused_pulldowns = replay != 0;
    lab.bind_u();
    const auto unused = lab.net(other_u);
    require(unused.known == (lab.unused_pulldowns ? other_u : 0U),
            "Unused U pins retain declared knowledge independently of native PU0 reads");
    link.restore_wires(wires);
    trace.str({});
    trace.clear();
    std::string error;
    try {
      for (unsigned count = 0; count < limit; ++count) { link.step(); }
      throw Fault("Original source exceeded its bounded frontier run");
    } catch (const Fault &fault) { error = fault.what(); }
    std::cerr << "Original native failure replay=" << replay << " mcu=" << hex(mcu.cpu().state().pc)
              << " dsp=" << hex(dsp.cpu().registers().pc, 6) << " frames=" << link.counts().frames
              << " writes=" << link.counts().writes << " reason=" << error << '\n';
    write_text(path + ".observed-" + std::to_string(replay) + ".trace", trace.str());
    owned_clock::check_nvm(mcu, McuFlashImage::from_known(firmware.mcu));
    require(mcu.timing_faulted() && link.schedule().faulted(),
            "Actual source failure quarantines the shared owner");
    require(error.contains("Unmapped DSP memory write at Y:0x002000") &&
                error.contains("pc=0x000072") && dsp.clock().faulted() &&
                dsp.cpu().registers().pc == clear_pc && dsp.cpu().registers().omr == stage_mode,
            "Original source retains its strict reserved-write frontier");
    require_quarantined(dsp);
    verify_payload(dsp, firmware, link);
    if (expected_mcu && expected_dsp && expected_ram) {
      require(owned_clock::same_cpu(mcu.cpu().state(), *expected_mcu) &&
                  dsp.cpu().registers() == *expected_dsp && expected_progress &&
                  dsp.cpu().progress() == *expected_progress && ram(mcu) == *expected_ram &&
                  error == expected_error && trace.str() == expected_trace,
              "Held-reset replay retains every event, full observed CPU/stack state and all MCU "
              "SRAM");
    } else {
      expected_mcu = mcu.cpu().state();
      expected_dsp = dsp.cpu().registers();
      expected_progress = dsp.cpu().progress();
      expected_ram = ram(mcu);
      expected_error = error;
      expected_trace = trace.str();
    }
    record(replay, lab, dsp, link, summary, error);
    const auto output = path + "." + std::to_string(replay);
    write_text(output + ".trace", trace.str());
    progress.census.write(output + ".mcu.jsonl");
    progress.phases.write(output + ".phases.json");
  }
  summary << "REPLAY native=1 ram=1 events=1\n";
}

void require_runtime_quarantined(Lab &lab, OwnDspMachine &dsp, Link &link) {
  const auto observation = [&] {
    return std::tuple{dsp.cpu().registers(),
                      dsp.cpu().progress(),
                      dsp.cpu().fault(),
                      dsp.clock().elapsed_ticks(),
                      dsp.bus().unselected_writes(),
                      lab.mcu.run_clock().bus_cycles(),
                      lab.mcu.run_clock().elapsed_ticks(),
                      ram(lab.mcu)};
  };
  const auto before = observation();
  const auto mcu = lab.mcu.cpu().state();
  std::string error;
  try {
    link.schedule().end_instruction();
  } catch (const Fault &fault) { error = fault.what(); }
  require(error == "MCU/DSP runtime requires its healthy inactive native epoch" &&
              observation() == before && owned_clock::same_cpu(lab.mcu.cpu().state(), mcu),
          "The actual failed runtime refuses continuation without any CPU, time or RAM change");
  require_quarantined(dsp, false);
}

void verify_selected_clear(OwnDspMachine &dsp) {
  struct Window {
    DspSpace space;
    unsigned first, last;
  };

  constexpr std::array windows{Window{.space = DspSpace::x, .first = 0, .last = 0x3000},
                               Window{.space = DspSpace::x, .first = 0xc000, .last = 0x10000},
                               Window{.space = DspSpace::y, .first = 0, .last = 0x2000},
                               Window{.space = DspSpace::y, .first = 0xc000, .last = 0x16000}};
  for (const auto window : windows) {
    for (unsigned address = window.first; address < window.last; ++address) {
      require(dsp.bus().read({.space = window.space, .address = address}) == 0,
              "Every selected X/Y cell is cleared by the actual original instruction");
    }
  }
}

void exercise_decoder(Lab &lab,
                      OwnDspMachine &dsp,
                      Link &link,
                      const Firmware &firmware,
                      std::ostringstream &trace,
                      const std::string &path,
                      std::ostream &summary) {
  std::string error;
  try {
    for (unsigned count = 0; count < limit; ++count) { link.step(); }
    throw Fault("Functional decoder source exceeded its execution bound");
  } catch (const Fault &fault) { error = fault.what(); }
  const auto state = dsp.cpu().registers();
  const auto pending = dsp.cpu().pending_instruction();
  const auto fault = dsp.cpu().fault();
  const auto &failure = dsp.io().failure();
  constexpr unsigned completed_pairs = 0x20000;
  constexpr unsigned gpio_pc = 0x73;
  constexpr unsigned direction_c = 0xffffbe;
  constexpr unsigned direction_value = 0xff00;
  constexpr unsigned selected_words = 0x3000 + 0x4000 + 0x2000 + 0xa000;
  constexpr unsigned vacant_writes = (2 * completed_pairs) - selected_words;
  constexpr unsigned later_port_writes = 71;
  std::cerr << "Original functional decoder mcu=" << hex(lab.mcu.cpu().state().pc)
            << " dsp=" << hex(state.pc, 6) << " reason=" << error << '\n';
  record(0, lab, dsp, link, summary, error);
  summary << "DECODER pairs=" << state.r[4] << " selected=" << selected_words
          << " unselected=" << dsp.bus().unselected_writes()
          << " policy=internal-writable-storage\n";
  summary.flush();
  write_text(path + ".0.trace", trace.str());
  lab.progress.census.write(path + ".0.mcu.jsonl");
  lab.progress.phases.write(path + ".0.phases.json");
  require(error.contains("Unqualified DSP GPIO reserved-bit write") &&
              error.contains("pc=0x000073") && lab.mcu.timing_faulted() &&
              link.schedule().faulted() && !dsp.clock().faulted(),
          "The actual completed native I/O period faults the shared runtime, not its clock");
  require(state.pc == gpio_pc && state.omr == stage_mode && state.r[4] == completed_pairs &&
              state.lc == 0 && state.a == 0 && !pending &&
              fault.kind == dsp56300::FaultKind::memory_write &&
              fault.space == dsp56300::Space::x && fault.pc == gpio_pc &&
              fault.address == direction_c && fault.value == direction_value && failure &&
              failure->operation == OwnDspIo::Operation::write &&
              failure->space == dsp56300::Space::x && failure->address == direction_c &&
              failure->pc == gpio_pc && failure->value == direction_value && failure->cause,
          "Original REP finishes every long store before its unchanged Port-C write");
  require(dsp.bus().write_contract() == DspWriteContract::internal_writable_storage &&
              dsp.bus().unselected_writes() == vacant_writes &&
              link.counts().writes == coupled::expected_port_writes + later_port_writes,
          "Only vacant/ROM writes lack storage; their actual native commits are counted");

  verify_selected_clear(dsp);
  verify_stage(dsp, firmware, link);
  owned_clock::check_nvm(lab.mcu, McuFlashImage::from_known(firmware.mcu));
  require_runtime_quarantined(lab, dsp, link);
}

void exercise_pins(Lab &lab,
                   OwnDspMachine &dsp,
                   Link &link,
                   const Firmware &firmware,
                   std::ostringstream &trace,
                   const std::string &path,
                   std::ostream &summary) {
  std::string error;
  try {
    for (unsigned count = 0; count < limit; ++count) { link.step(); }
    throw Fault("Functional GPIO source exceeded its execution bound");
  } catch (const Fault &fault) { error = fault.what(); }
  std::cerr << "Original logical GPIO mcu=" << hex(lab.mcu.cpu().state().pc)
            << " dsp=" << hex(dsp.cpu().registers().pc, 6) << " reason=" << error << '\n';
  record(0, lab, dsp, link, summary, error);
  for (const auto port : {DspPort::c, DspPort::e, DspPort::f, DspPort::d}) {
    const auto &gpio = dsp.bus().gpio(port);
    summary << "GPIO port=" << static_cast<unsigned>(port)
            << " contract=" << static_cast<unsigned>(gpio.contract())
            << " control=" << hex(gpio.read(DspGpioRegister::control), 6)
            << " direction=" << hex(gpio.read(DspGpioRegister::direction), 6) << '\n';
  }
  constexpr unsigned bcr = 0xfffffb;
  constexpr unsigned bcr_reset = 0x1fffff;
  const auto c = dsp.bus().gpio(DspPort::c).outputs();
  const auto f = dsp.bus().gpio(DspPort::f).outputs();
  const auto bus_control = dsp.bus().read({.space = DspSpace::x, .address = bcr});
  summary << "PINS unselected=" << dsp.bus().unselected_writes() << " bcr=" << hex(bus_control, 6)
          << " c_mask=" << hex(c.mask, 6) << " c_known=" << hex(c.known, 6)
          << " c_levels=" << hex(c.levels, 6) << " f_mask=" << hex(f.mask, 6)
          << " f_known=" << hex(f.known, 6) << " f_levels=" << hex(f.levels, 6) << '\n';
  summary.flush();
  write_text(path + ".0.trace", trace.str());
  lab.progress.census.write(path + ".0.mcu.jsonl");
  lab.progress.phases.write(path + ".0.phases.json");
  constexpr unsigned ready_pc = 0xbb;
  constexpr unsigned timer_control = 0xffff8b;
  constexpr unsigned ready_bit = 12;
  constexpr unsigned pairs = 0x20000;
  constexpr unsigned buffer = 0xc000;
  constexpr unsigned no_storage_writes = 184320;
  constexpr unsigned original_port_writes = 7346;
  const auto state = dsp.cpu().registers();
  const auto fault = dsp.cpu().fault();
  const auto &failure = dsp.io().failure();
  require(error.contains("Unknown DSP timer DI read at X:0xffff8b") &&
              error.contains("pc=0x0000bb") && lab.mcu.timing_faulted() &&
              link.schedule().faulted() && !dsp.clock().faulted() &&
              fault.kind == dsp56300::FaultKind::io_bit && fault.space == dsp56300::Space::x &&
              fault.pc == ready_pc && fault.address == timer_control && fault.value == ready_bit &&
              failure && failure->operation == OwnDspIo::Operation::bit &&
              failure->space == dsp56300::Space::x && failure->pc == ready_pc &&
              failure->address == timer_control && failure->action == dsp56300::BitAction::test &&
              failure->value == ready_bit && failure->cause && !dsp.cpu().pending_instruction(),
          "Actual original NAND-ready sampling fails visibly without fabricated external input");
  require(state.pc == ready_pc && state.r[4] == pairs && state.lc == 0 && state.r[0] == buffer &&
              state.r[2] == buffer && state.r[5] == 1 && state.omr == stage_mode &&
              dsp.bus().write_contract() == DspWriteContract::internal_writable_storage &&
              dsp.bus().unselected_writes() == no_storage_writes && bus_control == bcr_reset &&
              link.counts().writes == original_port_writes,
          "The original first refill follows the complete clear without a substituted BCR write");
  constexpr unsigned c_pins = 0xf00;
  constexpr unsigned idle = 0x300;
  constexpr unsigned f_pins = 0x4ff;
  require(c.mask == c_pins && c.known == c_pins && c.levels == idle && c.peripheral == 0 &&
              f.mask == f_pins && f.known == f_pins && f.levels == 0 && f.peripheral == 0,
          "Actual original stores drive NAND idle controls and the first row's zero output data");
  verify_selected_clear(dsp);
  verify_stage(dsp, firmware, link);
  owned_clock::check_nvm(lab.mcu, McuFlashImage::from_known(firmware.mcu));
  require_runtime_quarantined(lab, dsp, link);
}

void exercise_nand(Lab &lab,
                   OwnDspMachine &dsp,
                   Link &link,
                   NandPeer &nand,
                   std::ostringstream &trace,
                   const std::string &path,
                   std::ostream &summary) {
  constexpr unsigned maximum_cycles = 6000000;
  std::string error;
  try {
    for (unsigned count = 0; count < limit && link.counts().dsp_steps < maximum_cycles; ++count) {
      link.step();
    }
    throw Fault("Connected original NAND source exceeded its execution bound");
  } catch (const Fault &fault) { error = fault.what(); }
  record(0, lab, dsp, link, summary, error, false);
  const auto state = dsp.cpu().registers();
  summary << "NAND faulted=" << nand.faulted() << " runtime=" << link.counts().runtime_entered
          << " r0=" << hex(state.r[0], 6) << " r4=" << hex(state.r[4], 6)
          << " r5=" << hex(state.r[5], 6) << " timebase=" << dsp.clock().timebase_hz() << '\n';
  if (!nand.faulted()) {
    const auto actual = nand.detailed();
    summary << "MEDIUM ticks=" << actual.base.ticks << " page=" << actual.page
            << " column=" << actual.column
            << " pending=" << static_cast<unsigned>(actual.base.pending) << '\n';
  }
  summary.flush();
  write_text(path + ".0.trace", trace.str());
  lab.progress.census.write(path + ".0.mcu.jsonl");
  lab.progress.phases.write(path + ".0.phases.json");
  throw Fault(error);
}

enum class StartupContract : std::uint8_t {
  strict,
  writable_storage,
  existing_pins,
  nand,
  nand_runtime
};

struct Paths {
  std::filesystem::path firmware;
  std::filesystem::path rom;
  std::filesystem::path trace;
  StartupContract contract = StartupContract::strict;
  std::optional<std::filesystem::path> nand_image;
};

void run(const Paths &paths) {
  const auto &firmware_path = paths.firmware;
  const auto &rom_path = paths.rom;
  const auto trace_path = paths.trace.string();
  const auto firmware = Firmware::load(firmware_path);
  const auto rom = read_document_boot_rom(rom_path);
  const auto original = McuFlashImage::from_known(firmware.mcu);
  McuMachine mcu(
      original,
      BLOFELD_DEFAULT_SLA,
      McuRunContract{.clock = {.oscillator_hz = source::oscillator_hz,
                               .initial_oscillator_phase_ticks = 0,
                               .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                     .memory_timing = McuRunMemory::internal_flash_reads});
  source::Progress progress;
  std::ostringstream trace;
  std::ofstream summary(trace_path);
  summary.exceptions(std::ios::failbit | std::ios::badbit);
  const auto timebase = paths.contract == StartupContract::nand_runtime ? 2 * common : common;
  Lab lab(mcu, progress, trace, timebase);
  // Explicit initial-held profile; no pre-assert DSP execution is inferred.
  OwnDspMachine dsp({.extal_hz = source::oscillator_hz,
                     .timebase_hz = timebase,
                     .switch_timing = DspClockSwitch::reference_cycle_boundary});
  if (paths.contract != StartupContract::strict) {
    dsp.bus().set_write_contract(DspWriteContract::internal_writable_storage);
  }
  if (paths.contract == StartupContract::existing_pins || paths.nand_image) {
    for (const auto port : {DspPort::c, DspPort::e, DspPort::f, DspPort::d}) {
      dsp.bus().gpio(port).set_contract(DspGpioContract::logical_pins);
    }
  }
  if (paths.contract == StartupContract::nand_runtime) {
    dsp.bus().set_write_contract(DspWriteContract::functional_decode);
    dsp.bus().shi().set_reset_contract(ShiResetContract::selected_quiescent_reference);
    dsp.bus().set_esai_configuration_contract(
        DspEsaiInstance::secondary, DspEsaiConfigurationContract::cold_unclocked_reference);
    std::cerr << "FUNCTIONAL_BOOT IO=decoded-reserved-unselected SHI=selected-quiescent "
                 "ESAI1=cold-unclocked "
                 "SS/HBUSY=unchanged external-audio-clocks=unsupplied\n";
  }
  dsp.bus().install_rom({.start = coupled::bootstrap_base, .words = rom});
  std::unique_ptr<NandPeer> nand;
  if (paths.nand_image) {
    nand = std::make_unique<NandPeer>(
        dsp, Nand(load_nand_image(*paths.nand_image), NandReadContinuation::k9f1208_j));
  }
  Link link(lab, dsp, firmware, nand.get());
  lab.bind_u();
  link.attach();
  prefix(lab, link);
  const auto tail = lab.common_ticks() - lab.assert_tick();
  for (unsigned count = 0; count < limit && mcu.cpu().state().pc != release_pc; ++count) {
    link.step();
  }
  require(mcu.cpu().state().pc == release_pc && !link.counts().released &&
              dsp.clock().hardware_reset_active() && link.schedule().completed_cycles() == 0 &&
              dsp.cpu().registers().sr == reset_status,
          "Unchanged MCU delays execute with DSP held");
  const auto reset = dsp.clock().hardware_reset_state();
  if (!reset) { throw Fault("Expected the actual held native reset phase"); }
  summary << "HELD_CUT retired=" << progress.census.retired
          << " cycles=" << mcu.run_clock().bus_cycles()
          << " ticks=" << mcu.run_clock().elapsed_ticks() << " common=" << lab.common_ticks()
          << " assert=" << lab.assert_tick() << " tail=" << tail
          << " held=" << dsp.clock().elapsed_ticks() << " phase=" << reset->phase_ticks << '\n';
  write_text(std::string(trace_path) + ".prefix.trace", trace.str());
  progress.census.write(std::string(trace_path) + ".held.mcu.jsonl");
  progress.phases.write(std::string(trace_path) + ".held.phases.json");
  if (paths.nand_image && nand) {
    exercise_nand(lab, dsp, link, *nand, trace, trace_path, summary);
  } else if (paths.contract == StartupContract::existing_pins) {
    exercise_pins(lab, dsp, link, firmware, trace, trace_path, summary);
  } else if (paths.contract == StartupContract::writable_storage) {
    exercise_decoder(lab, dsp, link, firmware, trace, trace_path, summary);
  } else {
    exercise(lab, dsp, link, firmware, trace, trace_path, summary);
  }
}

}

int main(int count, char **args) {
  try {
    if (count != 4 && count != 5 && count != 6) {
      throw blofeld::Fault("firmware.zip private-rom private.trace "
                           "[writable-storage|writable-pins|nand image|nand-runtime image]");
    }
    auto contract = StartupContract::strict;
    std::optional<std::filesystem::path> nand;
    if (count >= 5) {
      const std::string_view name(args[4]);
      if (count == 6 && (name == "nand" || name == "nand-runtime")) {
        contract = name == "nand" ? StartupContract::nand : StartupContract::nand_runtime;
        nand = args[5];
      } else if (count == 6) {
        throw blofeld::Fault("Only the NAND contract accepts an image path");
      } else if (name == "writable-storage") {
        contract = StartupContract::writable_storage;
      } else if (name == "writable-pins") {
        contract = StartupContract::existing_pins;
      } else {
        throw blofeld::Fault("Unknown original startup contract");
      }
    }
    run({.firmware = args[1],
         .rom = args[2],
         .trace = args[3],
         .contract = contract,
         .nand_image = nand});
    std::string_view message =
        "Original reset-net source, dsp56300 runtime, SPI and held replay passed.\n";
    if (contract == StartupContract::writable_storage) {
      message = "Original reset-net source, functional decoder and complete clear passed.\n";
    }
    if (contract == StartupContract::existing_pins) {
      message = "Original reset-net source, logical GPIO and NAND-ready frontier passed.\n";
    }
    std::cout << message;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
