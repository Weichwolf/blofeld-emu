// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_timers.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::DspProgram;
using blofeld::test::require;
constexpr unsigned mask = 0xffffff, mode = 0x600389;
constexpr unsigned receiver = 0x191f, preparation = 0x2943, prepared = 0x2955;
constexpr unsigned sampler = 0x29be, silent = 0x2a7a;
constexpr unsigned ring = 0x1000, slot = 0x800, chain = 0x880, args = 0x780;
constexpr unsigned fm = 0x920, scratch = 0x700, output = 0x940, queue = 0x2700;
constexpr unsigned head = 0x2a14, tail = 0x2a15, active_pointer = 0x2a16;
constexpr unsigned active_count = 0x2a17, bank = 0xed86;
constexpr unsigned port_f_data = 0xfffff5, strobe = 0x100;
constexpr unsigned phase = 0x0c0000, marker = 0x7e0000, next_marker = 0x020000;
constexpr unsigned guard = 0x13579b, status = 0xc00300, refill_count = 264;
constexpr unsigned timer_arm = 0x300065;
constexpr unsigned header_words = 24, header_bytes = 72, seed_field = 22;
constexpr unsigned word_bits = 24, accumulator_bits = 48, a0_seed = 0x2468ac;
constexpr unsigned x1_seed = 0x123456, b1_seed = 0x765432, a2_seed = 0x12;
constexpr unsigned r1_seed = 0xaaa, r2_seed = 0xbbb;
constexpr unsigned page_bytes = 528, step_limit = 16000, absolute_jump = 0x0af080;
constexpr unsigned maximum_pages = 142, sample_request = 0xc100, ring_request = 0x8100;
constexpr unsigned pitch_correction = 0xc29c, minimum_end = 0x3400, maximum_end = 0x20000;
constexpr unsigned active_header = 0x2b, service_counter = 0x2c;
constexpr unsigned read_inputs = 0x3ff, upper_inputs = 0x300, full_read = 0x700;
constexpr std::array<unsigned, 6> records{39, 40, 41, 79, 84, 114};

struct Page {
  unsigned id = 0, ready = 0, rate = 0, next_phase = 0, result = 0;
  std::array<std::uint8_t, page_bytes> bytes{};
};

unsigned word(const Page &page, const unsigned index) {
  const auto start = index * 3;
  return (static_cast<unsigned>(page.bytes.at(start)) << 16U) |
         (static_cast<unsigned>(page.bytes.at(start + 1)) << 8U) | page.bytes.at(start + 2);
}

unsigned pcm(const Page &page, const unsigned index) {
  const auto start = header_bytes + (index * 2);
  return (static_cast<unsigned>(page.bytes.at(start)) << 16U) |
         (static_cast<unsigned>(page.bytes.at(start + 1)) << 8U);
}

void put(DspBus &bus, const DspSpace space, const unsigned address, const unsigned value) {
  bus.write({.space = space, .address = address, .value = value});
}

unsigned get(DspBus &bus, const DspSpace space, const unsigned address) {
  return bus.read({.space = space, .address = address});
}

std::array<unsigned, 3> caller(DspBus &bus) {
  DspProgram p;
  p.immediate("omr", mode);
  p.immediate("sr", status);
  for (unsigned i = 0; i < 8; ++i) { p.immediate("m" + std::to_string(i), mask); }
  p.emit("movep #>$01ffff,x:<<$fffffb");
  p.emit("movep #>$f00,x:<<$ffffbe");
  p.emit("movep #>$0,x:<<$ffffbd");
  p.emit("movep #>$ff,x:<<$ffffbf");
  p.emit("movep #>$0,y:<<$fffff7");
  p.emit("movep #>$400,y:<<$fffff6");
  p.emit("movep #>$400,y:<<$fffff5");
  p.emit("movep #>$3ff,y:<<$fffff7");
  p.immediate("r1", r1_seed);
  p.immediate("r2", r2_seed);
  p.immediate("m1", 3);
  p.immediate("m2", 7);
  p.immediate("a0", a0_seed);
  p.immediate("a1", guard);
  p.immediate("a2", a2_seed);
  p.immediate("b1", b1_seed);
  p.immediate("x1", x1_seed);
  p.immediate("r6", receiver);
  p.emit("jsr (r6)");
  const auto header_stop = p.pc();
  p.emit("nop");
  p.emit("movep #>" + blofeld::test::number(timer_arm) + ",x:<<$ffff8b");
  p.immediate("m1", mask);
  p.immediate("m2", mask);
  p.immediate("r3", chain + 2);
  p.immediate("r4", slot);
  p.immediate("r7", args);
  p.immediate("r6", preparation);
  p.emit("jsr (r6)");
  const auto prep_stop = p.pc();
  p.emit("nop");
  p.immediate("sr", status);
  for (unsigned i = 0; i < 8; ++i) { p.immediate("m" + std::to_string(i), mask); }
  p.immediate("m5", 7);
  p.immediate("r0", slot);
  p.immediate("r4", slot);
  p.immediate("r5", fm);
  p.immediate("r3", scratch);
  p.immediate("r7", output);
  p.immediate("n0", 3);
  p.immediate("n4", 5);
  p.immediate("n3", 2);
  p.immediate("n7", 2);
  p.emit("move x:>" + blofeld::test::number(slot - 7) + ",r6");
  p.emit("jmp (r6)");
  const auto render_stop = p.pc();
  p.emit("nop");
  p.word(absolute_jump);
  p.word(blofeld::test::boot);
  p.install(bus);
  return {header_stop, prep_stop, render_stop};
}

struct Peer {
  unsigned clocked = 0, reads = 0, steps = 0;
  std::optional<bool> previous;
  bool operator==(const Peer &) const = default;
};

struct Saved {
  DspCheckpoint owner;
  Peer peer;
};

class Fixture {
public:
  explicit Fixture(const Firmware &firmware) : original_(firmware) {
    auto &bus = machine_.bus();
    bus.observe_omr(mode);
    bus.advance_instruction_cycles(3);
    for (const auto index : records) {
      const auto &record = original_.dsp.at(index);
      const auto space = static_cast<DspSpace>(record.space);
      for (unsigned offset = 0; offset < record.words.size(); ++offset) {
        put(bus, space, record.address + offset, record.words.at(offset));
      }
    }
    stops_ = caller(bus);
    machine_.cpu().reset_spi_boot();
  }

  void start(const Page &page, const std::optional<unsigned> cutoff = {}) {
    page_ = &page;
    cutoff_ = cutoff;
    saved_.reset();
    peer_ = {};
    auto &bus = machine_.bus();
    for (unsigned i = 0; i < 512; ++i) { put(bus, DspSpace::y, ring + i, guard); }
    put(bus, DspSpace::y, ring + 256, slot);
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      for (unsigned i = 0; i < 9; ++i) { put(bus, space, slot + i, guard); }
    }
    for (unsigned i = 0; i < 256; ++i) { put(bus, DspSpace::x, queue + i, guard); }
    put(bus, DspSpace::x, head, queue);
    put(bus, DspSpace::x, tail, queue);
    put(bus, DspSpace::x, active_pointer, ring);
    put(bus, DspSpace::x, active_count, page_bytes);
    put(bus, DspSpace::x, active_header, sample_request);
    put(bus, DspSpace::x, service_counter, guard);
    put(bus, DspSpace::x, 6, 0);
    put(bus, DspSpace::x, 7, 1);
    put(bus, DspSpace::x, chain, preparation);
    put(bus, DspSpace::x, slot - 7, silent);
    put(bus, DspSpace::x, slot - 6, silent);
    put(bus, DspSpace::x, slot, phase);
    put(bus, DspSpace::x, slot + 1, stops_[2]);
    put(bus, DspSpace::x, slot + 2, fm);
    put(bus, DspSpace::x, slot + 3, ring);
    put(bus, DspSpace::x, slot + 5, marker);
    put(bus, DspSpace::x, slot + 7, 0);
    put(bus, DspSpace::y, slot + 1, 0);
    put(bus, DspSpace::y, slot + 2, fm);
    put(bus, DspSpace::y, args, (word(page, 10) - pitch_correction) & mask);
    put(bus, DspSpace::y, args + 1, 0x800000);
    put(bus, DspSpace::x, fm, 0);
    put(bus, DspSpace::y, scratch, guard);
    for (unsigned i = 0; i < 3; ++i) { put(bus, DspSpace::x, output + i, guard); }
    bus.timers().observe_synchronized_tio(1, false);
    drive();
  }

  void finish() {
    advance(stops_[0]);
    check_ring(page_bytes);
    check_header();
    advance(stops_[1]);
    check_preparation();
    advance(stops_[2]);
    require(peer_.reads == page_bytes && peer_.clocked == page_bytes,
            "Only528 actual read-strobe edges may consume the declared logical page");
    auto &bus = machine_.bus();
    require(get(bus, DspSpace::x, slot) == page_->next_phase &&
                get(bus, DspSpace::x, output) == page_->result,
            "The actual selected callback differs from the independent page-to-output model");
    require(get(bus, DspSpace::x, slot + 7) == page_->result &&
                get(bus, DspSpace::x, output + 1) == guard &&
                get(bus, DspSpace::x, output + 2) == guard && machine_.cpu().state().sp == 0,
            "Original callback, exposed sample, guards and native return frame must agree");
    check_ring(page_bytes);
  }

  void failure(const Page &page, const unsigned cutoff) {
    start(page, cutoff);
    bool faulted = false;
    try {
      finish();
    } catch (const Fault &error) {
      const std::string message(error.what());
      require(message.contains("Unknown DSP GPIO data read") && message.contains("0xfffff5"),
              "Only the deliberately unobserved peer byte may fault this receiver");
      faulted = true;
    }
    require(faulted && saved_.has_value() && peer_.reads == cutoff && peer_.clocked == cutoff + 1,
            "A missing byte retains the exact successful read prefix and a healthy prior snapshot");
    check_ring(cutoff);
    check_fields(cutoff >= header_bytes);
    try {
      machine_.cpu().step();
      throw Fault("A faulted native receiver silently retried");
    } catch (const Fault &error) {
      require(std::string(error.what()).contains("requires reset"),
              "Implicit retry must remain rejected");
    }
    if (!saved_) { throw Fault("No healthy receiver snapshot precedes the missing byte"); }
    const auto snapshot = *saved_;
    std::optional<DspDebugState> expected;
    for (unsigned replay = 0; replay < 2; ++replay) {
      machine_.restore(snapshot.owner);
      peer_ = snapshot.peer;
      cutoff_.reset();
      drive();
      finish();
      const auto state = machine_.cpu().debug_state();
      if (expected) {
        require(state == *expected, "Both trusted native suffix replays must match");
      }
      expected = state;
    }
  }

  void intact() {
    for (const auto index : records) {
      const auto &record = original_.dsp.at(index);
      for (unsigned i = 0; i < record.words.size(); ++i) {
        require(get(machine_.bus(), static_cast<DspSpace>(record.space), record.address + i) ==
                    record.words.at(i),
                "All complete original receiver/sampler/curve/coefficient records remain intact");
      }
    }
  }

private:
  void drive() {
    const bool missing = cutoff_ && peer_.clocked == *cutoff_ + 1;
    const auto byte = peer_.clocked == 0 ? 0U : page_->bytes.at(peer_.clocked - 1);
    machine_.bus()
        .gpio(DspPort::f)
        .set_drive({.mask = missing ? upper_inputs : read_inputs,
                    .levels = upper_inputs | (missing ? 0U : byte)});
  }

  void step() {
    require(++peer_.steps <= step_limit, "Original sample receive/prepare/render exceeded bound");
    machine_.cpu().step({.observe = [this](const DspNativeAccess &access) {
      if (access.kind != DspNativeAccessKind::read || access.stage != DspNativeAccessStage::after ||
          (access.target.space != DspSpace::y || access.target.address != port_f_data)) {
        return;
      }
      require(peer_.reads < page_bytes && peer_.clocked == peer_.reads + 1 &&
                  access.value == (full_read | page_->bytes.at(peer_.reads)),
              "Actual whole Port-F reads must consume the next strobe-selected observed byte");
      ++peer_.reads;
    }});
    machine_.bus().advance_instruction_cycles(1);
    const auto intent = machine_.bus().gpio(DspPort::c).outputs();
    const bool known = (intent.mask & intent.known & strobe) != 0;
    const bool level = (intent.levels & strobe) != 0;
    if (known && peer_.previous == false && level) {
      require(peer_.clocked < page_bytes, "Original receiver clocked beyond its528-byte response");
      ++peer_.clocked;
      drive();
    }
    peer_.previous = known ? std::optional<bool>{level} : std::nullopt;
    if (cutoff_ && !saved_ && peer_.clocked == *cutoff_ + 1 && peer_.reads == *cutoff_) {
      saved_.emplace(Saved{.owner = machine_.checkpoint(), .peer = peer_});
    }
  }

  void advance(const unsigned stop) {
    do { step(); } while (machine_.cpu().state().pc != stop);
  }

  void check_ring(const unsigned bytes) {
    const auto copied_header_words = std::min(bytes / 3, header_words);
    const auto payload_words = bytes < header_bytes ? 0 : (bytes - header_bytes) / 2;
    for (unsigned i = 0; i < 512; ++i) {
      unsigned expected = i == 256 ? slot : guard;
      if (i < copied_header_words) { expected = word(*page_, i); }
      if (i >= header_words && i - header_words < payload_words) {
        expected = pcm(*page_, i - header_words);
      }
      require(get(machine_.bus(), DspSpace::y, ring + i) == expected,
              "Every ring word retains its independent complete/partial-byte expectation");
    }
  }

  void check_fields(const bool copied) {
    auto &bus = machine_.bus();
    require(get(bus, DspSpace::x, slot + 4) == (copied ? word(*page_, 10) : guard) &&
                get(bus, DspSpace::y, slot + 4) == (copied ? word(*page_, 13) : guard) &&
                get(bus, DspSpace::y, slot + 5) == (copied ? word(*page_, 11) : guard) &&
                get(bus, DspSpace::x, slot + 6) == (copied ? word(*page_, 8) - 1 : guard) &&
                get(bus, DspSpace::y, slot + 6) == (copied ? word(*page_, 12) : guard) &&
                get(bus, DspSpace::y, slot) == (copied ? word(*page_, seed_field) : guard),
            "Original C100 receiver must copy exactly its six real header/slot fields");
  }

  void check_header() {
    check_fields(true);
    const auto state = machine_.cpu().state();
    require(state.sp == 0 && state.sr == status && state.r[1] == r1_seed && state.r[2] == r2_seed &&
                state.m[1] == 3 && state.m[2] == 7 && (state.x >> word_bits) == x1_seed &&
                ((state.a >> word_bits) & mask) == guard &&
                (state.a >> accumulator_bits) == a2_seed &&
                ((state.b >> word_bits) & mask) == b1_seed,
            "The genuine JSR/RTI frame restores exactly the original ISR saved context");
  }

  void check_preparation() {
    auto &bus = machine_.bus();
    const bool ready =
        word(*page_, 10) != 0 && word(*page_, 13) >= minimum_end && word(*page_, 13) <= maximum_end;
    require(page_->ready == static_cast<unsigned>(ready), "Independent raw-header gate disagrees");
    require(get(bus, DspSpace::x, chain) == (ready ? prepared : preparation) &&
                get(bus, DspSpace::x, slot - 7) == (ready ? sampler : silent) &&
                get(bus, DspSpace::x, slot - 6) == (ready ? sampler : silent),
            "Original readiness gates choose the real silent or active callbacks");
    require(get(bus, DspSpace::x, head) == queue + (ready ? 4 : 0) &&
                get(bus, DspSpace::x, tail) == queue &&
                get(bus, DspSpace::x, slot + 5) == (ready ? next_marker : marker),
            "Only eligible original preparation may publish the first refill descriptor");
    if (!ready) { return; }
    require(get(bus, DspSpace::y, slot) == page_->rate && get(bus, DspSpace::y, slot + 1) == 0 &&
                get(bus, DspSpace::y, slot + 3) == bank &&
                get(bus, DspSpace::x, slot + 6) == word(*page_, 8) &&
                get(bus, DspSpace::x, service_counter) == 1,
            "Real tuned preparation selects the original curve rate, zero FM depth and busy queue");
    const std::array<unsigned, 4> expected{ring_request, word(*page_, 8), ring + 252, refill_count};
    for (unsigned i = 0; i < expected.size(); ++i) {
      require(get(bus, DspSpace::x, queue + i) == expected.at(i),
              "Original first refill retains its raw command/address/pointer/count");
    }
  }

  const Firmware &original_;
  DspMachine machine_;
  std::array<unsigned, 3> stops_{};
  const Page *page_ = nullptr;
  Peer peer_;
  std::optional<unsigned> cutoff_;
  std::optional<Saved> saved_;
};

unsigned read_number(std::istream &stream) {
  unsigned result = 0;
  for (unsigned i = 0; i < 4; ++i) {
    const auto byte = stream.get();
    require(byte != std::char_traits<char>::eof(), "Truncated private sample page input");
    result = (result << 8U) | static_cast<unsigned>(byte);
  }
  return result;
}

std::vector<Page> load_pages(const std::string &path) {
  std::ifstream stream(path, std::ios::binary);
  std::array<char, 8> magic{};
  stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  require(std::string(magic.data(), magic.size()) == "BSRX0001", "Invalid sample page signature");
  const auto count = read_number(stream);
  require(count > 0 && count <= maximum_pages,
          "Sample receiver input count is outside its bounded scope");
  std::vector<Page> pages(count);
  for (auto &page : pages) {
    page.id = read_number(stream);
    page.ready = read_number(stream);
    page.rate = read_number(stream);
    page.next_phase = read_number(stream);
    page.result = read_number(stream);
    for (auto &byte : page.bytes) {
      const auto value = stream.get();
      require(value != std::char_traits<char>::eof(), "Truncated official page bytes");
      byte = static_cast<std::uint8_t>(value);
    }
  }
  require(stream.get() == std::char_traits<char>::eof(), "Extra private sample input bytes");
  return pages;
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Usage: original-sample-receiver firmware.zip private.pages private.trace");
    const auto firmware = Firmware::load(arguments[1]);
    const auto pages = load_pages(arguments[2]);
    Fixture fixture(firmware);
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    unsigned active = 0;
    for (const auto &page : pages) {
      fixture.start(page);
      fixture.finish();
      active += page.ready;
      trace << "PAGE " << page.id << ' ' << page.ready << ' ' << page.next_phase << ' '
            << page.result << '\n';
    }
    for (const auto cutoff : {0U, 1U, 71U, 72U, 527U}) {
      fixture.failure(pages.front(), cutoff);
      trace << "CUT " << cutoff << " replays=2\n";
    }
    fixture.intact();
    trace << "RESULT " << pages.size() << ' ' << active << " 5 10\n";
    std::cout << "Original C100 GPIO receive, preparation, refill queue, callbacks and trusted "
                 "partial-byte recovery verified under explicit logical preconditions.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
