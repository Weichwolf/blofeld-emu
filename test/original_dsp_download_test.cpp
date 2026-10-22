// SPDX-License-Identifier: GPL-3.0-only
// Execute original download commands with independent data/state expectations.
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <ios>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {
using enum blofeld::DspSpace;
using blofeld::test::require;
// DSP56300FM JMP absolute extension encoding, used only in our caller.
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned entry = 0x2361, destination = 0x600, buffer = 0x778;
constexpr unsigned memory_mode = 0x600389, mask = 0xffffff;
constexpr unsigned manufacturer = 0x3e;
constexpr unsigned product = 0x13;
constexpr unsigned all_devices = 0x7f;
constexpr unsigned configure_command = 0x73;
constexpr unsigned byte_command = 0x74;
constexpr unsigned group_command = 0x75;
constexpr unsigned header_bits = 28;
constexpr unsigned septet_mask = 127;
constexpr unsigned original_record_count = 127;
constexpr unsigned return_word = 4;
constexpr unsigned cursor_word = 5;
using Templates = std::array<unsigned, 6>;
constexpr unsigned word_bits = 24;
constexpr unsigned byte_mask = 255;
constexpr unsigned first_residual = 0x87;
constexpr unsigned last_residual = 0xfe;
constexpr unsigned context_record = 93;
constexpr unsigned command_record = 94;
constexpr unsigned command_words = 126;
constexpr unsigned context_base = 0x2c25;
constexpr unsigned masked_status = 0xc00300;
constexpr unsigned return_template = 0x23ca;
constexpr unsigned status_slot = 0x76c;
constexpr unsigned completed_status = 19;
constexpr unsigned last_instruction = 0x23d7;
constexpr unsigned cursor_slot = 0x2d;
constexpr unsigned length_slot = 0x2e;
constexpr unsigned end_slot = 0x2f;
constexpr unsigned start_slot = 0x30;
constexpr unsigned sequence_slot = 0x31;
constexpr unsigned store_template = 0x23c6;
constexpr unsigned guard = 0x456789;
constexpr unsigned mode_command_cases = 4 * 2;
constexpr unsigned normal_cases = mode_command_cases * 2;
// Per mode/command: five mismatch/recovery calls, 130 real wrap calls,
// two zero-address calls and two zero-group calls. No host sequence seeding.
constexpr unsigned expected_calls =
    (normal_cases * 2) + (mode_command_cases * (5 + septet_mask + 3 + 2 + 2));
constexpr unsigned expected_prefetches =
    normal_cases + (mode_command_cases * (3 + septet_mask + 2 + 1 + 1));
constexpr unsigned expected_stores = 4 * (normal_cases + mode_command_cases);
constexpr std::array payload{12U, 0x123456U, 0xfedcbaU, 0xa55a11U};

void put(blofeld::DspBus &bus,
         const blofeld::DspSpace space,
         const unsigned at,
         const unsigned value) {
  bus.write({.space = space, .address = at, .value = value});
}

unsigned get(blofeld::DspBus &bus, const blofeld::DspSpace space, const unsigned at) {
  return bus.read({.space = space, .address = at});
}

struct Configuration {
  unsigned mode;
  unsigned address = destination;
  unsigned length = 9;
};

std::vector<unsigned> configuration(const Configuration input) {
  std::vector<unsigned> bytes{manufacturer, product, all_devices, configure_command, input.mode};
  for (const unsigned value : {input.length, input.address}) {
    for (unsigned shift = 0; shift < header_bits; shift += 7) {
      bytes.push_back((value >> shift) & septet_mask);
    }
  }
  return bytes;
}

struct Case {
  unsigned mode;
  unsigned command;
  unsigned checksum;
  unsigned sequence = 0;
  bool empty = false;
};

std::vector<unsigned> packet(const Case input) {
  const auto command = input.command;
  const auto checksum = input.checksum;
  std::vector<unsigned> decoded;
  for (const auto word : payload) {
    for (unsigned shift = 0; shift < word_bits; shift += 8) {
      decoded.push_back((word >> shift) & byte_mask);
    }
  }
  decoded.push_back(first_residual); // Two residual bytes are not complete words.
  decoded.push_back(last_residual);
  if (input.empty) { decoded.clear(); }
  const auto groups = static_cast<unsigned>(decoded.size()) / 7;
  std::vector<unsigned> bytes{manufacturer,
                              product,
                              all_devices,
                              command,
                              input.sequence,
                              command == byte_command ? groups * 8 : groups};
  for (unsigned offset = 0; offset < decoded.size(); offset += 7) {
    unsigned flags = 0;
    for (unsigned index = 0; index < 7; ++index) {
      flags |= ((decoded[offset + index] >> 7U) & 1U) << (6U - index);
    }
    bytes.push_back(flags);
    for (unsigned index = 0; index < 7; ++index) {
      bytes.push_back(decoded[offset + index] & septet_mask);
    }
  }
  bytes.push_back(checksum); // The actual routine prefetches this without checking it.
  return bytes;
}

struct Header {
  unsigned address;
  unsigned length;
};

constexpr unsigned divisor_coefficient = 0x2aaaab;
constexpr unsigned address_integer_bit = 0x1000000;
constexpr unsigned word_sign = 0x800000;
constexpr unsigned table_base = context_base + 8;
constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;
constexpr std::array header_cases{Header{.address = 0, .length = 0},
                                  Header{.address = 0, .length = 1},
                                  Header{.address = 0, .length = 2},
                                  Header{.address = 0, .length = 3},
                                  Header{.address = destination, .length = 9},
                                  Header{.address = destination, .length = 0x7fffff},
                                  Header{.address = destination, .length = 0x800000},
                                  Header{.address = destination, .length = 0xfffffd},
                                  Header{.address = destination, .length = mask},
                                  Header{.address = 0x1000600, .length = 0x1000009},
                                  Header{.address = 0xfffffff, .length = 0xfffffff},
                                  Header{.address = 0x8000600, .length = 0x8000009}};
constexpr std::array header_modes{0U, 1U, 2U, 3U, 4U, 5U, 6U, 127U};
constexpr unsigned header_case_count = header_cases.size() * header_modes.size();
constexpr unsigned total_cases = 48 + header_case_count;
constexpr unsigned total_calls = expected_calls + header_case_count;

std::int64_t expected_header_accumulator(const Configuration input) {
  const auto raw_length = input.length & mask;
  const auto length = (raw_length & word_sign) != 0
                          ? static_cast<std::int64_t>(raw_length) - address_integer_bit
                          : static_cast<std::int64_t>(raw_length);
  return (static_cast<std::int64_t>(input.address) * address_integer_bit) +
         (2 * static_cast<std::int64_t>(divisor_coefficient) * length);
}

unsigned expected_header_end(const std::int64_t accumulator) {
  auto quotient = accumulator / address_integer_bit;
  if (accumulator < 0 && accumulator % address_integer_bit != 0) { --quotient; }
  return static_cast<unsigned>(static_cast<std::uint64_t>(quotient) & mask);
}

Templates read_templates(const std::string &path) {
  std::ifstream input(path);
  Templates words{};
  for (auto &word : words) {
    require(static_cast<bool>(input >> std::hex >> word) && word <= mask,
            "Expected six independently assembled own raw24 instruction words");
  }
  std::string extra;
  require(!(input >> extra) && input.eof(), "Unexpected trailing own instruction words");
  return words;
}

struct Fixture {
  const Templates templates;
  blofeld::DspBus bus;
  blofeld::DspCore cpu{bus};
  unsigned returned = 0, begin = 0, caller_end = 0, steps = 0, calls = 0, launched = 0,
           prefetches = 0;
  unsigned stores = 0;
  std::map<unsigned, unsigned> original;

  Fixture(const blofeld::Firmware &firmware, const Templates &own) : templates(own) {
    require(templates[return_word] == payload[0],
            "Own downloaded payload begins with independently assembled RTS");
    bus.observe_omr(memory_mode);
    bus.advance_instruction_cycles(3);
    for (const unsigned index : {context_record, command_record}) {
      const auto &record = firmware.dsp.at(index);
      require(record.address == (index == context_record ? context_base : entry) &&
                  record.space == (index == context_record ? 1U : 0U) &&
                  record.words.size() == (index == context_record ? 12U : command_words),
              "Pinned original record positions");
      for (unsigned i = 0; i < record.words.size(); ++i) {
        const auto space = static_cast<blofeld::DspSpace>(record.space);
        put(bus, space, record.address + i, record.words[i]);
        if (space == p) { original[record.address + i] = record.words[i]; }
      }
    }
    blofeld::test::DspProgram caller;
    caller.immediate("omr", memory_mode);
    for (unsigned i = 0; i < 3; ++i) { caller.emit("nop"); }
    begin = caller.pc();
    for (unsigned i = 0; i < 8; ++i) {
      caller.immediate("m" + std::to_string(i), mask);
      caller.immediate("n" + std::to_string(i), 0);
    }
    for (const auto *const reg : {"a0", "a1", "a2", "b0", "b1", "b2"}) { caller.immediate(reg, 0); }
    caller.immediate("sr", masked_status);
    caller.immediate("r7", entry);
    caller.emit("jsr (r7)");
    returned = caller.pc();
    caller.word(absolute_jump);
    caller.word(begin);
    caller_end = caller.pc();
    caller.install(bus);
    cpu.reset_spi_boot();
  }

  void call(const std::vector<unsigned> &bytes, const bool launch) {
    for (unsigned i = 0; i < bytes.size(); ++i) { put(bus, x, buffer + i, bytes[i] << 16U); }
    const auto prefetched = buffer + static_cast<unsigned>(bytes.size()) - 1;
    const blofeld::DspNativeAccessObserver observer{
        .observe = [&](const blofeld::DspNativeAccess &access) {
          if (access.stage == blofeld::DspNativeAccessStage::after &&
              access.kind == blofeld::DspNativeAccessKind::write && access.pc == store_template) {
            ++stores;
          }
          if (access.stage == blofeld::DspNativeAccessStage::after &&
              access.kind == blofeld::DspNativeAccessKind::read && access.target.space == x &&
              access.target.address == prefetched &&
              (bytes[3] == byte_command || bytes[3] == group_command)) {
            require(access.value == bytes.back() << 16U, "Actual trailing prefetch value");
            ++prefetches;
          }
        }};
    unsigned local = 0;
    do {
      require(++local < 2048, "Fixed original command instruction bound");
      const auto state = cpu.state();
      if (state.pc == destination) {
        require(launch && state.r[0] == context_base && state.sp == 1,
                "Real original JMP retains caller frame and supplies actual context");
        require(get(bus, p, return_template) == templates[return_word] &&
                    get(bus, x, status_slot) == completed_status,
                "Return template restored and status published before helper launch");
        ++launched;
      }
      require((state.pc >= blofeld::test::boot && state.pc < caller_end) ||
                  (state.pc >= entry && state.pc <= last_instruction) ||
                  (launch && state.pc == destination),
              "Only own caller, complete original command record and own downloaded RTS execute");
      cpu.step(observer);
      bus.advance_instruction_cycles(1);
      ++steps;
    } while (cpu.state().pc != returned);
    require(cpu.state().sp == 0, "Actual original/downloaded RTS restores the caller stack");
    ++calls;
  }

  void initialize_guards() {
    for (const auto bank : {p, x, y}) {
      for (unsigned i = 0; i < 6; ++i) { put(bus, bank, destination - 1 + i, guard); }
    }
    put(bus, x, status_slot, guard);
  }

  void verify_no_store() {
    for (const auto bank : {p, x, y}) {
      for (unsigned i = 0; i < 6; ++i) {
        require(get(bus, bank, destination - 1 + i) == guard,
                "Disabled/empty packet retains every destination and boundary word");
      }
    }
    require(get(bus, x, status_slot) == guard, "Disabled/empty packet does not report launch");
    for (const auto &[at, value] : original) {
      if (at != store_template && at != return_template) {
        require(bus.peek_program(at) == value, "Disabled/empty packet original program guards");
      }
    }
  }

  void verify_decoded() {
    unsigned index = 0;
    for (const auto word : payload) {
      for (unsigned shift = 0; shift < word_bits; shift += 8) {
        const auto expected = ((word >> shift) & byte_mask) << 16U;
        require(get(bus, x, buffer + 6 + index++) == expected,
                "Disabled destination still decodes the exact complete own payload");
      }
    }
    require(get(bus, x, buffer + 6 + index) == first_residual << 16U &&
                get(bus, x, buffer + 7 + index) == last_residual << 16U,
            "Disabled destination retains both decoded residual bytes");
  }

  void disabled_packet(const Case input) {
    const auto before = stores;
    call(packet(input), false);
    require(stores == before, "Disabled cursor performs no original destination-store instruction");
    require(get(bus, x, cursor_slot) == 0 &&
                get(bus, x, sequence_slot) == ((input.sequence + 1) & septet_mask) << 16U,
            "Matching disabled sequence advances modulo128 without reactivating stores");
    verify_decoded();
    verify_no_store();
  }

  void sequence_failure(const Case input) {
    initialize_guards();
    call(configuration({.mode = input.mode}), false);
    auto wrong = input;
    wrong.sequence = 7;
    const auto before = stores;
    call(packet(wrong), false);
    require(stores == before, "Mismatched sequence performs no original destination stores");
    require(get(bus, x, cursor_slot) == 0 && get(bus, x, sequence_slot) == 0x10000,
            "Mismatch advances OLD expected sequence before disabling only the cursor");
    verify_decoded();
    verify_no_store();
    auto following = input;
    following.sequence = 1;
    disabled_packet(following);
    run(input); // Only a new real original configuration re-enables writes.
    std::cout << "SEQUENCE " << input.mode << ' ' << input.command << " passed\n";
  }

  void sequence_wrap(const Case input) {
    initialize_guards();
    call(configuration({.mode = input.mode}), false);
    auto wrong = input;
    wrong.sequence = 7;
    const auto before = stores;
    call(packet(wrong), false);
    require(stores == before, "Mismatched sequence performs no original destination stores");
    require(get(bus, x, sequence_slot) == 0x10000 && get(bus, x, cursor_slot) == 0,
            "Actual mismatch establishes disabled cursor and sequence1");
    verify_decoded();
    verify_no_store();
    auto following = input;
    for (unsigned sequence = 1; sequence <= septet_mask; ++sequence) {
      following.sequence = sequence;
      disabled_packet(following);
    }
    following.sequence = 0;
    disabled_packet(following);
    std::cout << "WRAP " << input.mode << ' ' << input.command << " passed\n";
  }

  void zero_address(const Case input) {
    initialize_guards();
    call(configuration({.mode = input.mode, .address = 0}), false);
    require(get(bus, x, cursor_slot) == 0 && get(bus, x, start_slot) == 0 &&
                get(bus, x, end_slot) == 3,
            "An actual zero target header retains raw zero cursor/start and computed end");
    disabled_packet(input);
    std::cout << "ZERO_ADDRESS " << input.mode << ' ' << input.command << " passed\n";
  }

  void empty_packet(Case input) {
    initialize_guards();
    call(configuration({.mode = input.mode}), false);
    input.empty = true;
    const auto before = stores;
    call(packet(input), false);
    require(stores == before, "Normal-mode zero groups execute no destination stores");
    require(get(bus, x, cursor_slot) == destination && get(bus, x, sequence_slot) == 0x10000,
            "Normal-mode zero groups advance sequence but retain the live cursor");
    verify_no_store();
    std::cout << "EMPTY " << input.mode << ' ' << input.command << " passed\n";
  }

  void header_case(const Configuration input) {
    call(configuration(input), false);
    const auto mode = input.mode & 3U;
    const auto full_b = expected_header_accumulator(input);
    const auto raw_b = static_cast<std::uint64_t>(full_b) & accumulator_mask;
    const auto state = cpu.state();
    require(get(bus, x, cursor_slot) == (input.address & mask) &&
                get(bus, x, start_slot) == (input.address & mask) &&
                get(bus, x, length_slot) == (input.length & mask) &&
                get(bus, x, end_slot) == expected_header_end(full_b) &&
                get(bus, x, sequence_slot) == 0,
            "Independent full28 header and signed24 fixed-point end arithmetic");
    require(state.a == static_cast<std::uint64_t>(input.address) << word_bits && state.b == raw_b &&
                state.r[0] == buffer + 13 && state.r[1] == table_base && state.n[1] == mode &&
                state.m[1] == mask,
            "Full A/B accumulator bits and actual header-pointer/mode registers");
    require(get(bus, p, store_template) == templates[mode] &&
                get(bus, p, return_template) ==
                    (mode == 0 ? templates[cursor_word] : templates[return_word]),
            "The actual mode byte is masked to two bits before live program patches");
    std::cout << "HEADER " << input.mode << ' ' << input.address << ' ' << input.length
              << " passed\n";
  }

  void run(const Case input) {
    const auto mode = input.mode;
    const auto command = input.command;
    const auto checksum = input.checksum;
    const std::array target_spaces{p, p, x, y};
    const auto space = target_spaces.at(mode);
    initialize_guards();
    call(configuration({.mode = mode}), false);
    require(get(bus, x, cursor_slot) == destination && get(bus, x, length_slot) == 9 &&
                get(bus, x, end_slot) == destination + 3 &&
                get(bus, x, start_slot) == destination && get(bus, x, sequence_slot) == 0,
            "Independent decoded header length/cursors/sequence");
    require(get(bus, p, store_template) == templates[mode] &&
                get(bus, p, return_template) ==
                    (mode == 0 ? templates[cursor_word] : templates[return_word]),
            "Original configuration writes independently assembled live program templates");
    call(packet(input), mode == 0);
    require(get(bus, x, cursor_slot) == destination + 4 && get(bus, x, sequence_slot) == 0x10000,
            "Actual packet stores all four complete words, including declared-length overshoot");
    for (const auto bank : {p, x, y}) {
      require(get(bus, bank, destination - 1) == guard && get(bus, bank, destination + 4) == guard,
              "Destination boundary guards retain their independent values");
      for (unsigned i = 0; i < payload.size(); ++i) {
        require(get(bus, bank, destination + i) == (bank == space ? payload[i] : guard),
                "Exact little-endian payload and untouched other banks");
      }
    }
    for (const auto &[at, value] : original) {
      if (at != store_template && at != return_template) {
        require(bus.peek_program(at) == value, "Original P guard");
      }
    }
    std::cout << "CASE " << mode << ' ' << command << ' ' << checksum << " passed\n";
  }
};
}

int main(const int argc, char **argv) {
  try {
    require(argc == 3, "Expected local original firmware archive and own instruction fixture");
    const auto firmware = blofeld::Firmware::load(argv[1]);
    require(firmware.dsp.size() == original_record_count, "Complete original ordered record count");
    Fixture fixture(firmware, read_templates(argv[2]));
    for (unsigned mode = 0; mode < 4; ++mode) {
      for (const unsigned command : {byte_command, group_command}) {
        for (const unsigned checksum : {0U, septet_mask}) {
          fixture.run({.mode = mode, .command = command, .checksum = checksum});
        }
      }
    }
    for (unsigned mode = 0; mode < 4; ++mode) {
      for (const unsigned command : {byte_command, group_command}) {
        const Case input{.mode = mode, .command = command, .checksum = 0};
        fixture.sequence_failure(input);
        fixture.sequence_wrap(input);
        fixture.zero_address(input);
        fixture.empty_packet(input);
      }
    }
    for (const auto mode : header_modes) {
      for (const auto header : header_cases) {
        fixture.header_case({.mode = mode, .address = header.address, .length = header.length});
      }
    }
    require(fixture.calls == total_calls && fixture.launched == 6 &&
                fixture.prefetches == expected_prefetches && fixture.stores == expected_stores,
            "Complete directed header/sequence/disabled/empty matrix and exact I/O counts");
    std::cout << "TOTAL " << total_cases << " cases " << fixture.calls << " calls " << fixture.steps
              << " steps prefetches " << fixture.prefetches << " stores " << fixture.stores
              << " launches " << fixture.launched << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
