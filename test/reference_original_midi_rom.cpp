// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp_rom_program.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <vector>

namespace {
using Program = blofeld::test::DspRomProgram;
using Register = blofeld::test::DspRomRegister;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

constexpr auto x0 = Register::x0;
constexpr auto x1 = Register::x1;
constexpr auto r0 = Register::r0;
constexpr auto r1 = Register::r1;
constexpr auto r6 = Register::r6;
constexpr auto sr = Register::sr;
constexpr auto omr = Register::omr;
constexpr unsigned producer = 0x2cd9;
constexpr unsigned vector_address = 0x54;
constexpr unsigned parameter_program = 0x808;
constexpr unsigned parser_program = 0x967;
constexpr unsigned parser_entry = 0xc53;
constexpr unsigned physical_alias = 0xf000;
constexpr unsigned memory_mode = 0x600085;
constexpr unsigned data_base = Program::boot + 0x1000;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned field = 0xab6;
constexpr unsigned seed = 0x246813;
constexpr unsigned ring_base = 0x2f80;
constexpr unsigned ring_words = 64;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned ordinary_status = 0xc00000;
constexpr unsigned masked_status = 0xc00300;
constexpr unsigned producer_cursor = 0x3b;
constexpr unsigned consumer_cursor = 0x3c;
constexpr unsigned midi_channel = 0x744;
constexpr unsigned midi_clock = 0x750;

void initialize_registers(Program &program) {
  program.immediate(sr, masked_status);
  for (unsigned index = 0; index < 8; ++index) {
    constexpr unsigned modifier_base = 32;
    constexpr unsigned offset_base = 24;
    constexpr unsigned address_base = 16;
    program.immediate(static_cast<Register>(modifier_base + index), word_mask);
    program.immediate(static_cast<Register>(offset_base + index), 0);
    program.immediate(static_cast<Register>(address_base + index), 0);
  }
  // FM table12-14: A0/A1/A2, B0/B1/B2, X0/X1/Y0/Y1.
  constexpr std::array scalar_registers{Register::a0,
                                        Register::a1,
                                        Register::a2,
                                        Register::b0,
                                        Register::b1,
                                        Register::b2,
                                        Register::x0,
                                        Register::x1,
                                        Register::y0,
                                        Register::y1};
  for (const auto reg : scalar_registers) { program.immediate(reg, 0); }
}

void load_records(Program &program,
                  const blofeld::Firmware &firmware,
                  std::vector<std::uint32_t> &data) {
  unsigned programs = 0;
  for (const auto &record : firmware.dsp) {
    if (record.space == 0) {
      if (record.address != producer && record.address != vector_address &&
          record.address != parameter_program && record.address != parser_program) {
        continue;
      }
      ++programs;
    } else {
      const unsigned limit = record.space == 1 ? 0x3000 : 0x2000;
      if (record.address + record.words.size() > limit) { continue; }
    }
    require(!record.words.empty(), "Selected original component is nonempty");
    const bool alias = record.space == 0 && record.address == producer;
    const auto address = record.address + (alias ? physical_alias : 0U);
    program.immediate(r0, data_base + static_cast<unsigned>(data.size()));
    program.immediate(r1, address);
    program.immediate(x1, static_cast<unsigned>(record.words.size()));
    const auto start = program.pc();
    // DO's extension is LA, the final loop word; the source label follows it.
    constexpr unsigned do_x1 = 0x06c500;
    constexpr unsigned p_r0_post_to_x0 = 0x07d884;
    constexpr unsigned x0_to_x_r1_post = 0x445900;
    constexpr unsigned x0_to_y_r1_post = 0x4c5900;
    constexpr unsigned x0_to_p_r1_post = 0x075984;
    program.word(do_x1);
    program.word(start + 3);
    program.word(p_r0_post_to_x0);
    auto destination = x0_to_p_r1_post;
    if (alias || record.space == 1) {
      destination = x0_to_x_r1_post;
    } else if (record.space == 2) {
      destination = x0_to_y_r1_post;
    }
    program.word(destination);
    require(program.pc() == start + 4, "Authored native copy loop has the declared four words");
    data.insert(data.end(), record.words.begin(), record.words.end());
  }
  require(programs == 4, "Four complete original producer/vector/parser components selected");
}

struct Store {
  unsigned address;
  unsigned value;
};

void put(Program &program, const Store store) {
  program.immediate(x0, store.value);
  constexpr unsigned store_x0_absolute = 0x447000;
  program.word(store_x0_absolute); // MOVE X0,X:ea, absolute extension.
  program.word(store.address);
}

void caller(Program &program) {
  initialize_registers(program);
  program.immediate(omr, memory_mode);
  for (unsigned index = 0; index < 3; ++index) { program.word(0); }
  put(program, {.address = producer_cursor, .value = ring_base});
  put(program, {.address = consumer_cursor, .value = ring_base});
  put(program, {.address = 6, .value = 0});
  put(program, {.address = midi_channel, .value = 0});
  put(program, {.address = midi_clock, .value = 1});
  put(program, {.address = field, .value = seed});
  program.immediate(r1, ring_base);
  program.immediate(x0, 0);
  constexpr unsigned repeat_immediate = 0x0600a0;
  constexpr unsigned x0_to_x_r1_post = 0x445900;
  program.word(repeat_immediate | (ring_words << 8U));
  program.word(x0_to_x_r1_post);
  program.immediate(sr, ordinary_status);
  constexpr std::array shi_reset{0x07f410U, 0U};
  constexpr std::array timer_priority{0x08f4beU, 0x200U};
  program.append(shi_reset);
  program.append(timer_priority);
  program.immediate(r6, producer);
  constexpr unsigned jsr_r6 = 0x0be680;
  program.word(jsr_r6);
  const auto loop = program.pc();
  program.immediate(r6, parser_entry);
  program.word(jsr_r6);
  program.immediate(r6, loop);
  constexpr std::array read_field{0x56f000U, field}; // MOVE X:ea,A.
  constexpr unsigned test_a = 0x200003;
  constexpr unsigned jne_r6 = 0x0ae6a2;
  program.append(read_field);
  program.word(test_a);
  program.word(jne_r6);
  // Only the guest's observed original center-pitch result enables the marker.
  constexpr std::array request_enabled{0x07f411U, 0x89U};
  program.append(request_enabled);
  program.word(absolute_jump);
  program.word(loop);
}

void generate(const blofeld::Firmware &firmware, const std::filesystem::path &path) {
  Program program;
  initialize_registers(program);
  std::vector<std::uint32_t> data;
  load_records(program, firmware, data);
  caller(program);
  require(program.pc() < data_base, "Authored caller fits before private original data");
  while (program.pc() < data_base) { program.word(0); }
  for (const auto value : data) { program.word(value); }
  require(program.pc() - Program::boot <= 65536, "Private reference ROM input bound");
  blofeld::test::write_dsp_rom(path, program);
  std::cout << "private-original-data-words=" << data.size()
            << " ROM-words=" << program.pc() - Program::boot << '\n';
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected original firmware ZIP and private generated ROM path");
    generate(blofeld::Firmware::load(arguments[1]), arguments[2]);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
