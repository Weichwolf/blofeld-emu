// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {
using blofeld::test::require;
constexpr unsigned producer = 0x2cd9;
constexpr unsigned vector_address = 0x54;
constexpr unsigned parameter_program = 0x808;
constexpr unsigned parser_program = 0x967;
constexpr unsigned parser_entry = 0xc53;
constexpr unsigned physical_alias = 0xf000;
constexpr unsigned memory_mode = 0x600085;
constexpr unsigned data_base = blofeld::test::boot + 0x1000;
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

void initialize_registers(blofeld::test::DspProgram &program) {
  program.immediate("sr", masked_status);
  for (unsigned index = 0; index < 8; ++index) {
    program.immediate("m" + std::to_string(index), word_mask);
    program.immediate("n" + std::to_string(index), 0);
    program.immediate("r" + std::to_string(index), 0);
  }
  for (const auto *const reg : {"a0", "a1", "a2", "b0", "b1", "b2", "x0", "x1", "y0", "y1"}) {
    program.immediate(reg, 0);
  }
}

void load_records(blofeld::test::DspProgram &program,
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
    program.immediate("r0", data_base + static_cast<unsigned>(data.size()));
    program.immediate("r1", address);
    program.immediate("x1", static_cast<unsigned>(record.words.size()));
    const auto start = program.pc();
    // The assembler takes the label after both copy instructions, not the LA word.
    program.emit("do x1," + blofeld::test::number(start + 4));
    program.emit("move p:(r0)+,x0");
    char space = 'p';
    if (alias || record.space == 1) {
      space = 'x';
    } else if (record.space == 2) {
      space = 'y';
    }
    program.emit(std::string("move x0,") + space + ":(r1)+");
    require(program.pc() == start + 4, "Own native copy loop has the declared four words");
    data.insert(data.end(), record.words.begin(), record.words.end());
  }
  require(programs == 4, "Four complete original producer/vector/parser components selected");
}

struct Store {
  unsigned address;
  unsigned value;
};

void put(blofeld::test::DspProgram &program, const Store store) {
  program.immediate("x0", store.value);
  program.emit("move x0,x:>" + blofeld::test::number(store.address));
}

void caller(blofeld::test::DspProgram &program) {
  initialize_registers(program);
  program.immediate("omr", memory_mode);
  for (unsigned index = 0; index < 3; ++index) { program.emit("nop"); }
  put(program, {.address = producer_cursor, .value = ring_base});
  put(program, {.address = consumer_cursor, .value = ring_base});
  put(program, {.address = 6, .value = 0});
  put(program, {.address = midi_channel, .value = 0});
  put(program, {.address = midi_clock, .value = 1});
  put(program, {.address = field, .value = seed});
  program.immediate("r1", ring_base);
  program.immediate("x0", 0);
  program.emit("rep #" + std::to_string(ring_words));
  program.emit("move x0,x:(r1)+");
  program.immediate("sr", ordinary_status);
  program.emit("movep #>0,x:<<$ffff90");
  program.emit("movep #>$200,x:<<$fffffe");
  program.immediate("r6", producer);
  program.emit("jsr (r6)");
  const auto loop = program.pc();
  program.immediate("r6", parser_entry);
  program.emit("jsr (r6)");
  program.immediate("r6", loop);
  program.emit("move x:>" + blofeld::test::number(field) + ",a");
  program.emit("tst a");
  program.emit("jne (r6)");
  // Only the guest's observed original center-pitch result enables the marker.
  program.emit("movep #>$89,x:<<$ffff91");
  program.word(absolute_jump);
  program.word(loop);
}

void generate(const blofeld::Firmware &firmware, const std::filesystem::path &path) {
  blofeld::test::DspProgram program;
  initialize_registers(program);
  std::vector<std::uint32_t> data;
  load_records(program, firmware, data);
  caller(program);
  require(program.pc() < data_base, "Own caller fits before private original data");
  while (program.pc() < data_base) { program.word(0); }
  for (const auto value : data) { program.word(value); }
  require(program.pc() - blofeld::test::boot <= 65536, "Private reference ROM input bound");
  blofeld::DspBus bus;
  program.install(bus);
  std::ofstream stream(path, std::ios::binary);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  for (unsigned address = blofeld::test::boot; address < program.pc(); ++address) {
    const auto word = bus.inspect_program(address);
    constexpr unsigned high_shift = 16;
    const std::array bytes{static_cast<char>(word >> high_shift),
                           static_cast<char>(word >> 8U),
                           static_cast<char>(word)};
    stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }
  stream.close();
  std::cout << "private-original-data-words=" << data.size()
            << " ROM-words=" << program.pc() - blofeld::test::boot << '\n';
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
