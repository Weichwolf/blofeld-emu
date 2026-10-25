// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp_corpus_fixture.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

namespace blofeld::test {
namespace {
constexpr unsigned word_bits = 24;
constexpr unsigned data_bits = word_bits * 2;
constexpr unsigned extension_mask = 0xff;

void field(std::ostream &output, const std::string &name, const std::uint64_t value) {
  output << name << ' ' << std::hex << std::setfill('0') << std::setw(6) << value << '\n';
}

std::string memory_key(const std::string &prefix, const unsigned address) {
  std::ostringstream name;
  name << prefix << std::hex << std::setfill('0') << std::setw(4) << address;
  return name.str();
}

void dump_memory(DspBus &bus, std::ostream &output, const CorpusFill &fill) {
  for (const auto area : {DspSpace::x, DspSpace::y, DspSpace::p}) {
    unsigned begin = 0;
    unsigned end = y_words;
    std::string prefix = "ym";
    if (area == DspSpace::p) {
      begin = program_dump_begin;
      prefix = "pm";
    }
    if (area == DspSpace::x) {
      end = x_words;
      prefix = "xm";
    }
    for (unsigned address = begin; address < end; ++address) {
      if (area == DspSpace::x && address >= snapshot_ram_begin && address < snapshot_ram_end) {
        continue;
      }
      const auto value = bus.read({.space = area, .address = address});
      if (value != fill.memory(area, address)) {
        field(output, memory_key(prefix, address), value);
      }
    }
  }
}

void dump(const CorpusCase &item,
          DspBus &bus,
          const DspCore &cpu,
          std::ostream &output,
          const CorpusFill &fill) {
  const auto observation = cpu.debug_state();
  const auto &state = observation.cpu;
  output << "case " << item.name << '\n';
  field(output, "pc", state.pc);
  field(output, "sr", state.sr);
  field(output, "omr", state.omr);
  field(output, "la", state.la);
  field(output, "lc", state.lc);
  field(output, "sp", state.sp);
  field(output, "vba", state.vba);
  field(output, "ep", observation.ep);
  field(output, "sz", observation.sz);
  field(output, "sc", observation.sc);
  if (state.sp != 0) {
    field(output, "ssh", state.stack_pc);
    field(output, "ssl", state.stack_sr);
  }
  field(output, "x0", state.x & corpus_word_mask);
  field(output, "x1", (state.x >> word_bits) & corpus_word_mask);
  field(output, "y0", state.y & corpus_word_mask);
  field(output, "y1", (state.y >> word_bits) & corpus_word_mask);
  field(output, "a0", state.a & corpus_word_mask);
  field(output, "a1", (state.a >> word_bits) & corpus_word_mask);
  field(output, "a2", (state.a >> data_bits) & extension_mask);
  field(output, "b0", state.b & corpus_word_mask);
  field(output, "b1", (state.b >> word_bits) & corpus_word_mask);
  field(output, "b2", (state.b >> data_bits) & extension_mask);
  for (unsigned index = 0; index < state.r.size(); ++index) {
    field(output, "r" + std::to_string(index), state.r[index]);
    field(output, "n" + std::to_string(index), state.n[index]);
    field(output, "m" + std::to_string(index), state.m[index]);
  }
  for (unsigned index = 1; index < observation.stack_high.size(); ++index) {
    std::ostringstream suffix;
    suffix << std::hex << std::setfill('0') << std::setw(2) << index;
    field(output, "sh" + suffix.str(), observation.stack_high[index]);
    field(output, "sl" + suffix.str(), observation.stack_low[index]);
  }
  dump_memory(bus, output, fill);
  output << "end\n";
}

void run_corpus(const std::filesystem::path &directory, std::ostream &output) {
  const auto metadata = read_cases(directory / "corpus.meta");
  const auto &cases = metadata.cases;
  if (metadata.fill.enabled) {
    output << "fill";
    for (const auto value : metadata.fill.values) {
      output << ' ' << std::hex << std::setfill('0') << std::setw(6) << value;
    }
    output << '\n';
  }
  DspBus bus;
  initialize_memory(bus, metadata.fill);
  load_corpus(bus, directory / "corpus.lod");
  install_preamble(bus, cases.front().start, metadata.fill);
  DspCore cpu(bus);
  unsigned preamble = 0;
  while (cpu.state().pc != cases.front().start) {
    constexpr unsigned preamble_limit = 200;
    corpus_require(++preamble <= preamble_limit, "DSP corpus preamble instruction limit");
    cpu.step();
  }
  for (const auto &item : cases) {
    unsigned gap = 0;
    while (cpu.state().pc != item.start) {
      constexpr unsigned gap_limit = 100;
      corpus_require(++gap <= gap_limit, "DSP corpus gap instruction limit");
      cpu.step();
    }
    unsigned count = 0;
    while (cpu.state().pc != item.end) {
      corpus_require(++count <= item.limit, "DSP corpus instruction limit: " + item.name);
      cpu.step();
    }
    dump(item, bus, cpu, output, metadata.fill);
  }
}
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) { throw blofeld::Fault("Usage: dsp-corpus-test trusted-bank output.txt"); }
    std::ofstream output(arguments[2]);
    output.exceptions(std::ios::failbit | std::ios::badbit);
    blofeld::test::run_corpus(arguments[1], output);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
