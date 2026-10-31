// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "dma_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_bit_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using test::require;
namespace bit = test::peripheral_bit;
namespace dma = test::dma_test;
namespace banks = test::banks;
namespace pipeline = test::pipeline;
constexpr unsigned first = 0x123456, second = 0x654321;
unsigned refused = 0, native_refused = 0;

void rejection(const DspMemoryAccess access) {
  DspBus bus;
  const std::array fetch{DspMemoryAccess{.target = {.address = banks::p_rom_begin}}};
  static_cast<void>(bus.advance_dma(banks::inputs({.index = 0}), banks::use(fetch)));
  bus.write({.space = DspSpace::x, .address = dma::ring_start, .value = first});
  banks::configure(bus, {.control = dma::software_copy});
  const auto source = bus.advance_dma(banks::inputs({.index = 1}), banks::use(fetch));
  require(source.phase && !source.word && source.phase->kind == DspDmaPhaseKind::source_read,
          "A real source read precedes every unsupported peripheral overlap");
  bus.write({.space = DspSpace::x, .address = dma::ring_start, .value = second});
  const std::array core{fetch[0], access};
  const auto error = test::esai::rejects(
      [&] { static_cast<void>(bus.advance_dma(banks::inputs({.index = 2}), banks::use(core))); });
  require(error.contains("Unqualified") && bus.last_dma_cycle() == 1 &&
              bus.read({.space = DspSpace::x, .address = dma::source0}) == dma::ring_start,
          "Unsupported footprints reject before request sampling, pointer update or DMA write");
  const auto held =
      bus.advance_dma(banks::inputs({.index = 2, .grant = DspDmaGrant::wait}), banks::use(fetch));
  require(held.phase && held.phase->kind == DspDmaPhaseKind::destination_write &&
              held.phase->value == first && !held.word,
          "The rejected cycle leaves the original captured DMA payload available to wait");
  const auto completed = bus.advance_dma(banks::inputs({.index = 3}), banks::use(fetch));
  require(completed.word && completed.word->value == first &&
              bus.read({.space = DspSpace::y, .address = dma::destination}) == first &&
              bus.read({.space = DspSpace::x, .address = dma::control0}) ==
                  (dma::software_copy & ~dma::enable),
          "A later qualified cycle completes exactly the retained transfer");
  ++refused;
}

void footprints() {
  for (const auto port : bit::ports) {
    for (const auto operation : {DspMemoryOperation::read, DspMemoryOperation::write}) {
      for (const unsigned neighbor : {1U, 2U}) {
        rejection({.target = {.space = port.space, .address = port.address + neighbor},
                   .operation = operation,
                   .phase = DspMemoryPhase::peripheral_bit});
      }
      rejection({.target = {.space = port.space == DspSpace::x ? DspSpace::y : DspSpace::x,
                            .address = port.address},
                 .operation = operation,
                 .phase = DspMemoryPhase::peripheral_bit});
      rejection({.target = {.space = port.space, .address = port.address},
                 .operation = operation,
                 .phase = operation == DspMemoryOperation::write ? DspMemoryPhase::peripheral_sample
                                                                 : DspMemoryPhase::execute});
    }
    rejection({.target = {.space = port.space, .address = port.address},
               .phase = DspMemoryPhase::peripheral_sample});
  }
  constexpr std::array data{DspAddress{.space = DspSpace::x, .address = 0xffffa0},
                            DspAddress{.space = DspSpace::y, .address = 0xffff80},
                            DspAddress{.space = DspSpace::x, .address = 0xffffd2},
                            DspAddress{.space = DspSpace::x, .address = 0xffff90}};
  for (const auto address : data) {
    for (const auto operation : {DspMemoryOperation::read, DspMemoryOperation::write}) {
      rejection(
          {.target = address, .operation = operation, .phase = DspMemoryPhase::peripheral_bit});
    }
  }
}

void native(const bool reserved) {
  DspMachine machine;
  test::DspProgram code;
  code.immediate("sr", bit::status);
  for (const auto store : {DspStore{.address = dma::source0, .value = dma::ring_start},
                           DspStore{.address = dma::destination0, .value = dma::destination},
                           DspStore{.address = dma::counter0},
                           DspStore{.address = dma::control0, .value = dma::word_copy}}) {
    code.emit("movep #>" + test::number(store.value) + ",x:<<" + test::number(store.address));
  }
  code.word(0);
  constexpr unsigned reserved_gpio_set = 0x013d37;
  code.word(reserved ? reserved_gpio_set : bit::opcode({.operation = 1}));
  for (unsigned index = 0; index < 32; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  auto &gpio = machine.bus().gpio(DspPort::c);
  gpio.write({.reg = DspGpioRegister::control, .value = reserved ? 0U : 1U});
  gpio.write({.reg = DspGpioRegister::direction, .value = reserved ? bit::mask({}) : 0U});
  gpio.write({.reg = DspGpioRegister::data, .value = bit::mask({})});
  machine.bus().write({.space = DspSpace::x, .address = dma::ring_start, .value = first});
  static_cast<void>(machine.advance_dma(banks::inputs({.index = 0})));
  test::checkpoint::run(machine, 5);
  auto expected = machine.cpu().debug_state();
  ++expected.cpu.pc;
  ++expected.native_instructions;
  ++expected.native_cycles;
  static_cast<void>(machine.begin_reference_instruction(pipeline::execution));
  DspFetchRun run(bit::contract(), machine);
  const auto saved = run.checkpoint(machine);
  std::string prior;
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(saved, machine);
    const auto error = test::esai::rejects([&] {
      while (run.completed_cycles() < 10) {
        const auto next = run.completed_cycles() + 1;
        static_cast<void>(
            run.advance_cycle(machine,
                              {.known_requests = 15,
                               .requests = next == 6 ? 1U : 0U,
                               .grant = next >= 7 ? DspDmaGrant::wait : DspDmaGrant::available}));
      }
    });
    require(error.contains(reserved ? "reserved-bit write" : "Unknown DSP GPIO data read") &&
                error.contains("pc=") && error.contains("omr=") && run.faulted() &&
                !error.contains("with active DMA"),
            "Actual native validation still rejects unknown GPIO samples and reserved writes");
    require(run.completed_cycles() == (reserved ? 7U : 6U) &&
                machine.cpu().debug_state() == expected,
            "A fault retains the prior NOP retirement without fabricating bit flags or completion");
    if (replay == 0) { prior = error; }
    require(error == prior, "Joint restore reproduces the same strict failure with real DMA held");
    ++native_refused;
  }
}
}

int main() {
  try {
    footprints();
    native(false);
    native(true);
    std::cout << "GPIO/DMA guards=" << refused << " native-fault-suffixes=" << native_refused
              << ";captured DMA retained,unknown pins and reserved writes strict\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
