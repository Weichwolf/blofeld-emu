// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_dma_pipeline_fixture.hpp"

#include <exception>
#include <iostream>

int main() {
  try {
    blofeld::test::dma_pipeline::source_conflict();
    blofeld::test::dma_pipeline::captured_source();
    blofeld::test::dma_pipeline::destination_hold_restore();
    blofeld::test::dma_pipeline::disable_captured_word();
    blofeld::test::dma_pipeline::controller_guards();
    blofeld::test::dma_pipeline::request_knowledge_restore();
    std::cout << "scenarios=10 assertions=" << blofeld::test::dma_pipeline::assertions
              << " expected-refusals=" << blofeld::test::dma_pipeline::refusals
              << " attempted-advances=" << blofeld::test::dma_pipeline::attempted_advances
              << " completed-advances=" << blofeld::test::dma_pipeline::completed_advances
              << " physical-qualified=0\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
