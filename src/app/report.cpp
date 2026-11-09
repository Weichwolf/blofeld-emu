// SPDX-License-Identifier: GPL-3.0-only
#include "report.hpp"

#include "execution.hpp"

#include <cstddef>
#include <exception>
#include <ios>
#include <iostream>
#include <span>

namespace blofeld::app {
namespace {
const char *operation(const vi::Access access) {
  switch (access) {
    case vi::Access::read: return "read";
    case vi::Access::write: return "write";
    case vi::Access::fetch: return "fetch";
    case vi::Access::bit: return "bit";
    case vi::Access::execute: return "execute";
    case vi::Access::reset: return "reset";
    default: return "unknown";
  }
}

}

bool report_execution(const std::span<const vi::ExecutionReport> reports) {
  bool valid = true;
  for (std::size_t index = 0; index < reports.size(); ++index) {
    const auto &report = reports[index];
    if (report.exception) { std::rethrow_exception(report.exception); }
    const auto *name = index == 0 ? "MCU" : "DSP";
    std::cerr << name << " retired=" << report.instructions;
    if (report.stopped_pc) {
      std::cerr << " stopped-pc=0x" << std::hex << *report.stopped_pc << std::dec;
    }
    if (report.failure) {
      valid = false;
      const auto &fault = *report.failure;
      std::cerr << " " << operation(fault.operation) << " space=" << fault.space << " pc=0x"
                << std::hex << fault.pc << " address=0x" << fault.address << " value=0x"
                << fault.value << std::dec << " code=" << fault.code << ": " << fault.reason;
      if (fault.media) {
        std::cerr << "; media-page=0x" << std::hex << fault.media->page << " media-column=0x"
                  << fault.media->column << std::dec;
      }
    }
    std::cerr << '\n';
  }
  return valid;
}
}
