// SPDX-License-Identifier: GPL-3.0-only
#include "census_record.hpp"

#include "audio/wav_consumer.hpp"
#include "census.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <ostream>

namespace blofeld::app {
namespace {
void usage(std::ostream &out, const vi::Census::OwnerUsage &owner) {
  out << R"({"id":)" << owner.native.identifier << R"(,"error":)" << owner.native.error
      << R"(,"pmc":)" << (owner.native.pmc ? "true" : "false") << R"(,"wait_coherent":)"
      << (owner.wait.coherent ? "true" : "false") << R"(,"wait_calls":)" << owner.wait.calls
      << R"(,"wait_ns":)" << owner.wait.nanoseconds << R"(,"naming_error":)" << owner.naming_error
      << R"(,"levels":[)";
  for (unsigned i = 0; i < owner.native.count; ++i) {
    const auto &level = owner.native.levels[i];
    out << (i == 0 ? "" : ",") << R"({"cpu_ns":)" << level.cpu_nanoseconds << R"(,"instructions":)"
        << level.instructions << R"(,"cycles":)" << level.cycles << '}';
  }
  out << "]}";
}

void snapshot(std::ostream &out, const vi::Census::Snapshot &sample) {
  out << R"({"valid":)" << (sample.valid ? "true" : "false") << R"(,"opened_ns":)" << sample.opened
      << R"(,"closed_ns":)" << sample.closed << R"(,"guest":{"instructions":)"
      << sample.guest.instructions << R"(,"cycles":)" << sample.guest.cycles << R"(,"idle":)"
      << sample.guest.idle << R"(,"epoch":)" << sample.guest.epoch << R"(,"pc":)" << sample.guest.pc
      << R"(,"region_instructions":)" << sample.guest.region_instructions
      << R"(,"precise_instructions":)" << sample.guest.precise_instructions
      << R"(,"batched_instructions":)" << sample.guest.batched_instructions
      << R"(,"deferred_instructions":)" << sample.guest.deferred_instructions << R"(},"owners":[)";
  for (unsigned i = 0; i < sample.owners.size(); ++i) {
    if (i != 0) { out << ','; }
    usage(out, sample.owners[i]);
  }
  out << "]}";
}

void percentile(std::ostream &out,
                const vi::Census::Sessions sessions,
                const unsigned numerator,
                const unsigned denominator) {
  if (sessions.count == 0) {
    out << "null";
    return;
  }
  const auto rank = (sessions.count * numerator + denominator - 1) / denominator;
  std::uint64_t sum = 0;
  for (std::size_t i = 0; i < sessions.bins.size(); ++i) {
    sum += sessions.bins[i];
    if (sum < rank) { continue; }
    if (i + 1 == sessions.bins.size() && sessions.maximum > i) {
      out << "null";
    } else {
      out << i;
    }
    return;
  }
  out << "null";
}
}

void CensusRecord::observe(const vi::Census &census, const WavConsumer &wav) noexcept {
  if (!census.enabled() || wav.frames() == 0) { return; }
  if (first_frame_ == 0) {
    first_frame_ = wav.frames();
    first_source_ = wav.source_frames();
    first_ = census.capture();
    vi::Census::recording(true);
  }
  if (wav.complete()) {
    last_frame_ = wav.frames();
    last_source_ = wav.source_frames();
    last_ = census.capture();
    vi::Census::recording(false);
  }
}

void CensusRecord::report(const vi::Census &census) const {
  if (!census.enabled()) { return; }
  auto &out = std::cerr;
  out << R"(Census: {"first_frame":)" << first_frame_ << R"(,"last_frame":)" << last_frame_
      << R"(,"first_source_frame":)" << first_source_ << R"(,"last_source_frame":)" << last_source_
      << R"(,"begin":)";
  snapshot(out, first_);
  out << R"(,"end":)";
  snapshot(out, last_);
  const auto sessions = census.sessions(vi::Census::Actor::dsp);
  out << R"(,"sessions":{"count":)" << sessions.count << R"(,"retired":)" << sessions.sum
      << R"(,"max":)" << sessions.maximum << R"(,"quantile":"nearest_rank","p10":)";
  percentile(out, sessions, 1, 10);
  out << R"(,"p50":)";
  percentile(out, sessions, 1, 2);
  out << R"(,"p90":)";
  percentile(out, sessions, 9, 10);
  out << "}}\n";
  const auto units = census.units();
  out << R"(NativeUnits: {"entries":)" << units.histogram.count << R"(,"region_retirements":)"
      << units.histogram.sum << R"(,"max":)" << units.histogram.maximum << R"(,"p50":)";
  percentile(out, units.histogram, 1, 2);
  out << R"(,"p90":)";
  percentile(out, units.histogram, 9, 10);
  out << R"(,"boundary_crossings":)" << units.boundary_crossings
      << R"(,"unattributed_region_retirements":)" << units.unattributed
      << R"(,"retirements_by_entry_size":[)" << units.region_retirements[0] << ','
      << units.region_retirements[1] << ',' << units.region_retirements[2] << "]}\n";
  constexpr std::array names{"p-reserved", "x-rom", "x-reserved", "y-rom", "y-reserved"};
  const auto discarded = census.discarded_writes();
  out << "DiscardedWrites: [";
  for (std::size_t i = 0; i < discarded.size(); ++i) {
    const auto &record = discarded[i];
    out << (i == 0 ? "" : ",") << R"({"class":")" << names[i] << R"(","count":)" << record.count;
    if (record.count != 0) {
      out << R"(,"first":)" << record.first << R"(,"min":)" << record.minimum << R"(,"max":)"
          << record.maximum << R"(,"pc":)" << record.pc << R"(,"value":)" << record.value;
    }
    out << '}';
  }
  out << "]\n";
  out << "PreciseOperations: [";
  const auto operations = census.precise_operations();
  bool first = true;
  for (std::size_t i = 0; i < operations.size(); ++i) {
    if (operations[i] == 0) { continue; }
    out << (first ? "" : ",") << R"({"kind":)" << i << R"(,"retired":)" << operations[i] << '}';
    first = false;
  }
  out << "]\n";
}
}
