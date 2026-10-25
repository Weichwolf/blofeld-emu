// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_master_output_model.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld::test::master {
struct Access {
  DspNativeAccessKind kind;
  DspSpace space;
  unsigned address;
  unsigned value;
  bool operator==(const Access &) const = default;
};

struct Effects {
  unsigned steps = 0;
  std::vector<Access> data;
  bool operator==(const Effects &) const = default;
};

struct Saved {
  DspCheckpoint machine;
  Effects effects;
};

class Fixture {
public:
  explicit Fixture(const Firmware &firmware);
  void run(const Input &input, const Samples &source, const Prediction &expected, bool replay);
  void failures();
  void intact();

  [[nodiscard]] unsigned calls() const { return calls_; }

  [[nodiscard]] unsigned restores() const { return restores_; }

  [[nodiscard]] unsigned failed_reads() const { return failed_reads_; }

  [[nodiscard]] const Coefficients &coefficients() const { return coefficients_; }

private:
  void seed(const Input &input, const Samples &source, unsigned missing = 0);
  void execute(bool capture);
  void observe(const DspNativeAccess &access);
  void verify(const Input &input, const Samples &source, const Prediction &expected);
  void verify_accesses(const Input &input, const Samples &source, const Prediction &expected) const;
  void restore(const Saved &saved);
  DspMachine machine_;
  std::optional<DspCheckpoint> baseline_;
  std::optional<Saved> middle_;
  Effects effects_;
  std::vector<unsigned> program_;
  Coefficients coefficients_;
  std::array<unsigned, 64> output_{};
  unsigned stop_ = 0;
  unsigned calls_ = 0;
  unsigned restores_ = 0;
  unsigned failed_reads_ = 0;
};
}
