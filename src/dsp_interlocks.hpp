// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace blofeld {
// Selected normal-mode internal-memory instruction dependencies. This does not
// assign physical F2/AG/execute bus phases or qualify branch/IRQ timing.
struct DspInterlockInstruction {
  std::uint32_t opcode = 0;
  unsigned native_units = 0;
  unsigned agu_reads = 0;
  unsigned agu_writes = 0;
  unsigned move_reads = 0;
  unsigned move_writes = 0;
  unsigned arithmetic_writes = 0;
  unsigned agu_write_delay = 0;
  unsigned transfer_write_delay = 0;
  unsigned move_read_advance = 0;
  bool status_read = false;
};

[[nodiscard]] DspInterlockInstruction decode_interlock_instruction(std::uint32_t opcode,
                                                                   unsigned native_units);

class DspInterlocks {
public:
  [[nodiscard]] unsigned preview(std::uint32_t opcode, unsigned native_units) const;
  void prepare(std::uint32_t opcode, unsigned native_units);
  void observe_opcode(std::uint32_t opcode);
  [[nodiscard]] unsigned complete(unsigned native_units);
  void reset() noexcept;

private:
  [[nodiscard]] unsigned delay(const DspInterlockInstruction &instruction) const;
  void commit(const DspInterlockInstruction &instruction, unsigned stalls);
  std::array<unsigned, 8> agu_{};
  std::array<unsigned, 2> arithmetic_{};
  std::array<unsigned, 2> transfer_{};
  unsigned status_ = 0;
  unsigned unknown_status_ = 0;
  std::optional<DspInterlockInstruction> prepared_;
  unsigned prepared_stalls_ = 0;
  bool fetched_ = false;
};
}
