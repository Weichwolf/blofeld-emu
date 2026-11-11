// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/io.hpp"

#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>

namespace blofeld::vi::dsp {
extern "C" bool blofeld_dsp_read(dsp56300::Io *owner,
                                 dsp56300::Space space,
                                 dsp56300::Word address,
                                 dsp56300::Word pc,
                                 dsp56300::Word &value) noexcept {
  return static_cast<Io *>(owner)->read(space, address, pc, value);
}

extern "C" bool blofeld_dsp_write(dsp56300::Io *owner,
                                  dsp56300::Space space,
                                  dsp56300::Word address,
                                  dsp56300::Word pc,
                                  dsp56300::Word value) noexcept {
  return static_cast<Io *>(owner)->write(space, address, pc, value);
}

extern "C" bool blofeld_dsp_bit(dsp56300::Io *owner,
                                dsp56300::Space space,
                                dsp56300::Word address,
                                dsp56300::Word pc,
                                dsp56300::Word bit,
                                dsp56300::BitAction action,
                                bool &previous) noexcept {
  return static_cast<Io *>(owner)->bit(space, address, pc, bit, action, previous);
}

extern "C" bool blofeld_dsp_reset(dsp56300::Io *owner, dsp56300::Word pc) noexcept {
  return static_cast<Io *>(owner)->reset_peripherals(pc);
}

extern "C" bool blofeld_dsp_pending(const dsp56300::Io *owner, dsp56300::Word status) noexcept {
  return static_cast<const Io *>(owner)->interrupt_pending(status);
}

extern "C" std::uint64_t blofeld_dsp_revision(const dsp56300::Io *owner) noexcept {
  return static_cast<const Io *>(owner)->program_revision();
}
}
