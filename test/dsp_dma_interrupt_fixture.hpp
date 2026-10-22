// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dma_fixture.hpp"

namespace blofeld::test::dma_irq {
using dma_test::rejects;
using dma_test::require;
inline constexpr unsigned iprc = 0xffffff;
inline constexpr unsigned iprp = 0xfffffe;
inline constexpr unsigned original_priority = 0x4006;
inline constexpr unsigned first_vector = 0x18;
inline constexpr unsigned channels = 6;
inline constexpr unsigned priority_shift = 12;
inline constexpr unsigned interrupt_enable = 0x400000;
inline constexpr unsigned line_copy = 0x900244;   // DE,DTM2,IRQ A,X->Y,fixed/fixed.
inline constexpr unsigned repeat_copy = 0xa80244; // DE,DTM5,IRQ A,X->Y,fixed/fixed.
void native_checks();
}
