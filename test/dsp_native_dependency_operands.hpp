// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld::test {
unsigned native_dependency_operand_checks();
unsigned native_dependency_alu_checks();
unsigned native_dependency_scalar_checks();
unsigned native_dependency_parallel_checks();
unsigned native_dependency_address_checks();
}
