// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::saturation {
inline constexpr unsigned saturation_mode = 0x100000;
inline constexpr unsigned rounding_mode = 0x200000;
inline constexpr unsigned counter_seed = 0x234567;
inline constexpr unsigned preamble_steps = 12;

enum class Operation : unsigned char {
  add,
  subtract,
  adc,
  sbc,
  add_left,
  add_right,
  subtract_left,
  subtract_right,
  absolute,
  negate,
  shift_left,
  shift_right,
  bfu_left,
  bfu_right,
  normalize,
  multiply,
  accumulate,
  multiply_round,
  accumulate_round,
  double_accumulate,
  round,
  extract,
  insert,
  transfer,
  conditional_transfer,
  self_move,
};
enum class ProductForm : unsigned char { registers, immediate, power };
enum class Signedness : unsigned char { signed_signed, signed_unsigned, unsigned_unsigned };

struct Input {
  Operation operation = Operation::add;
  std::uint64_t destination = 0;
  std::uint64_t source = 0;
  unsigned x = 0;
  unsigned y = 0;
  unsigned status = 0;
  unsigned power = 1;
  bool destination_b = false;
  bool negative_product = false;
  ProductForm form = ProductForm::registers;
  Signedness signedness = Signedness::signed_signed;
};

struct Expected {
  std::uint64_t value = 0;
  unsigned flag_mask = 0;
  unsigned flags = 0;
  int counter_delta = 0;
  bool qualified_carry = true;
};

[[nodiscard]] Expected expected(const Input &input);
[[nodiscard]] std::string instruction(const Input &input);
[[nodiscard]] std::vector<Input> cases(std::string_view scope);
void run(std::span<const Input> inputs);
}
