// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_saturation_fixture.hpp"

#include <array>
#include <cstdint>

namespace blofeld::test::saturation {
namespace {
constexpr std::uint64_t range = std::uint64_t{1} << 56;
constexpr auto bound = static_cast<std::int64_t>(sign_bit);
constexpr unsigned data_flags = extension | unnormalized | negative | zero;
constexpr std::uint64_t positive_limit = 0x007fffffffffff;
constexpr std::uint64_t negative_limit = 0xff800000000000;
constexpr std::uint64_t rounded_positive_limit = 0x007fffff000000;

struct Arithmetic {
  std::int64_t value = 0;
  bool overflowed = false;
  bool carry_bit = false;
  bool carry_written = true;
  bool qualified_carry = true;
  bool saturates = true;
  bool rounded = false;
  unsigned flag_mask = carry | overflow | data_flags;
  unsigned sticky_flags = 0;
  int counter_delta = 0;
};

std::int64_t signed_accumulator(const std::uint64_t value) {
  return (value & sign_bit) != 0 ? -static_cast<std::int64_t>(range - value)
                                 : static_cast<std::int64_t>(value);
}

std::int64_t signed_word(const unsigned value) {
  constexpr unsigned word_range = 0x1000000;
  constexpr unsigned word_sign = 0x800000;
  return value >= word_sign ? -static_cast<std::int64_t>(word_range - value)
                            : static_cast<std::int64_t>(value);
}

std::uint64_t raw(const std::int64_t value) {
  return static_cast<std::uint64_t>(value) & accumulator_mask;
}

bool outside(const std::int64_t value) {
  return value < -bound || value >= bound;
}

std::int64_t floor_divide(const std::int64_t value, const std::int64_t divisor) {
  auto quotient = value / divisor;
  if (value < 0 && value % divisor != 0) { --quotient; }
  return quotient;
}

// FM3.2.2: quotient/remainder rounding, independently of the SDK's mask/add method.
std::int64_t rounded(const std::int64_t value, const Input &input) {
  const auto status = input.status;
  unsigned width = word_bits;
  if ((status & scale_up) != 0) { --width; }
  if ((status & scale_down) != 0) { ++width; }
  const auto grid = std::int64_t{1} << width;
  auto quotient = floor_divide(value, grid);
  const auto remainder = value - (quotient * grid);
  if (remainder > grid / 2 ||
      (remainder == grid / 2 && ((status & rounding_mode) != 0 || quotient % 2 != 0))) {
    ++quotient;
  }
  return quotient * grid;
}

Arithmetic addition(const Input &input, const bool subtract, const bool with_carry) {
  const auto destination = signed_accumulator(input.destination);
  const auto source = with_carry ? (signed_word(input.y) * (std::int64_t{1} << word_bits)) + input.x
                                 : signed_word(input.x) * (std::int64_t{1} << word_bits);
  const unsigned incoming = with_carry && (input.status & carry) != 0 ? 1U : 0U;
  const auto mathematical =
      subtract ? destination - source - incoming : destination + source + incoming;
  const auto source_raw = raw(source);
  const bool carry_bit = subtract ? input.destination < source_raw + incoming
                                  : (input.destination + source_raw + incoming) >= range;
  return {.value = mathematical, .overflowed = outside(mathematical), .carry_bit = carry_bit};
}

Arithmetic shifted(const Input &input, const bool left, const bool subtract) {
  const auto destination = signed_accumulator(input.destination);
  const auto shifted_value = left ? destination * 2 : floor_divide(destination, 2);
  const auto source = signed_accumulator(input.source);
  const auto mathematical = subtract ? shifted_value - source : shifted_value + source;
  const bool shift_overflow = outside(shifted_value);
  return {.value = mathematical,
          .overflowed = shift_overflow || outside(mathematical),
          .carry_bit = subtract ? raw(shifted_value) < input.source
                                : raw(shifted_value) + input.source >= range,
          .qualified_carry = !shift_overflow};
}

Arithmetic unary(const Input &input) {
  const auto destination = signed_accumulator(input.destination);
  Arithmetic result;
  if (input.operation == Operation::absolute) {
    result.value = destination < 0 ? -destination : destination;
  } else if (input.operation == Operation::negate) {
    result.value = -destination;
  } else if (input.operation == Operation::increment) {
    result.value = destination + 1;
    result.carry_bit = input.destination == accumulator_mask;
  } else if (input.operation == Operation::decrement) {
    result.value = destination - 1;
    result.carry_bit = input.destination == 0;
  } else if (input.operation == Operation::shift_left || input.operation == Operation::bfu_left) {
    result.value = destination * 2;
    result.carry_bit = (input.destination & sign_bit) != 0;
  } else if (input.operation == Operation::shift_right || input.operation == Operation::bfu_right) {
    result.value = floor_divide(destination, 2);
    result.carry_bit = (input.destination & 1U) != 0;
  } else {
    throw Fault("Expected a unary saturation-oracle operation");
  }
  if (input.operation == Operation::absolute || input.operation == Operation::negate) {
    result.carry_written = false;
    result.flag_mask &= ~carry;
  }
  result.saturates =
      input.operation != Operation::bfu_left && input.operation != Operation::bfu_right;
  result.overflowed = outside(result.value);
  return result;
}

Arithmetic normalize(const Input &input) {
  auto value = signed_accumulator(input.destination);
  int counter = 0;
  if ((input.status & extension) != 0) {
    value = floor_divide(value, 2);
    counter = 1;
  } else if ((input.status & (unnormalized | zero)) == unnormalized) {
    value *= 2;
    counter = -1;
  }
  return {.value = value,
          .overflowed = outside(value),
          .carry_written = false,
          .flag_mask = overflow | data_flags,
          .counter_delta = counter};
}

Arithmetic product(const Input &input) {
  const bool first_unsigned = input.signedness == Signedness::unsigned_unsigned;
  const bool second_unsigned = input.signedness != Signedness::signed_signed;
  const auto first = first_unsigned ? std::int64_t{input.x} : signed_word(input.x);
  auto second = second_unsigned ? std::int64_t{input.y} : signed_word(input.y);
  if (input.form == ProductForm::power) {
    second = std::int64_t{1} << ((word_bits - 1) - input.power);
  }
  auto value = first * second * 2;
  if (input.negative_product) { value = -value; }
  if (input.operation == Operation::accumulate || input.operation == Operation::accumulate_round) {
    value += signed_accumulator(input.destination);
  }
  if (input.operation == Operation::double_accumulate) {
    value += floor_divide(signed_accumulator(input.destination), std::int64_t{1} << word_bits);
  }
  const bool round = input.operation == Operation::multiply_round ||
                     input.operation == Operation::accumulate_round;
  if (round) { value = rounded(value, input); }
  return {.value = value,
          .overflowed = outside(value),
          .carry_written = false,
          .saturates = input.signedness == Signedness::signed_signed,
          .rounded = round,
          .flag_mask = overflow | data_flags};
}

Arithmetic bitfield(const Input &input) {
  constexpr unsigned width_shift = 12;
  constexpr unsigned control_mask = 0x3f;
  const auto width = (input.x >> width_shift) & control_mask;
  const auto offset = input.x & control_mask;
  auto value = input.operation == Operation::insert ? input.destination : 0;
  for (unsigned bit = 0; bit < width; ++bit) {
    if (input.operation == Operation::insert) {
      const auto position = offset + bit;
      const auto mask = std::uint64_t{1} << position;
      value = (value & ~mask) | (std::uint64_t{(input.y >> bit) & 1U} << position);
    } else {
      value |= ((input.source >> (offset + bit)) & 1U) << bit;
    }
  }
  return {.value = signed_accumulator(value), .saturates = false};
}

Arithmetic transfer(const Input &input) {
  auto value = input.source;
  if (input.operation == Operation::conditional_transfer && (input.status & zero) != 0) {
    value = input.destination;
  }
  if (input.operation != Operation::self_move) {
    return {.value = signed_accumulator(value),
            .carry_written = false,
            .saturates = false,
            .flag_mask = 0};
  }
  // Literal no-scaling self transfers separately exercise the data-bus limiter.
  auto data = signed_accumulator(input.destination);
  constexpr std::int64_t minimum = -0x800000000000;
  constexpr std::int64_t maximum = 0x7fffffffffff;
  unsigned flags = 0;
  if (data < minimum) {
    data = minimum;
    flags |= limit;
  }
  if (data > maximum) {
    data = maximum;
    flags |= limit;
  }
  constexpr unsigned growth_high = 46;
  if (((input.destination >> growth_high) & 1U) !=
      ((input.destination >> (growth_high - 1)) & 1U)) {
    flags |= sticky;
  }
  const auto bus_word = (raw(data) >> word_bits) & word_mask;
  return {.value = signed_word(static_cast<unsigned>(bus_word)) * (std::int64_t{1} << word_bits),
          .carry_written = false,
          .saturates = false,
          .flag_mask = 0,
          .sticky_flags = flags};
}

Arithmetic operation(const Input &input) {
  switch (input.operation) {
    case Operation::add: return addition(input, false, false);
    case Operation::subtract: return addition(input, true, false);
    case Operation::adc: return addition(input, false, true);
    case Operation::sbc: return addition(input, true, true);
    case Operation::add_left: return shifted(input, true, false);
    case Operation::add_right: return shifted(input, false, false);
    case Operation::subtract_left: return shifted(input, true, true);
    case Operation::subtract_right: return shifted(input, false, true);
    case Operation::absolute:
    case Operation::negate:
    case Operation::increment:
    case Operation::decrement:
    case Operation::shift_left:
    case Operation::shift_right:
    case Operation::bfu_left:
    case Operation::bfu_right: return unary(input);
    case Operation::normalize: return normalize(input);
    case Operation::multiply:
    case Operation::accumulate:
    case Operation::multiply_round:
    case Operation::accumulate_round:
    case Operation::double_accumulate: return product(input);
    case Operation::round: {
      const auto value = rounded(signed_accumulator(input.destination), input);
      return {.value = value,
              .overflowed = outside(value),
              .carry_written = false,
              .rounded = true,
              .flag_mask = overflow | data_flags};
    }
    case Operation::extract:
    case Operation::insert: return bitfield(input);
    case Operation::transfer:
    case Operation::conditional_transfer:
    case Operation::self_move: return transfer(input);
    default: throw Fault("Unknown own saturation-oracle operation");
  }
}
}

Expected expected(const Input &input) {
  auto result = operation(input);
  auto value = raw(result.value);
  if (result.saturates && (input.status & saturation_mode) != 0) {
    // FM3.2.3 table3-1, deliberately expressed as its literal eight outcomes.
    constexpr std::array table{0U, 1U, 1U, 1U, 2U, 2U, 2U, 0U};
    const unsigned selector = (((value & sign_bit) != 0) ? 4U : 0U) |
                              (((value & (std::uint64_t{1} << 48)) != 0) ? 2U : 0U) |
                              (((value & (std::uint64_t{1} << 47)) != 0) ? 1U : 0U);
    const auto action = table.at(selector);
    result.overflowed = action != 0;
    if (action == 1) { value = result.rounded ? rounded_positive_limit : positive_limit; }
    if (action == 2) { value = negative_limit; }
  }
  unsigned flags = value_flags({.value = value, .status = input.status}) & result.flag_mask;
  if (result.carry_written && result.carry_bit) { flags |= carry; }
  if (result.overflowed) { flags |= overflow | limit; }
  return {.value = value,
          .flag_mask = result.flag_mask,
          .flags = flags | result.sticky_flags,
          .counter_delta = result.counter_delta,
          .qualified_carry = result.qualified_carry};
}
}
