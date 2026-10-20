// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_operand_fixture.hpp"
#include "dsp_saturation_fixture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace blofeld::test::saturation {
namespace {
constexpr std::uint64_t positive_limit = 0x007fffffffffff;
constexpr std::uint64_t negative_limit = 0xff800000000000;
constexpr unsigned positive_word = 0x7fffff;
constexpr unsigned source_sentinel = 0x654321;
constexpr unsigned bitfield_pattern = 0xa55aa5;
constexpr std::uint64_t positive_rounding_tie = 0x007fffff800000;
// All eight EXT7/EXT0/MSP23 patterns, both 56-bit endpoints, and an unused
// extension bit: table 3-1 is deliberately narrower than a signed 48-bit clamp.
constexpr std::array values{std::uint64_t{0},
                            std::uint64_t{1},
                            accumulator_mask,
                            positive_limit,
                            positive_limit + 1,
                            negative_limit,
                            negative_limit - 1,
                            sign_bit,
                            sign_bit - 1,
                            std::uint64_t{0x01000000000000},
                            std::uint64_t{0x01800000000000},
                            std::uint64_t{0x80000000000000},
                            std::uint64_t{0x80800000000000},
                            std::uint64_t{0x81000000000000},
                            std::uint64_t{0x02000000000000}};
constexpr std::array words{0U, 1U, 0x7fffffU, 0x800000U, word_mask};
constexpr std::array sources{
    std::uint64_t{0}, std::uint64_t{1}, accumulator_mask, sign_bit, sign_bit - 1};
constexpr std::array scales{0U, scale_down, scale_up};
constexpr std::array statuses{0U, carry | overflow, operands::all_ccr};

void variants(std::vector<Input> &result, Input input, const bool round = false) {
  for (const auto mode : {0U, saturation_mode}) {
    for (const auto scale : scales) {
      for (const auto status : statuses) {
        for (const bool destination_b : {false, true}) {
          input.destination_b = destination_b;
          input.status = mode | scale | status;
          result.push_back(input);
          if (round) {
            input.status |= rounding_mode;
            result.push_back(input);
          }
        }
      }
    }
  }
}

std::vector<Input> arithmetic_cases() {
  std::vector<Input> result;
  constexpr std::array operations{Operation::add,
                                  Operation::subtract,
                                  Operation::adc,
                                  Operation::sbc,
                                  Operation::add_left,
                                  Operation::add_right,
                                  Operation::subtract_left,
                                  Operation::subtract_right};
  for (const auto operation : operations) {
    for (const auto destination : values) {
      for (std::size_t index = 0; index < words.size(); ++index) {
        const bool full_source = operation == Operation::adc || operation == Operation::sbc;
        constexpr std::array wide_sources{std::uint64_t{0},
                                          std::uint64_t{1},
                                          positive_limit,
                                          negative_limit & 0xffffffffffff,
                                          std::uint64_t{0xffffffffffff}};
        const auto source = wide_sources.at(index);
        variants(result,
                 {.operation = operation,
                  .destination = destination,
                  .source = sources.at(index),
                  .x = full_source ? static_cast<unsigned>(source & word_mask) : words.at(index),
                  .y = full_source ? static_cast<unsigned>(source >> word_bits) : source_sentinel});
      }
    }
  }
  for (const auto operation : {Operation::absolute,
                               Operation::negate,
                               Operation::shift_left,
                               Operation::shift_right,
                               Operation::normalize}) {
    for (const auto destination : values) {
      const Input input{
          .operation = operation, .destination = destination, .source = other_accumulator};
      variants(result, input);
      if (operation == Operation::normalize) {
        for (const auto flags : {extension, unnormalized}) {
          auto normalization = input;
          normalization.status = saturation_mode | flags;
          result.push_back(normalization);
        }
      }
    }
  }
  return result;
}

void product_form(std::vector<Input> &result, Input input) {
  const bool round = input.operation == Operation::multiply_round ||
                     input.operation == Operation::accumulate_round;
  for (const auto x : {0x800000U, positive_word, word_mask}) {
    for (const auto y : {0x800000U, positive_word, word_mask}) {
      input.x = x;
      input.y = y;
      variants(result, input, round);
      if (input.form != ProductForm::immediate) {
        input.negative_product = true;
        variants(result, input, round);
        input.negative_product = false;
      }
    }
  }
}

void extra_product_forms(std::vector<Input> &result, const Operation operation) {
  for (const auto form : {ProductForm::registers, ProductForm::immediate, ProductForm::power}) {
    for (const auto power : {1U, 8U, 22U}) {
      if (form != ProductForm::power && power != 1) { continue; }
      product_form(result,
                   {.operation = operation,
                    .destination = positive_limit,
                    .source = other_accumulator,
                    .power = power,
                    .form = form});
    }
  }
}

std::vector<Input> product_cases(const bool accumulate) {
  std::vector<Input> result;
  const auto plain = accumulate ? Operation::accumulate : Operation::multiply;
  const auto rounded_operation =
      accumulate ? Operation::accumulate_round : Operation::multiply_round;
  for (const auto operation : {plain, rounded_operation}) {
    const bool round = operation == rounded_operation;
    for (const auto destination : {std::uint64_t{0},
                                   positive_limit,
                                   negative_limit,
                                   sign_bit,
                                   sign_bit - 1,
                                   accumulator_mask}) {
      if (!accumulate && destination != 0) { continue; }
      for (const auto x : words) {
        for (const auto y : words) {
          variants(result,
                   {.operation = operation,
                    .destination = destination,
                    .source = other_accumulator,
                    .x = x,
                    .y = y},
                   round);
        }
      }
    }
    extra_product_forms(result, operation);
  }
  return result;
}

std::vector<Input> rounding_cases() {
  std::vector<Input> result;
  for (const auto base : {std::uint64_t{0},
                          positive_limit & ~std::uint64_t{word_mask},
                          negative_limit,
                          sign_bit - (std::uint64_t{1} << 26)}) {
    for (const auto half :
         {std::uint64_t{1} << 22, std::uint64_t{1} << 23, std::uint64_t{1} << 24}) {
      for (const auto offset : {half - 1, half, half + 1, half * 3}) {
        for (const bool negative_value : {false, true}) {
          auto value = base + offset;
          // Explicit two's-complement encoding without unsigned underflow.
          if (negative_value) { value = accumulator_mask - value + 1; }
          variants(
              result,
              {.operation = Operation::round, .destination = value, .source = other_accumulator},
              true);
        }
      }
    }
  }
  return result;
}

void transfer_exceptions(std::vector<Input> &result) {
  for (const auto operation : {Operation::bfu_left,
                               Operation::bfu_right,
                               Operation::transfer,
                               Operation::conditional_transfer}) {
    for (const auto value : values) {
      variants(result, {.operation = operation, .destination = value, .source = value});
      variants(result, {.operation = operation, .destination = value, .source = other_accumulator});
    }
  }
}

void mixed_product_exceptions(std::vector<Input> &result) {
  for (const auto operation :
       {Operation::multiply, Operation::accumulate, Operation::double_accumulate}) {
    for (const auto sign :
         {Signedness::signed_signed, Signedness::signed_unsigned, Signedness::unsigned_unsigned}) {
      if (sign == Signedness::signed_signed && operation != Operation::double_accumulate) {
        continue;
      }
      for (const auto x : words) {
        for (const auto y : words) {
          variants(result,
                   {.operation = operation,
                    .destination = positive_limit,
                    .source = other_accumulator,
                    .x = x,
                    .y = y,
                    .signedness = sign});
        }
      }
    }
  }
}

void bitfield_exceptions(std::vector<Input> &result) {
  for (const auto operation : {Operation::extract, Operation::insert}) {
    for (const auto value : values) {
      for (const auto control : {0x018000U, 0x00c018U, 0x008028U}) {
        variants(result,
                 {.operation = operation,
                  .destination = value,
                  .source = value,
                  .x = control,
                  .y = bitfield_pattern});
      }
    }
  }
}

void self_transfers(std::vector<Input> &result) {
  for (const auto value : values) {
    for (const auto status : statuses) {
      for (const auto mode : {0U, saturation_mode}) {
        for (const bool destination_b : {false, true}) {
          result.push_back({.operation = Operation::self_move,
                            .destination = value,
                            .source = other_accumulator,
                            .status = mode | status,
                            .destination_b = destination_b});
        }
      }
    }
  }
}

std::vector<Input> exception_cases() {
  std::vector<Input> result;
  transfer_exceptions(result);
  mixed_product_exceptions(result);
  bitfield_exceptions(result);
  self_transfers(result);
  return result;
}

std::vector<Input> regressions() {
  return {{.operation = Operation::add,
           .destination = positive_limit,
           .x = 1,
           .status = saturation_mode},
          {.operation = Operation::subtract,
           .destination = negative_limit,
           .x = 1,
           .status = saturation_mode},
          {.operation = Operation::absolute, .destination = sign_bit, .status = saturation_mode},
          {.operation = Operation::negate, .destination = sign_bit, .status = saturation_mode},
          {.operation = Operation::shift_left,
           .destination = 0x00400000000000,
           .status = saturation_mode},
          {.operation = Operation::round,
           .destination = positive_rounding_tie,
           .status = saturation_mode},
          {.operation = Operation::accumulate,
           .destination = positive_limit,
           .x = 1,
           .y = 1,
           .status = saturation_mode},
          {.operation = Operation::accumulate_round,
           .destination = positive_limit,
           .x = 1,
           .y = 1,
           .status = saturation_mode}};
}
}

std::vector<Input> cases(const std::string_view scope) {
  if (scope == "regressions") { return regressions(); }
  if (scope == "arithmetic") { return arithmetic_cases(); }
  if (scope == "multiply") { return product_cases(false); }
  if (scope == "accumulate") { return product_cases(true); }
  if (scope == "rounding") { return rounding_cases(); }
  if (scope == "exceptions") { return exception_cases(); }
  if (scope == "mac-host-overflow") {
    return {{.operation = Operation::accumulate,
             .destination = sign_bit - 1,
             .x = positive_word,
             .y = positive_word}};
  }
  throw Fault("Unknown own DSP saturation diagnostic scope");
}
}
