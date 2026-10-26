// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_postloop_context_fixture.hpp"

#include <exception>
#include <iostream>

int main() {
  using namespace blofeld::test::postloop_context;
  try {
    for (const auto form : {Form::direct, Form::short_immediate, Form::long_immediate}) {
      for (const unsigned count : {0U, 1U, 3U}) {
        for (const unsigned size : {1U, 2U}) {
          for (const unsigned gap : {0U, 1U, 2U}) {
            one_case({.count = count, .size = size, .gap = gap, .form = form});
          }
        }
      }
      restores({.form = form});
    }
    active_guard();
    std::cout << "54 literal loop/OMR/reopen streams,90 trusted suffixes and active guard passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
