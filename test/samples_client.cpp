// SPDX-License-Identifier: GPL-3.0-only
#include "samples/sample_client.hpp"

#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <string_view>

int main(const int argc, char **argv) {
  try {
    if (argc < 2 || std::string_view(argv[1]) != "samples") { return 1; }
    return blofeld::samples::run_client(std::span(argv + 2, static_cast<std::size_t>(argc - 2)),
                                        std::cout);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
