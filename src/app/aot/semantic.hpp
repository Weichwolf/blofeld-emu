// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "aot/artifact.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace blofeld::app::aot::detail {
class Inputs {
public:
  void number(const std::uint64_t value) {
    constexpr unsigned byte_bits = 8;
    for (unsigned byte = 0; byte < sizeof(value); ++byte) {
      data.push_back(static_cast<std::byte>(value >> (byte * byte_bits)));
    }
  }

  void bytes(const std::span<const std::byte> value) {
    number(value.size());
    data.insert(data.end(), value.begin(), value.end());
  }

  void text(const std::string_view value) { bytes(std::as_bytes(std::span(value))); }

  void sequence(const auto &values, const auto &append) {
    number(values.size());
    for (const auto &value : values) { append(value); }
  }

  std::vector<std::byte> data;
};

// Only compiler inputs are encoded, never RAM, device state or host addresses.
inline Inputs
semantic_inputs(const auto image, const auto options, const std::string_view compiler) {
  Inputs key;
  key.text("semantic-inputs-2");
  key.text(compiler);
  key.sequence(image.variants, [&](const auto &variant) {
    key.number(variant.id);
    key.sequence(variant.code, [&](const auto span) {
      key.number(span.address);
      key.bytes(std::as_bytes(span.words));
    });
    key.bytes(std::as_bytes(variant.entries));
    if constexpr (requires { variant.failed_fetches; }) {
      key.bytes(std::as_bytes(variant.failed_fetches));
    }
    if constexpr (requires { variant.region_starts; }) {
      key.bytes(std::as_bytes(variant.region_starts));
    }
    if constexpr (requires { variant.foreground_entries; }) {
      key.bytes(std::as_bytes(variant.foreground_entries));
    }
  });
  key.sequence(options.memory, [&](const auto &variant) {
    key.number(variant.id);
    key.sequence(variant.memory, [&](const auto region) {
      if constexpr (requires { region.space; }) {
        key.number(static_cast<std::uint64_t>(region.space));
      }
      key.number(region.first);
      key.number(region.size);
      key.number(region.writable);
      key.number(region.tracked);
      if constexpr (requires { region.aliases_program; }) { key.number(region.aliases_program); }
    });
  });
  key.sequence(options.interrupts, [&](const auto request) {
    key.number(request.vector);
    if constexpr (requires { request.kind; }) {
      key.number(static_cast<std::uint64_t>(request.kind));
    }
    if constexpr (requires { request.level; }) { key.number(request.level); }
  });
  key.sequence(options.transitions, [&](const auto edge) {
    key.number(edge.from);
    key.number(edge.to);
    key.number(edge.entry);
  });
  key.sequence(options.callbacks, [&](const auto binding) {
    key.number(static_cast<std::uint64_t>(binding.role));
    key.text(binding.symbol);
  });
  key.bytes(options.bindings);
  if constexpr (requires { options.memory_mode; }) {
    key.number(static_cast<std::uint64_t>(options.memory_mode));
  }
  if constexpr (requires { options.topology; }) {
    key.number(static_cast<std::uint64_t>(options.topology));
  }
  if constexpr (requires { options.region_modes; }) {
    key.number(options.region_modes.has_value());
    if (options.region_modes) { key.bytes(std::as_bytes(*options.region_modes)); }
  }
  if constexpr (requires { options.fixed_passes; }) {
    key.sequence(options.fixed_passes, [&](const auto &pass) {
      key.number(pass.variant);
      key.bytes(std::as_bytes(pass.path));
      key.sequence(pass.device_reads, [&](const auto &read) {
        key.number(static_cast<std::uint64_t>(read.space));
        key.number(read.address);
      });
    });
  }
  return key;
}

template <class Core>
auto prepare_semantic(const auto image,
                      const auto options,
                      const std::filesystem::path &cache,
                      const host::aot::Preparation preparation = host::aot::Preparation::allow) {
  if (cache.empty() && preparation == host::aot::Preparation::allow) {
    return Core::lower(image, options);
  }
  const auto inputs = semantic_inputs(image, options, Core::compiler_identity());
  const host::aot::Artifact entry(cache, inputs.data);
  if (auto bytes = entry.load()) {
    try {
      return Core::load(image, options, *bytes);
    } catch (const std::invalid_argument &) {
      // Discard the rejected record before a potentially large cold rebuild.
      bytes.reset();
    }
  }
  if (preparation == host::aot::Preparation::load_only) {
    throw std::runtime_error(
        "Missing or corrupt semantic AOT image. Run blofeld-headless --prepare.");
  }
  auto semantic = Core::lower(image, options);
  entry.store(semantic.artifact());
  return semantic;
}
}
