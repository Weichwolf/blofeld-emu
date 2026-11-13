// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "aot/mcu.hpp"
#include "mcu/program.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace blofeld::test {
class McuImage {
public:
  McuImage() : image_(std::make_unique<vi::FlashImage>()) { image_->known.fill(1); }

  vi::FlashImage *operator->() const { return image_.get(); }

  vi::FlashImage &operator*() const { return *image_; }

  void load(const cpu12::Address address, const std::span<const cpu12::Byte> bytes) {
    constexpr cpu12::Address first = 0x8000;
    if (address < first || bytes.empty() ||
        bytes.size() > vi::FlashImage::byte_count - (address - first)) {
      throw std::invalid_argument("Invalid authored MCU fixture extent");
    }
    std::ranges::copy(bytes, image_->bytes.begin() + (address - first));
    for (std::size_t offset = 0; offset < bytes.size(); ++offset) {
      code_.set(address - first + offset);
    }
    program_.reset();
  }

  [[nodiscard]] std::shared_ptr<const vi::mcu::Program> program() const {
    if (program_) { return program_; }
    auto code = source();
    std::vector<cpu12::Address> entries;
    entries.reserve(code.size());
    for (const auto span : code) { entries.push_back(span.address); }
    const std::array variants{cpu12::CodeVariant{.id = 0, .code = code, .entries = entries}};
    program_ =
        std::make_shared<vi::mcu::Program>(app::aot::prepare_mcu_program({.variants = variants}));
    return program_;
  }

  [[nodiscard]] std::unique_ptr<vi::FlashImage> release() {
    static_cast<void>(source());
    return std::move(image_);
  }

private:
  [[nodiscard]] std::vector<cpu12::CodeSpan> source() const {
    std::vector<cpu12::CodeSpan> code;
    for (std::size_t first = 0; first < code_.size();) {
      if (!code_[first]) {
        ++first;
        continue;
      }
      auto end = first + 1;
      while (end < code_.size() && code_[end]) { ++end; }
      if (end + 2 > image_->bytes.size() || (end + 1 < code_.size() && code_[end + 1])) {
        throw std::invalid_argument("Authored MCU fixture requires terminal padding");
      }
      // Complete the fixture's unreachable fallthrough without changing its authored prefix.
      image_->bytes[end] = 0x20;
      constexpr cpu12::Byte self_branch = 0xfe;
      image_->bytes[end + 1] = self_branch;
      const auto pc = static_cast<cpu12::Address>(0x8000 + first);
      code.push_back(
          {.address = pc, .words = std::span(image_->bytes).subspan(first, end + 2 - first)});
      first = end;
    }
    return code;
  }

  std::unique_ptr<vi::FlashImage> image_;
  std::bitset<vi::FlashImage::byte_count> code_;
  mutable std::shared_ptr<const vi::mcu::Program> program_;
};
}
