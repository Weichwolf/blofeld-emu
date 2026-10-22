// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/lcd_image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>
#include <zconf.h>
#include <zlib.h>

namespace blofeld {
namespace {
using Bytes = std::vector<std::uint8_t>;
constexpr unsigned byte_bits = 8;
constexpr unsigned word_bits = 32;
constexpr unsigned byte_mask = 255;
constexpr unsigned signature_size = 8;

void big(Bytes &bytes, const std::uint32_t value) {
  for (unsigned shift = word_bits; shift != 0; shift -= byte_bits) {
    bytes.push_back(static_cast<std::uint8_t>((value >> (shift - byte_bits)) & byte_mask));
  }
}

void chunk(Bytes &bytes, const std::string_view type, const std::span<const std::uint8_t> data) {
  big(bytes, static_cast<std::uint32_t>(data.size()));
  const auto offset = bytes.size();
  for (const auto value : type) { bytes.push_back(static_cast<std::uint8_t>(value)); }
  bytes.insert(bytes.end(), data.begin(), data.end());
  const auto crc = crc32_z(0, bytes.data() + offset, bytes.size() - offset);
  big(bytes, static_cast<std::uint32_t>(crc));
}

Bytes deflate_rows(const LcdImage &image) {
  const auto pbm = encode_lcd_pbm(image);
  const auto stride = static_cast<std::size_t>((image.width() + 7) / 8);
  const auto offset = pbm.size() - (stride * LcdImage::height);
  Bytes rows;
  for (unsigned y = 0; y < LcdImage::height; ++y) {
    rows.push_back(0); // PNG filter None, distinct from the PBM header.
    for (unsigned x = 0; x < stride; ++x) {
      // PNG grayscale is white=1, opposite the PBM black=1 convention.
      rows.push_back(static_cast<std::uint8_t>(pbm.at(offset + (y * stride) + x) ^ byte_mask));
    }
  }
  auto length = compressBound(static_cast<uLong>(rows.size()));
  Bytes compressed(length);
  if (compress2(compressed.data(), &length, rows.data(), rows.size(), Z_DEFAULT_COMPRESSION) !=
      Z_OK) {
    throw Fault("Cannot compress bounded LCD screenshot");
  }
  compressed.resize(length);
  return compressed;
}
}

Bytes encode_lcd_png(const LcdImage &image) {
  // W3C PNG Third Edition: signature, IHDR, zlib IDAT, IEND and type+data CRCs.
  constexpr std::array<std::uint8_t, signature_size> signature{137, 80, 78, 71, 13, 10, 26, 10};
  Bytes result(signature.begin(), signature.end());
  Bytes header;
  big(header, image.width());
  big(header, LcdImage::height);
  constexpr std::array<std::uint8_t, 5> format{1, 0, 0, 0, 0}; // 1-bit grayscale, no interlace.
  header.insert(header.end(), format.begin(), format.end());
  chunk(result, "IHDR", header);
  chunk(result, "IDAT", deflate_rows(image));
  chunk(result, "IEND", {});
  return result;
}
}
