// SPDX-License-Identifier: GPL-3.0-only
#include "archive/zip.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#include <zconf.h>
#include <zlib.h>

namespace blofeld::host {
namespace {
using Bytes = std::vector<std::uint8_t>;

class Reader {
public:
  explicit Reader(const std::span<const std::uint8_t> data) : data_(data) {}

  std::span<const std::uint8_t> take(const std::size_t size) {
    if (size > data_.size() - position_) { throw std::runtime_error("Truncated ZIP input"); }
    const auto result = data_.subspan(position_, size);
    position_ += size;
    return result;
  }

  std::uint32_t little(const unsigned size) {
    std::uint32_t value = 0;
    unsigned shift = 0;
    for (const auto byte : take(size)) {
      value |= static_cast<std::uint32_t>(byte) << shift;
      shift += 8;
    }
    return value;
  }

  [[nodiscard]] bool empty() const { return position_ == data_.size(); }

private:
  std::span<const std::uint8_t> data_;
  std::size_t position_ = 0;
};

bool matches(const std::string_view name, const std::string_view suffix) {
  if (suffix.empty() || suffix.size() > name.size()) { return false; }
  const auto tail = name.substr(name.size() - suffix.size());
  return std::ranges::equal(tail, suffix, [](const unsigned char left, const unsigned char right) {
    return std::tolower(left) == std::tolower(right);
  });
}

constexpr std::uint32_t directory_signature = 0x06054b50;
constexpr std::uint32_t central_signature = 0x02014b50;
constexpr std::uint32_t local_signature = 0x04034b50;
constexpr std::size_t end_header_size = 22;
constexpr std::size_t maximum_comment_size = 0xffff;
constexpr unsigned zip64_entries = 0xffff;
constexpr unsigned supported_flags = 0x0808;
constexpr unsigned descriptor_flag = 8;

Bytes inflate_member(const std::span<const std::uint8_t> compressed, const std::uint32_t size) {
  Bytes result(std::max(size, std::uint32_t{1}));
  z_stream stream{};
  // zlib's non-const ABI does not modify next_in.
  stream.next_in = const_cast<Bytef *>(compressed.data());
  stream.avail_in = static_cast<uInt>(compressed.size());
  stream.next_out = result.data();
  stream.avail_out = static_cast<uInt>(result.size());
  if (inflateInit2(&stream, -MAX_WBITS) != Z_OK) {
    throw std::runtime_error("Cannot initialize ZIP inflater");
  }
  const int status = inflate(&stream, Z_FINISH);
  inflateEnd(&stream);
  if (status != Z_STREAM_END || stream.total_in != compressed.size() || stream.total_out != size) {
    throw std::runtime_error("Invalid ZIP deflate stream");
  }
  result.resize(size);
  return result;
}

std::size_t end_directory(const std::span<const std::uint8_t> input) {
  if (input.size() < end_header_size) { throw std::runtime_error("Truncated ZIP directory"); }
  std::size_t offset = input.size() - end_header_size;
  const auto maximum_tail = maximum_comment_size + end_header_size;
  const std::size_t minimum = input.size() > maximum_tail ? input.size() - maximum_tail : 0;
  while (true) {
    Reader candidate(std::span(input).subspan(offset));
    if (candidate.little(4) == directory_signature) {
      candidate.take(16);
      if (offset + end_header_size + candidate.little(2) == input.size()) { return offset; }
    }
    if (offset == minimum) { throw std::runtime_error("Missing ZIP end directory"); }
    --offset;
  }
}

struct Member {
  std::uint32_t flags = 0;
  std::uint32_t method = 0;
  std::uint32_t checksum = 0;
  std::uint32_t packed_size = 0;
  std::uint32_t size = 0;
  std::uint32_t local = 0;
  std::string name;
};

Member central_entry(Reader &directory) {
  if (directory.little(4) != central_signature) {
    throw std::runtime_error("Invalid ZIP central entry");
  }
  directory.take(4);
  Member member;
  member.flags = directory.little(2);
  member.method = directory.little(2);
  directory.take(4);
  member.checksum = directory.little(4);
  member.packed_size = directory.little(4);
  member.size = directory.little(4);
  const auto name_size = directory.little(2);
  const auto extra_size = directory.little(2);
  const auto comment_size = directory.little(2);
  if (directory.little(2) != 0) { throw std::runtime_error("Unsupported split ZIP member"); }
  directory.take(6);
  member.local = directory.little(4);
  const auto name = directory.take(name_size);
  member.name.assign(reinterpret_cast<const char *>(name.data()), name.size());
  directory.take(extra_size);
  directory.take(comment_size);
  return member;
}

Bytes unpack_member(const Member &member,
                    const std::span<const std::uint8_t> storage,
                    const std::size_t maximum_bytes) {
  if ((member.flags & ~supported_flags) != 0 || member.size > maximum_bytes ||
      member.local > storage.size()) {
    throw std::runtime_error("Unsupported ZIP flags, member size or local offset");
  }
  Reader local(storage.subspan(member.local));
  if (local.little(4) != local_signature) { throw std::runtime_error("Invalid ZIP local entry"); }
  local.take(2);
  if (local.little(2) != member.flags || local.little(2) != member.method) {
    throw std::runtime_error("Inconsistent ZIP member flags or compression method");
  }
  local.take(4);
  const auto checksum = local.little(4);
  const auto packed_size = local.little(4);
  const auto size = local.little(4);
  if ((member.flags & descriptor_flag) == 0 &&
      (checksum != member.checksum || packed_size != member.packed_size || size != member.size)) {
    throw std::runtime_error("Inconsistent ZIP member checksum or size");
  }
  const auto name_size = local.little(2);
  const auto extra_size = local.little(2);
  const auto name = local.take(name_size);
  if (std::string_view(reinterpret_cast<const char *>(name.data()), name.size()) != member.name) {
    throw std::runtime_error("Inconsistent ZIP member filename");
  }
  local.take(extra_size);
  const auto packed = local.take(member.packed_size);
  Bytes result;
  if (member.method == 8) {
    result = inflate_member(packed, member.size);
  } else if (member.method == 0 && member.size == member.packed_size) {
    result.assign(packed.begin(), packed.end());
  } else {
    throw std::runtime_error("Unsupported ZIP compression");
  }
  if (crc32(0, result.data(), static_cast<uInt>(result.size())) != member.checksum) {
    throw std::runtime_error("ZIP CRC mismatch");
  }
  return result;
}
}

Bytes zip_member(const std::span<const std::uint8_t> input,
                 const std::string_view suffix,
                 const std::size_t maximum_bytes) {
  const auto end = end_directory(input);
  Reader footer(std::span(input).subspan(end + 4));
  if (footer.little(4) != 0) { throw std::runtime_error("Unsupported split ZIP directory"); }
  const auto disk_count = footer.little(2);
  const auto count = footer.little(2);
  const auto size = footer.little(4);
  const auto offset = footer.little(4);
  if (count == zip64_entries || count != disk_count || offset > end || size != end - offset) {
    throw std::runtime_error("Unsupported ZIP64 or inconsistent central directory");
  }
  Reader directory(std::span(input).subspan(offset, size));
  Bytes result;
  bool found = false;
  for (std::uint32_t index = 0; index < count; ++index) {
    const auto member = central_entry(directory);
    if (matches(member.name, suffix)) {
      if (found) { throw std::runtime_error("Ambiguous ZIP member suffix"); }
      found = true;
      result = unpack_member(member, input.first(offset), maximum_bytes);
    }
  }
  if (!found || !directory.empty()) {
    throw std::runtime_error("Missing member or trailing ZIP directory");
  }
  return result;
}
}
