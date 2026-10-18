// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "input.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <zconf.h>
#include <zlib.h>

namespace blofeld::input {
namespace {
constexpr std::uint32_t directory_signature = 0x06054b50;
constexpr std::uint32_t central_signature = 0x02014b50;
constexpr std::uint32_t local_signature = 0x04034b50;
constexpr std::size_t end_header_size = 22;
constexpr std::size_t maximum_comment_size = 0xffff;
constexpr unsigned zip64_entries = 0xffff;
constexpr unsigned supported_flags = 0x0808;
constexpr unsigned descriptor_flag = 8;

Bytes inflate_mid(const std::span<const std::uint8_t> compressed, const std::uint32_t size) {
  Bytes result(size);
  z_stream stream{};
  // zlib's non-const ABI does not modify next_in.
  stream.next_in = const_cast<Bytef *>(compressed.data());
  stream.avail_in = static_cast<uInt>(compressed.size());
  stream.next_out = result.data();
  stream.avail_out = size;
  if (inflateInit2(&stream, -MAX_WBITS) != Z_OK) { throw Fault("Cannot initialize ZIP inflater"); }
  const int status = inflate(&stream, Z_FINISH);
  inflateEnd(&stream);
  if (status != Z_STREAM_END || stream.total_in != compressed.size() || stream.total_out != size) {
    throw Fault("Invalid ZIP deflate stream");
  }
  return result;
}

std::size_t end_directory(const Bytes &input) {
  if (input.size() < end_header_size) { throw Fault("Truncated ZIP directory"); }
  std::size_t offset = input.size() - end_header_size;
  const auto maximum_tail = maximum_comment_size + end_header_size;
  const std::size_t minimum = input.size() > maximum_tail ? input.size() - maximum_tail : 0;
  while (true) {
    Reader candidate(std::span(input).subspan(offset));
    if (candidate.little(4) == directory_signature) {
      candidate.take(16);
      if (offset + end_header_size + candidate.little(2) == input.size()) { return offset; }
    }
    if (offset == minimum) { throw Fault("Missing ZIP end directory"); }
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
  if (directory.little(4) != central_signature) { throw Fault("Invalid ZIP central entry"); }
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
  if (directory.little(2) != 0) { throw Fault("Unsupported split ZIP member"); }
  directory.take(6);
  member.local = directory.little(4);
  const auto name = directory.take(name_size);
  member.name.assign(name.begin(), name.end());
  directory.take(extra_size);
  directory.take(comment_size);
  return member;
}

Bytes unpack_member(const Member &member, const std::span<const std::uint8_t> storage) {
  if ((member.flags & ~supported_flags) != 0 || member.size == 0 || member.size > input_limit ||
      member.local > storage.size()) {
    throw Fault("Unsupported ZIP flags, member size or local offset");
  }
  Reader local(storage.subspan(member.local));
  if (local.little(4) != local_signature) { throw Fault("Invalid ZIP local entry"); }
  local.take(2);
  if (local.little(2) != member.flags || local.little(2) != member.method) {
    throw Fault("Inconsistent ZIP member flags or compression method");
  }
  local.take(4);
  const auto checksum = local.little(4);
  const auto packed_size = local.little(4);
  const auto size = local.little(4);
  if ((member.flags & descriptor_flag) == 0 &&
      (checksum != member.checksum || packed_size != member.packed_size || size != member.size)) {
    throw Fault("Inconsistent ZIP member checksum or size");
  }
  const auto name_size = local.little(2);
  const auto extra_size = local.little(2);
  const auto name = local.take(name_size);
  if (std::string(name.begin(), name.end()) != member.name) {
    throw Fault("Inconsistent ZIP member filename");
  }
  local.take(extra_size);
  const auto packed = local.take(member.packed_size);
  Bytes result;
  if (member.method == 8) {
    result = inflate_mid(packed, member.size);
  } else if (member.method == 0 && member.size == member.packed_size) {
    result.assign(packed.begin(), packed.end());
  } else {
    throw Fault("Unsupported ZIP compression");
  }
  if (crc32(0, result.data(), static_cast<uInt>(result.size())) != member.checksum) {
    throw Fault("ZIP CRC mismatch");
  }
  return result;
}
}

Bytes archive_midi(const Bytes &input) {
  const auto end = end_directory(input);
  Reader footer(std::span(input).subspan(end + 4));
  if (footer.little(4) != 0) { throw Fault("Unsupported split ZIP directory"); }
  const auto disk_count = footer.little(2);
  const auto count = footer.little(2);
  const auto size = footer.little(4);
  const auto offset = footer.little(4);
  if (count == zip64_entries || count != disk_count || offset > end || size != end - offset) {
    throw Fault("Unsupported ZIP64 or inconsistent central directory");
  }
  Reader directory(std::span(input).subspan(offset, size));
  Bytes result;
  for (std::uint32_t index = 0; index < count; ++index) {
    const auto member = central_entry(directory);
    if (member.name.ends_with(".mid") || member.name.ends_with(".MID")) {
      if (!result.empty()) { throw Fault("Expected one MIDI member in firmware ZIP"); }
      result = unpack_member(member, std::span(input).first(offset));
    }
  }
  if (result.empty() || !directory.empty()) {
    throw Fault("Missing MIDI or trailing ZIP directory");
  }
  return result;
}
}
