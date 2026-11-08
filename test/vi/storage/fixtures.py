# SPDX-License-Identifier: GPL-3.0-only
"""Author synthetic update containers; no manufacturer programs or execution."""

from pathlib import Path
import struct
import sys
import zipfile
import zlib


def words(values):
    return b"".join((value & 0xffffff).to_bytes(3, "big") for value in values)


def checked(data):
    assert len(data) % 3 == 0
    checksum = -sum(int.from_bytes(data[i:i + 3], "big") for i in range(0, len(data), 3))
    return data + words([checksum])


def vlq(value):
    result = [value & 0x7f]
    while value >> 7:
        value >>= 7
        result.insert(0, (value & 0x7f) | 0x80)
    return bytes(result)


def messages(mode, address, data):
    def septets(value):
        return bytes((value >> shift) & 0x7f for shift in (0, 7, 14, 21))

    assert len(data) % 3 == 0
    yield b"\x3e\x13\x7f\x73" + bytes([mode]) + septets(len(data)) + septets(address) + b"\xf7"
    raw = b"".join(data[i:i + 3][::-1] for i in range(0, len(data), 3))
    raw += b"\x5a" * (-len(raw) % 7)
    packed = bytearray()
    for offset in range(0, len(raw), 7):
        group = raw[offset:offset + 7]
        packed.append(sum((byte >> 7) << (6 - i) for i, byte in enumerate(group)))
        packed.extend(byte & 0x7f for byte in group)
    for sequence, offset in enumerate(range(0, len(packed), 112)):
        packet = b"\x3e\x13\x7f\x74" + bytes([sequence % 128, len(packed[offset:offset + 112])])
        packet += packed[offset:offset + 112]
        checksum = 0
        for byte in packet:
            checksum ^= byte
        yield packet + bytes([checksum, 0xf7])


def midi(messages_to_write):
    track = bytearray()
    for index, message in enumerate(messages_to_write):
        if index == 0:
            # A legal split SysEx header, independent of subsequent data packets.
            for status, part in ((0xf0, message[:5]), (0xf7, message[5:])):
                track += bytes([0, status]) + vlq(len(part)) + part
        else:
            track += b"\0\xf0" + vlq(len(message)) + message
    track += b"\0\xff\x2f\0"
    return b"MThd" + struct.pack(">IHHH", 6, 0, 1, 480) + b"MTrk" + struct.pack(">I", len(track)) + track


def update(invalid_app=False):
    flash = bytearray(32768)
    flash[0] = 0xa5
    flash[512:514] = b"\x13\x34" if invalid_app else b"\x12\x34"
    flash[0x7f10:0x7f12] = b"\x12\x34"
    helper = bytes(6)
    secondary = words([0x380]) + checked(bytes(16 * 528)) + helper
    records = words([0, 2, 0x100, 0x112233, 0x445566, 0x100, 0x100, 0x100])
    tail = words([0x100] * 4 + [0x123123]) + helper
    main = helper + checked(flash + b"\0") + checked(records) + tail
    result = []
    for mode, address, payload in ((3, 0xef00, helper), (0, 0x2e00, helper),
                                   (3, 0x10000, secondary), (0, 0x2e00, helper),
                                   (3, 0xd000, main), (0, 0x2e00, helper)):
        result.extend(messages(mode, address, payload))
    return result


root = Path(sys.argv[1])
root.mkdir(parents=True, exist_ok=True)
valid = midi(update())
(root / "update.mid").write_bytes(valid)
with zipfile.ZipFile(root / "update.zip", "w", compression=zipfile.ZIP_DEFLATED) as archive:
    archive.writestr("UPDATE.MID", valid)
packets = update()
damaged = bytearray(packets[1])
damaged[6] ^= 1
packets[1] = bytes(damaged)
(root / "bad-packet.mid").write_bytes(midi(packets))
(root / "bad-app.mid").write_bytes(midi(update(invalid_app=True)))
flash = bytearray(32768)
flash[0], flash[100], flash[101], flash[-1] = 0xa5, 0xff, 0xff, 0x7a
known = bytearray(4096)
for index in (0, 100, 101, 32767):
    known[index // 8] |= 1 << (index % 8)
erased = bytearray(2048)
erased[50 // 8] = 1 << (50 % 8)
payload = flash + known + erased
image = b"BLFNVM\r\n" + struct.pack("<II", 1, len(payload)) + payload
(root / "flash.bfnvm").write_bytes(image + struct.pack("<I", zlib.crc32(image)))
media = bytearray(b"BLFNAND\n" + struct.pack("<IIIIII", 1, 1, 4, 1, 528, 0))
media += struct.pack("<HBB", 0, 0, 0)
for page in (0, 31, 32, 131071):
    media += struct.pack("<IBBH", page, 255, 255, 0)
    for column in range(528):
        media += bytes((0, 0)) if page == 32 and column == 17 else bytes(((page + column) & 255, 255))
(root / "media.bfnand").write_bytes(media + struct.pack("<I", zlib.crc32(media)))
# Writable metadata: valid, program-protected, erase-protected, invalid, unknown.
write_blocks = ((0, 7, 1), (1, 7, 3), (2, 7, 5), (3, 7, 0), (4, 0, 0), (4095, 7, 1))
writable = bytearray(b"BLFNAND\n" + struct.pack("<IIIIII", 1, 1, 4, len(write_blocks), 528, 0))
for record in write_blocks:
    writable += struct.pack("<HBB", *record)
writable += media[36:]
(root / "media-writable.bfnand").write_bytes(writable + struct.pack("<I", zlib.crc32(writable)))
# A page with known program history but one unobserved byte exercises knowledge propagation.
programmable = bytearray(writable)
page_offset = 32 + 4 * len(write_blocks)
programmable[page_offset + 4:page_offset + 6] = bytes((0, 0))
programmable[page_offset + 8:page_offset + 8 + 528 * 2] = bytes((255, 255)) * 528
programmable[page_offset + 8 + 17 * 2:page_offset + 10 + 17 * 2] = bytes((0, 0))
# An erase-protected but program-writable page checks the independent protection bits.
protected_page = struct.pack("<IBBH", 64, 0, 0, 0) + bytes((255, 255)) * 528
programmable[-1064:-1064] = protected_page
struct.pack_into("<I", programmable, 16, 5)
(root / "media-programmable.bfnand").write_bytes(programmable + struct.pack("<I", zlib.crc32(programmable)))
corrupt = bytearray(media)
corrupt[36 + 8] = 1
corrupt[36 + 9] = 0
(root / "bad-media.bfnand").write_bytes(corrupt + struct.pack("<I", zlib.crc32(corrupt)))
(root / "firmware.done").write_text("generated\n")
