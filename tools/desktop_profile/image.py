#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Build local Blofeld Desktop images from verified manufacturer inputs."""

from dataclasses import dataclass
from functools import reduce
import hashlib
from io import BytesIO
from operator import xor
import struct
from zipfile import ZipFile
import zlib

PAGE_BYTES, PAGE_COUNT, BLOCK_PAGES = 528, 131072, 32
SAMPLE_BLOCK_BYTES = PAGE_BYTES * BLOCK_PAGES
PREFIX = b"\x3e\x13\x7f"
INPUTS = {
    "firmware": ("BlofeldV125.mid", 169554,
                 "447f7ec928b5b4144fad2193351f0604011f2ab8ed9b9dbe3d1bb7c594e813c0",
                 "095c760a576dcb04fe3e379bed1b95a0360002295ab42b29e3335a39514f0c72"),
    "sounds": ("Blo Factory 2012.mid", 404506,
               "1345997b45cfbc53f4944fd2fc694602c4fb50a0e8a705ed0b39c5415dc05df3",
               "f57365289d9862c9ecf1aab25965d82cefc0e04e66cb8a03cb406df9a15195f7"),
    "samples": ("Blofeld-SamplesV1_00.mid", 29801086,
                "e5e34f8f2830dc394360083d7e9d896cba7fb89c6748a7ede2d73af564651b56",
                "a6f2adb407253392cfe9d6b211c3cfce29d652421373745c72e06634bf3ed1a0"),
}
# Explicit shared emulator settings, not a recovered factory-global dump.
GLOBALS = {0: 2, 35: 1, 38: 13, 39: 45, 40: 64, 41: 64, 48: 2, 50: 3,
           51: 2, 52: 4, 53: 11, 54: 12, 55: 127, 57: 1, 64: 127}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def input_midi(raw, kind):
    member, size, midi_hash, zip_hash = INPUTS[kind]
    if raw.startswith(b"PK\x03\x04"):
        require(digest(raw) == zip_hash, f"Unexpected {kind} archive SHA256")
        with ZipFile(BytesIO(raw)) as archive:
            matches = [entry for entry in archive.infolist() if entry.filename == member]
            require(len(matches) == 1 and matches[0].file_size == size,
                    f"Missing or ambiguous {kind} MIDI member")
            midi = archive.read(matches[0])
    else:
        midi = raw
    require(len(midi) == size and digest(midi) == midi_hash,
            f"Unexpected {kind} MIDI size/SHA256")
    return midi


class Reader:
    def __init__(self, data):
        self.data, self.position = data, 0

    def take(self, count):
        require(0 <= count <= len(self.data) - self.position, "Truncated MIDI input")
        begin = self.position
        self.position += count
        return self.data[begin:self.position]

    def byte(self):
        return self.take(1)[0]

    def vlq(self):
        result = 0
        for _ in range(4):
            value = self.byte()
            result = (result << 7) | (value & 127)
            if value < 128:
                return result
        raise ValueError("MIDI VLQ exceeds four bytes")


def sysex_messages(data):
    source = Reader(data)
    require(source.take(8) == b"MThd\0\0\0\x06", "Unsupported SMF header")
    fmt, tracks, division = struct.unpack(">HHH", source.take(6))
    require(fmt in (0, 1) and tracks > 0 and division > 0 and
            (fmt != 0 or tracks == 1), "Unsupported SMF layout")
    for _ in range(tracks):
        require(source.take(4) == b"MTrk", "Missing MIDI track")
        track = Reader(source.take(int.from_bytes(source.take(4), "big")))
        pending, running, ended = None, None, False
        while track.position < len(track.data):
            track.vlq()
            status = track.byte()
            if status < 128:
                require(running is not None, "Missing MIDI running status")
                track.position -= 1
                status = running
            if status == 255:
                kind = track.byte()
                payload = track.take(track.vlq())
                if kind == 47:
                    require(not payload and track.position == len(track.data), "Bad MIDI end")
                    ended = True
            elif status in (240, 247):
                running = None
                if status == 240:
                    require(pending is None, "Nested SysEx")
                    pending = bytearray()
                require(pending is not None, "Standalone SysEx escape")
                pending.extend(track.take(track.vlq()))
                if pending and pending[-1] == 247:
                    require(all(byte < 128 for byte in pending[:-1]), "Non-seven-bit SysEx")
                    yield bytes(pending)
                    pending = None
            else:
                require(128 <= status < 240, "Unsupported MIDI event")
                running = status
                size = 1 if status & 240 in (192, 208) else 2
                require(all(byte < 128 for byte in track.take(size)), "Invalid MIDI data")
        require(pending is None and ended, "Incomplete MIDI track")
    require(source.position == len(data), "Trailing MIDI bytes")


def septets(data):
    return sum(value << (7 * index) for index, value in enumerate(data))


def unpack(packed):
    require(len(packed) % 8 == 0, "Incomplete seven-bit packing group")
    raw = bytearray()
    for at in range(0, len(packed), 8):
        for index in range(7):
            raw.append(packed[at + index + 1] | (((packed[at] >> (6 - index)) & 1) << 7))
    # The real receiver stores complete little-endian triples per packet.
    return b"".join(raw[at:at + 3][::-1] for at in range(0, len(raw) - 2, 3))


@dataclass
class Block:
    mode: int
    address: int
    declared: int
    data: bytearray
    packets: int = 0


def download_blocks(midi):
    block = None
    for message in sysex_messages(midi):
        require(message[:3] == PREFIX and len(message) >= 5, "Unexpected download prefix")
        command = message[3]
        if command == 115:
            if block is not None:
                require(block.packets == 0 or len(block.data) >= block.declared,
                        "Incomplete download block")
                yield block
            require(len(message) == 14 and message[4] <= 3, "Invalid download target")
            block = Block(message[4], septets(message[9:13]), septets(message[5:9]), bytearray())
            require(block.declared <= 1024 * 1024 and block.declared % 3 == 0,
                    "Invalid download length")
        else:
            require(block is not None and command in (116, 117) and len(message) >= 8,
                    "Unsupported download command")
            count = message[5] * (8 if command == 117 else 1)
            require(len(message) == count + 8 and message[4] == block.packets % 128 and
                    reduce(xor, message[:-1], 0) == 0, "Bad download count/sequence/XOR")
            block.data.extend(unpack(message[6:-2]))
            block.packets += 1
            require(len(block.data) <= block.declared + 12, "Download exceeds its bounded tail")
    require(block is not None and len(block.data) >= block.declared, "Missing final download")
    yield block


def words(data):
    require(len(data) % 3 == 0, "Incomplete 24-bit words")
    return (int.from_bytes(data[at:at + 3], "big") for at in range(0, len(data), 3))


def firmware_parts(blocks):
    expected = [(3, 0xef00, 768), (0, 0x2e00, 189), (3, 0x10000, 8649),
                (0, 0x2e00, 1164), (3, 0xd000, 89643), (0, 0x2e00, 1200)]
    require([(b.mode, b.address, b.declared) for b in blocks] == expected,
            "Unsupported firmware block layout")
    main = blocks[4].data
    helper_bytes = int.from_bytes(main[:2], "big")
    mcu_at = ((helper_bytes + 4) // 3) * 3 + 3
    require(sum(words(main[:mcu_at])) & 0xffffff == 0, "Invalid flash helper checksum")
    mcu = main[mcu_at:mcu_at + 32768]
    runtime = main[mcu_at + 32772:]
    overlay = blocks[2].data[3:3 + 16 * PAGE_BYTES]
    require(digest(mcu) == "a530c788593bc9ab2427f9d8104aefe2ede8eb91a1be34d779731491f96ed4f8" and
            digest(runtime) == "cadd0873304467a701174afc196997e1d199dfff216201c216c3bc729f9e213c" and
            digest(overlay) == "6e3bbd97e1d1a0578c579eabbcf0002fed7847b9d769c228a4d7f628a77ae9a8",
            "Decoded MCU/runtime/overlay identity mismatch")
    return mcu, runtime, overlay


def sound_pages(midi):
    sounds = {}
    for message in sysex_messages(midi):
        require(len(message) == 391 and message[:2] == PREFIX[:2] and message[3] == 16 and
                sum(message[6:389]) & 127 == message[389], "Invalid factory sound dump")
        slot = message[4] * 128 + message[5]
        require(slot < 1024 and slot not in sounds, "Duplicate or invalid factory sound slot")
        sounds[slot] = message[6:389]
    require(len(sounds) == 1024, "Incomplete factory sound bank")
    return {0x400 + slot: b"\x80\0" + data + b"\xff" * 143 for slot, data in sounds.items()}


def sample_pages(midi):
    pages, groups, payload = {}, 0, hashlib.sha256()
    staging = (0x10000, 0x11610, 0x12c20, 0x14230)
    for block in download_blocks(midi):
        # The official stream repeats target headers without intervening data.
        if block.packets == 0 or block.declared != SAMPLE_BLOCK_BYTES + 24:
            continue
        require(block.mode == 3 and block.address == staging[groups % 4],
                "Invalid sample staging rotation")
        data = block.data
        row = int.from_bytes(data[SAMPLE_BLOCK_BYTES:SAMPLE_BLOCK_BYTES + 3], "big")
        require(data[SAMPLE_BLOCK_BYTES + 3:SAMPLE_BLOCK_BYTES + 6] == b"\xdd\xef\x0e" and
                row % BLOCK_PAGES == 0 and row + BLOCK_PAGES <= PAGE_COUNT,
                "Invalid sample row/marker")
        for offset in range(BLOCK_PAGES):
            require(row + offset not in pages, "Overlapping sample pages")
            pages[row + offset] = bytes(data[offset * PAGE_BYTES:(offset + 1) * PAGE_BYTES])
        groups += 1
    for row in sorted(pages):
        payload.update(pages[row])
    require(groups == 1488 and len(pages) == 47616 and payload.hexdigest() ==
            "dd4df3342269b582a27aa09835bbcae0cdb9d762f1226c2d718bbd2654755a22",
            "Standard sample-library page identity mismatch")
    return pages


def nvm(mcu, serial):
    raw = serial.encode("ascii")
    require(1 <= len(raw) <= 31 and all(32 <= byte <= 126 for byte in raw),
            "Serial must contain 1..31 printable ASCII characters")
    require(mcu[0x1df:0x200] == b"\xff" * 33, "Resident identity fields are not erased inputs")
    flash = bytearray(mcu)
    flash[0x1df] &= ~2  # Installed sample state in persistent flash.
    flash[0x1e0:0x1e0 + len(raw) + 1] = raw + b"\0"
    payload = flash + b"\xff" * 4096 + bytes(2048)
    data = b"BLFNVM\r\n" + struct.pack("<II", 1, len(payload)) + payload
    return data + struct.pack("<I", zlib.crc32(data))


def write_nand(path, selected):
    crc = 0
    with path.open("xb") as stream:
        def put(data):
            nonlocal crc
            stream.write(data)
            crc = zlib.crc32(data, crc)
        put(b"BLFNAND\n" + struct.pack("<6I", 1, 1, PAGE_COUNT,
                                        PAGE_COUNT // BLOCK_PAGES, PAGE_BYTES, 0))
        for block in range(PAGE_COUNT // BLOCK_PAGES):
            put(struct.pack("<HBB", block, 7, 1))
        erased = b"\xff" * (PAGE_BYTES * 2)
        for page in range(PAGE_COUNT):
            if page not in selected:
                put(struct.pack("<IBBBB", page, 0, 0, 0, 0) + erased)
                continue
            data, main, spare = selected[page]
            require(len(data) == PAGE_BYTES, "Invalid selected NAND page length")
            encoded = bytearray(PAGE_BYTES * 2)
            encoded[::2], encoded[1::2] = data, b"\xff" * PAGE_BYTES
            put(struct.pack("<IBBBB", page, main, spare, 0, 0) + encoded)
        stream.write(struct.pack("<I", crc))


def assemble_pages(runtime, overlay, sounds, samples):
    selected = {}
    for base, data, history in ((0, runtime, (255, 255)), (0x380, overlay, (1, 1))):
        for at in range(0, len(data), PAGE_BYTES):
            selected[base + at // PAGE_BYTES] = (bytes(data[at:at + PAGE_BYTES]).ljust(
                PAGE_BYTES, b"\xff"), *history)
    # Startup reads row $300 into X:$2800 and tests X:$280A for the L tag.
    # Only this installed-state field is selected; seed and other tags stay erased.
    require(0x300 not in selected, "Overlapping startup-state page")
    installed = bytearray(b"\xff" * PAGE_BYTES)
    installed[10] = ord("L")
    selected[0x300] = (bytes(installed), 1, 0)
    for row, data in sounds.items():
        require(row not in selected, "Overlapping firmware/sound page")
        selected[row] = (data, 255, 255)
    for row, data in samples.items():
        require(row not in selected, "Overlapping sample/firmware/sound page")
        selected[row] = (data, 1, 1)
    globals_ = bytearray(72)
    for field, value in GLOBALS.items():
        globals_[field] = value
    selected[0x11e0] = (bytes(globals_).ljust(PAGE_BYTES, b"\xff"), 1, 0)
    return selected
