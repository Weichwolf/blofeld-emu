# SPDX-License-Identifier: GPL-3.0-only
"""Identify the private original keyboard update without assigning device effects."""

import copy
import hashlib
import json

from original_dsp_voice import ROOT
from extract_runtime import hc12_object
from inspect_firmware import decode_block, load_input, read_smf, split_blocks

LAYOUTS = ((3, 0xef00, 42, 42, 6), (0, 0x2e00, 63, 63, 9),
           (3, 0xef00, 1542, 1542, 221), (0, 0x2e00, 57, 58, 9))


def digest(data):
    return hashlib.sha256(data).hexdigest()


def split_keyboard_messages(messages, reference):
    if len(messages) != reference['message_count']:
        raise ValueError('Changed original keyboard message count')
    positions = reference['unclassified_short_message_positions']
    for position in positions:
        message = messages[position]
        if (len(message) != reference['unclassified_short_message_bytes']
                or digest(message) != reference['unclassified_short_message_sha256']):
            raise ValueError('Changed unclassified original short message')
    # These identified messages are explicitly outside native transfer qualification.
    # Their original positions/identity remain in the receipt, without a fake effect.
    transfers = [message for index, message in enumerate(messages) if index not in positions]
    return split_blocks(transfers)


def verify_objects(blocks, reference):
    for expected in reference['hc12_objects']:
        data = decode_block(blocks[expected['block']])[1]
        helper, end = hc12_object(data)
        tail = data[end:]
        if (len(helper), digest(helper), end, len(tail), digest(tail)) != (
                expected['helper_bytes'], expected['helper_sha256'],
                expected['checked_container_bytes'], expected['opaque_tail_bytes'],
                expected['opaque_tail_sha256']):
            raise ValueError('Changed keyboard HC12 object or opaque trailing data')


def message_controls(messages, reference):
    bad_identity = copy.deepcopy(messages)
    bad_identity[17] = bytes([0, 0xf7])
    bad_order = copy.deepcopy(messages)
    bad_order[0], bad_order[17] = bad_order[17], bad_order[0]
    for changed in (bad_identity, bad_order, messages[:-1]):
        try:
            split_keyboard_messages(changed, reference)
        except ValueError:
            continue
        raise ValueError('Keyboard message classifier accepted a fixed invalid envelope')
    return 3


def verified_blocks(path):
    reference = json.loads((ROOT / 'doc/keyboard-update-reference.json').read_text())
    manifest = json.loads((ROOT / 'doc/downloads.json').read_text())['files']
    archive = next(row for row in manifest if row['file'] == reference['archive'])
    if path.stat().st_size != archive['bytes'] or digest(path.read_bytes()) != archive['sha256']:
        raise ValueError('Changed original keyboard archive')
    data, member = load_input(path)
    if (member, len(data), digest(data)) != (
            reference['member'], reference['midi_bytes'], reference['midi_sha256']):
        raise ValueError('Changed original keyboard MIDI member')
    messages, smf = read_smf(data)
    if smf != reference['smf']:
        raise ValueError('Changed original keyboard SMF events')
    blocks = split_keyboard_messages(messages, reference)
    for block, expected, layout in zip(blocks, reference['blocks'], LAYOUTS, strict=True):
        _, decoded, info = decode_block(block)
        actual = (info['mode'], info['address_words'], len(decoded) // 3,
                  expected['stored_words'], len(block['packets']))
        if actual != layout or digest(decoded) != expected['declared_sha256']:
            raise ValueError('Changed original keyboard transfer identity')
        padding = bytes.fromhex(info['transport_padding_hex'])
        if (len(padding), digest(padding)) != (
                expected['transport_padding_bytes'], expected['transport_padding_sha256']):
            raise ValueError('Changed original keyboard transport padding')
    verify_objects(blocks, reference)
    return blocks, reference, message_controls(messages, reference)
