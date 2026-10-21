# SPDX-License-Identifier: GPL-3.0-only
"""Compare complete original voice returns with independent integer equations."""

import hashlib
import re

from dsp_drive_validation import validate_drive_calls

from dsp_voice_model import (FULL_MASK, H, RESET_LEVEL, W, WORD_MASK, Voice,
                             initial_bus, mixer_model, output_model, signed,
                             waveform_model)


def validate_coverage(result, reference):
    match = re.search(r"Entry-streams visited PCs:([^\n]+)", result)
    if match is None:
        raise ValueError("Missing original voice instruction coverage")
    visited = {int(pc, 16) for pc in match[1].split()}
    encoded = "".join(f"{pc:06x}\n" for pc in sorted(visited)).encode("ascii")
    expected = reference["executed"]
    if (len(visited) != expected["instruction_starts"]
            or hashlib.sha256(encoded).hexdigest() != expected["sha256"]):
        raise ValueError("Changed original voice instruction coverage")


def new_totals():
    return dict(blocks=0, dependent_blocks=0, wraps=0, syncs=0, falling_steps=0,
                zero_steps=0, seed_writes=0, positive_limits=0, negative_limits=0,
                changed_prefetch_blocks=0)


def validate_stream(stream, xdata, ydata, tables, identity, totals, block_count=8, drives=None):
    kind, mode, color, profile, routing = identity
    high, twos, varied, serial = kind == 'sync', mode == '1', profile == '1', routing == '1'
    voice = Voice(xdata, ydata, tables[twos], high, twos, int(color), varied, serial, drives)

    def check(label, expected, text):
        rows = re.findall('^' + label + r' ([^\n]+)$', text, re.M)
        if len(rows) != 1:
            raise ValueError(f'{identity}: missing/duplicate {label}')
        actual = [int(value, 16) for value in rows[0].split()]
        if actual != expected:
            raise ValueError(f'{identity}, block {voice.block}, {label}: {actual} != {expected}')

    prefix = stream.split('BLOCK ')[0]
    check('CACHE', voice.gains, prefix)
    check('PARAM_CACHE', [p & WORD_MASK for p in voice.parameters], prefix)
    check('SETUP', [1, int(color) * 65536, 0, 1392, 1, 1392, 1], prefix)
    blocks = re.findall(r'BLOCK (\d+)\n([\s\S]*?)(?=BLOCK |$)', stream)
    if [int(n) for n, _ in blocks] != list(range(block_count)):
        raise ValueError('Incomplete consecutive stream')
    for number, block in blocks:
        validate_block(voice, number, block, xdata, ydata, check, twos, color, high,
                       varied, serial, totals)


def validate_block(voice, number, block, xdata, ydata, check, twos, color, high,
                   varied, serial, totals):
    n = int(number)
    root = xdata[50432] & 8388607
    initial_callbacks = [ydata[50432 + 3], ydata[50432 + 3], ydata[(50439 if high else 50432) + 4], 1723]
    steady_callbacks = [4856, 4856, 4820, 1736]
    check('CALLBACKS_BEFORE', [root | (H if n == 0 else 0), *(initial_callbacks if n == 0 else steady_callbacks)], block)
    check('HISTORY_INIT', voice.history_words(), block)
    writes = voice.start_controls() if n == 0 else []
    lfos = [[int(v, 16) for v in row.split()] for row in re.findall('^LFO ([^\\n]+)', block, re.M)]
    if lfos != ([[root + 1 + 10 * i, 4096 + 160 + 12 * i, 3 ** i] for i in range(3)] if n == 0 else []):
        raise ValueError('Unexpected LFO initialization in dependent stream')
    for stage in ('COPIED', 'ROUTED', 'GAINS'):
        check(stage, voice.gains, block)
        check('PARAM_' + stage, [p & WORD_MASK for p in voice.parameters], block)
    waves, wraps, syncs = voice.oscillators()
    before = voice.seed
    check('NOISE_INPUT', [before, voice.noise_history, int(color) * 65536], block)
    noise = voice.noise()
    writes.append([2039 if int(color) < 64 else 2054, before, voice.seed])
    actual_writes = [[int(v, 16) for v in row.split()] for row in re.findall('^SEED_WRITE ([^\\n]+)', block, re.M)]
    if writes != actual_writes:
        raise ValueError('Dependent shared seed updates differ')
    check('NOISE_STATE', [voice.seed, voice.noise_history], block)
    check('COEFFICIENTS', voice.coefficients, block)
    mixed = mixer_model(waves, noise, voice.coefficients, twos)
    observed_mix = {int(at, 16): [int(v, 16) for v in row.split()] for at, row in re.findall('^MIX_OUTPUT ([0-9a-f]+)([^\\n]+)', block, re.M)}
    if observed_mix != mixed:
        raise ValueError(f'{identity}, block {n}: mixed samples differ')
    work, routed, coefs, inputs, route_coefs, endpoint = voice.filters(mixed, waves)
    if voice.drives is not None:
        drive_metrics = validate_drive_calls(voice, block, xdata, waves)
        for key, value in drive_metrics.items():
            totals[key] = totals.get(key, 0) + value
    actual_outputs = {int(at, 16): [int(v, 16) for v in row.split()] for at, row in re.findall('^OUTPUT ([0-9a-f]+)([^\\n]+)', block, re.M)}
    expected_outputs = {1344 + 16 * i: row for i, row in enumerate(waves)} | {1392: noise} | work
    if actual_outputs != expected_outputs:
        raise ValueError(f'{identity}, block {n}: waveform/noise/filter outputs differ')
    check('HISTORIES', voice.history_words(), block)
    check('ROUTE_OUTPUT', routed, block)
    check('ROUTE_COEFFICIENTS', [v & WORD_MASK for v in route_coefs], block)
    check('FILTER_ENDPOINT', [1442 if serial else 1425, 1, WORD_MASK if serial else 32771], block)
    check('ROUTE', [int(serial)], block)
    filters = [[int(v, 16) for v in row.split()] for row in re.findall('^FILTER ([^\\n]+)', block, re.M)]
    if filters != [[1408, 864, 24, 2], [1424, 872, 29, 2]]:
        raise ValueError('Original dependent filter call context differs')
    cores = [[int(v, 16) for v in row.split()] for row in re.findall('^CORE ([^\\n]+)', block, re.M)]
    expected_cores = []
    for i, c in enumerate(coefs):
        expected_cores.append([4947 if c[5] else 4919, 1408 if i < 2 else 1424, (864 if i < 2 else 872) + 2 * (i % 2), (24 if i < 2 else 29) + (1 if i % 2 == 0 else 0), 3 if i % 2 == 0 else 2, *[v & WORD_MASK for v in c[:5]]])
    actual_inputs = [[int(v, 16) for v in row.split()] for row in re.findall('^CORE_INPUT ([^\\n]+)', block, re.M)]
    if cores != expected_cores or actual_inputs != inputs:
        raise ValueError('Dependent cascade coefficients/intermediate inputs differ')
    check('PITCH', voice.pitch, block)
    check('PHASE', [p & WORD_MASK for p in voice.phase], block)
    state = re.search('CURSOR ([0-9a-f]+) MASK ([0-9a-f]+)', block)
    counts = re.search('WRAPS (\\d+) (\\d+) (\\d+) SYNC (\\d+)', block)
    if state is None or [int(v, 16) for v in state.groups()] != [voice.cursor, 255]:
        raise ValueError('Dependent cursor/mask differs')
    if counts is None or list(map(int, counts.groups())) != wraps + [syncs]:
        raise ValueError('Dependent wrap/sync counts differ')
    returned = [endpoint[0] & FULL_MASK, endpoint[1] & FULL_MASK, voice.filter_return_r0, 874 if serial else 1440, 50432, 2208, voice.filter_return_r5, 29, voice.parameters[10] if serial else work[1424][0], voice.filter_return_x1, route_coefs[0] & WORD_MASK]
    check('FILTER_RETURN', returned, block)
    check('PRIOR_BUS', voice.bus, block)
    envelope = voice.envelopes[7]
    check('OUTPUT_DISPATCH', [envelope, voice.gain, RESET_LEVEL], block)
    out = output_model(voice.gain, envelope, RESET_LEVEL, routed, voice.bus)
    check('OUTPUT_BUS', out['bus'], block)
    check('FINAL_GAIN', [out['stored_target']], block)
    check('OUTPUT_KERNEL', [signed(voice.gain) * W & FULL_MASK, out['delta'] & FULL_MASK, out['stored_target'], 2176, 50621, 1440, 1], block)
    check('OUTPUT_END', [out['end_gain'], out['last_full'], 2209, 50621, 1472, envelope, out['step'], 1193047, out['last_gain']], block)
    totals['changed_prefetch_blocks'] += voice.bus[:2] != initial_bus(varied)[:2]
    voice.bus, voice.gain = (out['bus'], out['stored_target'])
    final_return = voice.finish_controls()
    check('WHOLE_RETURN', final_return, block)
    check('FINAL_LFO', voice.lfos, block)
    check('FINAL_ENVELOPES', voice.envelopes, block)
    check('CALLBACKS_AFTER', [root, *steady_callbacks], block)
    totals['blocks'] += 1
    totals['dependent_blocks'] += n > 0
    totals['wraps'] += sum(wraps)
    totals['syncs'] += syncs
    totals['falling_steps'] += signed(out['step']) < 0
    totals['zero_steps'] += out['step'] == 0
    totals['seed_writes'] += len(writes)
    totals['positive_limits'] += out['positive_limits']
    totals['negative_limits'] += out['negative_limits']

def validate(result, xdata, ydata, reference):
    tables = {twos: waveform_model(ydata, twos) for twos in (False, True)}
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)(?=CASE |Entry-streams visited PCs:)', result)
    seen = set()
    totals = new_totals()
    for *identity, stream in cases:
        identity = tuple(identity)
        if identity in seen:
            raise ValueError('Duplicated entry stream')
        seen.add(identity)
        validate_stream(stream, xdata, ydata, tables, identity, totals)

    matrix = {(kind, mode, str(color), profile, route) for kind in ('normal', 'sync') for mode in ('0', '1') for color in (0, 127) for profile in ('0', '1') for route in ('0', '1')}
    if seen != matrix or totals['blocks'] != 256 or totals['dependent_blocks'] != 224:
        raise ValueError('Incomplete persistent entry matrix')
    if min((totals[k] for k in ('falling_steps', 'zero_steps', 'changed_prefetch_blocks'))) <= 0:
        raise ValueError('Missing natural decay, steady-gain or persistent-bus distinction')
    validate_coverage(result, reference['coverage'])
    return totals
