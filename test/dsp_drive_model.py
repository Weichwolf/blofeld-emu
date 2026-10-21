# SPDX-License-Identifier: GPL-3.0-only
"""Independent integer equations for the original sixteen-word Drive callbacks.

Adapted from blofeld-src 8d7cdd313b1051b83b98721dc933162ea6def845;
the model predicts output words, not callback register or lookahead state.
"""

W, H = 1 << 24, 1 << 23
MASK = W - 1


def signed(word):
    return word if word < H else word - W


def q(value):
    return min(H - 1, max(-H, value // W))


def middle(value):
    return signed((value // W) & MASK)


def rounded(value, twos):
    integer, remainder = divmod(value, W)
    return (integer + (remainder > H or (remainder == H and (twos or integer % 2)))) * W


def log_encoding(value):
    exponent = value.bit_length() - 47
    normalized = value >> exponent if exponent >= 0 else value << -exponent
    return (exponent * (1 << 48) + ((normalized << 2) & ((1 << 48) - 1))) >> 7


def drive_block(curve, drive, twos, words, oscillator):
    if (curve not in range(16) or not 0 <= drive < H or len(words) != 16
            or len(oscillator) != 16 or any(not 0 <= v <= MASK for v in words + oscillator)):
        raise ValueError("Unsupported declared Drive block")
    if curve in (13, 14, 15) or (curve == 0 and drive == 0):
        return list(words)
    samples, gates = list(map(signed, words)), list(map(signed, oscillator))
    if curve == 0:
        gain = q(0x10000 * W + 2 * drive * drive)
        t = max(0, 0x100000 - drive)
        linear, clipped = q(8 * t * W), q((0x200000 - 2 * t) * W)
        output = [q(2 * linear * v + 2 * clipped * q((2 * v * gain) << 8)) for v in samples]
    elif curve == 1:
        gain = q(0x20000 * W + 2 * drive * drive)
        def shape(v):
            full = (2 * v * gain) << 7
            limited = q(full)
            return 2 * full + (2 * limited * limited if full < 0 else -2 * limited * limited)
        output = [q(2 * 0x200000 * q(shape(v))) for v in samples]
    elif curve == 2:
        t = max(0, 0x7f0000 - drive)
        base = t * t // 2 + W
        gain, floor_base, encoded = q((0x10000 + t) * W), q(base), q(log_encoding(base))
        def hard(v):
            delta = q(log_encoding(floor_base * W + 2 * v * v) - encoded * W)
            return (2 * gain * delta * (1 if v >= 0 else -1)) << 6
        output = [q(hard(v)) for v in samples]
    elif curve == 3:
        output = [q(rounded(v * W + 2 * q(16 * q(2 * v * v) * v) * drive, twos))
                  for v in samples]
    elif curve == 4:
        output = [q(v * W + (2 * q(4 * v * v) * drive if v < 0
                             else -2 * q(4 * v * v) * drive)) for v in samples]
    elif curve in (5, 6):
        gain = q(0x20000 * W + 2 * drive * drive)
        offset = 0x400000 if curve == 5 else 0x5a827a
        def pickup(v):
            shifted = ((2 * v * gain) << 7) + offset * W
            full = ((q(abs(shifted)) - offset) * W if curve == 5
                    else 2 * q(shifted) ** 2 - 0x400000 * W)
            limited = q(full)
            return rounded(full + (2 * limited * limited if full < 0
                                   else -2 * limited * limited), twos)
        output = [q(pickup(v)) for v in samples]
    elif curve == 7:
        t = max(0, 0x7f0000 - drive)
        bias = q(2 * t * t)
        output = [q((abs(v + bias) - bias) * W) for v in samples]
        # The first sample retains the square's full fractional remainder.
        output[0] = q(abs(samples[0] * W + 2 * t * t) - bias * W)
    elif curve == 8:
        t = max(0, 0x7f0000 - drive)
        output = [q(2 * v * t + 2 * drive * q(4 * v * v)) for v in samples]
    elif curve == 9:
        height = q(2 * drive * 0x180000)
        output = [q((-v + (height if v >= 0 else -height)) * W) for v in samples]
    elif curve == 10:
        gain = q(0x10000 * W + 2 * drive * drive)
        output = [q(2 * 0x200000 * middle((2 * v * gain) << 8)) for v in samples]
    elif curve == 11:
        gain = q(0x10000 * W + 2 * drive * drive)
        wrapped = [middle((2 * v * gain) << 8) for v in samples]
        output = [q(v * W + (2 * v * v if v < 0 else -2 * v * v)) for v in wrapped]
    else:
        t = max(0, 0x7f0000 - drive)
        output = [q(2 * v * t + 4 * drive * q(2 * gate * v))
                  for v, gate in zip(samples, gates)]
    return [v & MASK for v in output]


def drive_live_state(curve, drive, words, gates, incoming_x1, callback, buffer=0x580):
    """R0/X1 retained by the real caller, including unused speculative samples."""
    if len(words) != 18 or len(gates) != 17 or curve not in range(16):
        raise ValueError('Incomplete declared Drive lookahead context')
    last = signed(words[15])
    x1 = incoming_x1
    if curve == 0 and drive != 0:
        x1 = q((0x200000 - 2 * max(0, 0x100000 - drive)) * W)
    elif curve in (1, 10):
        x1 = 0x200000
    elif curve == 2:
        x1 = q((0x10000 + max(0, 0x7f0000 - drive)) * W)
    elif curve == 3:
        x1 = q(16 * q(2 * last * last) * last)
    elif curve == 4:
        x1 = q(4 * last * last)
    elif curve in (5, 6):
        x1 = 0x400000 if curve == 5 else 0x5a827a
    elif curve == 8:
        x1 = drive
    elif curve == 11:
        x1 = words[17]
    elif curve == 12:
        x1 = gates[16]
    read = 0 if curve in (13, 14, 15) or (curve == 0 and drive == 0) else 17
    if curve in (1, 5, 6, 8, 10, 11):
        read = 18
    return (0x551 if curve == 12 else callback), x1 & MASK, buffer + read
