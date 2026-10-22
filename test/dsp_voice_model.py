# SPDX-License-Identifier: GPL-3.0-only
"""Independent integer models for declared original held-note voice streams."""

from dsp_drive_model import drive_block, drive_live_state

W = 1 << 24
H = 1 << 23
WORD_MASK = W - 1
FULL_MASK = (1 << 56) - 1
RESET_LEVEL = 0x7f0000

def signed(value):
    return value if value < H else value - W

def qword(value):
    return max(-H, min(H - 1, value // W))

def round_word(value, twos):
    word, fraction = divmod(value, W)
    return (word + (fraction > H or (fraction == H and (twos or word % 2 != 0)))) * W

def waveform_model(original, twos):
    table = dict(original)
    table.update({at: 0 for at in range(2936, 3013)})
    sine = [0] * 1024
    accumulator, previous = (7915436 * W, 0)
    for j in range(1, 1024):
        current = qword(accumulator)
        accumulator = round_word(-previous * W + 2 * 5555042 * current, twos)
        sine[201 * j % 1024] = current
        previous = current
    for bank in range(7):
        body = 3303 + 258 * bank
        destination = [0] * 256
        for term in range(table[3257 + bank]):
            coefficient = signed(table[3264 + term])
            for i in range(256):
                sample = sine[(2 + 4 * term + (4 + 8 * term) * i) % 1024]
                destination[i] = qword(destination[i] * W + 2 * coefficient * sample)
        table.update({body + i: value & WORD_MASK for i, value in enumerate(destination)})
        table[body - 1] = destination[-1] & WORD_MASK
        table[body + 256] = destination[0] & WORD_MASK
    return table

def controls(original, varied):
    sound = {field: original[7717 + field] & 255 for field in range(383)}
    sound[67] = 64
    if varied:
        sound.update({61 + i: value for i, value in enumerate((101, 127, 89, 64, 73, 32, 96, 113))})
        sound[71], sound[72] = (77, 17)

    def balance(value):
        full = value * 131072 - H
        if full == 8257536:
            full += 8257536
        return qword(full * W)
    pairs = [(128 * sound[level] ** 2, balance(sound[pan])) for level, pan in ((61, 62), (65, 66), (63, 64), (71, 72), (67, 68))]
    return pairs

def coefficient_model(pairs):
    result = []
    for index, (gain, pan) in enumerate(pairs):
        gain = max(gain, 0)
        divisor = 2 if index == 4 else 1
        result.extend((qword(full // divisor) & WORD_MASK for full in (gain * W + 2 * gain * pan, gain * W - 2 * gain * pan)))
    return result

def mixer_model(waves, noise, coefficients, twos):
    coefficient = list(map(signed, coefficients))
    buffers = {1408: [], 1424: []}
    for i in range(16):
        samples = [signed(channel[i]) for channel in waves]
        partial = sum((2 * value * coefficient[2 * j] for j, value in enumerate(samples)))
        b = sum((2 * value * coefficient[2 * j + 1] for j, value in enumerate(samples)))
        ring = qword(16 * samples[0] * samples[2])
        a = qword(partial) * W + 2 * ring * coefficient[6] + 2 * signed(noise[i]) * coefficient[8]
        b += 2 * ring * coefficient[7] + 2 * signed(noise[i]) * coefficient[9]
        buffers[1424].append(qword(round_word(a, twos)) & WORD_MASK)
        buffers[1408].append(qword(round_word(b, twos)) & WORD_MASK)
    return buffers

def balance(value):
    result = value * 131072 - H
    return qword((result + (8257536 if result == 8257536 else 0)) * W)

def filter_parameters(data, varied, drives=None):
    sound = {i: data[7717 + i] & 255 for i in range(383)}
    if varied:
        sound.update({78: 29, 80: 117, 98: 101, 100: 85, 93: 23, 113: 105})
    if any((sound[i] != 0 for i in (81, 91, 92, 101, 111, 112))) or any((sound[i] != 64 for i in (86, 106))):
        raise ValueError('Changed declared zero-drive/FM and centered cutoff-keytrack inputs')
    if drives is not None:
        if len(drives) != 2 or any(curve not in range(16) or level not in range(128)
                                  for curve, level in drives):
            raise ValueError('Unsupported declared Drive sound fields')
        sound[81], sound[101] = drives[0][1], drives[1][1]
    params = []
    for offset in (0, 20):
        params.extend((sound[78 + offset] * 131072 - H, signed(data[5499 + sound[80 + offset]]), 512 * sound[92 + offset] ** 2, sound[81 + offset] * 65536, balance(sound[93 + offset])))
    level = sound[121] * 65536
    velocity = 4194304
    sensitivity = sound[122] * 131072 - H
    b = 2 * velocity * sensitivity
    a = level * W + 2 * level * qword(b)
    if b >= 0:
        a -= 2 * level * sensitivity
    params.append(qword(max(0, a)))
    return params

def lowpass_coefficients(data, f, resonance):
    segment, fraction = divmod(f, 131072)
    fraction *= 64
    left, right = map(signed, (data[2612 + segment], data[2613 + segment]))
    full = left * W + 2 * fraction * (right - left)
    g = qword(full)
    feed = (qword(g * g), qword(2 * g * g), qword(g * g))
    first = (*feed, qword(H * W - 2 * full), qword(2 * full - 4 * g * g - 4194304 * W), False)
    damping = 4063232 * W - 2 * 4325376 * resonance
    if damping < 0:
        damping //= 2
    damping = max(damping, -196608 * W + 2 * 393216 * f)
    e = qword(damping - 2 * qword(damping) * g)
    total = 4194304 * W - 2 * g * g - 2 * e * g
    wide = total > 4194304 * W
    second = (*feed, qword(total if wide else 2 * total), qword(-4194304 * W + 4 * e * g), wide)
    return (first, second)

def output_model(previous, envelope, controller, samples, bus):
    target = 2 * signed(envelope) * signed(controller)
    delta = (target - signed(previous) * W) // 16
    step = qword(delta)
    gain = signed(previous) * W
    output = []
    last = 0
    last_gain = 0
    positive = negative = 0
    for i in range(16):
        last_gain = qword(gain)
        for channel in range(2):
            at = 2 * i + channel
            last = signed(bus[at]) * W + 2 * last_gain * signed(samples[at])
            positive += last >= H * W
            negative += last < -H * W
            output.append(qword(last) & WORD_MASK)
        gain = (last_gain + step) * W
    return {'bus': output, 'stored_target': qword(target) & WORD_MASK, 'delta': delta, 'step': step & WORD_MASK, 'end_gain': gain & FULL_MASK, 'last_full': last & FULL_MASK, 'last_gain': last_gain & WORD_MASK, 'positive_limits': positive, 'negative_limits': negative}

def initial_bus(varied):
    return [1267611, 2386092] + [((H - 1 if i % 2 == 0 else -H) if varied else (i - 16) * 65536) & WORD_MASK for i in range(2, 32)]

class Voice:
    """Static held-note parameters, with all inter-block histories retained."""

    def __init__(self, xdata, ydata, table, high, twos, color, varied, serial, drives=None,
                 sound_base=0x1000):
        self.xdata, self.data, self.table = (xdata, ydata, table)
        self.high, self.twos, self.color, self.serial = (high, twos, color, serial)
        self.drives = drives
        self.sound_base = sound_base
        self.controller = RESET_LEVEL
        self.parameters = filter_parameters(ydata, varied, drives)
        self.gains = [v & WORD_MASK for pair in controls(ydata, varied) for v in pair]
        self.coefficients = coefficient_model(controls(ydata, varied))
        self.bus = initial_bus(varied)
        self.gain = 0
        self.seed, self.noise_history = (1, 0)
        self.phase, self.cursor = ([0, 0, 0], 0)
        self.histories = [[[0, 0, 0, 1], [0, 0, 0, 0]] for _ in range(2)]
        self.pitch, self.frequency, self.damping, self.bias = ([], [], [], [])
        self.sound = {i: ydata[7717 + i] & 255 for i in range(383)}
        for osc, shape in ((0, 3), (2, 3), (1, 4)):
            default = lambda field: self.sound[16 * osc + field]
            octave, semitone, detune = (112, 76, 127) if high else (default(1), default(2), default(3))
            note = 63 * 65536 if high else 0
            track = signed((default(5) - 64) * 131072 & WORD_MASK)
            full = (1024 * (64 * (octave + semitone) + detune) - 4784128) * W + 4 * track * note
            pitch = qword(full)
            self.pitch.append(pitch & WORD_MASK)
            segment, fraction = divmod(pitch, 65536)
            fraction *= 128

            def interpolate(at):
                return qword(signed(table[at]) * W + 2 * fraction * (signed(table[at + 1]) - signed(table[at])))
            self.frequency.append(interpolate(2807 + segment))
            self.damping.append(interpolate(2807 + segment + 257))
            self.bias.append(3431 if shape == 4 else table[3225 + pitch // 262144])
        self.lfos = []
        self.envelopes = []
        self.block = 0

    def history_words(self):
        result = []
        for h1, h2 in self.histories:
            result.extend((v & WORD_MASK for v in (h1[0], h1[1], h2[0], h2[1], h1[2], h1[3], h2[2], h2[3])))
        return result

    def start_controls(self):
        writes = []
        for i in range(3):
            base = 160 + 12 * i
            if [self.sound[base + j] for j in (0, 3, 4, 5, 6, 7, 10)] != [0, 0, 0, 0, 0, 64, 64]:
                raise ValueError('Changed declared free-sine LFO defaults')
            if self.data[5962] != 5888:
                raise ValueError('Changed free-sine callback table')
            seed = self.seed
            self.seed *= 3
            writes.append([5842, seed, self.seed])
            self.lfos.extend((5888, 0, seed - 131072 & WORD_MASK, 0, seed, 0, H - 1, 0, 5888, 0))
        self.lfos.extend((0, 0))
        for base in (196, 208, 220, 232):
            if self.sound[base] & 7 or self.sound[base + 3] != 0 or self.data[5832] != H - 1:
                raise ValueError('Changed zero-attack ADSR declarations')
            self.envelopes.extend((5726, H - 1, self.sound_base + base, self.sound[base + 4] * 65536, self.data[5832 - self.sound[base + 7]], self.sound[base + 8] * 65536))
        self.advance_lfos()
        return writes

    def advance_lfos(self):
        for i in range(3):
            at = 10 * i
            p = signed(self.lfos[at + 2])
            wave = qword(4 * (p * W + (-2 * p * p if p >= 0 else 2 * p * p)))
            self.lfos[at + 3] = qword(2 * wave * (H - 1)) & WORD_MASK
            increment = signed(self.xdata[9490 + self.sound[161 + 12 * i]])
            self.lfos[at + 2] = p + increment & WORD_MASK

    def finish_controls(self):
        self.advance_lfos()
        last = 0
        for i, base in enumerate((196, 208, 220, 232)):
            at = 6 * i
            if self.block == 0:
                last = 2 * (H - 1) * W
                self.envelopes[at:at + 2] = [5689, H - 1]
            else:
                u = signed(self.envelopes[at + 1])
                coefficient = signed(self.data[5832 - self.sound[base + 5]])
                target = self.sound[base + 6] * 65536
                last = u * W + 2 * coefficient * (target - u)
                self.envelopes[at + 1] = qword(last) & WORD_MASK
        self.block += 1
        if self.block == 1:
            return [5832 * W, last & FULL_MASK, 50506, 5832, 50488, 50506, 50432, 5726, 5689, 5832, self.data[50434]]
        return [5832 * W, last & FULL_MASK, 50506, 5832 - self.sound[237], 50488, 50506, 50432, 5689, 53, 5832, self.data[5832 - self.sound[237]]]

    def oscillators(self):
        output = [[], [], []]
        wraps, syncs, full_b = ([0, 0, 0], 0, 0)
        for sample in range(16):
            preceding_wrap = False
            for osc in range(3):
                if self.high and osc == 2 and preceding_wrap:
                    fraction = ((self.cursor << 16 | (full_b & WORD_MASK) >> 8) & WORD_MASK) >> 1
                    self.phase[osc] = signed(qword(self.frequency[osc] * W - 2 * self.frequency[osc] * fraction) & WORD_MASK)
                    syncs += 1
                segment, fraction = divmod(self.phase[osc], 65536)
                fraction *= 128
                at = self.bias[osc] + segment
                full_b = round_word(signed(self.table[at]) * W + 2 * fraction * (signed(self.table[at + 1]) - signed(self.table[at])), self.twos)
                output[osc].append(qword(full_b) & WORD_MASK)
                accumulator = (self.phase[osc] + self.frequency[osc]) * W
                self.phase[osc] = signed(accumulator // W & WORD_MASK)
                preceding_wrap = accumulator >= H * W
                if preceding_wrap:
                    wraps[osc] += 1
                    magnitude = accumulator // W & 8388607
                    full_b = -2 * self.damping[osc] * magnitude
                    main = full_b // W & WORD_MASK
                    full_b += ((main & 255) - main) * W
                    full_b += sample * 256 * W
                    self.cursor = full_b // W & WORD_MASK
        return (output, wraps, syncs)

    def noise(self):
        coefficient = 2 * self.color * 65536 - H
        amplitude = max(2 * self.color * 65536, 1048576) if coefficient < 0 else -H
        output = []
        previous = signed(self.noise_history)
        for _ in range(16):
            carry = self.seed & 1
            self.seed = self.seed >> 1 ^ (8650752 if carry else 0)
            excitation = -amplitude if carry else amplitude
            full = excitation * W - 2 * previous * coefficient
            output.append(qword(full) & WORD_MASK)
            previous = qword(full) if coefficient < 0 else qword(excitation * W)
        self.noise_history = previous & WORD_MASK
        return output

    def biquad(self, samples, c, history):
        b0, b1, b2, a1, a2, wide = c
        output = []
        divisor = 1 << 22
        for value in map(signed, samples):
            full = b0 * value + b1 * history[1] + b2 * history[0] + (2 if wide else 1) * a1 * history[3] + a2 * history[2]
            word, fraction = divmod(full, divisor)
            if wide and (fraction > divisor // 2 or (fraction == divisor // 2 and (self.twos or word % 2))):
                word += 1
            word = max(-H, min(H - 1, word))
            output.append(word & WORD_MASK)
            history[:] = (history[1], value, history[3], word)
        return output

    def filters(self, mixed, oscillators=None):
        if self.drives is not None and (oscillators is None or len(oscillators) != 3
                                       or any(len(channel) != 16 for channel in oscillators)):
            raise ValueError('Drive requires independently generated oscillator-1 samples')
        self.drive_inputs, self.drive_outputs = [], []
        self.drive_lookahead = [list(mixed[1424][:2]), list(self.bus[:2])]
        work = {at: list(row) for at, row in mixed.items()}
        coefficients, inputs = ([], [])
        for index, at in enumerate((1408, 1424)):
            if index == 1 and self.serial:
                work[at] = [qword((signed(a) + signed(b)) * W) & WORD_MASK for a, b in zip(work[1408], work[at])]
            for section, c in enumerate(lowpass_coefficients(self.data, *self.parameters[5 * index:5 * index + 2])):
                coefficients.append(c)
                inputs.append(list(work[at]))
                work[at] = self.biquad(work[at], c, self.histories[index][section])
            self.drive_inputs.append(list(work[at]))
            if self.drives is not None:
                work[at] = drive_block(self.drives[index][0], max(0, self.parameters[5 * index + 3]),
                                       self.twos, work[at], oscillators[0])
            self.drive_outputs.append(list(work[at]))
        self.filter_return_r0, self.filter_return_x1 = 11214, self.bus[1] if self.serial else self.bus[0]
        self.filter_return_r5 = 1424 if self.serial else 2145
        if self.drives is not None:
            curve = self.drives[1][0]
            r0, x1, r5 = drive_live_state(curve, max(0, self.parameters[8]),
                                      self.drive_inputs[1] + self.drive_lookahead[1],
                                      oscillators[0] + [oscillators[1][0]],
                                      self.bus[0], self.xdata[0x2f36 + curve], buffer=1424)
            self.filter_return_r0 = r0
            if self.serial:
                self.filter_return_r5 = r5
            else:
                self.filter_return_x1 = x1
        gain = max(0, self.parameters[10])

        def pair(p):
            return [qword(2 * gain * qword((H + p) * W)), qword(2 * gain * qword((H - p) * W))]
        c = pair(self.parameters[9]) if self.serial else pair(self.parameters[4]) + pair(self.parameters[9])
        routed = []
        a = b = 0
        for i in range(16):
            if self.serial:
                a, b = (2 * signed(work[1424][i]) * v for v in c)
            else:
                a = 2 * signed(work[1408][i]) * c[0] + 2 * signed(work[1424][i]) * c[2]
                b = 2 * signed(work[1408][i]) * c[1] + 2 * signed(work[1424][i]) * c[3]
            routed.extend((qword(a) & WORD_MASK, qword(b) & WORD_MASK))
        if self.serial:
            a, b = (2 * signed(self.bus[0]) * v for v in c)
        return (work, routed, coefficients, inputs, c, (a, b))
