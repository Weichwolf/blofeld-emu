# SPDX-License-Identifier: GPL-3.0-only
"""Independent struct/wave fixtures through the actual headless sample importer."""
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import wave
import xml.etree.ElementTree as ET

BINARY = Path(sys.argv.pop(1)).resolve()


def chunk(name, payload):
    return name + struct.pack('<I', len(payload)) + payload + b'\0' * (len(payload) % 2)


def riff(*chunks):
    body = b'WAVE' + b''.join(chunks)
    return b'RIFF' + struct.pack('<I', len(body)) + body


class WaveClient(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='blofeld-wave-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.wave = self.root / 'own-Ω.wav'
        self.library = self.root / 'library.xml'
        self.invoke('new', self.library)
        self.invoke('add-program', self.library, self.library, 'Own')

    def invoke(self, *args, success=True):
        result = subprocess.run([str(BINARY), 'samples', *map(str, args)],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode == 0, success, result.stderr)
        self.assertFalse(any(s in result.stderr for s in (
            'AddressSanitizer', 'runtime error:', 'UndefinedBehaviorSanitizer')))
        return result

    def tone(self, frequency=440, stereo=False):
        pcm = b''.join(struct.pack('<h', round(8192 * math.sin(2 * math.pi * frequency * i / 44100)))
                       for i in range(11025))
        if stereo:
            pcm = b''.join(pcm[i:i+2] + pcm[i:i+2] for i in range(0, len(pcm), 2))
        with wave.open(str(self.wave), 'wb') as out:
            out.setparams((2 if stereo else 1, 2, 44100, 0, 'NONE', 'not compressed'))
            out.writeframes(pcm)

    def listing(self, path=None):
        return json.loads(self.invoke('list', path or self.library).stdout)['programs'][0]['samples']

    def test_real_import_auto_pitch_and_rebased_source(self):
        self.tone(stereo=True)
        before = hashlib.sha256(self.wave.read_bytes()).hexdigest()
        info = json.loads(self.invoke('wave-info', self.wave).stdout)
        self.assertEqual((info['channels'], info['sample_rate'], info['frames']), (2, 44100, 11025))
        self.assertAlmostEqual(info['detected_pitch']['midi_note'], 69, delta=0.02)
        self.assertIsNone(info['metadata_root'])
        self.invoke('add-sample', self.library, self.library, 1, self.wave, 'metadata', 127, success=False)
        target = self.root / 'nested' / 'export.xml'
        target.parent.mkdir()
        self.invoke('add-sample', self.library, target, 1, self.wave, 'auto', 127)
        sample = self.listing(target)[0]
        self.assertEqual(sample['path'], str(self.wave))
        self.assertAlmostEqual(sample['pitch'], 69, delta=0.02)
        self.assertEqual(ET.parse(target).find('SampleProgram/Sample').attrib['RelativePath'], '../own-Ω.wav')
        self.assertEqual(hashlib.sha256(self.wave.read_bytes()).hexdigest(), before)

    def test_sampler_and_instrument_metadata_with_explicit_override(self):
        fmt = struct.pack('<HHIIHH', 1, 1, 8000, 16000, 2, 16)
        sampler = struct.pack('<9I6I', 0, 0, 125000, 60, 0x80000000, 0, 0, 1, 0, 7, 0, 1, 3, 0, 0)
        instrument = struct.pack('<BbbBBBB', 60, -25, -6, 0, 127, 0, 127)
        self.wave.write_bytes(riff(chunk(b'fmt ', fmt), chunk(b'data', b'\0' * 8),
                                   chunk(b'smpl', sampler), chunk(b'inst', instrument)))
        info = json.loads(self.invoke('wave-info', self.wave).stdout)
        self.assertEqual(info['metadata_root'], 60.5)
        self.assertIsNone(info['detected_pitch'])
        self.assertEqual(info['instrument']['correction_cents'], -25)
        self.assertEqual(info['loops'], [dict(identifier=7, type=0, start=1,
                                            end_inclusive=3, fraction=0, play_count=0)])
        self.invoke('add-sample', self.library, self.library, 1, self.wave, 'metadata', 100)
        self.assertEqual((self.listing()[0]['pitch'], self.listing()[0]['level_db']), (60.5, 0))
        self.invoke('add-sample', self.library, self.library, 1, self.wave, '48.25', 60)
        self.assertEqual([s['highest'] for s in self.listing()], [60, 100])
        self.assertEqual(self.listing()[0]['pitch'], 48.25)

    def test_unpitched_material_requires_explicit_root(self):
        fmt = struct.pack('<HHIIHH', 1, 1, 8000, 16000, 2, 16)
        self.wave.write_bytes(riff(chunk(b'fmt ', fmt), chunk(b'data', b'\0' * 512)))
        before = self.library.read_bytes()
        result = self.invoke('add-sample', self.library, self.library, 1, self.wave, 'auto', 127, success=False)
        self.assertIn('supply a numeric root', result.stderr)
        self.assertEqual(self.library.read_bytes(), before)
        self.invoke('add-sample', self.library, self.library, 1, self.wave, 60, 127)
        self.assertEqual(self.listing()[0]['pitch'], 60)

    def test_bad_wave_and_collision_preserve_output_and_sources(self):
        before = self.library.read_bytes()
        fmt = struct.pack('<HHIIHH', 3, 1, 8000, 32000, 4, 32)
        for raw in [b'RIFF', riff(chunk(b'fmt ', fmt), chunk(b'data', struct.pack('<f', float('nan'))))]:
            self.wave.write_bytes(raw)
            self.invoke('add-sample', self.library, self.library, 1, self.wave, 60, 127, success=False)
            self.assertEqual(self.library.read_bytes(), before)
        self.tone()
        raw = self.wave.read_bytes()
        self.invoke('add-sample', self.library, self.wave, 1, self.wave, 60, 127, success=False)
        self.assertEqual(self.wave.read_bytes(), raw)
        self.assertEqual(self.library.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
