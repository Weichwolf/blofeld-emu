# SPDX-License-Identifier: GPL-3.0-only
"""Build inert compiler artifacts; never execute a control binary."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from archive_hardening import archive_members, check_archive_control_flow


class ArchiveHardeningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix='blofeld-cf-controls-')
        cls.folder = Path(cls.temporary.name)
        cls.compiler = shutil.which('clang-19')
        cls.archiver = shutil.which('llvm-ar-19')
        cls.readelf = shutil.which('llvm-readelf-19')
        cls.disassembler = shutil.which('llvm-dis-19')
        if not all((cls.compiler, cls.archiver, cls.readelf, cls.disassembler)):
            cls.temporary.cleanup()
            raise RuntimeError('LLVM19 compiler, ar, readelf and disassembler are required')
        source = cls.folder / 'control.c'
        source.write_text('// SPDX-License-Identifier: GPL-3.0-only\n'
                          'int probe(int value) { return value + 1; }\n')
        cls.objects = {}
        for kind, flags in [('elf', []), ('thin', ['-flto=thin']), ('full', ['-flto'])]:
            for protection in ('full', 'none', 'branch', 'return'):
                directory = cls.folder / (kind + '-' + protection)
                directory.mkdir()
                target = directory / 'member.o'
                subprocess.run([cls.compiler, '-O3', '-fPIC', '-fcf-protection=' + protection,
                                *flags, '-c', str(source), '-o', str(target)],
                               check=True, capture_output=True, text=True)
                cls.objects[kind, protection] = target

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def archive(self, *members, format='gnu'):
        target = self.folder / (self.id().split('.')[-1] + '-' + str(len(list(self.folder.glob('*.a')))) + '.a')
        subprocess.run([self.archiver, '--format=' + format, 'qcs', str(target),
                        *(str(member) for member in members)],
                       check=True, capture_output=True, text=True)
        return target

    def check(self, path):
        return check_archive_control_flow(path, self.readelf, self.disassembler)

    def test_protected_elf_thin_full_and_mixed(self):
        for kind in ('elf', 'thin', 'full'):
            with self.subTest(kind=kind):
                self.assertEqual(self.check(self.archive(self.objects[kind, 'full'])), 1)
        members = [self.objects[kind, 'full'] for kind in ('elf', 'thin', 'full')]
        archive = self.archive(*members)
        self.assertEqual([name for name, _ in archive_members(archive)], ['member.o'] * 3)
        self.assertEqual(self.check(archive), 3)

    def test_each_missing_property_rejects_for_all_formats(self):
        for kind in ('elf', 'thin', 'full'):
            for protection in ('none', 'branch', 'return'):
                with self.subTest(kind=kind, protection=protection):
                    with self.assertRaisesRegex(ValueError, 'CF'):
                        self.check(self.archive(self.objects[kind, protection]))

    def test_duplicate_names_never_hide_a_weak_member(self):
        for kind in ('elf', 'thin', 'full'):
            for reverse in (False, True):
                members = [self.objects[kind, 'full'], self.objects[kind, 'none']]
                if reverse:
                    members.reverse()
                with self.subTest(kind=kind, reverse=reverse):
                    archive = self.archive(*members)
                    self.assertEqual(len(list(archive_members(archive))), 2)
                    with self.assertRaisesRegex(ValueError, 'CF'):
                        self.check(archive)

    def test_long_gnu_and_bsd_names(self):
        path = self.folder / 'a long native object name with spaces and enough characters.o'
        shutil.copyfile(self.objects['thin', 'full'], path)
        for format in ('gnu', 'bsd'):
            with self.subTest(format=format):
                archive = self.archive(path, format=format)
                self.assertEqual([name for name, _ in archive_members(archive)], [path.name])
                self.assertEqual(self.check(archive), 1)

    def test_unreferenced_metadata_cannot_claim_protection(self):
        source = self.folder / 'decoy.ll'
        source.write_text('define i32 @probe() { ret i32 1 }\n'
                          '!unrelated = !{!0, !1}\n'
                          '!0 = !{i32 8, !"cf-protection-return", i32 1}\n'
                          '!1 = !{i32 8, !"cf-protection-branch", i32 1}\n')
        target = self.folder / 'decoy.o'
        subprocess.run([self.compiler, '-x', 'ir', '-c', '-emit-llvm', str(source),
                        '-o', str(target)], check=True, capture_output=True, text=True)
        with self.assertRaisesRegex(ValueError, 'CF'):
            self.check(self.archive(target))

    def test_invalid_archive_or_member_fails_visibly(self):
        valid = self.archive(self.objects['elf', 'full'])
        broken = self.folder / 'broken.a'
        for data in (b'', b'!<thin>\n', valid.read_bytes()[:-1]):
            broken.write_bytes(data)
            with self.assertRaises(ValueError):
                self.check(broken)
        empty = self.folder / 'empty.a'
        empty.write_bytes(b'!<arch>\n')
        with self.assertRaisesRegex(ValueError, 'no object members'):
            self.check(empty)


if __name__ == '__main__':
    unittest.main()
