# SPDX-License-Identifier: GPL-3.0-only
"""Exercise the real headless sample editor with own XML and source files."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

BINARY = Path(sys.argv.pop(1)).resolve()


class SampleClient(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="blofeld-samples-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.input = self.root / "input.xml"
        self.output = self.root / "output.xml"
        self.wave = self.root / "own.wav"
        self.wave.write_bytes(b"Own unchanged source reference; metadata edits do not decode WAV.")
        self.input.write_text(r'''<?xml version="1.0" encoding="UTF-8"?>
<WaldorfSampleSet Path="Z:\old\input.xml" Version="0.5">
 <SampleProgram Name="First" Normalize="0">
  <Sample Filename="Own" Path="Z:\old\own.wav" RelativePath="own.wav"
          Pitch="69.05" HighKey="127" Normalize="0" FixedPitch="0" Level="-3.00"/>
 </SampleProgram>
 <SampleProgram Name="Second" Normalize="1"/>
</WaldorfSampleSet>''')

    def invoke(self, *args, success=True):
        result = subprocess.run([str(BINARY), "samples", *map(str, args)],
                                capture_output=True, text=True, timeout=30)
        if success:
            self.assertEqual(result.returncode, 0, result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0)
            self.assertTrue(result.stderr)
        self.assertFalse(any(marker in result.stderr for marker in (
            "AddressSanitizer", "runtime error:", "UndefinedBehaviorSanitizer")))
        return result

    def read(self, path=None):
        return json.loads(self.invoke("list", path or self.output).stdout)["programs"]

    def test_create_and_program_collection(self):
        self.invoke("new", self.output)
        self.assertEqual(self.read(), [])
        self.invoke("add-program", self.output, self.output, 'Own "Ω" & keys')
        self.assertEqual(self.read()[0]["name"], 'Own "Ω" & keys')
        self.invoke("append", self.output, self.output, self.input)
        self.assertEqual([p["name"] for p in self.read()], ['Own "Ω" & keys', "First", "Second"])
        self.invoke("move-program", self.output, self.output, 3, 1)
        self.assertEqual([p["name"] for p in self.read()], ["Second", 'Own "Ω" & keys', "First"])
        self.invoke("rename-program", self.output, self.output, 1, "Renamed")
        self.invoke("remove-program", self.output, self.output, 2)
        self.assertEqual([p["name"] for p in self.read()], ["Renamed", "First"])

    def test_export_and_relocate_preserves_source(self):
        target = self.root / "nested" / "selected.xml"
        target.parent.mkdir()
        self.invoke("export-program", self.input, target, 1)
        data = self.read(target)
        self.assertEqual(len(data), 1)
        self.assertEqual(data[0]["samples"][0]["path"], str(self.wave))
        sample = ET.parse(target).find("./SampleProgram/Sample")
        self.assertEqual(sample.attrib["RelativePath"], "../own.wav")
        self.assertEqual(self.wave.read_bytes(), b"Own unchanged source reference; metadata edits do not decode WAV.")

    def test_metadata_and_transposition(self):
        self.invoke("set-sample", self.input, self.output, 1, 1, "pitch", "60.25")
        self.invoke("transpose-roots", self.output, self.output, 1, "+12")
        self.invoke("set-sample", self.output, self.output, 1, 1, "level", "-6.50")
        self.invoke("set-sample", self.output, self.output, 1, 1, "normalize", 1)
        self.invoke("set-sample", self.output, self.output, 1, 1, "fixed-pitch", 1)
        self.invoke("set-program-normalize", self.output, self.output, 1, 1)
        model = self.read()[0]
        self.assertTrue(model["normalize"])
        sample = model["samples"][0]
        self.assertEqual((sample["pitch"], sample["level_db"]), (72.25, -6.5))
        self.assertTrue(sample["normalize"])
        self.assertTrue(sample["fixed_pitch"])
        self.invoke("remove-sample", self.output, self.output, 1, 1)
        self.assertEqual(self.read()[0]["samples"], [])
        self.assertTrue(self.wave.exists())

    def test_invalid_edit_leaves_output_unchanged(self):
        self.output.write_bytes(b"Retain this output on every invalid operation")
        invalid = [
            ("rename-program", self.input, self.output, 0, "bad"),
            ("move-program", self.input, self.output, 1, 3),
            ("set-sample", self.input, self.output, 1, 1, "pitch", "nan"),
            ("set-sample", self.input, self.output, 1, 1, "highest", 128),
            ("set-sample", self.input, self.output, 1, 1, "normalize", 2),
            ("set-sample", self.input, self.output, 1, 1, "unknown", 1),
            ("transpose-roots", self.input, self.output, 1, "1e999"),
            ("add-program", self.input, self.output, ""),
        ]
        for command in invalid:
            with self.subTest(command=command):
                self.invoke(*command, success=False)
                self.assertEqual(self.output.read_bytes(), b"Retain this output on every invalid operation")

    def test_output_must_not_replace_a_source(self):
        before = self.wave.read_bytes()
        self.invoke("rename-program", self.input, self.wave, 1, "bad", success=False)
        self.assertEqual(self.wave.read_bytes(), before)
        alias = self.root / "alias.xml"
        alias.hardlink_to(self.wave)
        self.invoke("rename-program", self.input, alias, 1, "bad", success=False)
        self.assertEqual(self.wave.read_bytes(), before)

    def test_unsupported_operations_and_bad_xml(self):
        result = self.invoke("transmit", self.input, success=False)
        self.assertIn("Unsupported samples command", result.stderr)
        self.input.write_text('<!DOCTYPE x [<!ENTITY e SYSTEM "file:///etc/passwd">]><x/>')
        self.invoke("list", self.input, success=False)
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
