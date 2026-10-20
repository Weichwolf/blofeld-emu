# SPDX-License-Identifier: GPL-3.0-only
"""Negative checks prevent partial native dumps from passing silicon comparison."""
from pathlib import Path
import tempfile
import unittest

from dsp_hardware_corpus import (
    FULL_CAPTURE, REGISTER_CAPTURE, STACK_FIELDS, compare, parse_dump, validate_capture,
)


FIELDS = """pc sr omr la lc sp ep sz sc vba x0 x1 y0 y1
a0 a1 a2 b0 b1 b2 r0 r1 r2 r3 r4 r5 r6 r7
n0 n1 n2 n3 n4 n5 n6 n7 m0 m1 m2 m3 m4 m5 m6 m7""".split()


def dump_text():
    lines = ["case own_diagnostic"] + [f"{name} 000000" for name in FIELDS]
    return "\n".join(lines + ["ssh -", "ssl -", "end"]) + "\n"


def native_case(fields):
    return {"own_diagnostic": {**dict.fromkeys(STACK_FIELDS, 0), **fields}}


class SiliconComparatorTest(unittest.TestCase):
    def parse(self, text):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "own-observation.txt"
            path.write_text(text)
            return parse_dump(path)

    def test_missing_control_register_and_truncation_fail(self):
        for text in (dump_text().replace("ep 000000\n", ""), dump_text().removesuffix("end\n")):
            with self.assertRaises(ValueError):
                self.parse(text)

    def test_duplicate_unknown_or_oversize_register_fails(self):
        for injected in ("sr 000000\n", "unknown 000000\n", "xm0001 1000000\n"):
            with self.assertRaises(ValueError):
                self.parse(dump_text().replace("end\n", injected + "end\n"))

    def test_empty_stack_is_undefined_and_nonempty_requires_top(self):
        parsed = self.parse(dump_text())
        self.assertIsNone(parsed["own_diagnostic"]["ssh"])
        with self.assertRaises(ValueError):
            self.parse(dump_text().replace("sp 000000", "sp 000001"))

    def test_deep_stack_and_sparse_memory_differences_fail(self):
        for key in ("sh0f", "sl07", "xm0001", "ym0700", "pm07c0"):
            actual = native_case({key: 2})
            expected = {"own_diagnostic": {key: 3}}
            with self.assertRaisesRegex(AssertionError, key):
                compare(actual, expected, observations=FULL_CAPTURE)

    def test_missing_nonzero_cell_fails_and_missing_zero_is_valid(self):
        with self.assertRaisesRegex(AssertionError, "xm0001"):
            compare(native_case({}), {"own_diagnostic": {"xm0001": 1}}, observations=FULL_CAPTURE)
        self.assertEqual(compare(native_case({}), {"own_diagnostic": {"xm0001": 0}},
                                 observations=FULL_CAPTURE), len(STACK_FIELDS) + 1)

    def test_case_order_or_missing_case_fails(self):
        expected = {"first": {}, "second": {}}
        for actual in ({"second": {}, "first": {}}, {"first": {}}):
            with self.assertRaises(AssertionError):
                compare(actual, expected, observations=FULL_CAPTURE)

    def test_full_capture_rejects_missing_even_zero_native_stack_slots(self):
        expected = {"own_diagnostic": {"xm0001": 0}}
        for key in STACK_FIELDS:
            actual = native_case({})
            del actual["own_diagnostic"][key]
            with self.assertRaisesRegex(AssertionError, key):
                compare(actual, expected, observations=FULL_CAPTURE)

    def test_register_capture_counts_only_recorded_registers(self):
        expected = self.parse(dump_text())
        validate_capture(expected, REGISTER_CAPTURE)
        actual = {name: {**values, "sh01": 3, "xm0001": 7} for name, values in expected.items()}
        self.assertEqual(compare(actual, expected, observations=REGISTER_CAPTURE), len(FIELDS))
        actual["own_diagnostic"]["a1"] = 1
        with self.assertRaisesRegex(AssertionError, "a1"):
            compare(actual, expected, observations=REGISTER_CAPTURE)

    def test_capture_scope_cannot_hide_recorded_fields(self):
        expected = {"own_diagnostic": {"xm0001": 1, "sh01": 2}}
        validate_capture(expected, FULL_CAPTURE)
        with self.assertRaises(ValueError):
            validate_capture(expected, REGISTER_CAPTURE)
        with self.assertRaises(ValueError):
            compare(native_case({}), expected, observations=REGISTER_CAPTURE)

    def test_full_capture_requires_promised_recording_and_known_profile(self):
        for fields in ({}, {"xm0001": 1}, {"sh01": 1}):
            with self.assertRaises(ValueError):
                validate_capture({"own_diagnostic": fields}, FULL_CAPTURE)
        with self.assertRaises(ValueError):
            validate_capture({}, "unknown")
        with self.assertRaises(ValueError):
            compare({}, {}, observations="unknown")
