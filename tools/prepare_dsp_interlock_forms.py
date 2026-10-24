# SPDX-License-Identifier: GPL-3.0-only
"""Own accumulator forms used by the independently assembled timing reference."""


def reference_program():
    instructions = ["move #>$305,omr"] + ["nop"] * 8
    cases = []

    def begin():
        instructions.extend(["move #>$c00300,sr", "move #>$400000,x0",
                             "move #>$400000,y0"] + ["nop"] * 8)

    def pair(kind, producer, body, gap, native, register, value):
        begin()
        producer_index = len(instructions)
        instructions.extend([producer] + ["nop"] * gap)
        body_index = len(instructions)
        instructions.extend([body] + ["nop"] * 8)
        cases.append(dict(kind=kind, gap=gap, native_producer=native,
                          producer_index=producer_index, body_index=body_index,
                          window_start=producer_index - 6, window_end=body_index + 5,
                          expected_register=register, expected_value=value))

    for kind, producer, native, value in (
            ("register", "move x0,a1", 1, 0x400000),
            ("long_immediate", "move #>$400000,a1", 2, 0x400000),
            ("long_memory", "move x:>$100,a1", 2, 0x13579b)):
        for gap in range(4):
            pair(kind, producer, "move a1,x1", gap, native, "x1", value)
    for kind, address in (("long_store", ">$80"), ("short_store", "<$20")):
        for gap in range(2):
            pair(kind, "mpy x0,y0,a", "move a1,x:" + address, gap, 1, "x0", 0x400000)
            body = len(instructions)
            instructions.extend(["move x:" + address + ",x1"] + ["nop"] * 8)
            cases.append(dict(kind="payload", gap=gap, body_index=body,
                              expected_register="x1", expected_value=0x200000))
    return dict(cases=cases, instructions=instructions)
