# SPDX-License-Identifier: GPL-3.0-only
"""Inspect every native archive member, including LLVM ThinLTO bitcode."""

from pathlib import Path
import os
import re
import subprocess
import tempfile


def require(condition, message):
    if not condition:
        raise ValueError(message)


def archive_members(path):
    """Read regular GNU/BSD ar members by position; duplicate names remain distinct."""
    names = b''
    with path.open('rb') as source:
        require(source.read(8) == b'!<arch>\n', f'{path}: expected a regular native archive')
        archive_size = os.fstat(source.fileno()).st_size
        while header := source.read(60):
            require(len(header) == 60 and header[-2:] == b'`\n',
                    f'{path}: invalid archive member header')
            size_field = header[48:58].strip()
            require(size_field.isdigit(), f'{path}: invalid archive member size')
            size = int(size_field)
            require(size <= archive_size - source.tell(), f'{path}: truncated archive member')
            data = source.read(size)
            require(len(data) == size, f'{path}: truncated archive member')
            if size & 1:
                require(source.read(1) == b'\n', f'{path}: invalid archive padding')
            name = header[:16].rstrip()
            if name == b'//':
                names = data
                continue
            if name in (b'/', b'/SYM64/', b'__.SYMDEF', b'__.SYMDEF SORTED'):
                continue
            if name.startswith(b'#1/'):
                length = name[3:]
                require(length.isdigit() and int(length) <= len(data),
                        f'{path}: invalid BSD archive name')
                name, data = data[:int(length)].rstrip(b'\0'), data[int(length):]
                if name in (b'__.SYMDEF', b'__.SYMDEF SORTED'):
                    continue
            elif name.startswith(b'/'):
                offset = name[1:]
                require(offset.isdigit() and int(offset) < len(names),
                        f'{path}: invalid GNU archive name offset')
                end = names.find(b'/\n', int(offset))
                require(end >= 0, f'{path}: unterminated GNU archive name')
                name = names[int(offset):end]
            else:
                name = name.removesuffix(b'/')
            require(name and data, f'{path}: empty native member')
            yield name.decode('utf-8', errors='replace'), data


def bitcode_control_flow(text, label):
    flags = re.search(r'^!llvm\.module\.flags\s*=\s*!\{([^}]*)\}', text, re.MULTILINE)
    require(flags, f'{label}: missing LLVM module CF properties')
    identifiers = set(re.findall(r'!(\d+)', flags[1]))
    for feature in ('return', 'branch'):
        records = re.findall(
            r'^!(\d+)\s*=\s*!\{i32 \d+, !"cf-protection-' + feature + r'", i32 (\d+)\}',
            text, re.MULTILINE)
        values = [value for identifier, value in records if identifier in identifiers]
        require(values == ['1'], f'{label}: missing full compiler CF {feature} property')


def check_archive_control_flow(path, readelf, llvm_dis):
    count = 0
    with tempfile.TemporaryDirectory(prefix='blofeld-archive-hardening-') as temporary:
        member = Path(temporary) / 'member.o'
        for count, (name, data) in enumerate(archive_members(path), 1):
            label = f'{path}: member {count} ({name})'
            member.write_bytes(data)
            if data.startswith((b'BC\xc0\xde', b'\xde\xc0\x17\x0b')):
                command = [llvm_dis, str(member), '-o', '-']
            else:
                require(data.startswith(b'\x7fELF'), f'{label}: unsupported native object format')
                command = [readelf, '--notes', str(member)]
            result = subprocess.run(command, capture_output=True, text=True, check=True, timeout=30)
            if data.startswith(b'\x7fELF'):
                require(re.search(r'x86 feature:.*\bIBT\b.*\bSHSTK\b', result.stdout),
                        f'{label}: missing full compiler CF properties')
            else:
                bitcode_control_flow(result.stdout, label)
    require(count, f'{path}: native archive has no object members')
    return count
