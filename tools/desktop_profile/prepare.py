# SPDX-License-Identifier: GPL-3.0-only
"""Prepare a local Desktop profile from the three original Waldorf downloads."""

import argparse
import fcntl
import json
from pathlib import Path
import platform
import subprocess
import tarfile
from tempfile import NamedTemporaryFile, TemporaryDirectory
from urllib.request import urlopen
from zipfile import BadZipFile

from bootstrap import build_boot_rom
import image

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'profiles/desktop'
MCU = 'mcu/mcu-catalogue.json'
DSP = 'dsp/dsp-catalogue.json'
MCU_HELPERS = {'main-initial': [(0, 0)], 'main-error': [(2, 8454), (4, 89448)],
               'main-flash': [(4, 0)]}
MEDIA = ('device.bfnvm', 'samples.bfnand')
RECEIPT = 'inputs.json'
MAX_INPUT = 64 * 1024 * 1024


def read_input(path):
    with path.open('rb') as stream:
        data = stream.read(MAX_INPUT + 1)
    image.require(len(data) <= MAX_INPUT, f'Input exceeds 64 MiB: {path}')
    return data


def verify(data, sha, name):
    image.require(image.digest(data) == sha, f'Unexpected {name} SHA-256')


def download(cache, name, url, sha):
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / name
    if path.exists():
        verify(read_input(path), sha, name)
        return path
    print(f'Downloading {name}', flush=True)
    with urlopen(url, timeout=30) as response:
        data = response.read(MAX_INPUT + 1)
    image.require(len(data) <= MAX_INPUT, f'Oversized download: {name}')
    verify(data, sha, name)
    with NamedTemporaryFile(dir=cache, delete=False) as stream:
        temporary = Path(stream.name)
    try:
        temporary.write_bytes(data)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    return path


def boot_rom(cache, pdftotext, staging):
    hardware = json.loads((ROOT / 'doc/hardware/sources.json').read_bytes())
    reference = next(row for row in hardware if row['file'] == 'DSP56371UM.pdf')
    dependencies = json.loads((ROOT / 'cmake/dependencies.json').read_bytes())
    tool = dependencies['dsp56300_assembly_tool']
    host = f'{platform.system()}-{platform.machine().lower()}'
    asset = tool['platforms'].get(host)
    image.require(asset is not None, f'No pinned DSP assembler for {host}')
    package = f"dsp56300-{tool['version']}-{asset['target']}"
    archive = download(cache, package + '.tar.gz',
                       f"{tool['source']}/releases/download/v{tool['version']}/{package}.tar.gz",
                       asset['sha256'])
    manual = download(cache, reference['file'], reference['source'], reference['sha256'])
    assembler = staging / 'dsp56300-asm'
    with tarfile.open(archive, 'r:gz') as tar:
        member = tar.getmember(package + '/bin/dsp56300-asm')
        image.require(member.isfile() and 0 < member.size <= MAX_INPUT,
                      'Invalid DSP assembler archive member')
        with tar.extractfile(member) as stream:
            assembler.write_bytes(stream.read())
    assembler.chmod(0o700)
    return build_boot_rom(manual, assembler, pdftotext, staging / 'bootstrap')


def mcu_payloads(catalogue, mcu, overlay, blocks):
    verify(mcu, catalogue['inputs']['mcu'], 'MCU flash')
    sources = {f'ui{page}': overlay[page * image.PAGE_BYTES:(page + 1) * image.PAGE_BYTES]
               for page in range(16)}
    for helper, positions in MCU_HELPERS.items():
        copies = []
        for block, offset in positions:
            data = blocks[block]
            size = int.from_bytes(data[offset:offset + 2], 'big')
            copies.append(data[offset + 2:offset + 2 + size])
        image.require(all(copy == copies[0] for copy in copies[1:]),
                      f'Mismatched firmware copies of {helper}')
        sources[helper] = copies[0]
    for entry in catalogue['inputs']['blobs']:
        data = sources[entry['name']]
        verify(data, entry['sha256'], entry['name'])
        yield 'mcu/' + entry['path'], data


def dsp_records(data):
    offset = 0
    while offset + 9 <= len(data):
        space, count, address = image.words(data[offset:offset + 9])
        offset += 9
        if (space, count, address) == (0x100,) * 3:
            return
        end = offset + count * 3
        image.require(space <= 2 and 0 < count <= 0x1000000 - address and end <= len(data),
                      'Invalid DSP memory record')
        yield space, address, data[offset:end]
        offset = end
    raise ValueError('Missing DSP memory record terminator')


def dsp_view(writes, boot, records, blocks):
    words = {}
    for write in writes:
        kind = write['kind']
        if kind == 'records':
            count = write['count']
            image.require(count <= len(records), 'DSP view exceeds firmware record count')
            for space, address, data in records[:count]:
                if space == 0:
                    words.update(enumerate(image.words(data), address))
            continue
        start = write['address']
        if kind in boot:
            offset = 3 * write['word_offset']
            data = boot[kind]
        elif kind == 'block':
            offset = write['byte_offset']
            data = blocks[write['block']]
        elif kind == 'copy':
            source = write['from']
            copied = [words[source + index] for index in range(write['words'])]
            words.update(enumerate(copied, start))
            continue
        elif kind == 'field':
            source = words[write['from']]
            words[start] = ((source & write['mask']) << write['shift']) & 0xffffff
            continue
        else:
            raise ValueError(f'Unknown DSP write kind {kind}')
        size = 3 * write['words']
        image.require(offset + size <= len(data), 'DSP view exceeds source extent')
        words.update(enumerate(image.words(data[offset:offset + size]), start))
    return words


def dsp_catalogue(catalogue, rom, runtime, blocks):
    identities = catalogue['inputs']
    for index, (block, expected) in enumerate(zip(blocks, identities['blocks'], strict=True)):
        verify(block, expected, f'firmware block {index}')
    verify(runtime, identities['nand'], 'DSP download stream')
    boot = {'boot-main': rom[:576], 'boot-eeprom': rom[576:]}
    for segment, data in boot.items():
        verify(data, identities[segment], segment)
    records = list(dsp_records(runtime))
    for variant in catalogue['variants']:
        words = dsp_view(variant['writes'], boot, records, blocks)
        for guard in variant['guards']:
            if 'value' not in guard:
                guard['value'] = words[guard['address']]
    return (json.dumps(catalogue, separators=(',', ':')) + '\n').encode()


def factory_files(downloads, templates, cache, pdftotext, staging):
    midi = image.input_midi(downloads['firmware.zip'], 'firmware')
    sounds = image.input_midi(downloads['sounds.zip'], 'sounds')
    samples = image.input_midi(downloads['samples.zip'], 'samples')
    blocks = list(image.download_blocks(midi))
    mcu, runtime, overlay = image.firmware_parts(blocks)
    blocks = [block.data[:block.declared] for block in blocks]
    rom = boot_rom(cache, pdftotext, staging)
    files = templates | {'firmware.zip': downloads['firmware.zip'], 'boot-rom.bin': rom,
                         DSP: dsp_catalogue(json.loads(templates[DSP]), rom, runtime, blocks)}
    files.update(mcu_payloads(json.loads(templates[MCU]), mcu, overlay, blocks))
    pages = image.assemble_pages(runtime, overlay, image.sound_pages(sounds), image.sample_pages(samples))
    return files, mcu, pages


def prepare(download_dir, output, cache, pdftotext):
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.with_suffix('.lock').open('ab') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        downloads = {name + '.zip': read_input(download_dir / (name + '.zip'))
                     for name in ('firmware', 'sounds', 'samples')}
        templates = {name: (PROFILE / name).read_bytes() for name in ('desktop.profile', MCU, DSP)}
        identities = {name: image.digest(data) for name, data in (downloads | templates).items()}
        if output.exists():
            receipt = json.loads((output / RECEIPT).read_bytes())
            image.require(receipt.get('inputs') == identities,
                          f'Inputs differ from the existing profile in {output}. '
                          'Use a separate build directory for a fresh profile.')
            for name, sha in receipt['files'].items():
                verify(read_input(output / name), sha, name)
            for name in MEDIA:
                image.require((output / name).is_file(), f'Missing saved storage: {output / name}')
            return
        with TemporaryDirectory(prefix='.desktop-', dir=output.parent) as directory:
            staging = Path(directory)
            files, mcu, pages = factory_files(downloads, templates, cache, pdftotext, staging)
            profile = staging / 'profile'
            for name, data in files.items():
                file = profile / name
                file.parent.mkdir(parents=True, exist_ok=True)
                file.write_bytes(data)
            (profile / MEDIA[0]).write_bytes(image.nvm(mcu, 'EMUBN44AJRB5'))
            image.write_nand(profile / MEDIA[1], pages)
            receipt = {'inputs': identities,
                       'files': {name: image.digest(data) for name, data in files.items()}}
            (profile / RECEIPT).write_text(json.dumps(receipt, indent=2) + '\n')
            profile.rename(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('downloads', 'output', 'cache', 'pdftotext'):
        parser.add_argument(f'--{name}', type=Path, required=True)
    args = parser.parse_args()
    try:
        prepare(args.downloads, args.output, args.cache, args.pdftotext)
    except (BadZipFile, KeyError, OSError, ValueError, subprocess.SubprocessError,
            tarfile.TarError) as error:
        parser.exit(1, f'Desktop profile preparation failed: {error}\n')
    print(args.output / 'desktop.profile')


if __name__ == '__main__':
    main()
