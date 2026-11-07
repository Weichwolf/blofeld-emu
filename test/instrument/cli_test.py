# SPDX-License-Identifier: GPL-3.0-only
"""Check the profile-only instrument entry point without running firmware."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def invoke(binary, arguments, environment=None):
    return subprocess.run([binary, *arguments], env=environment, capture_output=True,
                          text=True, timeout=3, check=False)


def main():
    binary = sys.argv[1]
    help_result = invoke(binary, ['--help'])
    assert help_result.returncode == 0 and '--profile' in help_result.stdout
    for option in ('--instructions', '--reference-board', '--sla', '--port-e-input', '--midi-port'):
        result = invoke(binary, [option, 'unused'])
        assert result.returncode == 1 and 'Usage:' in result.stderr, result
    with tempfile.TemporaryDirectory(prefix='blofeld-profile-') as directory:
        root = Path(directory)
        profile = root / 'input.profile'
        for text, message in (
            ('clock 12000000\n', 'Unknown instrument profile field'),
            ('firmware a\nfirmware b\n', 'duplicate'),
            ('firmware\n', 'Invalid'),
            ('firmware "a" trailing\n', 'Invalid'),
            ('firmware a\n', 'requires'),
            ('firmware "a\x00b"\n', 'NUL'),
        ):
            profile.write_text(text)
            result = invoke(binary, ['--profile', str(profile)])
            assert result.returncode == 1 and message in result.stderr, result
        config = root / 'blofeld-emu'
        config.mkdir()
        (config / 'default.profile').write_text('clock 12000000\n')
        environment = dict(os.environ, XDG_CONFIG_HOME=str(root))
        result = invoke(binary, [], environment)
        assert result.returncode == 1 and 'Unknown instrument profile field' in result.stderr, result
        assert not list(root.rglob('*.bfnvm')) and not list(root.rglob('*.bfnand'))


if __name__ == '__main__':
    main()
