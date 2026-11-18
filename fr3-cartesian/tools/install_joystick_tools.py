"""Fetch Ubuntu diagnostic utilities locally, without sudo or system changes."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ['jstest-gtk', 'joystick', 'evtest', 'libatkmm-1.6-1v5',
            'libcairomm-1.0-1v5', 'libglibmm-2.4-1t64', 'libgtkmm-3.0-1t64',
            'libsigc++-2.0-0v5', 'libpangomm-1.4-1v5']


def main():
    base = ROOT/'.tools/linux-joystick'
    packages, target = base/'packages', base/'root'
    packages.mkdir(parents=True, exist_ok=True)
    target.mkdir(parents=True, exist_ok=True)
    subprocess.run(['apt-get', 'download', *PACKAGES], cwd=packages, check=True)
    for package in sorted(packages.glob('*.deb')):
        subprocess.run(['dpkg-deb', '-x', str(package), str(target)], check=True)
    print(f'Run {ROOT}/tools/joystick-tools jstest-gtk')


if __name__ == '__main__':
    main()
