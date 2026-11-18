"""Download a pinned official FAIRINO SDK without modifying its source."""
import hashlib
from pathlib import Path
import urllib.request

COMMIT = 'f2d762e37f0e9c18d63340eba5972bdb81159276'
SHA256 = '9aa5c2b3109fd9aeb6f40974ef69388b59ab44a02fe4f3478ca53fec14768275'
URL = f'https://raw.githubusercontent.com/FAIR-INNOVATION/fairino-python-sdk/{COMMIT}/linux/fairino/Robot.py'


def main():
    target = Path(__file__).resolve().parents[1] / 'vendor' / 'fairino'
    data = urllib.request.urlopen(URL, timeout=45).read()
    if hashlib.sha256(data).hexdigest() != SHA256:
        raise RuntimeError('SDK checksum mismatch')
    target.mkdir(parents=True, exist_ok=True)
    previous = target / 'Robot.py'
    if previous.exists() and previous.read_bytes() != data:
        (target / 'Robot.py.previous').write_bytes(previous.read_bytes())
    (target / 'Robot.py').write_bytes(data)
    (target / '__init__.py').write_text('', encoding='utf-8')
    print(f'Installed official SDK v2.1.5 (controller release 3.8.5) at {target}')
    print('Restart all running app windows to load the updated SDK (SDK folder: vendor).')


if __name__ == '__main__':
    main()
