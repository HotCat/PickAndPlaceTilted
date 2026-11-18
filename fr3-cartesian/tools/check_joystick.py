"""Read-only joystick diagnostic. Never imports the robot SDK or sends motion."""
from dataclasses import asdict
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fr3_cartesian.joystick import LinuxJoystick, devices

for device in devices():
    try:
        reader = LinuxJoystick(device)
        axes, pressed = reader.snapshot()
        print(json.dumps(dict(device=asdict(device), ranges=reader.ranges,
                              buttons=reader.buttons, axes=axes, pressed=sorted(pressed)), ensure_ascii=False, indent=2))
        reader.close()
    except (OSError, ValueError) as error:
        print(f'{device.path}: {error}', file=sys.stderr)
