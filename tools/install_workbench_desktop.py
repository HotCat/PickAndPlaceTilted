"""Install menu and desktop shortcuts without root or a hardcoded checkout path."""
from pathlib import Path
import os
import shutil
import subprocess

REPOSITORY = Path(__file__).resolve().parents[1]


def desktop_quote(value):
    # Desktop Exec quoting is neither shell syntax nor JSON. Escape reserved
    # characters and literal percent field codes in a path as required by spec.
    value = str(value).replace('%', '%%')
    for char in ('\\', '"', '`', '$'):
        value = value.replace(char, '\\'+char)
    return '"'+value+'"'


def main():
    launcher = REPOSITORY/'launch-workbench.sh'
    menu = Path(os.environ.get('XDG_DATA_HOME', Path.home()/'.local/share'))/'applications/pickplace-workbench.desktop'
    content = ('[Desktop Entry]\nVersion=1.0\nType=Application\n'
               'Name=Pick & Place Workbench\nName[zh_CN]=取放工作台启动器\n'
               'Comment=Launch camera preview, FR3 studio and configured tools\n'
               f'Exec={desktop_quote(launcher)}\n'
               'Icon=applications-engineering\nTerminal=false\nCategories=Utility;\nStartupNotify=true\n')
    menu.parent.mkdir(parents=True, exist_ok=True)
    menu.write_text(content)
    menu.chmod(0o755)
    print('Installed menu shortcut:', menu)
    if shutil.which('xdg-user-dir'):
        result = subprocess.run(['xdg-user-dir', 'DESKTOP'], capture_output=True, text=True, check=True)
        directory = Path(result.stdout.strip())
        if directory.is_dir() and directory != Path.home():
            desktop = directory/menu.name
            desktop.write_text(content)
            desktop.chmod(0o755)
            if shutil.which('gio'):
                subprocess.run(['gio', 'set', str(desktop), 'metadata::trusted', 'true'], capture_output=True)
            print('Installed desktop shortcut:', desktop)


if __name__ == '__main__':
    main()
