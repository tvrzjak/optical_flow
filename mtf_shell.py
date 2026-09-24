#!/usr/bin/env python3
"""Interactive "menu" shell for MTF-01P - used as the login shell of the
telnet account (see scripts/setup_telnet.sh and docs/MTF01P_manual.md,
ch. 12), so that after connecting you can type short commands right
away, e.g.

    forward 192.168.60.5:12345
    sensor 192.168.60.5:12345
    listen 12345

with no need for "cd" into the project directory or knowing the
script's name. Not a general-purpose shell (no pipes/redirects/
arbitrary command execution) - just a fixed set of subcommands mapped
to the project's tools.

All user-facing text here is plain English on purpose - a plain telnet
session (no proper UTF-8 negotiation) tends to mangle non-ASCII
characters.
"""
import os
import shlex
import subprocess
import sys
from pathlib import Path

DIR = Path(__file__).resolve().parent
PY = sys.executable or 'python3'
PROMPT = 'mtf> '

HELP = """\
Available commands (parameters from docs/MTF01P_manual.md ch. 8 can be
appended after the syntax below unchanged, e.g. "sensor 1.2.3.4:12345 --quality-min 40"):

  sensor <ip:port> [height_cm] [more parameters...]
      operational velocity streaming over UDP (sensor_node.py)
      without height_cm the initial height is estimated purely from the sensor

  forward [ip:port] [more parameters...]
      forwards raw Micolink frames over UDP (raw_forwarder.py)
      without ip:port the default broadcast 255.255.255.255:12346 is used

  listen <port> [more parameters...]
      test receiver for the resulting velocity over UDP (example_listener.py)

  calib [height_cm] [more parameters...]
      local calibration/diagnostics (calibration_tool.py --serial)
      WARNING: needs a graphical window (matplotlib) - DISPLAY only

  calib-remote <port> [more parameters...]
      calibration/diagnostics from raw_forwarder.py data

  help, ?     this help text
  exit, quit  log out

Stop a running tool with Ctrl+C (returns to the "mtf>" prompt).
"""


def _cmd_sensor(args):
    argv = [PY, str(DIR / 'sensor_node.py'), '--udp-target']
    if not args:
        print('usage: sensor <ip:port> [height_cm] [more parameters...]')
        return None
    argv.append(args[0])
    rest = args[1:]
    if rest and not rest[0].startswith('-'):
        argv += ['--height', rest[0]]
        rest = rest[1:]
    return argv + rest


def _cmd_forward(args):
    argv = [PY, str(DIR / 'raw_forwarder.py')]
    if args and not args[0].startswith('-'):
        argv += ['--udp-target', args[0]]
        args = args[1:]
    return argv + args


def _cmd_listen(args):
    if not args:
        print('usage: listen <port> [more parameters...]')
        return None
    return [PY, str(DIR / 'example_listener.py'), '--port', args[0]] + args[1:]


def _cmd_calib(args):
    if not os.environ.get('DISPLAY'):
        print('Warning: calib opens a graphical window (matplotlib) - '
              'without DISPLAY (X) set, this will not work over telnet.')
    argv = [PY, str(DIR / 'calibration_tool.py'), '--serial', '/dev/ttyAMA0']
    if args and not args[0].startswith('-'):
        argv += ['--height', args[0]]
        args = args[1:]
    return argv + args


def _cmd_calib_remote(args):
    if not args:
        print('usage: calib-remote <port> [more parameters...]')
        return None
    return [PY, str(DIR / 'calibration_tool.py'), '--udp-raw', args[0]] + args[1:]


COMMANDS = {
    'sensor': _cmd_sensor,
    'forward': _cmd_forward,
    'listen': _cmd_listen,
    'calib': _cmd_calib,
    'calib-remote': _cmd_calib_remote,
}


def run(argv):
    """Runs the tool in the foreground; Ctrl+C stops only it, not this menu."""
    proc = subprocess.Popen(argv, cwd=str(DIR))
    try:
        proc.wait()
    except KeyboardInterrupt:
        print()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.terminate()
            proc.wait()


def main():
    # Newly created files (calibration.json) should stay writable from the
    # other account in the same group too (project shared between "of" and
    # this telnet account) - see scripts/setup_telnet.sh.
    os.umask(0o002)

    print("MTF-01P - quick tool launcher. Type 'help' for help.")
    while True:
        try:
            line = input(PROMPT)
        except (EOFError, KeyboardInterrupt):
            print()
            break

        try:
            tokens = shlex.split(line)
        except ValueError as e:
            print(f'command error: {e}')
            continue
        if not tokens:
            continue

        cmd, args = tokens[0].lower(), tokens[1:]
        if cmd in ('exit', 'quit'):
            break
        if cmd in ('help', '?'):
            print(HELP)
            continue

        handler = COMMANDS.get(cmd)
        if handler is None:
            print(f"unknown command '{cmd}' - type 'help'")
            continue

        try:
            argv = handler(args)
        except Exception as e:  # bad parameters etc. - keep the menu alive
            print(f'error: {e}')
            continue
        if argv is None:
            continue

        try:
            run(argv)
        except FileNotFoundError as e:
            print(f'cannot run: {e}')

    print('Goodbye.')


if __name__ == '__main__':
    main()
