#!/usr/bin/env python3
"""Serial console for assemble_squro. Sends KEEP, never starts a move itself."""
import argparse
import datetime
from pathlib import Path
import select
import sys
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--log', type=Path, required=True)
    args = parser.parse_args()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    commands = {'STATUS', 'LEGS', 'CENTERHEAD', 'ALIGNFL', 'ALIGNFR', 'ALIGNBL', 'ALIGNBR', 'RELEASE', '!'}
    port = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=2)
    port.dtr = False
    port.rts = False
    port.port = args.port
    port.open()
    with args.log.open('a', buffering=1) as log:
        def emit(value):
            print(value, end='', flush=True)
            log.write(value)

        def send(command):
            if command != 'KEEP':
                stamp = datetime.datetime.now().isoformat(timespec='seconds')
                emit(f'\n[HOST {stamp}] Sending {command}\n')
            port.write((command + '\n').encode())
            port.flush()

        emit(f'\n[HOST] Connected to {args.port}. Automatic KEEP only.\n'
             'Commands: STATUS, LEGS, CENTERHEAD, ALIGNFL, ALIGNFR, ALIGNBL, ALIGNBR, RELEASE, !, EXIT\n')
        last_keep = time.monotonic()
        try:
            while True:
                data = port.read(8192)
                if data:
                    emit(data.decode(errors='replace'))
                if time.monotonic() - last_keep >= 1:
                    send('KEEP')
                    last_keep = time.monotonic()
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    line = sys.stdin.readline()
                    if not line or line.strip() == 'EXIT':
                        break
                    command = line.strip()
                    if command in commands:
                        send(command)
                    else:
                        emit('[HOST] Command rejected. No movement sent.\n')
        except KeyboardInterrupt:
            emit('\n[HOST] Interrupted.\n')
        finally:
            # If this process crashes or is killed, firmware KEEP timeout also
            # releases. On a normal exit, request release immediately.
            try:
                send('!')
                end = time.monotonic() + 2
                while time.monotonic() < end:
                    data = port.read(8192)
                    if data:
                        emit(data.decode(errors='replace'))
                    time.sleep(0.05)
            except serial.SerialException as error:
                emit(f'[HOST] Serial release could not be confirmed: {error}\n')
            finally:
                port.close()
                emit('[HOST] Console closed. Check release result above before handling the outputs.\n')


if __name__ == '__main__':
    main()
