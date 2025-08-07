#!/usr/bin/env python3
"""
serial_rs485_tool.py — RS-485 packet sniffer & sender with precise timing.
"""

import argparse
import time
import serial
from serial.rs485 import RS485Settings

class RS485Tool:
    def __init__(self, port: str, baud: int, rts_on_send: bool = True):
        """
        Initialize RS-485 port with automatic DE/RE control on RTS.
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=None
        )
        # Enable RS-485 mode: RTS high for send, low for receive
        self.ser.rs485_mode = RS485Settings(
            rts_level_for_tx= True,
            rts_level_for_rx= False,
            delay_before_tx= 0.00005,   # 50 µs
            delay_before_rx= 0.00005    # 50 µs
        )

    def capture(self, logfile: str = None):
        """
        Listen indefinitely, printing/logging each received chunk with ns timestamps.
        """
        ts_fn = time.perf_counter_ns
        last_ts = None
        log_f = open(logfile, 'a') if logfile else None

        print(f"▶ Capturing on {self.ser.port}@{self.ser.baudrate} …")
        try:
            while True:
                data = self.ser.read(self.ser.in_waiting or 1)
                if not data:
                    continue
                now = ts_fn()
                delta = now - last_ts if last_ts is not None else 0
                hexstr = data.hex().upper()
                line = f"[{now}] [+{delta}] {hexstr}"
                print(line)
                if log_f:
                    log_f.write(line + "\n")
                    log_f.flush()
                last_ts = now
        except KeyboardInterrupt:
            print("\n🛑 Capture stopped by user.")
        finally:
            if log_f: log_f.close()
            self.ser.close()

    def send_sequence(self, sequence: list, unit: str = "ms"):
        """
        Send a series of (delay, payload) entries.
        - delay: number (in ms or ns) to wait BEFORE sending.
        - payload: hex string (e.g. "7E7701FF") or ASCII.
        """
        factor = 1e-6 if unit == "ns" else 1e-3
        print(f"▶ Sending sequence on {self.ser.port}@{self.ser.baudrate} …")
        for delay, payload in sequence:
            if delay > 0:
                time.sleep(delay * factor)
            try:
                raw = bytes.fromhex(payload)
            except ValueError:
                raw = payload.encode("utf-8")
            self.ser.write(raw)
            ts = time.perf_counter_ns()
            print(f"[{ts}] → Sent {len(raw)} bytes: {payload}")
        self.ser.close()
        print("✅ All packets sent.")

def parse_sequence(items: list) -> list:
    seq = []
    for it in items:
        try:
            d, p = it.split(":", 1)
            seq.append((int(d), p))
        except ValueError:
            raise ValueError(f"Invalid sequence item '{it}'. Use DELAY:PAYLOAD format.")
    return seq

def main():
    parser = argparse.ArgumentParser(
        description="RS-485 sniffer/send tool (with auto DE/RE via RTS)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    cap = sub.add_parser("capture", help="Listen & timestamp incoming RS-485 data")
    cap.add_argument("-p","--port", required=True)
    cap.add_argument("-b","--baud", type=int, default=115200)
    cap.add_argument("-l","--log", help="File to append capture lines")

    snd = sub.add_parser("send", help="Send custom packet sequence over RS-485")
    snd.add_argument("-p","--port", required=True)
    snd.add_argument("-b","--baud", type=int, default=115200)
    snd.add_argument("-u","--unit", choices=["ms","ns"], default="ms",
                     help="Delay unit before each packet")
    snd.add_argument(
        "-s","--send", metavar="DELAY:PAYLOAD",
        action="append", required=True,
        help="e.g. 100:HELLO or 200:7E7701FF"
    )

    args = parser.parse_args()

    tool = RS485Tool(port=args.port, baud=args.baud)

    if args.cmd == "capture":
        tool.capture(logfile=args.log)

    elif args.cmd == "send":
        seq = parse_sequence(args.send)
        tool.send_sequence(sequence=seq, unit=args.unit)

if __name__ == "__main__":
    main()

