#!/usr/bin/env python3
import serial
import time
import argparse
import sys

def sniff(port: str,
          baud: int,
          output: str,
          by_ns: bool = True):
    """
    Listen on `port` at `baud`, dump each received packet as hex + timestamp to `output`.

    Args:
      port: COM port (e.g. 'COM3' or '/dev/ttyUSB0')
      baud: Baud rate (e.g. 9600, 115200)
      output: Path to output logfile
      by_ns: If True timestamp in nanoseconds, else in milliseconds
    """
    ts_fn = time.perf_counter_ns if by_ns else lambda: int(time.perf_counter() * 1e3)
    with serial.Serial(port, baud, timeout=None) as ser, open(output, 'w') as f:
        print(f"Sniffing on {port}@{baud}, logging to {output}")
        try:
            while True:
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                ts = ts_fn()
                # write comma-separated: timestamp,hexdata
                hexstr = data.hex().upper()
                f.write(f"{ts},{hexstr}\n")
                f.flush()
        except KeyboardInterrupt:
            print("\nStopping sniff.")

def replay(port: str,
           baud: int,
           input_file: str,
           delay_unit: str = "ms"):
    """
    Read `input_file` of timestamp,hexdata lines and replay on serial port,
    sleeping the original intervals between packets.

    Args:
      port: COM port for sending
      baud: Baud rate
      input_file: Path to sniffed logfile
      delay_unit: "ms" or "ns" — decides how to interpret timestamps
    """
    factor = 1e-6 if delay_unit == "ns" else 1e-3  # ns→s or ms→s
    with serial.Serial(port, baud) as ser, open(input_file, 'r') as f:
        print(f"Replaying on {port}@{baud} from {input_file}")
        prev_ts = None
        for line in f:
            line = line.strip()
            if not line:
                continue
            ts_str, hexstr = line.split(",", 1)
            ts = int(ts_str)
            raw = bytes.fromhex(hexstr)
            if prev_ts is not None:
                dt = (ts - prev_ts) * factor
                if dt > 0:
                    time.sleep(dt)
            ser.write(raw)
            prev_ts = ts
        print("Replay complete.")

def main():
    parser = argparse.ArgumentParser(
        description="Serial sniffer & replayer")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_sniff = sub.add_parser("sniff", help="Capture incoming serial data")
    p_sniff.add_argument("--port",  "-p", required=True)
    p_sniff.add_argument("--baud",  "-b", type=int, default=115200)
    p_sniff.add_argument("--out",   "-o", required=True,
                         help="Output file for logs")
    p_sniff.add_argument("--ns",    action="store_true",
                         help="Timestamp in nanoseconds (default)")

    p_rep = sub.add_parser("replay", help="Replay logged data with delays")
    p_rep.add_argument("--port",  "-p", required=True)
    p_rep.add_argument("--baud",  "-b", type=int, default=115200)
    p_rep.add_argument("--in",    "-i", dest="input",
                         required=True, help="Input log file")
    p_rep.add_argument("--unit",  choices=["ms","ns"], default="ms",
                         help="Timestamp unit used in log")

    args = parser.parse_args()

    if args.cmd == "sniff":
        sniff(args.port, args.baud, args.out, by_ns=args.ns)
    elif args.cmd == "replay":
        replay(args.port, args.baud, args.input, delay_unit=args.unit)
    else:
        parser.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()


