import sys, time, binascii
from datetime import datetime
import serial

# ================= USER SETTINGS =================
PORT = "COM3"              # e.g., "COM5" on Windows, "/dev/ttyUSB0" on Linux
BAUD = 19200               # match the controller/TSCView
BYTESIZE = serial.EIGHTBITS
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE

HEX_FRAME = "7E 77 01 00 01 00 40 49 FF ED 01 B4 A8 96 22 0A"  # CHANGE ME
INTER_BYTE_US = 0          # microseconds between bytes (try 0, 200, 500, 1000…)
BUS_IDLE_MS = 60           # how long the line must be quiet before we transmit
TURNAROUND_WAIT_MS = 2     # wait after last byte before switching to RX (if RTS used)
RESPONSE_TIMEOUT_MS = 120  # how long to wait for a reply after we send
USE_RTS_FOR_DE = False     # True if your adapter drives RS-485 DE via RTS
RTS_POLARITY_ACTIVE = True # If True: RTS=True = DE enabled. Flip if your adapter is opposite.
# ==================================================

def hexstr_to_bytes(s):
    s = s.replace("0x","").replace(","," ").replace("\n"," ").replace("\t"," ")
    parts = [p for p in s.split() if p]
    try:
        return bytes(int(p,16) for p in parts)
    except ValueError as e:
        print("Bad hex input:", e)
        sys.exit(1)

def now():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def bus_idle(ser, idle_ms):
    """Return True if no bytes arrive for idle_ms."""
    ser.timeout = idle_ms/1000.0
    data = ser.read(1)
    if data:
        # We consumed one byte; print it and flush rest quickly
        print(f"[{now()}] BUS ACTIVITY (idle check) RX: {data.hex().upper()}")
        # slurp rest of immediate buffer
        ser.timeout = 0.002
        more = ser.read(4096)
        if more:
            print(f"[{now()}] (burst) {more.hex().upper()}")
        return False
    return True

def wait_for_quiet_gap(ser, idle_ms):
    """Block until we see an idle window of idle_ms."""
    start = time.time()
    while True:
        if bus_idle(ser, idle_ms):
            return
        # brief guard to avoid busy spin
        time.sleep(0.005)
        # optional: give up after N seconds
        if time.time() - start > 30:
            print(f"[{now()}] Could not find a quiet gap within 30s; still trying…")
            start = time.time()

def set_de(ser, enable):
    """If using RTS to control DE, set it here."""
    if not USE_RTS_FOR_DE:
        return
    ser.rts = (enable == RTS_POLARITY_ACTIVE)

def send_with_interbyte_delay(ser, frame: bytes, inter_byte_us: int):
    for i, b in enumerate(frame):
        ser.write(bytes([b]))
        ser.flush()  # push immediately
        if inter_byte_us > 0 and i != len(frame)-1:
            time.sleep(inter_byte_us/1_000_000.0)

def read_response(ser, timeout_ms):
    ser.timeout = timeout_ms/1000.0
    buf = bytearray()
    # read some; extend if continuous bytes keep coming with short gaps
    first = ser.read(1)
    if not first:
        return bytes()
    buf += first

    # after first byte, keep grabbing rapidly until a small gap occurs
    ser.timeout = 0.008  # 8 ms inter-byte window
    while True:
        chunk = ser.read(4096)
        if not chunk:
            break
        buf += chunk
    return bytes(buf)

def main():
    frame = hexstr_to_bytes(HEX_FRAME)
    print(f"[{now()}] Opening {PORT} @ {BAUD} 8N1")
    with serial.Serial(PORT, BAUD, bytesize=BYTESIZE, parity=PARITY, stopbits=STOPBITS) as ser:
        # Prepare control lines
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        if USE_RTS_FOR_DE:
            # Ensure we're in RX mode initially
            set_de(ser, False)

        print(f"[{now()}] Waiting for {BUS_IDLE_MS} ms bus idle before transmit…")
        wait_for_quiet_gap(ser, BUS_IDLE_MS)

        # Collision check one more time right before send
        ser.timeout = 0.0
        if ser.read(1):
            print(f"[{now()}] Activity detected at send time; aborting this attempt.")
            return

        # Transmit
        print(f"[{now()}] TX: {frame.hex(' ').upper()}")
        if USE_RTS_FOR_DE:
            set_de(ser, True)
            time.sleep(0.001)  # guard
        send_with_interbyte_delay(ser, frame, INTER_BYTE_US)
        if USE_RTS_FOR_DE:
            time.sleep(TURNAROUND_WAIT_MS/1000.0)
            set_de(ser, False)

        # Listen for reply
        resp = read_response(ser, RESPONSE_TIMEOUT_MS)
        if resp:
            print(f"[{now()}] RX ({len(resp)} B): {resp.hex(' ').upper()}")
        else:
            print(f"[{now()}] RX: <no response within {RESPONSE_TIMEOUT_MS} ms>")

if __name__ == "__main__":
    main()


