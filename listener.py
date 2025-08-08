import time
from datetime import datetime
import serial

# ===== USER SETTINGS =====
PORT = "COM5"        # Change to your COM port
BAUD = 19200         # Baud rate (match your device)
FRAME_GAP_MS = 3.0   # Silence threshold to start a new frame
# =========================

def now_time():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def hexs(b: bytes) -> str:
    return b.hex(" ").upper()

# Open serial port
ser = serial.Serial(
    PORT, BAUD,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.0
)

print(f"[{now_time()}] Listening on {PORT} @ {BAUD} 8N1")
print("ABS_TIME          idx  BYTE  Δprev_us  since_frame_us")

frames = []
cur_data = bytearray()
cur_times = []
last_byte_ns = None
byte_index = 0
FRAME_GAP_NS = int(FRAME_GAP_MS * 1_000_000)

def close_frame():
    global cur_data, cur_times
    if cur_data:
        frames.append({
            "t0_ns": cur_times[0],
            "t1_ns": cur_times[-1],
            "data": bytes(cur_data),
            "times": list(cur_times)
        })
        cur_data.clear()
        cur_times.clear()

while True:
    b = ser.read(1)
    if b:
        t_ns = time.perf_counter_ns()

        # Check if gap between bytes means a new frame
        if last_byte_ns is not None and (t_ns - last_byte_ns) >= FRAME_GAP_NS and cur_data:
            close_frame()

        cur_data.extend(b)
        cur_times.append(t_ns)

        dprev_us = 0.0 if last_byte_ns is None else (t_ns - last_byte_ns) / 1000.0
        since_frame_us = 0.0 if len(cur_times) == 1 else (t_ns - cur_times[0]) / 1000.0
        print(f"{now_time():<16}  {byte_index:5d}  {b.hex().upper():>2}   {dprev_us:8.1f}      {since_frame_us:11.1f}")

        last_byte_ns = t_ns
        byte_index += 1
    else:
        time.sleep(0.0002)  # Small pause to reduce CPU load
