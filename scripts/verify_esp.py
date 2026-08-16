#!/usr/bin/env python3
"""Bench-verify CC-02 ESP firmware over USB CDC (Mac side)."""
import struct, sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem101"
TELEM = struct.Struct("<B9f2H2HB")
CMD = struct.Struct("<2HBHB2B")

def crc16(d):
    crc = 0xFFFF
    for b in d:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def cobs_enc(data):
    out = bytearray(); idx = 0
    while True:
        block = data[idx:idx+254]
        z = block.find(0)
        if z < 0:
            out.append(len(block)+1); out += block
            if len(block) < 254: break
            idx += 254
        else:
            out.append(z+1); out += block[:z]; idx += z+1
            if idx > len(data): break
        if idx == len(data): out.append(1); break
    return bytes(out)

def cobs_dec(data):
    out = bytearray(); i = 0
    while i < len(data):
        c = data[i]
        if c == 0 or i + c > len(data): return b""
        out += data[i+1:i+c]; i += c
        if c != 0xFF and i < len(data): out.append(0)
    return bytes(out)

def frame(ftype, payload):
    raw = bytes([ftype]) + payload
    c = crc16(raw)
    return cobs_enc(raw + struct.pack("<H", c)) + b"\x00"

def parse(buf, stats):
    for chunk in buf.split(b"\x00"):
        if len(chunk) < 4: continue
        dec = cobs_dec(chunk)
        if len(dec) < 3: continue
        if crc16(dec[:-2]) != struct.unpack("<H", dec[-2:])[0]:
            stats["badcrc"] += 1; continue
        if dec[0] == 0x01 and len(dec) == 1 + TELEM.size + 2:
            stats["telem"] += 1
            stats["last"] = TELEM.unpack(dec[1:-2])

s = serial.Serial(PORT, 921600, timeout=0.2)
stats = {"telem": 0, "badcrc": 0, "last": None}

# Phase 1: passive listen 2s — expect TELEM @100Hz with FAILSAFE flag set
t0 = time.time(); buf = b""
while time.time() - t0 < 2.0:
    buf += s.read(4096)
parse(buf, stats)
t = stats["last"]
print(f"P1 telem_frames={stats['telem']} badcrc={stats['badcrc']}")
if t:
    print(f"P1 seq={t[0]} roll={t[7]*57.3:.1f} pitch={t[8]*57.3:.1f} servo={t[10]} esc={t[11]} vbus={t[12]} batt={t[13]} flags=0x{t[14]:02x} (expect FAILSAFE bit2=1, servo/esc=1500)")

# Phase 2: send CMD steer=1600 throttle=1500 @50Hz for 1.5s — expect servo_us=1600, failsafe clear
cmd = CMD.pack(1600, 1500, 0, 200, 0b0110, 45, 45)  # steer_center+tip_enable
stats2 = {"telem": 0, "badcrc": 0, "last": None}
t0 = time.time(); buf = b""
while time.time() - t0 < 1.5:
    s.write(frame(0x10, cmd))
    buf += s.read(1024)
    time.sleep(0.02)
parse(buf, stats2)
t = stats2["last"]
print(f"P2 telem_frames={stats2['telem']}")
if t:
    print(f"P2 servo={t[10]} esc={t[11]} flags=0x{t[14]:02x} (expect servo=1600, flags bit0=1 serial_ok, bit2=0)")

# Phase 3: stop sending, wait 0.6s — failsafe must re-engage, servo recenters
time.sleep(0.6)
s.reset_input_buffer()
buf = s.read(2000); time.sleep(0.3); buf += s.read(4096)
stats3 = {"telem": 0, "badcrc": 0, "last": None}
parse(buf, stats3)
t = stats3["last"]
if t:
    print(f"P3 servo={t[10]} esc={t[11]} flags=0x{t[14]:02x} (expect 1500/1500, FAILSAFE bit2=1)")
s.close()
