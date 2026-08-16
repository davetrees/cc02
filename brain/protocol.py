"""CC-02 serial protocol v1 (FROZEN).
COBS-encoded frames delimited by 0x00.
Decoded frame = [type:u8][payload][crc16:u16 LE], CRC16-CCITT 0x1021 init 0xFFFF over type+payload.
"""
import struct

TELEM = 0x01    # ESP -> Pi ~100Hz
MODEREQ = 0x02  # ESP -> Pi
CMD = 0x10      # Pi -> ESP 50Hz
DISP = 0x11     # Pi -> ESP 10Hz

# seq, ax,ay,az, gx,gy,gz, roll,pitch, yaw_rate, servo_us, esc_us, vbus_mv, batt_mv, flags
TELEM_STRUCT = struct.Struct('<B9f2H2HB')   # 46 bytes
# steer_us, throttle_us, mode, failsafe_timeout_ms, flags, tip_roll_deg, tip_pitch_deg
CMD_STRUCT = struct.Struct('<2HBHB2B')      # 10 bytes
# wifi_clients, yolo_fps, det_count, flags, link_hz, ip (16s dotted quad NUL padded)
DISP_STRUCT = struct.Struct('<5B16s')       # 21 bytes

TELEM_FIELDS = ('seq', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'roll', 'pitch',
                'yaw_rate', 'servo_us', 'esc_us', 'vbus_mv', 'batt_mv', 'flags')

# TELEM flags bits
TF_SERIAL_OK = 1 << 0
TF_TIP_CUT = 1 << 1
TF_FAILSAFE = 1 << 2
TF_ESTOP = 1 << 3

# CMD flags bits
CF_ESTOP = 1 << 0
CF_FAILSAFE_STEER_CENTER = 1 << 1
CF_TIP_ENABLE = 1 << 2

# DISP flags bits (matches firmware protocol.h CC02_DF_*)
DF_COLLISION = 1 << 0
DF_CAL_REQUEST = 1 << 1   # ask ESP to re-zero the AH (LEVEL/ZERO button)

# modes
MODE_MANUAL, MODE_ASSIST, MODE_AUTO, MODE_RTH, MODE_ESTOP = 0, 1, 2, 3, 4
MODE_NAMES = {0: 'MANUAL', 1: 'ASSIST', 2: 'AUTO', 3: 'RTH', 4: 'ESTOP'}


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    code = 1
    code_idx = 0
    out.append(0)
    for b in data:
        if b == 0:
            out[code_idx] = code
            code = 1
            code_idx = len(out)
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code = 1
                code_idx = len(out)
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes):
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            return None
        i += 1
        end = i + code - 1
        if end > n:
            return None
        out.extend(data[i:end])
        i = end
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def build_frame(ftype: int, payload: bytes) -> bytes:
    """Return COBS-encoded frame with trailing 0x00 delimiter."""
    body = bytes([ftype]) + payload
    crc = crc16_ccitt(body)
    body += struct.pack('<H', crc)
    return cobs_encode(body) + b'\x00'


def parse_frame(chunk: bytes):
    """chunk = raw bytes between 0x00 delimiters. Returns (type, payload) or None."""
    if not chunk:
        return None
    body = cobs_decode(chunk)
    if body is None or len(body) < 3:
        return None
    crc_rx = struct.unpack('<H', body[-2:])[0]
    if crc16_ccitt(body[:-2]) != crc_rx:
        return None
    return body[0], body[1:-2]


def build_cmd(steer_us, throttle_us, mode, flags, tip_roll_deg, tip_pitch_deg,
              failsafe_timeout_ms=200):
    payload = CMD_STRUCT.pack(int(steer_us), int(throttle_us), int(mode) & 0xFF,
                              int(failsafe_timeout_ms), int(flags) & 0xFF,
                              int(tip_roll_deg) & 0xFF, int(tip_pitch_deg) & 0xFF)
    return build_frame(CMD, payload)


def build_disp(wifi_clients, yolo_fps, det_count, flags, link_hz, ip: str):
    ipb = ip.encode('ascii', 'ignore')[:15]
    payload = DISP_STRUCT.pack(min(int(wifi_clients), 255), min(int(yolo_fps), 255),
                               min(int(det_count), 255), int(flags) & 0xFF,
                               min(int(link_hz), 255), ipb)
    return build_frame(DISP, payload)


def parse_telem(payload: bytes):
    if len(payload) != TELEM_STRUCT.size:
        return None
    return dict(zip(TELEM_FIELDS, TELEM_STRUCT.unpack(payload)))
