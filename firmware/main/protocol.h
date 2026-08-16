/*
 * CC-02 USB CDC serial protocol — FROZEN 2026-08-16 v1
 * Framing: COBS-encoded frames delimited by 0x00.
 * Decoded frame = [type u8][payload][crc16 u16 LE]
 * CRC16-CCITT (poly 0x1021, init 0xFFFF) over type+payload.
 * All multi-byte fields little-endian, structs packed.
 * ESP32-S3 native USB-Serial-JTAG CDC; baud value irrelevant.
 */
#pragma once
#include <stdint.h>

#define CC02_PROTO_VERSION 1

/* Frame types */
#define CC02_T_TELEM   0x01  /* ESP -> Pi @100Hz */
#define CC02_T_MODEREQ 0x02  /* ESP -> Pi on touch event */
#define CC02_T_CMD     0x10  /* Pi -> ESP @50Hz */
#define CC02_T_DISP    0x11  /* Pi -> ESP @10Hz */

/* Modes */
enum { CC02_MODE_MANUAL = 0, CC02_MODE_ASSIST = 1, CC02_MODE_AUTO = 2,
       CC02_MODE_RTH = 3, CC02_MODE_ESTOP = 4 };

/* TELEM flags */
#define CC02_F_SERIAL_OK 0x01
#define CC02_F_TIP_CUT   0x02
#define CC02_F_FAILSAFE  0x04
#define CC02_F_ESTOP     0x08

/* CMD flags */
#define CC02_CF_ESTOP        0x01
#define CC02_CF_STEER_CENTER 0x02  /* failsafe steer: 1=center, 0=hold */
#define CC02_CF_TIP_ENABLE   0x04

/* DISP flags */
#define CC02_DF_COLLISION   0x01
#define CC02_DF_CAL_REQUEST 0x02  /* capture level/gyro-zero reference now */

/* MODEREQ requests */
enum { CC02_REQ_MODE_CYCLE = 1, CC02_REQ_RTH = 2,
       CC02_REQ_ESTOP_ON = 3, CC02_REQ_ESTOP_OFF = 4 };

#pragma pack(push, 1)
typedef struct {            /* 46 bytes; python: <B9f2H2HB */
    uint8_t  seq;
    float    ax, ay, az;    /* m/s^2 */
    float    gx, gy, gz;    /* rad/s */
    float    roll, pitch;   /* rad, complementary filter */
    float    yaw_rate;      /* rad/s */
    uint16_t servo_us, esc_us;   /* actually output now */
    uint16_t vbus_mv, batt_mv;   /* from AXP2101, 0 if unavailable */
    uint8_t  flags;         /* CC02_F_* */
} cc02_telem_t;

typedef struct {            /* 10 bytes; python: <2HBHB2B */
    uint16_t steer_us, throttle_us;  /* 1000..2000 */
    uint8_t  mode;
    uint16_t failsafe_timeout_ms;    /* default 200 */
    uint8_t  flags;         /* CC02_CF_* */
    uint8_t  tip_roll_deg;  /* default 45 */
    uint8_t  tip_pitch_deg; /* default 45 */
} cc02_cmd_t;

typedef struct {            /* 21 bytes; python: <5B16s */
    uint8_t  wifi_clients;
    uint8_t  yolo_fps;
    uint8_t  det_count;
    uint8_t  flags;         /* CC02_DF_* */
    uint8_t  link_hz;       /* CMD rate the Pi thinks it sends */
    char     ip[16];        /* dotted quad, NUL padded */
} cc02_disp_t;

typedef struct { uint8_t request; } cc02_modereq_t;
#pragma pack(pop)
