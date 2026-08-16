/* Shared state between control task, link RX and the LCD cluster. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "protocol.h"
#include "qmi8658.h"

typedef struct {
    cc02_cmd_t cmd;             /* last CMD from Pi */
    int64_t last_cmd_us;        /* esp_timer stamp of last valid CMD, 0 = never */
    cc02_disp_t disp;           /* last DISP from Pi */
    bool local_estop;           /* touch ESTOP latch (works with Pi dead) */
    bool tip_cut;
    bool failsafe;              /* CMD timeout active */
    bool imu_ok;
    float roll, pitch, yaw_rate;    /* rad, rad/s */
    qmi_sample_t imu;
    uint16_t out_steer_us, out_esc_us;  /* actually on the wire */
    uint16_t vbus_mv, batt_mv;
    float cmd_hz;               /* measured CMD ingress rate */
    bool cal_request;           /* UI tap: capture level reference */
    bool cal_active;            /* sampling in progress (UI shows yellow) */
} cc02_state_t;

extern cc02_state_t g_state;
extern portMUX_TYPE g_state_mux;

#define STATE_LOCK()   taskENTER_CRITICAL(&g_state_mux)
#define STATE_UNLOCK() taskEXIT_CRITICAL(&g_state_mux)
