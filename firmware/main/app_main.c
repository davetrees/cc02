/*
 * CC-02 bridge — Waveshare ESP32-S3-Touch-LCD-3.5B
 * Owns servo/ESC PWM + failsafe; talks to the Pi over USB CDC; LCD cluster.
 * Init order lifted from the bench-proven av-controller-v2 (01_factory demo):
 * PMU -> expander pulse -> display -> touch -> brightness -> LVGL.
 * PWM comes up FIRST so outputs are at neutral before anything else.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "bsp_i2c.h"
#include "bsp_display.h"
#include "bsp_touch.h"
#include "bsp_axp2101_ext.h"
#include "esp_io_expander_tca9554.h"

#include "lv_port.h"
#include "lvgl.h"

#include "pins.h"
#include "protocol.h"
#include "state.h"
#include "link.h"
#include "qmi8658.h"
#include "cluster_ui.h"

static const char *TAG = "cc02";

/* Portrait native panel */
#define LCD_H_RES 320
#define LCD_V_RES 480
#define LCD_BUFFER_SIZE (LCD_H_RES * LCD_V_RES)

cc02_state_t g_state;
portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;

static esp_io_expander_handle_t expander_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

/* ---------------- attitude math (mount-calibrated) ---------------- */
typedef struct { float x, y, z; } vec3;

static vec3 v_cross(vec3 a, vec3 b)
{
    return (vec3){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                   a.x * b.y - a.y * b.x };
}
static float v_dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static vec3 v_norm(vec3 a)
{
    float m = sqrtf(v_dot(a, a));
    if (m < 1e-6f) return (vec3){0, 0, 1};
    return (vec3){ a.x / m, a.y / m, a.z / m };
}

/* Calibrated mount basis: e3 = gravity when the CAR is level; e1/e2 span the
 * car's horizontal plane. Defaults = identity (board flat, screen up). */
static vec3 s_e1 = {1, 0, 0}, s_e2 = {0, 1, 0}, s_e3 = {0, 0, 1};

static void cal_load(void)
{
    nvs_handle_t h;
    if (nvs_open("cc02", NVS_READONLY, &h) != ESP_OK) return;
    float b[9];
    size_t len = sizeof(b);
    if (nvs_get_blob(h, "mountcal", b, &len) == ESP_OK && len == sizeof(b)) {
        s_e1 = (vec3){b[0], b[1], b[2]};
        s_e2 = (vec3){b[3], b[4], b[5]};
        s_e3 = (vec3){b[6], b[7], b[8]};
        ESP_LOGI(TAG, "mount cal loaded");
    }
    nvs_close(h);
}

static void cal_save(void)
{
    nvs_handle_t h;
    if (nvs_open("cc02", NVS_READWRITE, &h) != ESP_OK) return;
    float b[9] = { s_e1.x, s_e1.y, s_e1.z, s_e2.x, s_e2.y, s_e2.z,
                   s_e3.x, s_e3.y, s_e3.z };
    nvs_set_blob(h, "mountcal", b, sizeof(b));
    nvs_commit(h);
    nvs_close(h);
}

static void cal_apply(vec3 g_avg)
{
    s_e3 = v_norm(g_avg);
    vec3 helper = (fabsf(s_e3.x) < 0.9f) ? (vec3){1, 0, 0} : (vec3){0, 1, 0};
    float d = v_dot(helper, s_e3);
    s_e1 = v_norm((vec3){ helper.x - d * s_e3.x, helper.y - d * s_e3.y,
                          helper.z - d * s_e3.z });
    s_e2 = v_cross(s_e3, s_e1);
    cal_save();
    ESP_LOGI(TAG, "mount cal captured + saved");
}

/* ---------------- PWM ---------------- */
static void pwm_set_us(ledc_channel_t ch, uint16_t us)
{
    if (us < PWM_US_MIN) us = PWM_US_MIN;
    if (us > PWM_US_MAX) us = PWM_US_MAX;
    uint32_t duty = (uint32_t)us * (1 << PWM_RES_BITS) / PWM_PERIOD_US;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void pwm_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES_BITS,
        .timer_num = LEDC_TIMER_1,      /* timer 0 is the backlight's */
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));
    ledc_channel_config_t c = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0, .hpoint = 0,
    };
    c.channel = LEDC_CHANNEL_2; c.gpio_num = PIN_STEER_PULSE;    /* ch 0/1 kept clear of BL */
    ESP_ERROR_CHECK(ledc_channel_config(&c));
    c.channel = LEDC_CHANNEL_3; c.gpio_num = PIN_THROTTLE_PULSE;
    ESP_ERROR_CHECK(ledc_channel_config(&c));
    pwm_set_us(LEDC_CHANNEL_2, PWM_US_NEUTRAL);
    pwm_set_us(LEDC_CHANNEL_3, PWM_US_NEUTRAL);
}

/* ---------------- link RX ---------------- */
static void on_frame(uint8_t type, const uint8_t *payload, size_t len)
{
    int64_t now = esp_timer_get_time();
    if (type == CC02_T_CMD && len == sizeof(cc02_cmd_t)) {
        STATE_LOCK();
        memcpy(&g_state.cmd, payload, sizeof(cc02_cmd_t));
        if (g_state.last_cmd_us) {
            float dt = (now - g_state.last_cmd_us) / 1e6f;
            if (dt > 0.0005f)
                g_state.cmd_hz = 0.9f * g_state.cmd_hz + 0.1f * (1.0f / dt);
        }
        g_state.last_cmd_us = now;
        STATE_UNLOCK();
    } else if (type == CC02_T_DISP && len == sizeof(cc02_disp_t)) {
        STATE_LOCK();
        memcpy(&g_state.disp, payload, sizeof(cc02_disp_t));
        if ((g_state.disp.flags & CC02_DF_CAL_REQUEST) && !g_state.cal_active)
            g_state.cal_request = true;
        STATE_UNLOCK();
    }
}

/* ---------------- 100 Hz control / failsafe / telemetry ---------------- */
static void control_task(void *arg)
{
    (void)arg;
    uint8_t seq = 0;
    int batt_div = 0;
    float roll = 0, pitch = 0, yaw_rate = 0;
    vec3 gest = {0, 0, 1};
    bool gest_init = false;
    int cal_ticks = 0;
    vec3 cal_acc = {0, 0, 0};
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
        int64_t now = esp_timer_get_time();

        /* IMU + gravity-vector complementary filter in the CAR frame */
        qmi_sample_t s = {0};
        bool imu_ok = (qmi8658_read(&s) == ESP_OK);
        if (imu_ok) {
            const float dt = 0.01f;
            /* gyro centering: learn bias whenever rotation reads dead-still
             * (<3 dps), subtract it from everything downstream */
            static vec3 gbias = {0, 0, 0};
            float graw = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
            if (graw < 0.052f) {
                gbias.x += 0.005f * (s.gx - gbias.x);
                gbias.y += 0.005f * (s.gy - gbias.y);
                gbias.z += 0.005f * (s.gz - gbias.z);
            }
            s.gx -= gbias.x; s.gy -= gbias.y; s.gz -= gbias.z;
            vec3 a = {s.ax, s.ay, s.az};
            if (!gest_init) { gest = v_norm(a); gest_init = true; }
            /* propagate gravity estimate by gyro: dg/dt = -w x g */
            vec3 w = {s.gx, s.gy, s.gz};
            vec3 c = v_cross(w, gest);
            gest.x -= c.x * dt; gest.y -= c.y * dt; gest.z -= c.z * dt;
            /* blend toward accel only when |a| ~ 1g (not mid-crash/launch) */
            float am = sqrtf(v_dot(a, a));
            if (am > 6.0f && am < 14.0f) {
                vec3 an = {a.x / am, a.y / am, a.z / am};
                gest.x = 0.98f * gest.x + 0.02f * an.x;
                gest.y = 0.98f * gest.y + 0.02f * an.y;
                gest.z = 0.98f * gest.z + 0.02f * an.z;
            }
            gest = v_norm(gest);
            float gx1 = v_dot(gest, s_e1), gy1 = v_dot(gest, s_e2),
                  gz1 = v_dot(gest, s_e3);
            roll = atan2f(gy1, gz1);
            pitch = atan2f(-gx1, sqrtf(gy1 * gy1 + gz1 * gz1));
            yaw_rate = v_dot(w, gest);   /* rotation about the car's vertical */

            /* level-calibration capture (UI tap): average 1s of accel */
            if (g_state.cal_request && !g_state.cal_active) {
                STATE_LOCK();
                g_state.cal_request = false; g_state.cal_active = true;
                STATE_UNLOCK();
                cal_ticks = 100; cal_acc = (vec3){0, 0, 0};
            }
            if (g_state.cal_active && cal_ticks > 0) {
                cal_acc.x += a.x; cal_acc.y += a.y; cal_acc.z += a.z;
                if (--cal_ticks == 0) {
                    cal_apply(cal_acc);
                    STATE_LOCK();
                    g_state.cal_active = false;
                    STATE_UNLOCK();
                }
            }
        }

        /* snapshot inputs */
        STATE_LOCK();
        cc02_cmd_t cmd = g_state.cmd;
        int64_t last_cmd = g_state.last_cmd_us;
        bool local_estop = g_state.local_estop;
        bool tip_cut = g_state.tip_cut;
        STATE_UNLOCK();

        /* tip detection: LIVE condition — cut while over the limit, release as
         * soon as we're back under (5°/50 dps hysteresis band, no latch) */
        bool tip_en = (last_cmd == 0) || (cmd.flags & CC02_CF_TIP_ENABLE);
        float thr_r = (cmd.tip_roll_deg ? cmd.tip_roll_deg : TIP_ROLL_DEG_DEFAULT) * (float)M_PI / 180.0f;
        float thr_p = (cmd.tip_pitch_deg ? cmd.tip_pitch_deg : TIP_PITCH_DEG_DEFAULT) * (float)M_PI / 180.0f;
        const float hyst = 5.0f * (float)M_PI / 180.0f;
        const float gthr = TIP_GYRO_DPS * (float)M_PI / 180.0f;
        float gyro_mag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
        bool over = imu_ok && (fabsf(roll) > thr_r || fabsf(pitch) > thr_p ||
                               gyro_mag > gthr);
        bool under = !imu_ok ||
                     (fabsf(roll) < thr_r - hyst && fabsf(pitch) < thr_p - hyst &&
                      gyro_mag < gthr - 50.0f * (float)M_PI / 180.0f);
        if (tip_en && over) tip_cut = true;
        else if (!tip_en || under) tip_cut = false;

        /* failsafe: no CMD within timeout */
        uint16_t to_ms = cmd.failsafe_timeout_ms ? cmd.failsafe_timeout_ms
                                                 : FAILSAFE_TIMEOUT_MS_DEFAULT;
        bool failsafe = (last_cmd == 0) || (now - last_cmd > (int64_t)to_ms * 1000);

        /* output arbitration */
        uint16_t steer = cmd.steer_us ? cmd.steer_us : PWM_US_NEUTRAL;
        uint16_t esc = cmd.throttle_us ? cmd.throttle_us : PWM_US_NEUTRAL;
        bool estop = local_estop || (cmd.flags & CC02_CF_ESTOP) || cmd.mode == CC02_MODE_ESTOP;
        static int64_t estop_since = 0;
        static uint16_t last_drive_esc = PWM_US_NEUTRAL;
        if (estop) {
            steer = PWM_US_NEUTRAL;
            if (!estop_since) estop_since = now;
            /* ACTIVE BRAKE: neutral only coasts. Drive the ESC to the opposite
             * side of neutral for a short window to actually arrest motion,
             * then settle to neutral. Direction from the last driven pulse. */
            if (now - estop_since < (int64_t)ESTOP_BRAKE_MS * 1000 &&
                last_drive_esc != PWM_US_NEUTRAL) {
                int dir = (last_drive_esc > PWM_US_NEUTRAL) ? 1 : -1;
                esc = (uint16_t)(PWM_US_NEUTRAL - dir * ESTOP_BRAKE_US);
            } else {
                esc = PWM_US_NEUTRAL;
            }
        } else if (failsafe) {
            estop_since = 0;
            esc = PWM_US_NEUTRAL;
            if (cmd.flags & CC02_CF_STEER_CENTER || last_cmd == 0) steer = PWM_US_NEUTRAL;
            else steer = g_state.out_steer_us;   /* hold */
        }
        if (tip_cut) esc = PWM_US_NEUTRAL;

        /* ---- ANTI-ROLL COUNTER-STEER (continuous closed loop) ----
         * Runs every tick on the ESP, on TOP of whatever steer was chosen
         * (manual / auto / debug / failsafe). When the car starts to tip, it
         * steers into the fall to drive back under the CG. Roll rate is the
         * fast term (catches a dynamic flick), roll angle the slow term
         * (holds correction while leaned). Steering only — never throttle —
         * so ESTOP's neutral throttle is untouched. Not defeatable from the
         * Pi: this is the layer that must save it when the Pi is wrong. */
        static float roll_f = 0.0f, rollrate_f = 0.0f, prev_roll = 0.0f;
        if (imu_ok) {
            float rr = (roll - prev_roll) / 0.01f;      /* rad/s */
            prev_roll = roll;
            rollrate_f = 0.6f * rollrate_f + 0.4f * rr;
            roll_f = roll;
            float roll_deg = roll_f * 57.2958f;
            float rate_dps = rollrate_f * 57.2958f;
            float bias = 0.0f;                          /* µs of steer offset */
            /* Only within a RECOVERABLE lean band. Past ARS_ENGAGE_MAX the car
             * is either already flopped (can't steer back up — documented) or
             * the horizon isn't zeroed (reads ~180°); either way pegging the
             * wheels helps nothing, so stand down. */
            if (fabsf(roll_deg) < ARS_ENGAGE_MAX) {
                if (fabsf(roll_deg) > ARS_ANGLE_DB)
                    bias += ARS_KP_ANGLE * (roll_deg - copysignf(ARS_ANGLE_DB, roll_deg));
                if (fabsf(rate_dps) > ARS_RATE_DB)
                    bias += ARS_KP_RATE * (rate_dps - copysignf(ARS_RATE_DB, rate_dps));
            }
            if (bias > ARS_MAX_US) bias = ARS_MAX_US;
            if (bias < -ARS_MAX_US) bias = -ARS_MAX_US;
            int s2 = (int)steer + (int)(ARS_DIR * bias);
            if (s2 < PWM_US_MIN) s2 = PWM_US_MIN;
            if (s2 > PWM_US_MAX) s2 = PWM_US_MAX;
            steer = (uint16_t)s2;
        }

        pwm_set_us(LEDC_CHANNEL_2, steer);
        pwm_set_us(LEDC_CHANNEL_3, esc);

        /* battery at 1 Hz (I2C traffic) */
        uint16_t vbus = g_state.vbus_mv, batt = g_state.batt_mv;
        if (++batt_div >= 100) {
            batt_div = 0;
            batt = bsp_axp2101_get_batt_voltage();
            vbus = bsp_axp2101_is_vbus_in() ? 5000 : 0;
        }

        /* publish + telemetry */
        cc02_telem_t tm = {
            .seq = seq++,
            .ax = s.ax, .ay = s.ay, .az = s.az,
            .gx = s.gx, .gy = s.gy, .gz = s.gz,
            .roll = roll, .pitch = pitch, .yaw_rate = yaw_rate,
            .servo_us = steer, .esc_us = esc,
            .vbus_mv = vbus, .batt_mv = batt,
            .flags = (uint8_t)((failsafe ? 0 : CC02_F_SERIAL_OK) |
                               (tip_cut ? CC02_F_TIP_CUT : 0) |
                               (failsafe ? CC02_F_FAILSAFE : 0) |
                               (estop ? CC02_F_ESTOP : 0)),
        };
        STATE_LOCK();
        g_state.imu = s; g_state.imu_ok = imu_ok;
        g_state.roll = roll; g_state.pitch = pitch; g_state.yaw_rate = yaw_rate;
        g_state.tip_cut = tip_cut; g_state.failsafe = failsafe;
        g_state.out_steer_us = steer; g_state.out_esc_us = esc;
        g_state.vbus_mv = vbus; g_state.batt_mv = batt;
        STATE_UNLOCK();
        link_send(CC02_T_TELEM, &tm, sizeof(tm));
    }
}

/* ---------------- LCD glue (verbatim from proven base) ---------------- */
static void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static lv_coord_t last_x = 0, last_y = 0;
    touch_data_t td;
    bsp_touch_read();
    if (bsp_touch_get_coordinates(&td)) {
        last_x = td.coords[0].x;
        last_y = td.coords[0].y;
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    data->point.x = last_x;
    data->point.y = last_y;
}

static void io_expander_init(i2c_master_bus_handle_t bus_handle)
{
    ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(bus_handle,
        ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &expander_handle));
    ESP_ERROR_CHECK(esp_io_expander_set_dir(expander_handle, IO_EXPANDER_PIN_NUM_1, IO_EXPANDER_OUTPUT));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander_handle, IO_EXPANDER_PIN_NUM_1, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander_handle, IO_EXPANDER_PIN_NUM_1, 1));
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void lv_port_setup(void)
{
    lvgl_port_cfg_t port_cfg = {};
    port_cfg.task_priority = 4;
    port_cfg.task_stack = 1024 * 5;
    port_cfg.task_affinity = 1;
    port_cfg.task_max_sleep_ms = 500;
    port_cfg.timer_period_ms = 5;
    lvgl_port_init(&port_cfg);

    lvgl_port_display_cfg_t disp_cfg = {};
    disp_cfg.io_handle = io_handle;
    disp_cfg.panel_handle = panel_handle;
    disp_cfg.buffer_size = LCD_BUFFER_SIZE;
    disp_cfg.sw_rotate = LV_DISP_ROT_NONE;   /* portrait = panel native */
    disp_cfg.hres = LCD_H_RES;
    disp_cfg.vres = LCD_V_RES;
    disp_cfg.trans_size = LCD_BUFFER_SIZE / 20;
    disp_cfg.draw_wait_cb = NULL;
    disp_cfg.flags.buff_dma = false;
    disp_cfg.flags.buff_spiram = true;
    lvgl_port_add_disp(&disp_cfg);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    lv_indev_drv_register(&indev_drv);
}

void app_main(void)
{
    /* Failsafe outputs first — neutral on both channels before anything else */
    pwm_init();

    memset(&g_state, 0, sizeof(g_state));
    g_state.out_steer_us = PWM_US_NEUTRAL;
    g_state.out_esc_us = PWM_US_NEUTRAL;

    esp_err_t nerr = nvs_flash_init();
    if (nerr == ESP_ERR_NVS_NO_FREE_PAGES || nerr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    cal_load();

    i2c_master_bus_handle_t bus = bsp_i2c_init();
    bsp_axp2101_init_c(bus);
    io_expander_init(bus);
    bsp_display_init(&io_handle, &panel_handle, LCD_BUFFER_SIZE);
    bsp_touch_init(bus, LCD_H_RES, LCD_V_RES, 0);

    bsp_display_brightness_init();
    bsp_display_set_brightness(80);

    lv_port_setup();

    qmi8658_init(bus);      /* graceful if absent */
    link_init(on_frame);

    if (lvgl_port_lock(0)) {
        cluster_ui_init();
        lvgl_port_unlock();
    }

    xTaskCreatePinnedToCore(control_task, "control", 6144, NULL, 12, NULL, 0);
    ESP_LOGI(TAG, "CC-02 ready");
}
