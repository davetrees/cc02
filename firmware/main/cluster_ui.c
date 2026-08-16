/* CC-02 instrument cluster — portrait 320x480, LVGL 8.4.
 * Full-refresh panel: every write is deadbanded/gated on change. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "esp_timer.h"
#include "lvgl.h"
#include "state.h"
#include "link.h"
#include "pins.h"
#include "bsp_display.h"
#include "cluster_ui.h"

static lv_obj_t *s_mode_box, *s_mode_lbl;
static lv_obj_t *s_link_lbl, *s_att_lbl, *s_horizon, *s_att_box;
static lv_obj_t *s_steer_bar, *s_esc_bar, *s_drive_lbl;
static lv_obj_t *s_vision_lbl, *s_power_lbl, *s_fault_lbl;
static lv_obj_t *s_estop_btn, *s_estop_lbl;
static lv_point_t s_hpts[2];

static const char *MODE_NAMES[] = { "MANUAL", "ASSIST", "AUTO", "RTH", "ESTOP" };
static const uint32_t MODE_COLORS[] = { 0x2266cc, 0x22aa66, 0xcc8800, 0x8844cc, 0xcc2222 };

static uint8_t s_brightness = 80;

/* ---- caches for deadbanding ---- */
static int c_mode = -1, c_linklost = -1, c_roll = 999, c_pitch = 999;
static int c_steer = 0, c_esc = 0, c_hz = -1, c_age10 = -1;
static char c_vision[48], c_power[48], c_fault[64];

static void estop_cb(lv_event_t *e)
{
    (void)e;
    bool on;
    STATE_LOCK();
    g_state.local_estop = !g_state.local_estop;
    on = g_state.local_estop;
    STATE_UNLOCK();
    cc02_modereq_t rq = { .request = on ? CC02_REQ_ESTOP_ON : CC02_REQ_ESTOP_OFF };
    link_send(CC02_T_MODEREQ, &rq, sizeof(rq));
    lv_label_set_text(s_estop_lbl, on ? "ESTOP ON - TAP TO CLEAR" : "ESTOP");
    lv_obj_set_style_bg_color(s_estop_btn, lv_color_hex(on ? 0x660000 : 0xcc2222), 0);
}

static void mode_cb(lv_event_t *e)
{
    (void)e;
    cc02_modereq_t rq = { .request = CC02_REQ_MODE_CYCLE };
    link_send(CC02_T_MODEREQ, &rq, sizeof(rq));
}

static void rth_cb(lv_event_t *e)
{
    (void)e;
    cc02_modereq_t rq = { .request = CC02_REQ_RTH };
    link_send(CC02_T_MODEREQ, &rq, sizeof(rq));
}

static void cal_cb(lv_event_t *e)
{
    (void)e;
    STATE_LOCK();
    g_state.cal_request = true;
    STATE_UNLOCK();
}

static void brt_cb(lv_event_t *e)
{
    (void)e;
    s_brightness = (s_brightness >= 100) ? 25 : s_brightness + 25;
    bsp_display_set_brightness(s_brightness);
}

static lv_obj_t *mk_label(lv_obj_t *parent, int x, int y, const lv_font_t *f, uint32_t color)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_pos(l, x, y);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    return l;
}

static lv_obj_t *mk_btn(lv_obj_t *parent, int x, int y, int w, int h, const char *txt,
                        uint32_t color, lv_event_cb_t cb, lv_obj_t **out_lbl)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, lv_color_hex(color), 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_PRESSED, NULL); /* PRESSED = instant */
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_22, 0);
    lv_obj_center(l);
    if (out_lbl) *out_lbl = l;
    return b;
}

static void ui_tick(lv_timer_t *t);

void cluster_ui_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    /* MODE banner */
    s_mode_box = lv_obj_create(scr);
    lv_obj_set_pos(s_mode_box, 0, 0);
    lv_obj_set_size(s_mode_box, 320, 52);
    lv_obj_set_style_radius(s_mode_box, 0, 0);
    lv_obj_set_style_border_width(s_mode_box, 0, 0);
    lv_obj_set_style_bg_color(s_mode_box, lv_color_hex(MODE_COLORS[0]), 0);
    lv_obj_clear_flag(s_mode_box, LV_OBJ_FLAG_SCROLLABLE);
    s_mode_lbl = lv_label_create(s_mode_box);
    lv_obj_set_style_text_font(s_mode_lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_mode_lbl, lv_color_white(), 0);
    lv_label_set_text(s_mode_lbl, "MANUAL");
    lv_obj_center(s_mode_lbl);

    /* LINK */
    s_link_lbl = mk_label(scr, 8, 60, &lv_font_montserrat_16, 0xa0c0e0);
    lv_label_set_text(s_link_lbl, "LINK: waiting");

    /* ATTITUDE: horizon box + numbers */
    lv_obj_t *hb = lv_obj_create(scr);
    lv_obj_set_pos(hb, 8, 86);
    lv_obj_set_size(hb, 140, 84);
    lv_obj_set_style_bg_color(hb, lv_color_hex(0x182028), 0);
    lv_obj_set_style_border_color(hb, lv_color_hex(0x304050), 0);
    lv_obj_set_style_radius(hb, 4, 0);
    lv_obj_clear_flag(hb, LV_OBJ_FLAG_SCROLLABLE);
    /* tap the horizon box with the car sitting LEVEL = capture mount cal */
    lv_obj_add_flag(hb, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(hb, cal_cb, LV_EVENT_PRESSED, NULL);
    s_att_box = hb;
    s_horizon = lv_line_create(hb);
    lv_obj_set_style_line_width(s_horizon, 3, 0);
    lv_obj_set_style_line_color(s_horizon, lv_color_hex(0x40d080), 0);
    s_hpts[0].x = 10;  s_hpts[0].y = 35;
    s_hpts[1].x = 120; s_hpts[1].y = 35;
    lv_line_set_points(s_horizon, s_hpts, 2);
    s_att_lbl = mk_label(scr, 158, 96, &lv_font_montserrat_20, 0xffffff);
    lv_label_set_text(s_att_lbl, "R:  --\nP:  --");

    /* DRIVE bars */
    s_steer_bar = lv_bar_create(scr);
    lv_obj_set_pos(s_steer_bar, 8, 180);
    lv_obj_set_size(s_steer_bar, 200, 14);
    lv_bar_set_range(s_steer_bar, PWM_US_MIN, PWM_US_MAX);
    lv_bar_set_value(s_steer_bar, PWM_US_NEUTRAL, LV_ANIM_OFF);
    s_esc_bar = lv_bar_create(scr);
    lv_obj_set_pos(s_esc_bar, 8, 202);
    lv_obj_set_size(s_esc_bar, 200, 14);
    lv_bar_set_range(s_esc_bar, PWM_US_MIN, PWM_US_MAX);
    lv_bar_set_value(s_esc_bar, PWM_US_NEUTRAL, LV_ANIM_OFF);
    s_drive_lbl = mk_label(scr, 216, 180, &lv_font_montserrat_16, 0xffffff);
    lv_label_set_text(s_drive_lbl, "S 1500\nT 1500");

    /* VISION + POWER + FAULT */
    s_vision_lbl = mk_label(scr, 8, 226, &lv_font_montserrat_16, 0xc0c0a0);
    lv_label_set_text(s_vision_lbl, "VISION: --");
    s_power_lbl = mk_label(scr, 8, 250, &lv_font_montserrat_16, 0xa0e0a0);
    lv_label_set_text(s_power_lbl, "POWER: --");
    s_fault_lbl = mk_label(scr, 8, 276, &lv_font_montserrat_18, 0xff6060);
    lv_label_set_long_mode(s_fault_lbl, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(s_fault_lbl, 304);
    lv_label_set_text(s_fault_lbl, "");

    /* Buttons */
    s_estop_btn = mk_btn(scr, 8, 306, 304, 84, "ESTOP", 0xcc2222, estop_cb, &s_estop_lbl);
    lv_obj_set_style_text_font(s_estop_lbl, &lv_font_montserrat_28, 0);
    mk_btn(scr, 8, 400, 72, 70, "MODE", 0x2a3a4a, mode_cb, NULL);
    mk_btn(scr, 86, 400, 72, 70, "RTH", 0x2a3a4a, rth_cb, NULL);
    mk_btn(scr, 164, 400, 72, 70, "BRT", 0x2a3a4a, brt_cb, NULL);
    mk_btn(scr, 242, 400, 70, 70, "ZERO", 0x805020, cal_cb, NULL);

    lv_timer_create(ui_tick, 100, NULL);
}

static void ui_tick(lv_timer_t *t)
{
    (void)t;
    cc02_state_t st;
    STATE_LOCK();
    st = g_state;
    STATE_UNLOCK();

    /* MODE (ESTOP display state wins over commanded mode) */
    int mode = (st.local_estop || (st.cmd.flags & CC02_CF_ESTOP)) ? CC02_MODE_ESTOP
               : (st.cmd.mode <= CC02_MODE_ESTOP ? st.cmd.mode : CC02_MODE_MANUAL);
    if (mode != c_mode) {
        c_mode = mode;
        lv_label_set_text(s_mode_lbl, MODE_NAMES[mode]);
        lv_obj_set_style_bg_color(s_mode_box, lv_color_hex(MODE_COLORS[mode]), 0);
    }

    /* LINK: quantized so jitter doesn't repaint */
    int64_t now = esp_timer_get_time();
    int age_ms = st.last_cmd_us ? (int)((now - st.last_cmd_us) / 1000) : 9999;
    int age10 = age_ms > 999 ? 999 : age_ms / 10;
    int hz = (int)(st.cmd_hz + 0.5f);
    int lost = st.failsafe;
    if (age10 != c_age10 || hz != c_hz || lost != c_linklost) {
        c_age10 = age10; c_hz = hz; c_linklost = lost;
        if (lost)
            lv_label_set_text(s_link_lbl, "LINK LOST - FAILSAFE");
        else
            lv_label_set_text_fmt(s_link_lbl, "LINK %dHz age %dms  WiFi:%d",
                                  hz, age10 * 10, st.disp.wifi_clients);
        lv_obj_set_style_text_color(s_link_lbl,
                                    lv_color_hex(lost ? 0xff4040 : 0xa0c0e0), 0);
    }

    /* calibration feedback: yellow border while sampling */
    static int c_cal = -1;
    int calno = st.cal_active ? 1 : 0;
    if (calno != c_cal) {
        c_cal = calno;
        lv_obj_set_style_border_color(s_att_box,
            lv_color_hex(calno ? 0xf0c000 : 0x304050), 0);
    }

    /* ATTITUDE: 1-degree deadband */
    int rolld = (int)lrintf(st.roll * 57.2958f);
    int pitchd = (int)lrintf(st.pitch * 57.2958f);
    if (rolld != c_roll || pitchd != c_pitch) {
        c_roll = rolld; c_pitch = pitchd;
        lv_label_set_text_fmt(s_att_lbl, "R: %d°\nP: %d°", rolld, pitchd);
        float r = st.roll;
        int cy = 35 - (pitchd > 30 ? 30 : (pitchd < -30 ? -30 : pitchd));
        int dx = (int)(55.0f * cosf(r)), dy = (int)(55.0f * sinf(r));
        s_hpts[0].x = 65 - dx; s_hpts[0].y = cy - dy;
        s_hpts[1].x = 65 + dx; s_hpts[1].y = cy + dy;
        lv_line_set_points(s_horizon, s_hpts, 2);
    }

    /* DRIVE: 5 µs deadband */
    if (abs(st.out_steer_us - c_steer) > 5 || abs(st.out_esc_us - c_esc) > 5) {
        c_steer = st.out_steer_us; c_esc = st.out_esc_us;
        lv_bar_set_value(s_steer_bar, c_steer, LV_ANIM_OFF);
        lv_bar_set_value(s_esc_bar, c_esc, LV_ANIM_OFF);
        lv_label_set_text_fmt(s_drive_lbl, "S %d\nT %d", c_steer, c_esc);
    }

    /* VISION */
    char buf[48];
    snprintf(buf, sizeof(buf), "VISION: %dfps %ddet%s", st.disp.yolo_fps,
             st.disp.det_count, (st.disp.flags & CC02_DF_COLLISION) ? " COLL!" : "");
    if (strcmp(buf, c_vision)) { strcpy(c_vision, buf); lv_label_set_text(s_vision_lbl, buf); }

    /* POWER (values change at 1 Hz in control task) */
    snprintf(buf, sizeof(buf), "PWR: vbus %umV batt %umV", st.vbus_mv, st.batt_mv);
    if (strcmp(buf, c_power)) { strcpy(c_power, buf); lv_label_set_text(s_power_lbl, buf); }

    /* FAULT ticker */
    char f[64] = "";
    if (st.failsafe) strlcat(f, "LINK LOST  ", sizeof(f));
    if (st.tip_cut) strlcat(f, "TIP CUT  ", sizeof(f));
    if (st.disp.flags & CC02_DF_COLLISION) strlcat(f, "COLLISION  ", sizeof(f));
    if (st.local_estop || (st.cmd.flags & CC02_CF_ESTOP)) strlcat(f, "ESTOP  ", sizeof(f));
    if (!st.imu_ok) strlcat(f, "IMU FAIL  ", sizeof(f));
    if (strcmp(f, c_fault)) { strcpy(c_fault, f); lv_label_set_text(s_fault_lbl, f); }
}
