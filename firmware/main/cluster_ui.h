#pragma once
/* Build the instrument cluster (call under lvgl_port_lock). Updates itself
 * via an lv_timer at 10 Hz with deadbanded writes (full_refresh panel). */
void cluster_ui_init(void);
