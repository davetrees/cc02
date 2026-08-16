/* CC-02 pin map — FROZEN, matches docs/WIRING.md. Schematic-verified (J8 header). */
#pragma once

#define PIN_STEER_PULSE     41  /* header pin 13 — camera-DVP GPIO, free (no ESP camera fitted) */
#define PIN_THROTTLE_PULSE  42  /* header pin 15 — same */

/* LEDC: 50 Hz, 14-bit — 1 duty tick = 20000/16384 us */
#define PWM_FREQ_HZ    50
#define PWM_RES_BITS   14
#define PWM_PERIOD_US  20000

#define PWM_US_MIN     1000
#define PWM_US_NEUTRAL 1500
#define PWM_US_MAX     2000

/* Tip-cut defaults (overridable from Pi CMD frame) */
#define TIP_ROLL_DEG_DEFAULT   45
#define TIP_PITCH_DEG_DEFAULT  45
#define TIP_GYRO_DPS           400
#define FAILSAFE_TIMEOUT_MS_DEFAULT 200

/* Anti-roll counter-steer (continuous closed loop, firmware-owned) */
#define ARS_ANGLE_DB   15.0f   /* deg: ignore lean below this (normal cornering) */
#define ARS_RATE_DB    40.0f   /* deg/s: ignore roll rate below this */
#define ARS_KP_ANGLE   8.0f    /* µs of steer per deg of lean past deadband */
#define ARS_KP_RATE    3.0f    /* µs of steer per deg/s of roll rate past deadband */
#define ARS_MAX_US     500.0f  /* clamp: full steer lock in a real tip */
#define ARS_ENGAGE_MAX 60.0f   /* deg: above this = flopped or un-zeroed, stand down */
#define ARS_DIR        (-1)    /* flipped 2026-08-16 per bench test */

/* ESTOP active brake: neutral only coasts, so drive the ESC to the opposite
 * side of neutral briefly to actually arrest motion, then settle to neutral. */
#define ESTOP_BRAKE_MS 350     /* brake-pulse duration */
#define ESTOP_BRAKE_US 300     /* µs offset from neutral during the brake pulse */
