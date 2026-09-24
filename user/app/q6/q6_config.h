#ifndef Q6_CONFIG_H
#define Q6_CONFIG_H

/* Q6 target/capture and controller-adaptation parameters. Q3/Q4/Q5 tuning
 * tables remain owned by their original modules and are unchanged here. */
#define Q6_CAPTURE_MIN_FRAMES 15U
#define Q6_CAPTURE_MIN_SPAN_MS 500U
#define Q6_CAPTURE_MAX_FRAME_GAP_MS 100U
#define Q6_CAPTURE_MAX_AGE_MS 100U
#define Q6_CAPTURE_MIN_CONFIDENCE 60U
#define Q6_CAPTURE_MAX_POSITION_RANGE_CM 0.20f
#define Q6_CAPTURE_MAX_SPEED_CM_S 0.50f
#define Q6_EXPLICIT_START_TOLERANCE_CM 1.00f
#define Q6_TEST_TARGET_LIMIT_CM 5.00f
/* Retained legacy threshold; current Q6 integral and stiction geometry use
 * Q6_STICTION_ENGAGE_CM instead. */
#define Q6_HOLD_ENGAGE_TOLERANCE_CM 0.20f
/* Full 250 mm physical tube, centered at the calibrated origin. */
#define Q6_DYNAMIC_CAPTURE_MAX_POSITION_CM 12.50f
#define Q6_EXPLICIT_CAPTURE_MAX_POSITION_CM Q6_DYNAMIC_CAPTURE_MAX_POSITION_CM
/* Maximum learned Q6 beam-bias contribution, in degrees. */
#define Q6_INTEGRAL_LIMIT_DEG 0.80f
/* Q6-specific bounded stiction-push retry budget. */
#define Q6_STICTION_MAX_COUNT 10U
/* Motor-side angle amplitude for the bounded Q6 stiction pulse. */
#define Q6_STICTION_PUSH_MOTOR_DEG 10.0f
/* Position error threshold and pulse duration for Q6 stiction assistance.
 * Values are Q6-only; Q4/Q5 timing and gates remain unchanged. */
#define Q6_STICTION_ENGAGE_CM 0.55f
#define Q6_STICTION_PUSH_HOLD_MS 200U
/* Q6 BRAKE damping gains in beam-deg per (cm/s), including the outward-motion
 * branch. Q4/Q5 continue to use their frozen values. */
#define Q6_BRAKE_KD_BEAM_DEG_PER_CM_S 0.18f
#define Q6_BRAKE_KD_OUTWARD_BEAM_DEG_PER_CM_S 0.24f
/* Select BRAKE as the Q6 profile fallback over long target distances; this
 * prevents the shared CRUISE entry law from being selected for Q6. */
#define Q6_BRAKE_FALLBACK_CM 18.00f
/* Q6-only visual admission band. Samples outside it are ignored by the
 * Q6 consumer and sustained rejection remains subject to the freshness
 * watchdog; the vision protocol/parser is unchanged. */
#define Q6_BALL_PIXEL_CY_CENTER 165U
#define Q6_BALL_PIXEL_CY_BAND 60U
/* Reserved acceptance thresholds for static-trial tooling. The current
 * runtime control path does not consume these values. */
#define Q6_STATIC_SETTLE_MS 2000U
#define Q6_STATIC_HOLD_MS 10000U
#define Q6_STATIC_MAX_ERROR_CM 0.50f
#define Q6_STATIC_MIN_FRAMES 100U
#define Q6_STATIC_REPEAT 3U
#define Q6_STATIC_ABORT_DEVIATION_CM 3.00f
/* Master switch for the Q6 task; PB24 maps to Q6 when enabled. */
#ifndef Q6_TASK_ENABLE
#define Q6_TASK_ENABLE 1U
#endif
/* Bench image 1: hold-position trial with the chassis parked. */
#ifndef Q6_STATIC_TEST_MODE
#define Q6_STATIC_TEST_MODE 0U
#endif
/* Bench image 2 (default OFF): once Q6 is selected, a captured target may
 * launch without the normal second Q6 start-key press. */
#ifndef Q6_BENCH_AUTO_START
#define Q6_BENCH_AUTO_START 0U
#endif
/* Bench image 3 (default OFF): enter Q6 automatically one second after
 * boot so a flash reset starts the whole trial without any key press. */
#ifndef Q6_BOOT_AUTO_SELECT
#define Q6_BOOT_AUTO_SELECT 0U
#endif
/* Bench image 4 (default OFF): pin the boot auto-trial hold target to an
 * explicit value in millimetres (50 = +5.0 cm).  The sentry -32768 keeps
 * the trial in Q6_TARGET_CAPTURE mode (lock wherever the ball rests). */
#ifndef Q6_BENCH_EXPLICIT_TARGET_MM
#define Q6_BENCH_EXPLICIT_TARGET_MM (-32768)
#endif

#endif
