/* =====================================================================
 *  edr_loops.h  -  The two parallel control loops
 * ---------------------------------------------------------------------
 *  LOOP A  (fast, EDR_FAST_HZ, IMU driven)   -> delta-V crash severity
 *  LOOP B  (slow, EDR_SLOW_HZ, CAN driven)   -> sustained decel rate
 *
 *  They are deliberately independent: different sources, different
 *  rates, different failure modes, different regulatory status.
 *
 *   +------------------+  833 Hz    +---------------------------+
 *   | IMU ASM330LHBG1  |----------->| LOOP A  dV discrimination |--+
 *   +------------------+            +---------------------------+  |
 *                                     |  main dV / boundary / safing |
 *                                     v                              v
 *   +------------------+  100 Hz    +---------------------------+  +----------+
 *   | VSS / ABS (CAN)  |----------->| LOOP B  decel rate        |->| ARBITER  |
 *   +------------------+            +---------------------------+  +----------+
 *                                                                    |
 *   +------------------+  event     +---------------------------+    v
 *   | SRS ECU (CAN)    |----------->| lock path (AIS-220 5.3.2) | record/lock
 *   +------------------+            +---------------------------+
 *
 *  CONCURRENCY NOTE (target, not simulator):
 *  Loop B publishes its result through a small double-buffered snapshot
 *  (edr_decel_snapshot_t) with a sequence counter, so the fast loop can
 *  read a coherent copy without taking a lock. On the S32K312 the two
 *  loops are separate OS tasks/ISRs; use __DMB() around the seqlock
 *  increments. See edr_decel_publish()/edr_decel_read().
 * ===================================================================== */
#ifndef EDR_LOOPS_H
#define EDR_LOOPS_H

#include "edr_types.h"
#include "edr_config.h"
#include "edr_filter.h"

/* ===================== LOOP A ======================================= */
typedef enum {
    EDR_A_IDLE = 0,
    EDR_A_ARMED,        /* entry criteria met, boundary clock running   */
    EDR_A_CAPTURE,      /* fired; still integrating the post-trigger tail*/
    EDR_A_FIRED,        /* one-tick pulse: record is complete            */
    EDR_A_LOCKOUT
} edr_loopA_state_t;

typedef struct {
    edr_loopA_state_t state;

    edr_biquad_t  lp_disc[EDR_AXIS_COUNT];   /* discrimination path     */
    edr_biquad_t  lp_dv[EDR_AXIS_COUNT];     /* integration path        */
    edr_bias_t    bias[EDR_AXIS_COUNT];
    edr_dvring_t  ring[EDR_AXIS_COUNT];

    int32_t  a_filt[EDR_AXIS_COUNT];         /* bias-removed, filtered  */
    int32_t  a_prev[EDR_AXIS_COUNT];
    int32_t  jerk[EDR_AXIS_COUNT];

    uint32_t t_arm_ms;
    uint32_t t_trigger_ms;      /* instant the decision was made         */
    uint32_t t_ms;
    uint32_t lockout_until_ms;

    /* misuse discrimination */
    uint8_t  osc_count;
    uint16_t osc_win_samples;
    uint16_t osc_n;
    int8_t   last_sign;
    int32_t  peak_jerk;

    /* clip accounting -> dV validity */
    uint16_t clip_count;

    /* latched metrics for the event record */
    int32_t  dv_max_mm_s;
    uint16_t t_to_dv_max_ms;
    int32_t  peak_a[EDR_AXIS_COUNT];

    /* outputs, valid for one tick */
    uint16_t trigger_mask;      /* one-tick: record ready to be filed    */
    uint16_t trigger_latched;   /* which criteria fired, held to capture  */
    uint16_t inhibit_mask;
    bool     safing_ok;
    int32_t  dv_final_mm_s;     /* cumulative dV at end of capture        */
} edr_loopA_t;

void edr_loopA_init(edr_loopA_t *A, const edr_config_t *cfg);
void edr_loopA_step(edr_loopA_t *A, const edr_config_t *cfg,
                    const edr_imu_sample_t *s, uint32_t t_ms);

/* ===================== LOOP B ======================================= */
typedef enum {
    EDR_B_IDLE = 0,
    EDR_B_DWELL,        /* above threshold, accumulating dwell time     */
    EDR_B_FIRED,
    EDR_B_LATCHED       /* stays fired until decel released             */
} edr_loopB_state_t;

typedef struct {
    edr_loopB_state_t state;

    int32_t  speed[EDR_SPEED_RING_LEN];  /* mm/s ring for LS regression */
    uint16_t head;
    uint32_t n;
    uint16_t win;                        /* regression window, samples  */
    int64_t  sxx;                        /* precomputed Sum (k-kbar)^2  */

    edr_biquad_t lp_accel_fb;            /* accel fallback path (~1 Hz) */

    int32_t  decel_mm_s2;                /* +ve = decelerating          */
    uint16_t dwell_ms;
    uint16_t dropout_ms;
    int32_t  speed_now_mm_s;
    bool     src_is_vss;
    bool     fired_edge;
} edr_loopB_t;

/* Seqlock snapshot published to the fast loop. */
typedef struct {
    volatile uint32_t seq;
    int32_t  decel_mm_s2;
    uint16_t dwell_ms;
    int32_t  speed_mm_s;
    bool     fired;
    bool     src_is_vss;
} edr_decel_snapshot_t;

void edr_loopB_init(edr_loopB_t *B, const edr_config_t *cfg);
void edr_loopB_step(edr_loopB_t *B, const edr_config_t *cfg,
                    const edr_vehicle_sample_t *v, int32_t a_long_mm_s2);

void edr_decel_publish(edr_decel_snapshot_t *snap, const edr_loopB_t *B);
void edr_decel_read(const edr_decel_snapshot_t *snap, edr_decel_snapshot_t *out);

#endif /* EDR_LOOPS_H */
