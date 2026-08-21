/* =====================================================================
 *  edr_api.h  -  Public entry points
 * ---------------------------------------------------------------------
 *  Integration on the S32K312:
 *
 *    IMU data-ready ISR / 1.2 ms task  ->  edr_fast_tick()
 *    100 Hz CAN Rx task                ->  edr_slow_tick()
 *    background task                   ->  edr_poll_events()
 *
 *  edr_fast_tick() is bounded, allocation-free, branch-predictable and
 *  contains no division by a variable. Measured worst case ~4.1 us on a
 *  120 MHz Cortex-M7 with I-cache enabled (see README for the method).
 * ===================================================================== */
#ifndef EDR_API_H
#define EDR_API_H

#include "edr_types.h"
#include "edr_config.h"
#include "edr_loops.h"

typedef struct {
    edr_config_t          cfg;
    edr_loopA_t           A;
    edr_loopB_t           B;
    edr_decel_snapshot_t  snap;

    uint32_t t_fast_ms;
    uint32_t t_slow_ms;
    uint32_t fast_n;
    uint32_t slow_n;

    /* multi-event sequencing */
    edr_event_t events[EDR_MAX_EVENTS_PER_SEQ];
    uint8_t     event_count;
    uint8_t     event_read;
    uint32_t    seq_t0_ms;
    bool        seq_open;
    int32_t     seq_cum_dv_mm_s;

    /* Annexure 4 record buffer: 4 Hz circular pre-crash + post-crash */
    edr_record_sample_t rec[EDR_PRECRASH_SAMPLES];
    uint8_t   rec_head;
    uint8_t   rec_n;
    uint32_t  rec_acc;          /* Bresenham accumulator for 4 Hz      */
    edr_biquad_t rec_aa[EDR_AXIS_COUNT];   /* anti-alias before 4 Hz    */

    bool      srs_seen;
    uint32_t  last_decel_seq;   /* Loop B edge de-duplication          */
    uint8_t   prevA;            /* previous Loop A state, for edges    */
    int32_t   evt_peak_decel;   /* peak decel seen during this event   */
    int32_t   evt_speed_at_arm; /* road speed when Loop A armed        */
} edr_ctx_t;

void     edr_init(edr_ctx_t *ctx, const edr_config_t *cfg);

/* Fast loop. Call once per IMU sample. Returns trigger mask fired on
 * THIS tick (0 = nothing). */
uint16_t edr_fast_tick(edr_ctx_t *ctx, const edr_imu_sample_t *s);

/* Slow loop. Call once per CAN cycle. Returns non-zero on Loop B edge. */
uint16_t edr_slow_tick(edr_ctx_t *ctx, const edr_vehicle_sample_t *v);

/* Drain completed event records. Returns number written. */
uint32_t edr_poll_events(edr_ctx_t *ctx, edr_event_t *out, uint32_t max);

/* Introspection hooks for the simulator / HIL bench (not used on target
 * in the safety path, but useful for calibration logging). */
typedef struct {
    int32_t  a_long_filt, a_lat_filt;
    int32_t  bias_long, bias_lat;
    int32_t  dv_short, dv_main, dv_safing, dv_cum;
    int32_t  dv_boundary;
    int32_t  decel_mm_s2;
    uint16_t decel_dwell_ms;
    uint8_t  stateA, stateB;
    uint8_t  osc_count;
    int32_t  jerk;
    uint16_t trigger_mask, inhibit_mask;
    bool     safing_ok;
} edr_debug_t;

void edr_get_debug(const edr_ctx_t *ctx, edr_debug_t *dbg);

/* Evaluate the dV-vs-time boundary (exposed for calibration tooling). */
int32_t edr_boundary_at(const edr_config_t *cfg, uint32_t t_ms);

#endif /* EDR_API_H */
