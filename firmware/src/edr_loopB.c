/* =====================================================================
 *  LOOP B  -  sustained deceleration rate          (slow loop, 100 Hz)
 * ---------------------------------------------------------------------
 *  AIS-220 sudden-deceleration trigger: >= 3.25 m/s^2 sustained for
 *  >= 0.7 s, measured from VEHICLE SPEED - not from a dV threshold and
 *  not, in the conformant path, from the accelerometer.
 *
 *  WHY SPEED AND NOT ACCELERATION:
 *  On a 6 % downgrade a body-mounted accelerometer reads ~0.59 m/s^2 of
 *  gravity projection with the vehicle at constant speed. Feed that into
 *  a decel detector and you get grade-dependent false triggers all day.
 *  Wheel-speed differentiation is grade-immune. The accelerometer path
 *  below exists only as a degraded fallback when VSS is invalid, and
 *  every record it produces is tagged decel_src_is_vss = false.
 *
 *  WHY LEAST-SQUARES AND NOT BACKWARD DIFFERENCE:
 *  CAN speed is quantised (commonly 1/256 or 0.0625 km/h). A two-point
 *  difference at 100 Hz amplifies one LSB of 0.0625 km/h into 1.74 m/s^2
 *  of noise - over half the trigger threshold. A least-squares slope over
 *  a 200 ms window cuts that by roughly sqrt(N)*... in practice ~15x,
 *  with a bounded 100 ms group delay that we account for in t0.
 * ===================================================================== */
#include "edr_loops.h"

static uint16_t ms_to_samples_b(uint16_t ms, uint16_t hz)
{
    uint32_t n = ((uint32_t)ms * (uint32_t)hz + 500u) / 1000u;
    if (n < 3u) { n = 3u; }
    return (uint16_t)n;
}

void edr_loopB_init(edr_loopB_t *B, const edr_config_t *cfg)
{
    uint16_t i;
    const int64_t n = (int64_t)ms_to_samples_b(cfg->decel_regress_ms, cfg->slow_hz);

    B->state = EDR_B_IDLE;
    for (i = 0u; i < EDR_SPEED_RING_LEN; i++) { B->speed[i] = 0; }
    B->head = 0u;
    B->n = 0u;
    B->win = (uint16_t)n;
    /* Sum (k - kbar)^2 for k = 0..N-1  =  N(N^2 - 1)/12 */
    B->sxx = (n * (n * n - 1)) / 12;
    if (B->sxx <= 0) { B->sxx = 1; }

    edr_biquad_design_lp(&B->lp_accel_fb, 2, cfg->slow_hz);   /* ~2 Hz    */

    B->decel_mm_s2 = 0;
    B->dwell_ms = 0u;
    B->dropout_ms = 0u;
    B->speed_now_mm_s = 0;
    B->src_is_vss = true;
    B->fired_edge = false;
}

/* Least-squares slope over the last `win` speed samples.
 * Returns deceleration (positive = slowing down) in mm/s^2. */
static int32_t ls_decel(const edr_loopB_t *B, uint16_t hz)
{
    const int32_t N = (int32_t)B->win;
    int64_t sxy = 0;
    int32_t k;

    if ((int32_t)B->n < N) { return 0; }

    /* Sum (k - (N-1)/2) * y[k], k oldest..newest. The (k - kbar) term is
     * kept in halves to stay integral: use (2k - (N-1)) and divide by 2
     * at the end. */
    for (k = 0; k < N; k++) {
        const uint16_t idx = (uint16_t)((B->head - (uint16_t)(N - 1 - k)) & (EDR_SPEED_RING_LEN - 1u));
        sxy += (int64_t)(2 * k - (N - 1)) * (int64_t)B->speed[idx];
    }

    /* slope [mm/s per sample] = sxy / (2 * Sxx)
     * decel [mm/s^2]          = -slope * hz                            */
    {
        const int64_t num = -sxy * (int64_t)hz;
        const int64_t den = 2 * B->sxx;
        return (int32_t)(num / den);
    }
}

void edr_loopB_step(edr_loopB_t *B, const edr_config_t *cfg,
                    const edr_vehicle_sample_t *v, int32_t a_long_mm_s2)
{
    const uint16_t dt_ms = (uint16_t)(1000u / cfg->slow_hz);
    int32_t decel;

    B->fired_edge = false;

    if (v->speed_valid) {
        B->head = (uint16_t)((B->head + 1u) & (EDR_SPEED_RING_LEN - 1u));
        B->speed[B->head] = v->speed_mm_s;
        if (B->n < 0xFFFFFFFFu) { B->n++; }
        B->speed_now_mm_s = v->speed_mm_s;
        B->src_is_vss = true;
        decel = ls_decel(B, cfg->slow_hz);
    } else if (cfg->allow_accel_fallback) {
        /* Degraded path. -a_long because +X is forward: braking gives a
         * negative longitudinal acceleration. Heavily filtered because
         * this signal carries road grade and pitch as an error term. */
        B->src_is_vss = false;
        decel = -edr_biquad_step(&B->lp_accel_fb, a_long_mm_s2);
        B->n = 0u;                       /* invalidate the speed history */
    } else {
        B->decel_mm_s2 = 0;
        B->dwell_ms = 0u;
        B->state = EDR_B_IDLE;
        return;
    }

    B->decel_mm_s2 = decel;

    /* Below walking pace the slope estimate is dominated by quantisation
     * and by the wheel-speed sensor dropping out; suppress. */
    if (B->src_is_vss && B->speed_now_mm_s < cfg->decel_min_speed_mm_s &&
        B->state != EDR_B_DWELL) {
        B->state = EDR_B_IDLE;
        B->dwell_ms = 0u;
        return;
    }

    switch (B->state) {

    case EDR_B_IDLE:
        if (decel >= cfg->decel_thresh_mm_s2) {
            B->state = EDR_B_DWELL;
            /* GROUP-DELAY COMPENSATION. A centred least-squares slope over
             * an N-sample window estimates the derivative at the window
             * CENTRE, so the estimate lags the physical event by
             * regress_ms/2. Starting the dwell clock at zero on the first
             * threshold crossing therefore under-measures every dwell by
             * 100 ms, and a genuine 780 ms deceleration gets measured as
             * 680 ms and missed - a false negative on a MANDATED AIS-220
             * trigger. Credit the half-window back. */
            B->dwell_ms = (uint16_t)(cfg->decel_regress_ms / 2u);
            B->dropout_ms = 0u;
        } else {
            B->dwell_ms = 0u;
        }
        break;

    case EDR_B_DWELL:
        if (decel >= cfg->decel_rearm_mm_s2) {
            /* Hysteresis band: once dwelling, we tolerate a dip to the
             * re-arm level (gear shift, ABS cycling, brake modulation)
             * without discarding the accumulated dwell. */
            B->dwell_ms = (uint16_t)(B->dwell_ms + dt_ms);
            B->dropout_ms = 0u;
            if (B->dwell_ms >= cfg->decel_dwell_ms) {
                B->state = EDR_B_FIRED;
                B->fired_edge = true;
            }
        } else {
            B->dropout_ms = (uint16_t)(B->dropout_ms + dt_ms);
            if (B->dropout_ms > cfg->decel_dropout_ms) {
                B->state = EDR_B_IDLE;      /* not sustained -> discard   */
                B->dwell_ms = 0u;
                B->dropout_ms = 0u;
            } else {
                B->dwell_ms = (uint16_t)(B->dwell_ms + dt_ms);
            }
        }
        break;

    case EDR_B_FIRED:
        B->state = EDR_B_LATCHED;
        B->dwell_ms = (uint16_t)(B->dwell_ms + dt_ms);
        break;

    case EDR_B_LATCHED:
        /* Stay latched while the deceleration persists, so one long
         * braking event yields exactly one record, not one per tick. */
        if (decel < cfg->decel_rearm_mm_s2) {
            B->dropout_ms = (uint16_t)(B->dropout_ms + dt_ms);
            if (B->dropout_ms > cfg->decel_dropout_ms) {
                B->state = EDR_B_IDLE;
                B->dwell_ms = 0u;
                B->dropout_ms = 0u;
            }
        } else {
            B->dropout_ms = 0u;
            if (B->dwell_ms < 0xF000u) { B->dwell_ms = (uint16_t)(B->dwell_ms + dt_ms); }
        }
        break;

    default:
        B->state = EDR_B_IDLE;
        break;
    }
}

/* ---------------- seqlock publish / read ---------------------------- */

void edr_decel_publish(edr_decel_snapshot_t *snap, const edr_loopB_t *B)
{
    snap->seq++;                       /* odd = write in progress        */
    /* __DMB() here on target */
    snap->decel_mm_s2 = B->decel_mm_s2;
    snap->dwell_ms    = B->dwell_ms;
    snap->speed_mm_s  = B->speed_now_mm_s;
    snap->fired       = B->fired_edge;
    snap->src_is_vss  = B->src_is_vss;
    /* __DMB() here on target */
    snap->seq++;                       /* even = stable                  */
}

void edr_decel_read(const edr_decel_snapshot_t *snap, edr_decel_snapshot_t *out)
{
    uint32_t s0, s1;
    uint8_t  guard = 0u;
    do {
        s0 = snap->seq;
        *out = *snap;
        s1 = snap->seq;
        guard++;
    } while (((s0 & 1u) != 0u || s0 != s1) && guard < 4u);
    out->seq = s1;
}
