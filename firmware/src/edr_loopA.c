/* =====================================================================
 *  LOOP A  -  delta-V crash discrimination        (fast loop, 833 Hz)
 * ---------------------------------------------------------------------
 *  Pipeline per sample, per axis:
 *
 *    raw -> clip detect -> LPF(disc) -> bias removal ------> jerk, osc
 *                      \-> LPF(dV)   -> bias removal -> integrate -> dV
 *
 *  Firing logic (an event needs BOTH a severity criterion AND safing):
 *
 *      ( dV_150ms >= 25 km/h        [contractual spec]
 *        OR  dV_since_arm > boundary(t)   [severity discrimination] )
 *      AND  dV_300ms >= safing threshold  [independent concurrence]
 *      AND  NOT rough-road      AND  NOT jerk-spike
 *
 *  The three windows exist for three different jobs and must not be
 *  collapsed into one:
 *     30 ms  - arming only; fast enough not to lose the pulse front
 *    150 ms  - the specified severity measure
 *    300 ms  - safing; long enough that a pothole/hammer cannot fake it
 * ===================================================================== */
#include "edr_loops.h"

static uint16_t ms_to_samples(uint16_t ms, uint16_t hz)
{
    uint32_t n = ((uint32_t)ms * (uint32_t)hz + 500u) / 1000u;
    if (n == 0u) { n = 1u; }
    return (uint16_t)n;
}

int32_t edr_boundary_at(const edr_config_t *cfg, uint32_t t_ms)
{
    uint8_t i;

    if (cfg->boundary_pts == 0u) { return 0x7FFFFFFF; }
    if (t_ms <= cfg->boundary[0].t_ms) { return cfg->boundary[0].dv_mm_s; }

    for (i = 1u; i < cfg->boundary_pts; i++) {
        if (t_ms <= cfg->boundary[i].t_ms) {
            const int32_t  t0 = (int32_t)cfg->boundary[i - 1u].t_ms;
            const int32_t  t1 = (int32_t)cfg->boundary[i].t_ms;
            const int32_t  v0 = cfg->boundary[i - 1u].dv_mm_s;
            const int32_t  v1 = cfg->boundary[i].dv_mm_s;
            const int32_t  dt = t1 - t0;
            if (dt <= 0) { return v1; }
            return v0 + (int32_t)(((int64_t)(v1 - v0) * (int64_t)((int32_t)t_ms - t0)) / dt);
        }
    }
    /* beyond the last knee: hold, do not extrapolate */
    return cfg->boundary[cfg->boundary_pts - 1u].dv_mm_s;
}

void edr_loopA_init(edr_loopA_t *A, const edr_config_t *cfg)
{
    const uint32_t dt_us = 1000000u / cfg->fast_hz;
    int ax;

    A->state = EDR_A_IDLE;
    A->t_ms = 0u;
    A->t_arm_ms = 0u;
    A->lockout_until_ms = 0u;
    A->osc_count = 0u;
    A->osc_n = 0u;
    A->osc_win_samples = ms_to_samples(cfg->osc_win_ms, cfg->fast_hz);
    A->last_sign = 0;
    A->peak_jerk = 0;
    A->clip_count = 0u;
    A->dv_max_mm_s = 0;
    A->t_to_dv_max_ms = 0u;
    A->trigger_mask = 0u;
    A->trigger_latched = 0u;
    A->inhibit_mask = 0u;
    A->safing_ok = false;
    A->dv_final_mm_s = 0;
    A->t_trigger_ms = 0u;

    for (ax = 0; ax < EDR_AXIS_COUNT; ax++) {
        edr_biquad_design_lp(&A->lp_disc[ax], cfg->lp_disc_hz, cfg->fast_hz);
        edr_biquad_design_lp(&A->lp_dv[ax],   cfg->lp_dv_hz,   cfg->fast_hz);
        edr_bias_init(&A->bias[ax], cfg->bias_track_shift, cfg->bias_track_gate,
                      ms_to_samples(cfg->bias_settle_ms, cfg->fast_hz));
        edr_dvring_init(&A->ring[ax], dt_us,
                        ms_to_samples(cfg->dv_short_win_ms,  cfg->fast_hz),
                        ms_to_samples(cfg->dv_win_ms,        cfg->fast_hz),
                        ms_to_samples(cfg->dv_safing_win_ms, cfg->fast_hz),
                        ms_to_samples(cfg->dv_seed_win_ms,   cfg->fast_hz));
        A->a_filt[ax] = 0;
        A->a_prev[ax] = 0;
        A->jerk[ax] = 0;
        A->peak_a[ax] = 0;
    }
}

void edr_loopA_step(edr_loopA_t *A, const edr_config_t *cfg,
                    const edr_imu_sample_t *s, uint32_t t_ms)
{
    const int32_t raw[EDR_AXIS_COUNT] = { s->a_long_mm_s2, s->a_lat_mm_s2 };
    const bool    armed = (A->state == EDR_A_ARMED) ||
                          (A->state == EDR_A_CAPTURE);
    const uint32_t dt_us = 1000000u / cfg->fast_hz;
    int ax;
    int32_t dv_short_res, dv_main_res, dv_safing_res, dv_cum_res;
    int32_t a_disc_res;

    A->t_ms = t_ms;
    A->trigger_mask = 0u;
    A->inhibit_mask = 0u;

    if (!s->valid) {
        A->inhibit_mask |= EDR_INHIBIT_NOT_VALID;
        return;
    }
    if (s->clipped && A->clip_count < 0xFFFFu) { A->clip_count++; }

    /* ---------------- per-axis conditioning ------------------------- */
    for (ax = 0; ax < EDR_AXIS_COUNT; ax++) {
        const int32_t d = edr_biquad_step(&A->lp_disc[ax], raw[ax]);
        const int32_t v = edr_biquad_step(&A->lp_dv[ax],   raw[ax]);

        /* Bias is estimated on the discrimination path and applied to
         * BOTH, so the two paths share one DC reference. Frozen while
         * armed - see edr_filter.h for why this matters. */
        const int32_t d_nb = edr_bias_step(&A->bias[ax], d, armed);
        const int32_t b    = edr_bias_get(&A->bias[ax]);
        const int32_t v_nb = v - b;

        A->jerk[ax] = (int32_t)(((int64_t)(d_nb - A->a_prev[ax]) * 1000000LL) / (int64_t)dt_us);
        A->a_prev[ax] = d_nb;
        A->a_filt[ax] = d_nb;

        edr_dvring_push(&A->ring[ax], v_nb);

        if (armed || A->state == EDR_A_CAPTURE) {
            const int32_t mag  = (d_nb < 0) ? -d_nb : d_nb;
            const int32_t pmag = (A->peak_a[ax] < 0) ? -A->peak_a[ax] : A->peak_a[ax];
            if (mag > pmag) { A->peak_a[ax] = d_nb; }
        }
    }

    /* ---------------- resultant metrics ----------------------------- */
    a_disc_res    = edr_hypot(A->a_filt[EDR_AXIS_LONG],  A->a_filt[EDR_AXIS_LAT]);
    dv_short_res  = edr_hypot(edr_dvring_short(&A->ring[EDR_AXIS_LONG]),
                              edr_dvring_short(&A->ring[EDR_AXIS_LAT]));
    dv_main_res   = edr_hypot(edr_dvring_main(&A->ring[EDR_AXIS_LONG]),
                              edr_dvring_main(&A->ring[EDR_AXIS_LAT]));
    dv_safing_res = edr_hypot(edr_dvring_safing(&A->ring[EDR_AXIS_LONG]),
                              edr_dvring_safing(&A->ring[EDR_AXIS_LAT]));
    dv_cum_res    = edr_hypot(edr_dvring_cumulative(&A->ring[EDR_AXIS_LONG]),
                              edr_dvring_cumulative(&A->ring[EDR_AXIS_LAT]));

    /* ---------------- rough-road oscillation counter ----------------- */
    {
        const int8_t sign = (A->a_filt[EDR_AXIS_LONG] > (cfg->armA_accel_mm_s2 / 4)) ?  1 :
                            (A->a_filt[EDR_AXIS_LONG] < -(cfg->armA_accel_mm_s2 / 4)) ? -1 : 0;
        if (sign != 0) {
            if (A->last_sign != 0 && sign != A->last_sign && A->osc_count < 255u) {
                A->osc_count++;
            }
            A->last_sign = sign;
        }
        A->osc_n++;
        if (A->osc_n >= A->osc_win_samples) {   /* tumbling window reset */
            A->osc_n = 0u;
            A->osc_count = 0u;
            A->last_sign = 0;
        }
    }

    {
        const int32_t jl = (A->jerk[EDR_AXIS_LONG] < 0) ? -A->jerk[EDR_AXIS_LONG] : A->jerk[EDR_AXIS_LONG];
        if (jl > A->peak_jerk) { A->peak_jerk = jl; }
    }

    /* ---------------- state machine --------------------------------- */
    switch (A->state) {

    case EDR_A_LOCKOUT:
        if (t_ms >= A->lockout_until_ms) { A->state = EDR_A_IDLE; }
        else { A->inhibit_mask |= EDR_INHIBIT_LOCKOUT; }
        break;

    case EDR_A_IDLE:
        /* Hold the cumulative accumulator at zero while disarmed. A
         * free-running integrator would otherwise bank every gust, grade
         * and gear change of the whole drive cycle. */
        edr_dvring_reset_cumulative(&A->ring[EDR_AXIS_LONG]);
        edr_dvring_reset_cumulative(&A->ring[EDR_AXIS_LAT]);
        /* Two-of-two entry: an instantaneous level AND a short-window dV.
         * The level alone would arm on every kerb strike; the dV alone
         * would be too slow off the mark on a hard pulse. */
        if (!A->bias[EDR_AXIS_LONG].settled) {
            A->inhibit_mask |= EDR_INHIBIT_NOT_VALID;
        } else if (a_disc_res >= cfg->armA_accel_mm_s2 &&
                   dv_short_res >= cfg->armA_dv_mm_s) {
            A->state = EDR_A_ARMED;
            A->t_arm_ms = t_ms;
            A->dv_max_mm_s = 0;
            A->t_to_dv_max_ms = 0u;
            A->clip_count = s->clipped ? 1u : 0u;
            A->peak_jerk = 0;
            A->peak_a[EDR_AXIS_LONG] = A->a_filt[EDR_AXIS_LONG];
            A->peak_a[EDR_AXIS_LAT]  = A->a_filt[EDR_AXIS_LAT];
            edr_dvring_seed_cumulative(&A->ring[EDR_AXIS_LONG]);
            edr_dvring_seed_cumulative(&A->ring[EDR_AXIS_LAT]);
        }
        break;

    case EDR_A_ARMED: {
        const uint32_t dt = t_ms - A->t_arm_ms;
        const int32_t  bnd = edr_boundary_at(cfg, dt);
        uint16_t sev = 0u;

        if (dv_cum_res > A->dv_max_mm_s) {
            A->dv_max_mm_s = dv_cum_res;
            A->t_to_dv_max_ms = (uint16_t)dt;
        }

        A->safing_ok = (dv_safing_res >= cfg->dv_safing_mm_s);

        /* --- severity criteria (OR) --- */
        if (dv_main_res >= cfg->dv_win_thresh_mm_s) { sev |= EDR_TRG_DELTA_V_WINDOW; }
        if (dv_cum_res  >= bnd)                     { sev |= EDR_TRG_DELTA_V_CURVE;  }

        /* --- inhibits (AND-NOT) --- */
        if (A->osc_count >= cfg->osc_max_count && dv_main_res < cfg->dv_win_thresh_mm_s) {
            A->inhibit_mask |= EDR_INHIBIT_ROUGH_ROAD;
        }
        if (A->peak_jerk >= cfg->jerk_limit_mm_s3 && dv_safing_res < cfg->jerk_dv_floor_mm_s) {
            A->inhibit_mask |= EDR_INHIBIT_JERK_SPIKE;
        }
        if (!A->safing_ok) {
            A->inhibit_mask |= EDR_INHIBIT_NO_SAFING;
        }

        if (sev != 0u && A->inhibit_mask == 0u) {
            /* Decision is made HERE - t_trigger_ms is the legally relevant
             * instant and is what goes in the record as t0. Data capture
             * continues for the post-trigger window so the stored dV is
             * the dV of the collision, not the dV at the moment the
             * threshold happened to be crossed. */
            A->trigger_latched = sev;
            A->t_trigger_ms = t_ms;
            A->state = EDR_A_CAPTURE;
        } else if (dt >= cfg->eval_timeout_ms) {
            /* Nothing crossed inside the evaluation horizon: this was not
             * a crash. Disarm, unfreeze the bias tracker, do not lock. */
            A->state = EDR_A_LOCKOUT;
            A->lockout_until_ms = t_ms + cfg->lockout_ms;
        }
        break;
    }

    case EDR_A_CAPTURE:
        if (dv_cum_res > A->dv_max_mm_s) {
            A->dv_max_mm_s = dv_cum_res;
            A->t_to_dv_max_ms = (uint16_t)(t_ms - A->t_arm_ms);
        }
        if ((t_ms - A->t_trigger_ms) >= cfg->postcrash_ms) {
            A->dv_final_mm_s = dv_cum_res;
            A->trigger_mask = A->trigger_latched;   /* one-tick: file it */
            A->state = EDR_A_FIRED;
        }
        break;

    case EDR_A_FIRED:
        /* The arbiter consumed the record on the previous tick; drop into
         * lockout so one collision cannot emit a burst of duplicates. */
        A->trigger_latched = 0u;
        A->state = EDR_A_LOCKOUT;
        A->lockout_until_ms = t_ms + cfg->lockout_ms;
        break;

    default:
        A->state = EDR_A_IDLE;
        break;
    }
}
