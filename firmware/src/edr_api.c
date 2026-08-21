/* =====================================================================
 *  edr_api.c  -  Arbiter, multi-event sequencing, Annexure 4 recorder
 * ---------------------------------------------------------------------
 *  LOCKING POLICY (this is where regulation and product part company):
 *
 *    AIS-220 5.3.2 : an event record is write-protected (locked) when the
 *                    SRS deploys. On an N2/N3 truck with no airbag that
 *                    condition never occurs, so a strictly conformant
 *                    implementation never locks anything.
 *
 *    MileEdge      : cfg.lock_on_delta_v adds a second lock path driven
 *                    by Loop A. This is a PRODUCT decision, not a
 *                    compliance requirement, and it is behind a config
 *                    flag precisely so the safety case can state which
 *                    behaviour was shipped for which platform.
 * ===================================================================== */
#include "edr_api.h"

static uint16_t ms_to_samp(uint16_t ms, uint16_t hz)
{
    uint32_t n = ((uint32_t)ms * (uint32_t)hz + 500u) / 1000u;
    if (n == 0u) { n = 1u; }
    return (uint16_t)n;
}

void edr_init(edr_ctx_t *ctx, const edr_config_t *cfg)
{
    int ax;

    ctx->cfg = *cfg;
    edr_loopA_init(&ctx->A, &ctx->cfg);
    edr_loopB_init(&ctx->B, &ctx->cfg);

    ctx->snap.seq = 0u;
    ctx->snap.decel_mm_s2 = 0;
    ctx->snap.dwell_ms = 0u;
    ctx->snap.speed_mm_s = 0;
    ctx->snap.fired = false;
    ctx->snap.src_is_vss = true;

    ctx->t_fast_ms = 0u;
    ctx->t_slow_ms = 0u;
    ctx->fast_n = 0u;
    ctx->slow_n = 0u;

    ctx->event_count = 0u;
    ctx->event_read = 0u;
    ctx->seq_t0_ms = 0u;
    ctx->seq_open = false;
    ctx->seq_cum_dv_mm_s = 0;
    ctx->srs_seen = false;
    ctx->last_decel_seq = 0u;
    ctx->prevA = 0u;
    ctx->evt_peak_decel = 0;
    ctx->evt_speed_at_arm = 0;

    ctx->rec_head = 0u;
    ctx->rec_n = 0u;
    /* Bresenham, not integer division: 833/4 truncates to 208, giving a
     * 4.0048 Hz record clock. Over a 5 s pre-crash buffer that is 6 ms of
     * timebase drift on an evidential record - small, but it is exactly
     * the kind of thing a defence expert gets to ask about. */
    ctx->rec_acc = 0u;

    /* Anti-alias ahead of 4 Hz decimation. Without this, a 30 Hz cab
     * resonance folds straight into the regulated record as a plausible
     * looking 2 Hz artefact. Corner at record_hz/2 rounded down. */
    for (ax = 0; ax < EDR_AXIS_COUNT; ax++) {
        uint16_t fc = (uint16_t)(ctx->cfg.record_hz / 2u);
        if (fc == 0u) { fc = 1u; }
        edr_biquad_design_lp(&ctx->rec_aa[ax], fc, ctx->cfg.fast_hz);
    }
}

/* ---------------- Annexure 4 quantiser ------------------------------ */
static int8_t quantise_dg(int32_t a_mm_s2, const edr_config_t *cfg, bool *sat)
{
    int32_t q;
    const int32_t lsb = cfg->record_lsb_mm_s2;
    const int32_t fs  = cfg->record_fs_mm_s2;

    *sat = false;
    if (a_mm_s2 >  fs) { a_mm_s2 =  fs; *sat = true; }
    if (a_mm_s2 < -fs) { a_mm_s2 = -fs; *sat = true; }

    q = (a_mm_s2 >= 0) ? (a_mm_s2 + lsb / 2) / lsb
                       : (a_mm_s2 - lsb / 2) / lsb;
    if (q >  127) { q =  127; }
    if (q < -128) { q = -128; }
    return (int8_t)q;
}

static void record_push(edr_ctx_t *ctx, int32_t a_long, int32_t a_lat, int32_t speed_mm_s)
{
    bool s1, s2;
    edr_record_sample_t *r;
    const uint8_t depth = (uint8_t)((ctx->cfg.precrash_ms * ctx->cfg.record_hz) / 1000u);
    const uint8_t cap = (depth < EDR_PRECRASH_SAMPLES) ? depth : (uint8_t)EDR_PRECRASH_SAMPLES;

    ctx->rec_head = (uint8_t)((ctx->rec_head + 1u) % cap);
    r = &ctx->rec[ctx->rec_head];
    r->a_long_dg = quantise_dg(a_long, &ctx->cfg, &s1);
    r->a_lat_dg  = quantise_dg(a_lat,  &ctx->cfg, &s2);
    r->flags     = (uint8_t)((s1 || s2) ? 1u : 0u);
    /* speed in 0.1 km/h : mm/s * 10 / 277.78 = mm/s * 36 / 1000 */
    {
        int32_t kph10 = (int32_t)(((int64_t)speed_mm_s * 36) / 1000);
        if (kph10 < 0) { kph10 = 0; }
        if (kph10 > 65535) { kph10 = 65535; }
        r->speed_kph_x10 = (uint16_t)kph10;
    }
    if (ctx->rec_n < cap) { ctx->rec_n++; }
}

/* ---------------- event construction -------------------------------- */
static void push_event(edr_ctx_t *ctx, uint16_t trigger_mask, bool lock,
                       uint32_t t0_ms, bool use_cumulative)
{
    edr_event_t *e;
    const edr_loopA_t *A = &ctx->A;
    edr_decel_snapshot_t sn;

    /* Multi-event sequencing: AIS-220 treats collisions inside one window
     * as a single sequence with ordered sub-events. Open the sequence on
     * the first trigger, close it when the window expires. */
    if (ctx->seq_open && (ctx->t_fast_ms - ctx->seq_t0_ms) > ctx->cfg.multi_event_win_ms) {
        ctx->seq_open = false;
        ctx->seq_cum_dv_mm_s = 0;
    }
    if (!ctx->seq_open) {
        ctx->seq_open = true;
        ctx->seq_t0_ms = ctx->t_fast_ms;
        ctx->event_count = 0u;
        ctx->event_read = 0u;
        ctx->seq_cum_dv_mm_s = 0;
    }
    if (ctx->event_count >= EDR_MAX_EVENTS_PER_SEQ) { return; }

    edr_decel_read(&ctx->snap, &sn);

    e = &ctx->events[ctx->event_count];
    /* t0 = the instant the decision was made, NOT the instant the record
     * was finalised. For a Loop A event these differ by the capture
     * window, and it is the decision instant that a reconstruction has to
     * line up against the rest of the vehicle's data. */
    e->t0_ms            = t0_ms;
    e->seq_index        = ctx->event_count;
    e->trigger_mask     = trigger_mask;
    e->inhibit_mask     = A->inhibit_mask;

    /* Loop A: dV of the COLLISION = cumulative since arming (seeded from
     * the pre-arm window), integrated through the post-trigger capture.
     * The 150 ms sliding window is the DISCRIMINATION measure; it is not
     * the quantity a crash reconstructionist needs out of the record.
     * Loop B: there is no collision to integrate, so the sliding window
     * is the honest figure - it describes the braking interval itself. */
    if (use_cumulative) {
        e->dv_long_mm_s = edr_dvring_cumulative(&A->ring[EDR_AXIS_LONG]);
        e->dv_lat_mm_s  = edr_dvring_cumulative(&A->ring[EDR_AXIS_LAT]);
    } else {
        e->dv_long_mm_s = edr_dvring_main(&A->ring[EDR_AXIS_LONG]);
        e->dv_lat_mm_s  = edr_dvring_main(&A->ring[EDR_AXIS_LAT]);
    }
    e->dv_res_mm_s      = edr_hypot(e->dv_long_mm_s, e->dv_lat_mm_s);
    e->dv_max_mm_s      = use_cumulative ? A->dv_max_mm_s : e->dv_res_mm_s;
    e->t_to_dv_max_ms   = A->t_to_dv_max_ms;

    e->peak_a_long_mm_s2 = A->peak_a[EDR_AXIS_LONG];
    e->peak_a_lat_mm_s2  = A->peak_a[EDR_AXIS_LAT];
    /* PDOF: 0 deg = force from straight ahead. dV_long is negative in a
     * frontal impact, so negate to get the force direction. */
    e->pdof_ddeg        = edr_atan2_ddeg(-e->dv_lat_mm_s, -e->dv_long_mm_s);

    /* For a collision the instantaneous decel at filing time is useless
     * (the vehicle has usually stopped); file the peak seen during the
     * event and the road speed at arming. */
    e->decel_rate_mm_s2 = use_cumulative ? ctx->evt_peak_decel : sn.decel_mm_s2;
    e->decel_dwell_ms   = sn.dwell_ms;
    e->speed_at_t0_mm_s = use_cumulative ? ctx->evt_speed_at_arm : sn.speed_mm_s;
    e->decel_src_is_vss = sn.src_is_vss;

    e->locked           = lock;
    /* If the front end clipped during the window, the integral is a
     * LOWER BOUND. Recording that fact is a chain-of-custody matter: a
     * reconstructionist must never read a clipped dV as the true dV. */
    e->dv_is_lower_bound = (A->clip_count > 0u);

    ctx->seq_cum_dv_mm_s += e->dv_res_mm_s;
    ctx->event_count++;
}

/* ---------------- fast tick ----------------------------------------- */
uint16_t edr_fast_tick(edr_ctx_t *ctx, const edr_imu_sample_t *s)
{
    uint16_t fired = 0u;
    edr_decel_snapshot_t sn;

    ctx->t_fast_ms = (uint32_t)(((uint64_t)ctx->fast_n * 1000u) / ctx->cfg.fast_hz);
    ctx->fast_n++;

    edr_loopA_step(&ctx->A, &ctx->cfg, s, ctx->t_fast_ms);

    /* --- Annexure 4 record path (independent of any trigger) --------- */
    {
        const int32_t ral = edr_biquad_step(&ctx->rec_aa[EDR_AXIS_LONG], ctx->A.a_filt[EDR_AXIS_LONG]);
        const int32_t rat = edr_biquad_step(&ctx->rec_aa[EDR_AXIS_LAT],  ctx->A.a_filt[EDR_AXIS_LAT]);
        ctx->rec_acc += ctx->cfg.record_hz;
        if (ctx->rec_acc >= ctx->cfg.fast_hz) {
            ctx->rec_acc -= ctx->cfg.fast_hz;
            edr_decel_read(&ctx->snap, &sn);
            record_push(ctx, ral, rat, sn.speed_mm_s);
        }
    }

    /* --- arbiter ----------------------------------------------------- */
    edr_decel_read(&ctx->snap, &sn);

    /* Latch vehicle context on the Loop A arming edge and track the peak
     * deceleration across the whole event window. */
    if (ctx->A.state == EDR_A_ARMED && ctx->prevA == (uint8_t)EDR_A_IDLE) {
        ctx->evt_peak_decel   = sn.decel_mm_s2;
        ctx->evt_speed_at_arm = sn.speed_mm_s;
    } else if (ctx->A.state == EDR_A_ARMED || ctx->A.state == EDR_A_CAPTURE) {
        if (sn.decel_mm_s2 > ctx->evt_peak_decel) { ctx->evt_peak_decel = sn.decel_mm_s2; }
    }
    ctx->prevA = (uint8_t)ctx->A.state;

    /* 1. SRS deployment: the AIS-220 conformant lock path. */
    if (ctx->srs_seen) {
        ctx->srs_seen = false;
        fired |= EDR_TRG_SRS_DEPLOY;
        push_event(ctx, EDR_TRG_SRS_DEPLOY, true, ctx->t_fast_ms, false);
    }

    /* 2. Loop A: severity + safing already AND-ed inside the loop. */
    if (ctx->A.trigger_mask != 0u) {
        const bool lock = ctx->cfg.lock_on_delta_v && !ctx->cfg.lock_on_srs_only;
        fired |= ctx->A.trigger_mask;
        push_event(ctx, ctx->A.trigger_mask, lock, ctx->A.t_trigger_ms, true);
    }

    /* 3. Loop B: mandated recording trigger, never locks on its own.
     * The snapshot is refreshed at slow_hz but read at fast_hz, so the
     * edge must be consumed exactly once - keyed on the seqlock counter. */
    if (sn.fired && sn.seq != ctx->last_decel_seq) {
        ctx->last_decel_seq = sn.seq;
        fired |= EDR_TRG_SUDDEN_DECEL;
        push_event(ctx, EDR_TRG_SUDDEN_DECEL, false, ctx->t_fast_ms, false);
    }

    return fired;
}

/* ---------------- slow tick ----------------------------------------- */
uint16_t edr_slow_tick(edr_ctx_t *ctx, const edr_vehicle_sample_t *v)
{
    ctx->t_slow_ms = (uint32_t)(((uint64_t)ctx->slow_n * 1000u) / ctx->cfg.slow_hz);
    ctx->slow_n++;

    edr_loopB_step(&ctx->B, &ctx->cfg, v, ctx->A.a_filt[EDR_AXIS_LONG]);
    edr_decel_publish(&ctx->snap, &ctx->B);

    if (v->srs_deployed) { ctx->srs_seen = true; }

    return ctx->B.fired_edge ? (uint16_t)EDR_TRG_SUDDEN_DECEL : 0u;
}

/* ---------------- event drain --------------------------------------- */
uint32_t edr_poll_events(edr_ctx_t *ctx, edr_event_t *out, uint32_t max)
{
    uint32_t n = 0u;
    while (ctx->event_read < ctx->event_count && n < max) {
        out[n] = ctx->events[ctx->event_read];
        ctx->event_read++;
        n++;
    }
    return n;
}

/* ---------------- debug --------------------------------------------- */
void edr_get_debug(const edr_ctx_t *ctx, edr_debug_t *d)
{
    const edr_loopA_t *A = &ctx->A;

    d->a_long_filt = A->a_filt[EDR_AXIS_LONG];
    d->a_lat_filt  = A->a_filt[EDR_AXIS_LAT];
    d->bias_long   = edr_bias_get(&A->bias[EDR_AXIS_LONG]);
    d->bias_lat    = edr_bias_get(&A->bias[EDR_AXIS_LAT]);

    d->dv_short  = edr_hypot(edr_dvring_short(&A->ring[EDR_AXIS_LONG]),
                             edr_dvring_short(&A->ring[EDR_AXIS_LAT]));
    d->dv_main   = edr_hypot(edr_dvring_main(&A->ring[EDR_AXIS_LONG]),
                             edr_dvring_main(&A->ring[EDR_AXIS_LAT]));
    d->dv_safing = edr_hypot(edr_dvring_safing(&A->ring[EDR_AXIS_LONG]),
                             edr_dvring_safing(&A->ring[EDR_AXIS_LAT]));
    d->dv_cum    = edr_hypot(edr_dvring_cumulative(&A->ring[EDR_AXIS_LONG]),
                             edr_dvring_cumulative(&A->ring[EDR_AXIS_LAT]));

    /* Report the boundary through CAPTURE as well as ARMED: the tick on
     * which the decision is taken has already advanced the state machine,
     * so gating on ARMED alone hides the boundary value at exactly the
     * sample a calibration engineer most wants to see. */
    d->dv_boundary = (A->state == EDR_A_ARMED || A->state == EDR_A_CAPTURE)
                   ? edr_boundary_at(&ctx->cfg, ctx->t_fast_ms - A->t_arm_ms)
                   : 0;

    d->decel_mm_s2    = ctx->B.decel_mm_s2;
    d->decel_dwell_ms = ctx->B.dwell_ms;
    d->stateA = (uint8_t)A->state;
    d->stateB = (uint8_t)ctx->B.state;
    d->osc_count = A->osc_count;
    d->jerk = A->jerk[EDR_AXIS_LONG];
    d->trigger_mask = A->trigger_mask;
    d->inhibit_mask = A->inhibit_mask;
    d->safing_ok = A->safing_ok;

    (void)ms_to_samp;
}
