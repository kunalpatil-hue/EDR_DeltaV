/* =====================================================================
 *  edr_sim_shim.c  -  PC-simulator shim (NOT compiled into the ECU image)
 * ---------------------------------------------------------------------
 *  Exposes a flat, ctypes-friendly batch API so the Python harness can
 *  drive the *identical* compiled core that ships on the S32K312.
 *
 *  Deliberately avoids exporting C structs: mirroring struct layout in
 *  ctypes is the classic way for a simulator to silently diverge from
 *  the target after someone adds a field. Everything crosses the
 *  boundary as scalars and flat arrays keyed by name.
 * ===================================================================== */
#include "edr_api.h"
#include <stdlib.h>
#include <string.h>

#define EV_STRIDE 22   /* doubles per event - keep in sync with edr_core.py */

typedef struct {
    edr_ctx_t    ctx;
    edr_config_t cfg;
    int          initialised;
} sim_t;

void *edr_sim_create(void)
{
    sim_t *h = (sim_t *)calloc(1u, sizeof(sim_t));
    if (h != NULL) {
        edr_config_defaults(&h->cfg);
        h->initialised = 0;
    }
    return (void *)h;
}

void edr_sim_destroy(void *p) { free(p); }

/* ---- name-keyed config access -------------------------------------- */
#define SET_U(field)  if (strcmp(key, #field) == 0) { h->cfg.field = (uint16_t)v;  return 0; }
#define SET_I(field)  if (strcmp(key, #field) == 0) { h->cfg.field = (int32_t)v;   return 0; }
#define SET_B(field)  if (strcmp(key, #field) == 0) { h->cfg.field = (v != 0.0);   return 0; }
#define SET_8(field)  if (strcmp(key, #field) == 0) { h->cfg.field = (uint8_t)v;   return 0; }

int edr_sim_set(void *p, const char *key, double v)
{
    sim_t *h = (sim_t *)p;
    if (h == NULL || key == NULL) { return -1; }

    SET_U(fast_hz) SET_U(slow_hz) SET_U(lp_disc_hz) SET_U(lp_dv_hz)
    SET_I(fs_limit_mm_s2) SET_8(bias_track_shift) SET_I(bias_track_gate)
    SET_U(bias_settle_ms)
    SET_I(armA_accel_mm_s2) SET_I(armA_dv_mm_s)
    SET_U(dv_win_ms) SET_I(dv_win_thresh_mm_s) SET_U(dv_short_win_ms)
    SET_U(dv_safing_win_ms) SET_I(dv_safing_mm_s)
    SET_U(eval_timeout_ms) SET_U(lockout_ms)
    SET_8(osc_max_count) SET_U(osc_win_ms)
    SET_I(jerk_limit_mm_s3) SET_I(jerk_dv_floor_mm_s)
    SET_I(decel_thresh_mm_s2) SET_I(decel_rearm_mm_s2)
    SET_U(decel_dwell_ms) SET_U(decel_dropout_ms) SET_U(decel_regress_ms)
    SET_I(decel_min_speed_mm_s) SET_B(allow_accel_fallback)
    SET_U(multi_event_win_ms) SET_B(lock_on_srs_only) SET_B(lock_on_delta_v)
    SET_U(record_hz) SET_I(record_fs_mm_s2) SET_I(record_lsb_mm_s2)
    SET_U(precrash_ms) SET_U(postcrash_ms)

    return -2;   /* unknown key */
}

/* Replace the whole dV-vs-t boundary. */
int edr_sim_set_boundary(void *p, const int *t_ms, const int *dv_mm_s, int n)
{
    sim_t *h = (sim_t *)p;
    int i;
    if (h == NULL || n < 2 || n > (int)EDR_BOUNDARY_MAX_PTS) { return -1; }
    for (i = 0; i < n; i++) {
        h->cfg.boundary[i].t_ms   = (uint16_t)t_ms[i];
        h->cfg.boundary[i].dv_mm_s = dv_mm_s[i];
    }
    h->cfg.boundary_pts = (uint8_t)n;
    return 0;
}

int edr_sim_init(void *p)
{
    sim_t *h = (sim_t *)p;
    int rc;
    if (h == NULL) { return -100; }
    rc = edr_config_validate(&h->cfg);
    if (rc != 0) { return rc; }
    edr_init(&h->ctx, &h->cfg);
    h->initialised = 1;
    return 0;
}

double edr_sim_boundary_at(void *p, int t_ms)
{
    sim_t *h = (sim_t *)p;
    return (double)edr_boundary_at(&h->cfg, (uint32_t)t_ms);
}

/* ---- batch run ------------------------------------------------------ */
/* Inputs are expected at cfg.fast_hz (resampled by the Python harness).
 * Accelerations in g, speed in km/h. */
int edr_sim_run(void *p, int n,
                const double *ax_g, const double *ay_g, const double *az_g,
                const double *speed_kph, int speed_valid,
                const int *srs_flag,
                double *o_a_long, double *o_a_lat,
                double *o_bias_long,
                double *o_dv_main, double *o_dv_cum, double *o_dv_safing,
                double *o_boundary, double *o_decel, double *o_dwell,
                int *o_stateA, int *o_stateB,
                int *o_trigger, int *o_inhibit,
                int max_ev, double *ev_out, int *n_ev_out)
{
    sim_t *h = (sim_t *)p;
    edr_debug_t d;
    edr_imu_sample_t s;
    edr_vehicle_sample_t v;
    edr_event_t evbuf[EDR_MAX_EVENTS_PER_SEQ];
    int i, k, nev = 0;
    uint32_t slow_acc = 0u;
    const double fs = (double)h->cfg.fs_limit_mm_s2;

    if (h == NULL || h->initialised == 0) { return -1; }

    for (i = 0; i < n; i++) {
        /* -------- slow loop first: Loop B publishes before the arbiter
         * reads it on this tick, matching the target task priority. --- */
        /* Bresenham rather than fast_hz/slow_hz: 833/100 truncates to 8,
         * which would run the slow loop at 104.125 Hz while Loop B scales
         * its slope by the configured 100 Hz - a silent 4% error on every
         * deceleration reading. On the ECU the slow loop has its own
         * timer and the issue does not arise; in the simulator it must be
         * reproduced exactly or the calibration is tuned against a lie. */
        slow_acc += h->cfg.slow_hz;
        if (slow_acc >= h->cfg.fast_hz) {
            slow_acc -= h->cfg.fast_hz;
            v.speed_mm_s   = (speed_kph != NULL)
                           ? (int32_t)(speed_kph[i] * 277.7778 + 0.5) : 0;
            v.speed_valid  = (speed_kph != NULL) && (speed_valid != 0);
            v.srs_deployed = (srs_flag != NULL) ? (srs_flag[i] != 0) : false;
            v.brake_active = false;
            (void)edr_slow_tick(&h->ctx, &v);
        }

        /* -------- fast loop --------------------------------------- */
        {
            double al = ax_g[i] * (double)EDR_G_MM_S2;
            double at = ay_g[i] * (double)EDR_G_MM_S2;
            double av = (az_g != NULL) ? az_g[i] * (double)EDR_G_MM_S2 : 0.0;
            s.clipped = false;
            if (al >  fs) { al =  fs; s.clipped = true; }
            if (al < -fs) { al = -fs; s.clipped = true; }
            if (at >  fs) { at =  fs; s.clipped = true; }
            if (at < -fs) { at = -fs; s.clipped = true; }
            s.a_long_mm_s2 = (int32_t)al;
            s.a_lat_mm_s2  = (int32_t)at;
            s.a_vert_mm_s2 = (int32_t)av;
            s.valid = true;
        }
        (void)edr_fast_tick(&h->ctx, &s);

        edr_get_debug(&h->ctx, &d);
        o_a_long[i]    = (double)d.a_long_filt / (double)EDR_G_MM_S2;
        o_a_lat[i]     = (double)d.a_lat_filt  / (double)EDR_G_MM_S2;
        o_bias_long[i] = (double)d.bias_long   / (double)EDR_G_MM_S2;
        o_dv_main[i]   = (double)d.dv_main   / 1000.0;
        o_dv_cum[i]    = (double)d.dv_cum    / 1000.0;
        o_dv_safing[i] = (double)d.dv_safing / 1000.0;
        o_boundary[i]  = (double)d.dv_boundary / 1000.0;
        o_decel[i]     = (double)d.decel_mm_s2 / 1000.0;
        o_dwell[i]     = (double)d.decel_dwell_ms;
        o_stateA[i]    = (int)d.stateA;
        o_stateB[i]    = (int)d.stateB;
        o_trigger[i]   = (int)d.trigger_mask;
        o_inhibit[i]   = (int)d.inhibit_mask;

        /* drain events as they appear so multi-event sequences are not
         * overwritten when a new sequence opens */
        {
            uint32_t got = edr_poll_events(&h->ctx, evbuf, EDR_MAX_EVENTS_PER_SEQ);
            for (k = 0; k < (int)got && nev < max_ev; k++) {
                double *e = &ev_out[(size_t)nev * EV_STRIDE];
                e[0]  = (double)evbuf[k].t0_ms / 1000.0;
                e[1]  = (double)evbuf[k].seq_index;
                e[2]  = (double)evbuf[k].trigger_mask;
                e[3]  = (double)evbuf[k].inhibit_mask;
                e[4]  = (double)evbuf[k].dv_long_mm_s / 1000.0;
                e[5]  = (double)evbuf[k].dv_lat_mm_s  / 1000.0;
                e[6]  = (double)evbuf[k].dv_res_mm_s  / 1000.0;
                e[7]  = (double)evbuf[k].dv_max_mm_s  / 1000.0;
                e[8]  = (double)evbuf[k].t_to_dv_max_ms;
                e[9]  = (double)evbuf[k].peak_a_long_mm_s2 / (double)EDR_G_MM_S2;
                e[10] = (double)evbuf[k].peak_a_lat_mm_s2  / (double)EDR_G_MM_S2;
                e[11] = (double)evbuf[k].pdof_ddeg / 10.0;
                e[12] = (double)evbuf[k].decel_rate_mm_s2 / 1000.0;
                e[13] = (double)evbuf[k].decel_dwell_ms;
                e[14] = (double)evbuf[k].speed_at_t0_mm_s * 0.0036;
                e[15] = evbuf[k].locked ? 1.0 : 0.0;
                e[16] = evbuf[k].dv_is_lower_bound ? 1.0 : 0.0;
                e[17] = evbuf[k].decel_src_is_vss ? 1.0 : 0.0;
                e[18] = 0.0; e[19] = 0.0; e[20] = 0.0; e[21] = 0.0;
                nev++;
            }
        }
    }

    *n_ev_out = nev;
    return 0;
}

/* ---- Annexure 4 record readback ------------------------------------ */
int edr_sim_get_record(void *p, double *a_long_g, double *a_lat_g,
                       double *speed_kph, int *sat, int max_n)
{
    sim_t *h = (sim_t *)p;
    int i, n;
    if (h == NULL) { return 0; }
    n = (int)h->ctx.rec_n;
    if (n > max_n) { n = max_n; }
    for (i = 0; i < n; i++) {
        /* oldest first */
        const uint8_t cap = h->ctx.rec_n;
        const uint8_t idx = (uint8_t)((h->ctx.rec_head + 1u + (uint8_t)i) % cap);
        a_long_g[i]  = (double)h->ctx.rec[idx].a_long_dg / 10.0;
        a_lat_g[i]   = (double)h->ctx.rec[idx].a_lat_dg  / 10.0;
        speed_kph[i] = (double)h->ctx.rec[idx].speed_kph_x10 / 10.0;
        sat[i]       = (int)(h->ctx.rec[idx].flags & 1u);
    }
    return n;
}
