#include "edr_config.h"

void edr_config_defaults(edr_config_t *cfg)
{
    /* ---- rates ----------------------------------------------------- */
    cfg->fast_hz = 833;            /* ASM330LHBG1 ODR                    */
    cfg->slow_hz = 100;            /* CAN VSS frame rate                 */

    /* ---- front end -------------------------------------------------- */
    cfg->lp_disc_hz     = 100;     /* discrimination path                */
    cfg->lp_dv_hz       = 60;      /* ~SAE J211 CFC 60 for the dV path   */
    cfg->fs_limit_mm_s2 = 16 * EDR_G_MM_S2;   /* ASM330LHBG1 at +/-16 g  */
    cfg->bias_track_shift = 13;    /* tau ~ 10 s at 833 Hz               */
    cfg->bias_track_gate  = (3 * EDR_G_MM_S2) / 10;   /* 0.3 g           */
    cfg->bias_settle_ms   = 300;

    /* ---- LOOP A ----------------------------------------------------- */
    cfg->armA_accel_mm_s2  = (20 * EDR_G_MM_S2) / 10;  /* 2.0 g          */
    cfg->armA_dv_mm_s      = 400;                      /* 0.4 m/s in 30 ms*/
    cfg->dv_win_ms         = 150;                      /* [PRODUCT]      */
    cfg->dv_win_thresh_mm_s= EDR_KPH_TO_MM_S(25);      /* 6944 mm/s      */
    cfg->dv_short_win_ms   = 30;
    cfg->dv_seed_win_ms    = 60;
    cfg->dv_safing_win_ms  = 300;
    cfg->dv_safing_mm_s    = 2000;                     /* 2.0 m/s        */
    cfg->eval_timeout_ms   = 400;
    cfg->lockout_ms        = 500;

    /* dV-vs-time deployment boundary.
     * Monotonically INCREASING: a violent pulse banks its dV fast and
     * crosses early; a slow accumulation (hard braking, long downgrade)
     * can never catch a rising boundary. This is what separates "crash"
     * from "severe but survivable driving" without a second sensor.
     * The 150 ms point is pinned to the 25 km/h contractual spec.      */
    cfg->boundary[0] = (edr_dv_boundary_pt_t){  20,  3500 };
    cfg->boundary[1] = (edr_dv_boundary_pt_t){  40,  4500 };
    cfg->boundary[2] = (edr_dv_boundary_pt_t){  60,  5200 };
    cfg->boundary[3] = (edr_dv_boundary_pt_t){  90,  6000 };
    cfg->boundary[4] = (edr_dv_boundary_pt_t){ 120,  6600 };
    cfg->boundary[5] = (edr_dv_boundary_pt_t){ 150,  6944 };
    cfg->boundary[6] = (edr_dv_boundary_pt_t){ 250,  8500 };
    cfg->boundary[7] = (edr_dv_boundary_pt_t){ 400, 11000 };
    cfg->boundary_pts = 8;

    /* ---- immunity --------------------------------------------------- */
    cfg->osc_max_count      = 6;       /* sign changes in the window     */
    cfg->osc_win_ms         = 200;
    cfg->jerk_limit_mm_s3   = 2500000; /* 2500 m/s^3                     */
    cfg->jerk_dv_floor_mm_s = 2500;    /* high jerk + <2.5 m/s => misuse */

    /* ---- LOOP B (AIS-220 mandated numbers) -------------------------- */
    cfg->decel_thresh_mm_s2   = 3250;  /* 3.25 m/s^2                     */
    cfg->decel_rearm_mm_s2    = 3000;
    cfg->decel_dwell_ms       = 700;   /* sustained >= 0.7 s             */
    cfg->decel_dropout_ms     = 40;
    cfg->decel_regress_ms     = 200;
    cfg->decel_min_speed_mm_s = EDR_KPH_TO_MM_S(5);
    cfg->allow_accel_fallback = true;

    /* ---- event management ------------------------------------------- */
    cfg->multi_event_win_ms = 5000;
    cfg->lock_on_srs_only   = false;   /* see note below                 */
    cfg->lock_on_delta_v    = true;    /* [PRODUCT] no-SRS lock path     */

    /* ---- Annexure 4 recording --------------------------------------- */
    cfg->record_hz       = 4;
    cfg->record_fs_mm_s2 = (15 * EDR_G_MM_S2) / 10;    /* +/-1.5 g       */
    cfg->record_lsb_mm_s2= EDR_G_MM_S2 / 10;           /* 0.1 g          */
    cfg->precrash_ms     = 5000;
    cfg->postcrash_ms    = 250;
}

int edr_config_validate(const edr_config_t *cfg)
{
    uint32_t w;
    uint8_t  i;

    if (cfg->fast_hz < 100u || cfg->fast_hz > 2000u)          { return -1; }
    if (cfg->slow_hz < 10u  || cfg->slow_hz > 500u)           { return -2; }

    /* every window must fit the statically allocated ring */
    w = ((uint32_t)cfg->dv_safing_win_ms * cfg->fast_hz) / 1000u;
    if (w >= EDR_DV_WINDOW_MAX)                               { return -3; }
    if (cfg->dv_short_win_ms >= cfg->dv_win_ms)               { return -4; }
    if (cfg->dv_seed_win_ms  >  cfg->dv_win_ms)               { return -14; }
    if (cfg->dv_win_ms > cfg->dv_safing_win_ms)               { return -5; }

    w = ((uint32_t)cfg->decel_regress_ms * cfg->slow_hz) / 1000u;
    if (w < 3u || w >= EDR_SPEED_RING_LEN)                    { return -6; }

    if (cfg->boundary_pts < 2u ||
        cfg->boundary_pts > EDR_BOUNDARY_MAX_PTS)             { return -7; }
    for (i = 1u; i < cfg->boundary_pts; i++) {
        /* strictly increasing in time AND in threshold */
        if (cfg->boundary[i].t_ms  <= cfg->boundary[i - 1u].t_ms)  { return -8; }
        if (cfg->boundary[i].dv_mm_s < cfg->boundary[i - 1u].dv_mm_s) { return -9; }
    }

    if (cfg->decel_rearm_mm_s2 > cfg->decel_thresh_mm_s2)     { return -10; }
    if (cfg->record_lsb_mm_s2 == 0)                           { return -11; }
    if (cfg->record_hz == 0u || cfg->record_hz > 100u)        { return -12; }

    /* Anti-alias sanity: recording at 4 Hz demands a <2 Hz analogue-side
     * bandwidth. Enforced in edr_init() by the rec_aa biquad. */
    w = ((uint32_t)cfg->precrash_ms * cfg->record_hz) / 1000u;
    if (w > EDR_PRECRASH_SAMPLES)                             { return -13; }

    return 0;
}
