/* =====================================================================
 *  edr_config.h  -  Calibration set (data-driven trigger engine)
 * ---------------------------------------------------------------------
 *  NOTHING in the algorithm is hard-coded. Every threshold lives here so
 *  the same binary can be recalibrated per platform (MTB tractor vs AL
 *  N3 truck) from a signed calibration blob, without a code change.
 *
 *  REGULATORY PROVENANCE OF EACH VALUE IS TAGGED:
 *    [AIS-220]  = mandated by AIS-220 / UN R169
 *    [PRODUCT]  = MileEdge/AEPL value-add, NOT a compliance requirement
 *    [ENG]      = engineering/immunity value, tune on vehicle
 * ===================================================================== */
#ifndef EDR_CONFIG_H
#define EDR_CONFIG_H

#include "edr_types.h"

/* dV-vs-time deployment boundary point */
typedef struct {
    uint16_t  t_ms;               /* time since Loop A armed             */
    int32_t   dv_mm_s;            /* required cumulative dV at that time */
} edr_dv_boundary_pt_t;

typedef struct {
    /* ---------------- sample rates ---------------------------------- */
    uint16_t  fast_hz;            /* IMU / Loop A     (833 or 1667 Hz)   */
    uint16_t  slow_hz;            /* CAN  / Loop B    (100 Hz typical)   */

    /* ---------------- front-end conditioning ------------------------ */
    uint16_t  lp_disc_hz;         /* [ENG] LPF for discrimination path   */
    uint16_t  lp_dv_hz;           /* [ENG] LPF for dV integration path   */
    int32_t   fs_limit_mm_s2;     /* IMU full scale (clip detection)     */
    uint8_t   bias_track_shift;   /* leaky-integrator shift (2^-N)       */
    int32_t   bias_track_gate;    /* freeze tracker above this |a|       */
    uint16_t  bias_settle_ms;     /* startup blanking                    */

    /* ---------------- LOOP A : delta-V ------------------------------ */
    int32_t   armA_accel_mm_s2;   /* [ENG] entry threshold on |a_filt|   */
    int32_t   armA_dv_mm_s;       /* [ENG] entry threshold on 30 ms dV   */
    uint16_t  dv_win_ms;          /* [PRODUCT] 150 ms sliding window     */
    int32_t   dv_win_thresh_mm_s; /* [PRODUCT] 25 km/h = 6944 mm/s       */
    uint16_t  dv_short_win_ms;    /* [ENG] 30 ms arming window           */
    uint16_t  dv_seed_win_ms;     /* [ENG] pre-arm seed, recovers onset  */
    uint16_t  dv_safing_win_ms;   /* [ENG] 300 ms independent safing     */
    int32_t   dv_safing_mm_s;     /* [ENG] safing concurrence threshold  */
    uint16_t  eval_timeout_ms;    /* [ENG] disarm if boundary not met    */
    uint16_t  lockout_ms;         /* [ENG] re-arm inhibit after an event */

    edr_dv_boundary_pt_t boundary[EDR_BOUNDARY_MAX_PTS];
    uint8_t   boundary_pts;

    /* ---------------- misuse / immunity discrimination -------------- */
    uint8_t   osc_max_count;      /* [ENG] zero-crossings => rough road  */
    uint16_t  osc_win_ms;
    int32_t   jerk_limit_mm_s3;   /* [ENG] pothole/hammer signature      */
    int32_t   jerk_dv_floor_mm_s; /* dV below this + high jerk => inhibit*/

    /* ---------------- LOOP B : sustained deceleration --------------- */
    int32_t   decel_thresh_mm_s2; /* [AIS-220] 3250 mm/s^2 = 3.25 m/s^2  */
    int32_t   decel_rearm_mm_s2;  /* [ENG] hysteresis, e.g. 3000         */
    uint16_t  decel_dwell_ms;     /* [AIS-220] 700 ms sustained          */
    uint16_t  decel_dropout_ms;   /* [ENG] tolerated dip below threshold */
    uint16_t  decel_regress_ms;   /* [ENG] LS-slope window over speed    */
    int32_t   decel_min_speed_mm_s;/*[ENG] ignore below this road speed  */
    bool      allow_accel_fallback;/*[ENG] use accel if VSS invalid      */

    /* ---------------- event management ------------------------------ */
    uint16_t  multi_event_win_ms; /* [AIS-220] group events, 5000 ms     */
    bool      lock_on_srs_only;   /* [AIS-220] 5.3.2 = true (conformant) */
    bool      lock_on_delta_v;    /* [PRODUCT] true = no-SRS lock path   */

    /* ---------------- recording (Annexure 4) ------------------------ */
    uint16_t  record_hz;          /* [AIS-220] 4 Hz                      */
    int32_t   record_fs_mm_s2;    /* [AIS-220] +/-1.5 g = 14710 mm/s^2   */
    int32_t   record_lsb_mm_s2;   /* [AIS-220] 0.1 g = 981 mm/s^2        */
    uint16_t  precrash_ms;        /* [AIS-220] 5000 ms                   */
    uint16_t  postcrash_ms;       /* [AIS-220] 250 ms                    */
} edr_config_t;

/* Populate cfg with the validated default calibration. */
void edr_config_defaults(edr_config_t *cfg);

/* Range/consistency check. Returns 0 on success, negative error code
 * identifying the first offending field. Call before edr_init(). */
int  edr_config_validate(const edr_config_t *cfg);

#endif /* EDR_CONFIG_H */
