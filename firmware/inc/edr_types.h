/* =====================================================================
 *  edr_types.h  -  Fixed-point types, units and event definitions
 *  EDR crash-detection core   |  MileEdge Labs  |  AIS-220 / UN R169
 * ---------------------------------------------------------------------
 *  UNIT CONVENTION  (integer-only; no floating point anywhere in core)
 *    acceleration ... int32  mm/s^2      (1 g = 9807 mm/s^2)
 *    velocity     ... int32  mm/s        (25 km/h = 6944 mm/s)
 *    dv accum     ... int64  um/s        (integration is done in um/s
 *                                         so per-sample truncation is
 *                                         < 1 um/s instead of ~0.5 mm/s)
 *    time         ... uint32 microseconds / milliseconds as named
 *    angle        ... int16  deci-degrees (PDOF)
 *
 *  RATIONALE: MISRA-C friendly, deterministic, no FPU dependency, and
 *  bit-identical results between the PC simulator and the S32K312.
 * ===================================================================== */
#ifndef EDR_TYPES_H
#define EDR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ---- physical constants ------------------------------------------- */
#define EDR_G_MM_S2                 (9807)          /* 9.80665 m/s^2    */
#define EDR_KPH_TO_MM_S(kph)        (((kph) * 2778) / 10)   /* 1 km/h = 277.78 mm/s */

/* ---- ring geometry (compile-time, static allocation only) ---------- */
#define EDR_DV_RING_LEN             (1024u)         /* power of two     */
#define EDR_DV_RING_MASK            (EDR_DV_RING_LEN - 1u)
#define EDR_DV_WINDOW_MAX           (1000u)         /* must be < LEN    */

#define EDR_SPEED_RING_LEN          (64u)           /* Loop B regression*/
#define EDR_BOUNDARY_MAX_PTS        (12u)
#define EDR_MAX_EVENTS_PER_SEQ      (4u)
#define EDR_PRECRASH_SAMPLES        (24u)           /* 5 s @ 4 Hz + slack*/

/* ---- axes ---------------------------------------------------------- */
typedef enum {
    EDR_AXIS_LONG = 0,      /* +X forward  : braking/frontal = negative */
    EDR_AXIS_LAT  = 1,      /* +Y left                                   */
    EDR_AXIS_COUNT
} edr_axis_t;

/* ---- trigger / event classification -------------------------------- */
typedef enum {
    EDR_TRG_NONE            = 0u,
    EDR_TRG_SUDDEN_DECEL    = (1u << 0), /* Loop B : AIS-220 mandatory   */
    EDR_TRG_DELTA_V_WINDOW  = (1u << 1), /* Loop A : sliding 150 ms dV   */
    EDR_TRG_DELTA_V_CURVE   = (1u << 2), /* Loop A : dV-vs-t boundary    */
    EDR_TRG_SRS_DEPLOY      = (1u << 3), /* external, via CAN            */
    EDR_TRG_ROLLOVER        = (1u << 4)  /* reserved (Rev B)             */
} edr_trigger_t;

typedef enum {
    EDR_INHIBIT_NONE        = 0u,
    EDR_INHIBIT_ROUGH_ROAD  = (1u << 0),
    EDR_INHIBIT_JERK_SPIKE  = (1u << 1), /* pothole / kerb / hammer blow */
    EDR_INHIBIT_NO_SAFING   = (1u << 2), /* safing channel did not concur*/
    EDR_INHIBIT_LOCKOUT     = (1u << 3),
    EDR_INHIBIT_NOT_VALID   = (1u << 4)  /* sensor fault / bias unsettled*/
} edr_inhibit_t;

/* ---- one recorded event -------------------------------------------- */
typedef struct {
    uint32_t  t0_ms;              /* time of trigger (ECU uptime)        */
    uint16_t  seq_index;          /* 0 = first event in a multi-event seq*/
    uint16_t  trigger_mask;       /* edr_trigger_t bitfield              */
    uint16_t  inhibit_mask;       /* edr_inhibit_t bitfield (diagnostic) */

    int32_t   dv_long_mm_s;       /* dV long. over the 150 ms window     */
    int32_t   dv_lat_mm_s;
    int32_t   dv_res_mm_s;        /* resultant magnitude                 */
    int32_t   dv_max_mm_s;        /* peak resultant seen in the event    */
    uint16_t  t_to_dv_max_ms;

    int32_t   peak_a_long_mm_s2;  /* signed peak of filtered long. accel */
    int32_t   peak_a_lat_mm_s2;
    int16_t   pdof_ddeg;          /* principal direction of force, 0.1 deg*/

    int32_t   decel_rate_mm_s2;   /* Loop B: VSS-derived decel at trigger */
    uint16_t  decel_dwell_ms;     /* how long it was sustained            */
    int32_t   speed_at_t0_mm_s;

    bool      locked;             /* write-protected record               */
    bool      dv_is_lower_bound;  /* accel clipped => dV underestimated   */
    bool      decel_src_is_vss;   /* false = derived from accel (degraded)*/
} edr_event_t;

/* ---- AIS-220 Annexure 4 recorded sample (4 Hz, +/-1.5 g, 0.1 g) ----- */
typedef struct {
    int8_t    a_long_dg;          /* deci-g, saturating at +/-15 (=1.5 g)*/
    int8_t    a_lat_dg;
    uint8_t   flags;              /* bit0 = saturated                    */
    uint16_t  speed_kph_x10;
} edr_record_sample_t;

/* ---- raw inputs ----------------------------------------------------- */
typedef struct {                  /* fast loop, IMU @ EDR_FAST_HZ        */
    int32_t   a_long_mm_s2;       /* already scaled from LSB             */
    int32_t   a_lat_mm_s2;
    int32_t   a_vert_mm_s2;
    bool      clipped;            /* |raw| at or beyond full scale       */
    bool      valid;              /* sensor self-test / data-ready ok    */
} edr_imu_sample_t;

typedef struct {                  /* slow loop, CAN @ EDR_SLOW_HZ        */
    int32_t   speed_mm_s;         /* from VSS / ABS wheel speed          */
    bool      speed_valid;
    bool      srs_deployed;       /* SRS ECU deployment flag             */
    bool      brake_active;
} edr_vehicle_sample_t;

#endif /* EDR_TYPES_H */
