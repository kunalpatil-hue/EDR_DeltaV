/* =====================================================================
 *  edr_filter.h  -  Integer signal conditioning primitives
 * ---------------------------------------------------------------------
 *  1. edr_biquad_t   2-pole Butterworth low-pass, Direct Form I,
 *                    Q28 coefficients, int64 accumulator.
 *                    Anti-alias + noise rejection ahead of integration.
 *
 *  2. edr_bias_t     Slow leaky-integrator DC/gravity estimator.
 *                    *** FROZEN WHILE ARMED *** - this is the single
 *                    most important detail in the whole front end. A
 *                    static 0.02 g mounting/pitch offset integrates to
 *                    0.03 m/s over 150 ms (harmless) but to 2.0 m/s over
 *                    10 s, which would silently corrupt any long-window
 *                    or free-running integrator.
 *
 *  3. edr_dvring_t   ONE ring of per-sample dV increments (um/s) with
 *                    THREE independent running sums (short / main /
 *                    safing windows). O(1) per sample, no re-summation.
 * ===================================================================== */
#ifndef EDR_FILTER_H
#define EDR_FILTER_H

#include "edr_types.h"

/* ------------------------- biquad ---------------------------------- */
#define EDR_BQ_SHIFT   (28)

typedef struct {
    int32_t b0, b1, b2, a1, a2;   /* Q28, a0 normalised to 1.0          */
    int64_t x1, x2, y1, y2;
} edr_biquad_t;

/* Design a 2-pole Butterworth LPF (bilinear transform, pre-warped).
 * Uses double internally - called ONCE at init, never in the loop. On
 * the target this can be replaced by a const table to keep the runtime
 * strictly integer / FPU-free. */
void    edr_biquad_design_lp(edr_biquad_t *bq, uint32_t fc_hz, uint32_t fs_hz);
void    edr_biquad_reset(edr_biquad_t *bq, int32_t x0);
int32_t edr_biquad_step(edr_biquad_t *bq, int32_t x);

/* ------------------------- bias tracker ----------------------------- */
typedef struct {
    int32_t  bias_q8;             /* estimate in mm/s^2, Q8 sub-LSB     */
    uint8_t  shift;
    int32_t  gate;
    uint32_t settle_samples;
    uint32_t n;
    bool     frozen;
    bool     settled;
} edr_bias_t;

void    edr_bias_init(edr_bias_t *b, uint8_t shift, int32_t gate, uint32_t settle_samples);
int32_t edr_bias_step(edr_bias_t *b, int32_t x, bool freeze);
int32_t edr_bias_get(const edr_bias_t *b);

/* ------------------------- sliding dV ring -------------------------- */
typedef struct {
    int32_t  inc[EDR_DV_RING_LEN]; /* per-sample dV increment, um/s     */
    uint16_t head;
    uint32_t n;                    /* total samples pushed              */

    uint16_t w_short, w_main, w_safing, w_seed; /* lengths in samples   */
    int64_t  s_short, s_main, s_safing, s_seed; /* running sums, um/s   */

    int64_t  cumulative;           /* reset-on-arm accumulator, um/s    */
    uint32_t dt_us;
} edr_dvring_t;

void edr_dvring_init(edr_dvring_t *r, uint32_t dt_us, uint16_t w_short,
                     uint16_t w_main, uint16_t w_safing, uint16_t w_seed);
void edr_dvring_reset_cumulative(edr_dvring_t *r);
/* Restart the cumulative accumulator seeded with the dV already banked
 * in the seed window - see edr_dvring_seed_cumulative() in the .c for
 * why an integrator that starts at zero on arming under-reports. */
void edr_dvring_seed_cumulative(edr_dvring_t *r);
void edr_dvring_push(edr_dvring_t *r, int32_t accel_mm_s2);

/* Accessors return mm/s (rounded from the um/s accumulators). */
int32_t edr_dvring_short(const edr_dvring_t *r);
int32_t edr_dvring_main(const edr_dvring_t *r);
int32_t edr_dvring_safing(const edr_dvring_t *r);
int32_t edr_dvring_seed(const edr_dvring_t *r);
int32_t edr_dvring_cumulative(const edr_dvring_t *r);

/* ------------------------- helpers ---------------------------------- */
int32_t edr_isqrt(int64_t v);                       /* integer sqrt     */
int32_t edr_hypot(int32_t x, int32_t y);            /* resultant magnitude */
int16_t edr_atan2_ddeg(int32_t y, int32_t x);       /* PDOF, 0.1 deg    */

#endif /* EDR_FILTER_H */
