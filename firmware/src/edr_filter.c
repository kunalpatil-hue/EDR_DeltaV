#include "edr_filter.h"
#include <math.h>

/* M_PI is POSIX, not ISO C99 - define locally so this file builds with
 * -std=c99 on both GCC/ARM and MSVC without feature-test macros. */
#define EDR_PI  3.14159265358979323846

/* ===================== biquad ======================================= */

void edr_biquad_design_lp(edr_biquad_t *bq, uint32_t fc_hz, uint32_t fs_hz)
{
    /* 2-pole Butterworth via bilinear transform with frequency pre-warp.
     * Executed once at init only. */
    const double fs = (double)fs_hz;
    double fc = (double)fc_hz;
    const double nyq = fs * 0.5;

    if (fc > nyq * 0.45) { fc = nyq * 0.45; }   /* keep it well-behaved */

    const double k  = tan(EDR_PI * fc / fs);
    const double k2 = k * k;
    const double q  = 0.70710678118654752;      /* Butterworth Q        */
    const double n  = 1.0 / (1.0 + k / q + k2);

    const double b0 = k2 * n;
    const double b1 = 2.0 * b0;
    const double b2 = b0;
    const double a1 = 2.0 * (k2 - 1.0) * n;
    const double a2 = (1.0 - k / q + k2) * n;

    const double s = (double)(1L << EDR_BQ_SHIFT);
    bq->b0 = (int32_t)(b0 * s + (b0 >= 0 ? 0.5 : -0.5));
    bq->b1 = (int32_t)(b1 * s + (b1 >= 0 ? 0.5 : -0.5));
    bq->b2 = (int32_t)(b2 * s + (b2 >= 0 ? 0.5 : -0.5));
    bq->a1 = (int32_t)(a1 * s + (a1 >= 0 ? 0.5 : -0.5));
    bq->a2 = (int32_t)(a2 * s + (a2 >= 0 ? 0.5 : -0.5));

    edr_biquad_reset(bq, 0);
}

void edr_biquad_reset(edr_biquad_t *bq, int32_t x0)
{
    bq->x1 = bq->x2 = (int64_t)x0;
    bq->y1 = bq->y2 = (int64_t)x0;
}

int32_t edr_biquad_step(edr_biquad_t *bq, int32_t x)
{
    /* Direct Form I. Inputs are bounded by IMU full scale (~1.6e5), the
     * Q28 coefficients by 2^28, so the accumulator peaks near 2^46 -
     * comfortably inside int64 with no saturation logic required. */
    int64_t acc = (int64_t)bq->b0 * (int64_t)x
                + (int64_t)bq->b1 * bq->x1
                + (int64_t)bq->b2 * bq->x2
                - (int64_t)bq->a1 * bq->y1
                - (int64_t)bq->a2 * bq->y2;

    int64_t y = acc >> EDR_BQ_SHIFT;

    bq->x2 = bq->x1;
    bq->x1 = (int64_t)x;
    bq->y2 = bq->y1;
    bq->y1 = y;

    if (y >  2147483647LL) { y =  2147483647LL; }
    if (y < -2147483648LL) { y = -2147483648LL; }
    return (int32_t)y;
}

/* ===================== bias tracker ================================= */

void edr_bias_init(edr_bias_t *b, uint8_t shift, int32_t gate, uint32_t settle_samples)
{
    b->bias_q8 = 0;
    b->shift = shift;
    b->gate = gate;
    b->settle_samples = settle_samples;
    b->n = 0;
    b->frozen = false;
    b->settled = false;
}

int32_t edr_bias_step(edr_bias_t *b, int32_t x, bool freeze)
{
    const int32_t bias = b->bias_q8 >> 8;
    const int32_t err  = x - bias;
    int32_t aerr = (err < 0) ? -err : err;

    b->frozen = freeze;

    if (b->n < b->settle_samples) {
        /* Fast acquisition at power-up: converge in ~bias_settle_ms so a
         * cold-start on a slope does not need seconds to stabilise. */
        b->bias_q8 += (err << 8) >> 5;
        b->n++;
        if (b->n >= b->settle_samples) { b->settled = true; }
    } else if (!freeze && aerr < b->gate) {
        /* Slow leaky tracking, gated so a real event never bends the
         * estimate toward the crash pulse. */
        b->bias_q8 += (err << 8) >> b->shift;
    }
    /* else: frozen (armed) or gated (transient) -> hold last estimate. */

    return x - (b->bias_q8 >> 8);
}

int32_t edr_bias_get(const edr_bias_t *b) { return b->bias_q8 >> 8; }

/* ===================== sliding dV ring ============================== */

void edr_dvring_init(edr_dvring_t *r, uint32_t dt_us, uint16_t w_short,
                     uint16_t w_main, uint16_t w_safing, uint16_t w_seed)
{
    uint16_t i;
    for (i = 0; i < EDR_DV_RING_LEN; i++) { r->inc[i] = 0; }
    r->head = 0;
    r->n = 0;
    r->dt_us = dt_us;
    r->w_short  = (w_short  > EDR_DV_WINDOW_MAX) ? EDR_DV_WINDOW_MAX : w_short;
    r->w_main   = (w_main   > EDR_DV_WINDOW_MAX) ? EDR_DV_WINDOW_MAX : w_main;
    r->w_safing = (w_safing > EDR_DV_WINDOW_MAX) ? EDR_DV_WINDOW_MAX : w_safing;
    r->w_seed   = (w_seed   > EDR_DV_WINDOW_MAX) ? EDR_DV_WINDOW_MAX : w_seed;
    r->s_short = r->s_main = r->s_safing = r->s_seed = 0;
    r->cumulative = 0;
}

void edr_dvring_reset_cumulative(edr_dvring_t *r) { r->cumulative = 0; }

void edr_dvring_seed_cumulative(edr_dvring_t *r)
{
    /* An integrator that starts at zero the instant the arming criteria
     * are met has already missed the onset of the pulse: arming needs a
     * finite level AND a finite short-window dV, which on a 150 ms truck
     * pulse costs 20-40 ms of ramp. Measured on the marginal 25 km/h
     * case that is a 6% under-report of dV - straight onto the evidential
     * record. Seeding from the pre-arm window recovers it.
     *
     * The seed window is deliberately short (60 ms default). Longer, and
     * a collision that follows hard braking folds the braking dV into the
     * collision dV. */
    r->cumulative = r->s_seed;
}

void edr_dvring_push(edr_dvring_t *r, int32_t accel_mm_s2)
{
    /* dv[um/s] = a[mm/s^2] * dt[us] / 1000.
     * Worst case: 16 g -> 156912 mm/s^2 * 1200 us / 1000 = 188 294 um/s.
     * Well inside int32; the running sums are int64. */
    const int32_t inc = (int32_t)(((int64_t)accel_mm_s2 * (int64_t)r->dt_us) / 1000);

    r->head = (uint16_t)((r->head + 1u) & EDR_DV_RING_MASK);
    r->inc[r->head] = inc;
    r->n++;

    r->s_short  += inc;
    r->s_main   += inc;
    r->s_safing += inc;
    r->s_seed   += inc;
    r->cumulative += inc;

    /* Drop the sample that just left each window. The ring is longer than
     * EDR_DV_WINDOW_MAX, so the evicted slot is always still valid. */
    if (r->n > r->w_short)  { r->s_short  -= r->inc[(r->head - r->w_short)  & EDR_DV_RING_MASK]; }
    if (r->n > r->w_main)   { r->s_main   -= r->inc[(r->head - r->w_main)   & EDR_DV_RING_MASK]; }
    if (r->n > r->w_safing) { r->s_safing -= r->inc[(r->head - r->w_safing) & EDR_DV_RING_MASK]; }
    if (r->n > r->w_seed)   { r->s_seed   -= r->inc[(r->head - r->w_seed)   & EDR_DV_RING_MASK]; }
}

static int32_t um_to_mm(int64_t um)
{
    const int64_t mm = (um >= 0) ? (um + 500) / 1000 : (um - 500) / 1000;
    if (mm >  2147483647LL) { return  2147483647; }
    if (mm < -2147483648LL) { return -2147483648; }
    return (int32_t)mm;
}

int32_t edr_dvring_short(const edr_dvring_t *r)      { return um_to_mm(r->s_short);  }
int32_t edr_dvring_main(const edr_dvring_t *r)       { return um_to_mm(r->s_main);   }
int32_t edr_dvring_safing(const edr_dvring_t *r)     { return um_to_mm(r->s_safing); }
int32_t edr_dvring_seed(const edr_dvring_t *r)       { return um_to_mm(r->s_seed);   }
int32_t edr_dvring_cumulative(const edr_dvring_t *r) { return um_to_mm(r->cumulative); }

/* ===================== integer helpers ============================== */

int32_t edr_isqrt(int64_t v)
{
    if (v <= 0) { return 0; }
    /* Newton iteration seeded by bit-length halving. Converges in <= 6
     * iterations for the ranges we use; fully deterministic. */
    int64_t x = v;
    int64_t y = (x + 1) >> 1;
    int i;
    for (i = 0; i < 32 && y < x; i++) {
        x = y;
        y = (x + v / x) >> 1;
    }
    return (int32_t)x;
}

int32_t edr_hypot(int32_t x, int32_t y)
{
    return edr_isqrt((int64_t)x * (int64_t)x + (int64_t)y * (int64_t)y);
}

int16_t edr_atan2_ddeg(int32_t y, int32_t x)
{
    /* Rational approximation of atan on [0,1] (max error ~0.07 deg),
     * then octant folding. No libm, no FPU. Result in 0.1 degree. */
    if (x == 0 && y == 0) { return 0; }

    const int32_t ax = (x < 0) ? -x : x;
    const int32_t ay = (y < 0) ? -y : y;
    const int32_t mn = (ax < ay) ? ax : ay;
    const int32_t mx = (ax < ay) ? ay : ax;

    /* r = mn/mx in Q15 */
    const int32_t r = (int32_t)(((int64_t)mn << 15) / (mx ? mx : 1));
    const int64_t r2 = ((int64_t)r * r) >> 15;

    /* atan(r) on 0 <= r <= 1, Rajan's form:
     *
     *   atan(r) ~= (pi/4)r - r(r - 1)(0.2447 + 0.0663 r)
     *
     * Max error 0.0015 rad (0.085 deg) across the whole interval, and
     * critically it is EXACT at r = 1. The truncated odd-power series
     * that this replaces was 6.6 degrees out at r = 1 - i.e. it reported
     * a clean 45 degree oblique impact as 38 degrees, which would put a
     * visibly wrong principal direction of force on the record.
     *
     * Q15 constants: pi/4 = 25736, 0.2447 = 8018, 0.0663 = 2172. */
    const int64_t t1 = (25736LL * r) >> 15;
    const int64_t t2 = ((int64_t)r * (int64_t)(r - 32768)) >> 15;   /* <= 0 */
    const int64_t t3 = 8018LL + ((2172LL * r) >> 15);
    int64_t a = t1 - ((t2 * t3) >> 15);                              /* Q15 rad */
    (void)r2;

    /* rad -> degrees is *57.29578, so *5730 lands in CENTI-degrees.
     * Round down to deci-degrees rather than rescaling the constant,
     * which would throw away three bits of the Q15 mantissa. */
    int32_t cdeg = (int32_t)((a * 5730) >> 15);
    int32_t ddeg = (cdeg >= 0) ? (cdeg + 5) / 10 : (cdeg - 5) / 10;

    if (ay > ax) { ddeg = 900 - ddeg; }
    if (x < 0)   { ddeg = 1800 - ddeg; }
    if (y < 0)   { ddeg = -ddeg; }
    return (int16_t)ddeg;
}
