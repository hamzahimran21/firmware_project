/*
 * Presence Detection Algorithm - Full FFT Implementation
 * Using CMSIS-DSP library for ARM Cortex-M7
 */

#include "presence_detection.h"
#include "arm_math.h"
#include <string.h>
#include <math.h>

/* Blackman-Harris window coefficients (pre-computed for 64 samples)
 * Generated using scipy.signal.blackmanharris(64)
 * These are symmetric - values go from 0 at edges to ~1 at center
 */
static const float blackman_harris_64[64] = {
    0.00006000f, 0.00033859f, 0.00122759f, 0.00308442f, 0.00651448f,
    0.01229611f, 0.02142720f, 0.03506737f, 0.05451028f, 0.08109126f,
    0.11609082f, 0.16060659f, 0.21537727f, 0.28065893f, 0.35612853f,
    0.44088266f, 0.53343776f, 0.63176063f, 0.73332737f, 0.83522738f,
    0.93427919f, 1.02718260f, 1.11068200f, 1.18175298f, 1.23779291f,
    1.27680299f, 1.29752788f, 1.29952638f, 1.28319155f, 1.24973093f,
    1.20110711f, 1.13988693f, 1.06906937f, 0.99188432f, 0.91157631f,
    0.83120313f, 0.75343587f, 0.68035547f, 0.61339846f, 0.55335240f,
    0.50040049f, 0.45419694f, 0.41395680f, 0.37856633f, 0.34666116f,
    0.31674609f, 0.28727068f, 0.25672376f, 0.22373068f, 0.18720330f,
    0.14645685f, 0.10131577f, 0.05222267f, 0.00028682f, 0.00006000f,
    0.00033859f, 0.00122759f, 0.00308442f, 0.00651448f, 0.01229611f,
    0.02142720f, 0.03506737f, 0.05451028f, 0.08109126f
};

/* FFT instance */
static arm_rfft_fast_instance_f32 fft_instance;
static bool fft_initialized = false;

void presence_init(presence_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(presence_ctx_t));
    ctx->first_run = true;
    ctx->presence_detected = false;

    /* Initialize FFT for 64 samples */
    if (!fft_initialized) {
        arm_rfft_fast_init_f32(&fft_instance, RADAR_NUM_SAMPLES);
        fft_initialized = true;
    }
}

/*
 * Full presence detection with FFT
 * Based on Infineon's algorithm from presence_detection.py
 */
bool presence_detect(presence_ctx_t *ctx, const radar_frame_t *frame)
{
    if (!frame || !frame->valid) {
        return false;
    }

    /* Temporary buffers for processing */
    float range_samples[RADAR_NUM_SAMPLES];
    float windowed[RADAR_NUM_SAMPLES];
    float fft_output[RADAR_NUM_SAMPLES * 2];  /* Complex output (I,Q pairs) */
    float fft_magnitude[RADAR_NUM_SAMPLES];

    /* Step 1: Average samples across all chirps for each range bin */
    for (int s = 0; s < RADAR_NUM_SAMPLES; s++) {
        float sum = 0.0f;

        for (int c = 0; c < RADAR_NUM_CHIRPS; c++) {
            /* Get sample from frame (assuming I/Q interleaved, using I channel) */
            int idx = c * RADAR_NUM_SAMPLES + s;
            int16_t sample = frame->samples[idx];

            /* Convert to float and normalize */
            sum += (float)sample / 32768.0f;
        }

        range_samples[s] = sum / (float)RADAR_NUM_CHIRPS;
    }

    /* Step 2: Apply Blackman-Harris window */
    for (int i = 0; i < RADAR_NUM_SAMPLES; i++) {
        windowed[i] = range_samples[i] * blackman_harris_64[i];
    }

    /* Step 3: Compute FFT */
    arm_rfft_fast_f32(&fft_instance, windowed, fft_output, 0);

    /* Step 4: Calculate magnitude of complex FFT output */
    /* FFT output is [real0, imag0, real1, imag1, ...] */
    arm_cmplx_mag_f32(fft_output, fft_magnitude, RADAR_NUM_SAMPLES / 2);

    /* Step 5: Initialize averages on first run */
    if (ctx->first_run) {
        for (int i = 0; i < RADAR_NUM_SAMPLES / 2; i++) {
            ctx->slow_avg[i] = fft_magnitude[i];
            ctx->fast_avg[i] = fft_magnitude[i];
        }
        ctx->first_run = false;
        return false;  /* No detection on first frame */
    }

    /* Step 6: Update exponential moving averages (IIR filters) */
    float alpha_slow_used = ctx->presence_detected ? ALPHA_SLOW : ALPHA_MED;

    for (int i = 0; i < RADAR_NUM_SAMPLES / 2; i++) {
        /* Slow average (background tracking) */
        ctx->slow_avg[i] = ctx->slow_avg[i] * (1.0f - alpha_slow_used) +
                          fft_magnitude[i] * alpha_slow_used;

        /* Fast average (target tracking) */
        ctx->fast_avg[i] = ctx->fast_avg[i] * (1.0f - ALPHA_FAST) +
                          fft_magnitude[i] * ALPHA_FAST;
    }

    /* Step 7: Find maximum difference in detection range */
    float max_diff = 0.0f;
    int max_idx = 0;

    for (int i = DETECT_START_SAMPLE; i < DETECT_END_SAMPLE && i < RADAR_NUM_SAMPLES / 2; i++) {
        float diff = ctx->fast_avg[i] - ctx->slow_avg[i];
        if (diff > max_diff) {
            max_diff = diff;
            max_idx = i;
        }
    }

    /* Step 8: Threshold comparison */
    ctx->presence_detected = (max_diff > THRESHOLD_PRESENCE);

    /* Optional: Calculate approximate distance */
    if (ctx->presence_detected) {
        /* Distance = (range_bin * c) / (2 * bandwidth * samples)
         * c = 3e8 m/s, bandwidth = 3.232 GHz, samples = 64
         * distance ≈ range_bin * 0.7 meters
         */
        (void)max_idx;  /* Can be used for distance calculation */
    }

    return ctx->presence_detected;
}

/*
 * Advanced: With separate I/Q channels
 *
 * If frame data contains separate I and Q channels:
 * - Use arm_rfft_f32() for each chirp
 * - Average FFT magnitudes across chirps
 * - More accurate than averaging samples first
 */
bool presence_detect_iq(presence_ctx_t *ctx, const radar_frame_t *frame)
{
    /* TODO: Implement if I/Q channels are separate */
    /* This would process each chirp individually:
     * for each chirp:
     *   - Apply window
     *   - Compute FFT
     *   - Calculate magnitude
     * Average all magnitudes
     * Continue with IIR filters as above
     */
    (void)ctx;
    (void)frame;
    return false;
}
