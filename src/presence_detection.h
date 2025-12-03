/*
 * Presence Detection Algorithm
 * Based on Infineon's presence sensing algorithm
 */

#ifndef PRESENCE_DETECTION_H
#define PRESENCE_DETECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "avian_radar.h"

/* Algorithm configuration */
#define DETECT_START_SAMPLE     (RADAR_NUM_SAMPLES / 8)    /* Start at sample 8 */
#define DETECT_END_SAMPLE       (RADAR_NUM_SAMPLES / 2)    /* End at sample 32 */

#define THRESHOLD_PRESENCE      0.0007f
#define ALPHA_SLOW              0.001f
#define ALPHA_MED               0.05f
#define ALPHA_FAST              0.6f

/* Presence detection state */
typedef struct {
    float slow_avg[RADAR_NUM_SAMPLES];
    float fast_avg[RADAR_NUM_SAMPLES];
    bool first_run;
    bool presence_detected;
} presence_ctx_t;

/*
 * Initialize presence detection algorithm
 */
void presence_init(presence_ctx_t *ctx);

/*
 * Run presence detection on radar frame
 * Returns true if presence detected
 */
bool presence_detect(presence_ctx_t *ctx, const radar_frame_t *frame);

#endif /* PRESENCE_DETECTION_H */
