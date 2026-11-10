#include "laser_prediction.h"
#include <math.h>
#include <string.h>

LaserPredictionOutput laser_prediction_step(LaserPrediction* s, const LaserSnapshot* l,
    uint32_t now, float vx, float vy, float yaw, bool enabled) {
    LaserPredictionOutput out = {0};
    memcpy(out.mm, l->accepted_mm, sizeof(out.mm));
    const uint32_t dt_ms = now - s->last_ms;
    if(!enabled || !isfinite(vx) || !isfinite(vy) || !isfinite(yaw) ||
       hypotf(vx, vy) > LASER_PREDICT_MAX_SPEED_MPS || (s->initialized && dt_ms > 50u)) {
        memset(s, 0, sizeof(*s));
        return out;
    }
    if(s->initialized) {
        const float dt = (float)dt_ms * 0.001f;
        s->x += vx * dt;
        s->path += hypotf(vx, vy) * dt;
        s->turn += fabsf(remainderf(yaw - s->yaw, 6.283185307f));
    }
    s->last_ms = now; s->yaw = yaw; s->initialized = true;
    for(unsigned k = 0; k < 2u; ++k) {
        const unsigned ch = k + 2u; /* rear, front */
        const uint8_t bit = (uint8_t)(1u << ch);
        const uint32_t age = l->sample_age_ms[ch];
        /* updated_mask refers to the latest board frame. Its receipt stamp is
         * authoritative; caller time minus snapshot age can race a SysTick. */
        const uint32_t stamp = l->last_rx_ms;
        const bool measured = l->frame_fresh && (l->valid_mask & bit) &&
            (l->updated_mask & bit) && age <= 200u;
        if(measured && (!(s->anchors & (1u << k)) || stamp != s->stamp[k])) {
            s->anchors |= (uint8_t)(1u << k);
            s->stamp[k] = stamp;
            s->anchor_mm[k] = l->accepted_mm[ch];
            /* Distance may already be age ms old: account conservatively for
             * travel toward the obstacle before this adapter observed it. */
            const float closing = k ? fmaxf(0.0f, vx) : fmaxf(0.0f, -vx);
            s->anchor_x[k] = s->x + (k ? -1.0f : 1.0f) * closing * (float)age * 0.001f;
            s->anchor_path[k] = s->path; s->anchor_turn[k] = s->turn;
        }
        if(measured || !(s->anchors & (1u << k))) continue;
        const float path = s->path - s->anchor_path[k];
        const float turn = s->turn - s->anchor_turn[k];
        const uint32_t real_age = now - s->stamp[k];
        /* Lost CAN transport is still a stop condition; only individual bad
         * returns on a live board stream can be bridged. Near real data wins. */
        if(!l->frame_fresh || real_age > LASER_PREDICT_MAX_MS || path > LASER_PREDICT_MAX_PATH_M ||
           turn > 0.034906585f || s->anchor_mm[k] <= LASER_FILTER_NEAR_MM ||
           ((l->valid_mask & bit) && l->accepted_mm[ch] <= LASER_FILTER_NEAR_MM)) continue;
        const float delta = (s->x - s->anchor_x[k]) * 1000.0f * (k ? -1.0f : 1.0f);
        /* Never enlarge clearance on odometry alone. Inflate uncertainty for
         * path and yaw sweep; this is not a claim to see a new obstacle. */
        const float margin = 20.0f + 200.0f * path + (float)s->anchor_mm[k] * turn;
        const float estimate = (float)s->anchor_mm[k] + fminf(0.0f, delta) - margin;
        out.mm[ch] = (uint16_t)fmaxf(0.0f, floorf(estimate));
        out.mask |= bit;
    }
    return out;
}
