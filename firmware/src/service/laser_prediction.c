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

static float ray_yaw(unsigned ch) {
    const float offset[4]={1.570796327f,-1.570796327f,3.141592654f,0};
    return offset[ch];
}
static LaserMotionAnchor motion_anchor(const LaserMotionFilter* s,unsigned ch,
    float mm,uint32_t stamp,float wx,float wy,float age_s) {
    (void)ch;
    return (LaserMotionAnchor){.x=s->x-wx*age_s,.y=s->y-wy*age_s,
        .path=s->path-hypotf(wx,wy)*age_s,.turn=s->turn,.yaw=s->yaw,.mm=mm,.stamp=stamp};
}
static float motion_expected(const LaserMotionAnchor* a,float x,float y,unsigned ch) {
    const float angle=a->yaw+ray_yaw(ch);
    return a->mm-1000*((x-a->x)*cosf(angle)+(y-a->y)*sinf(angle));
}
static bool motion_matches(const LaserMotionAnchor* a,const LaserMotionAnchor* sample,unsigned ch) {
    const float path=hypotf(sample->x-a->x,sample->y-a->y);
    const float turn=fabsf(remainderf(sample->yaw-a->yaw,6.283185307f));
    /* Translation is compensated, while wheel scale and changing ray angle
     * get an explicit innovation allowance. Large scene changes need a new
     * independently confirmed track instead of overwriting the old anchor. */
    const float tolerance=60+150*path+.5f*(a->mm+sample->mm)*turn;
    return turn<=.052359878f &&
        fabsf(sample->mm-motion_expected(a,sample->x,sample->y,ch))<=tolerance;
}
LaserMotionOutput laser_motion_filter_step(LaserMotionFilter* s,const LaserSnapshot* l,
    uint32_t now,float vx,float vy,float yaw,float emergency_decel,bool enabled) {
    LaserMotionOutput out={.checked=*l};
    const uint32_t dt_ms=now-s->last_ms;
    if(!enabled || !isfinite(vx)||!isfinite(vy)||!isfinite(yaw)||
       !isfinite(emergency_decel)||emergency_decel<=0 || hypotf(vx,vy)>2.0f ||
       (s->initialized && dt_ms>50u)) {
        memset(s,0,sizeof(*s));return out;
    }
    const float wx=vx*cosf(yaw)-vy*sinf(yaw),wy=vx*sinf(yaw)+vy*cosf(yaw);
    if(s->initialized) {
        const float dt=dt_ms*.001f;
        s->x+=wx*dt;s->y+=wy*dt;s->path+=hypotf(vx,vy)*dt;
        s->turn+=fabsf(remainderf(yaw-s->yaw,6.283185307f));
    }
    const bool new_frame=!s->initialized || l->frame_count!=s->frame;
    s->last_ms=now;s->yaw=yaw;s->frame=l->frame_count;s->initialized=true;
    for(unsigned ch=0;ch<4u;++ch) {
        LaserMotionTrack* t=&s->channel[ch];const uint8_t bit=(uint8_t)(1u<<ch);
        const uint32_t age=l->sample_age_ms[ch],stamp=now-age;
        const uint16_t mm=l->accepted_mm[ch];
        const bool real=l->frame_fresh && (l->valid_mask&bit) && age<=LASER_FRAME_TIMEOUT_MS && mm<=LASER_ACCEPT_MAX_MM;
        const bool advanced=!t->seen || (stamp!=t->seen_stamp && (uint32_t)(stamp-t->seen_stamp)<0x80000000u);
        const bool fresh=real && ((!t->seen && !t->trusted) ||
            (new_frame && (l->updated_mask&bit) && advanced));
        const float angle=ray_yaw(ch),closing=fmaxf(0,vx*cosf(angle)+vy*sinf(angle));
        const float urgent_mm=fmaxf(200,100+500*closing*closing/emergency_decel+100*closing);
        const bool urgent=real && mm<=urgent_mm;
        if(fresh) {
            const LaserMotionAnchor sample=motion_anchor(s,ch,mm,stamp,wx,wy,age*.001f);
            t->seen=true;t->seen_stamp=stamp;t->sample_mm=mm;
            if(!t->checkpoint_valid || fabsf(sample.mm-t->checkpoint.mm)>5) {
                t->checkpoint=sample;t->checkpoint_valid=true;t->frozen=false;
            } else if(closing>.10f &&
                fabsf(motion_expected(&t->checkpoint,sample.x,sample.y,ch)-t->checkpoint.mm)>30) {
                /* A rolling frame counter is not evidence that a physically
                 * frozen range has updated. Do not ratchet the anchor every
                 * few millimetres until a metre of disagreement disappears. */
                t->frozen=true;
            }
            if(t->frozen) {
                t->accepted=false;t->confirmations=0;
            } else if(!t->trusted || motion_matches(&t->anchor,&sample,ch)) {
                t->anchor=sample;t->trusted=t->accepted=true;t->confirmations=0;
            } else {
                if(!t->confirmations || !motion_matches(&t->candidate,&sample,ch) ||
                   (uint32_t)(stamp-t->candidate.stamp)>300u) {
                    t->confirmations=1;t->candidate_first_ms=stamp;
                } else if(t->confirmations<3u) ++t->confirmations;
                t->candidate=sample;t->accepted=false;
                if(t->confirmations>=3u && (uint32_t)(stamp-t->candidate_first_ms)>=180u) {
                    t->anchor=sample;t->trusted=t->accepted=true;t->confirmations=0;
                    out.confirmed_mask|=bit;
                }
            }
        }
        const bool accepted=real && t->accepted && t->sample_mm==mm;
        if(urgent) out.urgent_mask|=bit;
        if(!accepted && real) out.rejected_mask|=bit;
        if(!accepted && !urgent) {
            out.checked.valid_mask&=(uint8_t)~bit;
            out.checked.updated_mask&=(uint8_t)~bit;
            out.checked.median_valid_mask&=(uint8_t)~bit;
            out.checked.median_updated_mask&=(uint8_t)~bit;
            const LaserMotionAnchor* a=&t->anchor;
            const float path=s->path-a->path,turn=s->turn-a->turn;
            if(l->frame_fresh && t->trusted && (uint32_t)(now-a->stamp)<=LASER_MOTION_MAX_MS &&
               path<=LASER_MOTION_MAX_PATH_M && turn<=.052359878f && a->mm>100) {
                const float estimate=fminf(a->mm,motion_expected(a,s->x,s->y,ch));
                /* Forward travel along a side wall is not closing travel on
                 * that ray. Penalizing the whole path made a 160mm lateral
                 * observation shrink into a false stop while driving straight. */
                const float normal_travel_mm=fabsf(motion_expected(a,s->x,s->y,ch)-a->mm);
                const float margin=20+.15f*normal_travel_mm+a->mm*turn;
                out.estimated_mm[ch]=(uint16_t)fmaxf(0,floorf(estimate-margin));
                out.predicted_mask|=bit;
            }
        } else if(!accepted) {
            /* A fresh close return inside the actual stopping envelope remains
             * independently protective, even when its innovation is implausible.
             * It does not renew a disputed far anchor. */
            out.checked.valid_mask|=bit;
        }
    }
    /* Preserve the independent median source ages/update flags. Rejection
     * above removes the ray from both consumers without inventing pair evidence. */
    return out;
}
