#include "manual_ramp.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void step(ManualRamp* r, DriveVelocity target, bool stopped, float dt) {
    const DriveVelocity before = r->output;
    assert(manual_ramp_step(r, target, stopped, true, dt));
    assert(hypotf(r->output.vx - before.vx, r->output.vy - before.vy) <= MANUAL_RAMP_LINEAR_DECEL * dt + 0.000001f);
    assert(fabsf(r->output.wz - before.wz) <= MANUAL_RAMP_ANGULAR_DECEL * dt + 0.000001f);
}
static void zero(ManualRamp r) {
    assert(r.output.vx == 0 && r.output.vy == 0 && r.output.wz == 0);
    assert(r.last_motion.vx == 0 && r.last_motion.vy == 0 && r.last_motion.wz == 0 && !r.reversing);
}
int main(void) {
    ManualRamp r = {0};
    const DriveVelocity forward = {1.5f, 0, 0}, reverse = {-1.5f, 0, 0}, stop = {0};
    /* Real irregular dt; a full-throttle jump reaches 0.2 m/s after one second. */
    for(unsigned i = 0; i < 50; ++i) {
        step(&r, forward, false, 0.007f);
        step(&r, forward, false, 0.013f);
    }
    assert(fabsf(r.output.vx - 0.2f) < 0.000001f);
    for(unsigned i = 0; i < 140; ++i) step(&r, forward, false, 0.05f);
    assert(r.output.vx == 1.5f);
    /* Selecting a lower gear decelerates continuously, rather than clamping. */
    step(&r, (DriveVelocity){0.4f, 0, 0}, false, 0.01f);
    assert(fabsf(r.output.vx - 1.497f) < 0.000001f);
    for(unsigned i = 0; i < 400; ++i) step(&r, (DriveVelocity){0.4f, 0, 0}, false, 0.01f);
    assert(fabsf(r.output.vx - 0.4f) < 0.000001f);
    step(&r, stop, false, 0.01f);
    assert(fabsf(r.output.vx - 0.397f) < 0.000001f);
    /* Reverse ramps to zero and cannot resume on opposite input until feedback stops. */
    for(unsigned i = 0; i < 200; ++i) { step(&r, reverse, false, 0.01f); assert(r.output.vx >= 0); }
    assert(r.reversing && r.output.vx == 0);
    step(&r, reverse, true, 0.01f); zero(r);
    step(&r, reverse, true, 0.01f); assert(fabsf(r.output.vx + 0.002f) < 0.000001f);
    /* A release-then-reverse is gated too, even though the command reached zero. */
    for(unsigned i = 0; i < 100; ++i) step(&r, reverse, false, 0.01f);
    for(unsigned i = 0; i < 100; ++i) step(&r, stop, false, 0.01f);
    assert(r.output.vx == 0);
    step(&r, forward, false, 0.01f); assert(r.reversing && r.output.vx == 0);
    manual_ramp_reset(&r); zero(r);
    const DriveVelocity diagonal = {1.5f / 1.414213562f, 1.5f / 1.414213562f, 0};
    for(unsigned i = 0; i < 100; ++i) {
        const DriveVelocity before = r.output;
        step(&r, diagonal, false, 0.01f);
        assert(hypotf(r.output.vx - before.vx, r.output.vy - before.vy) <= 0.002001f);
    }
    assert(fabsf(hypotf(r.output.vx, r.output.vy) - 0.2f) < 0.000001f);
    for(unsigned i = 0; i < 100; ++i) step(&r, (DriveVelocity){0, 1.5f, 0}, false, 0.01f);
    assert(r.output.vx >= 0 && r.output.vy > 0);
    manual_ramp_reset(&r);
    for(unsigned i = 0; i < 100; ++i) step(&r, (DriveVelocity){0, 0, 6}, false, 0.01f);
    assert(fabsf(r.output.wz - 0.4f) < 0.000001f);
    step(&r, stop, false, 0.01f); assert(fabsf(r.output.wz - 0.394f) < 0.000001f);
    for(unsigned i = 0; i < 100; ++i) { step(&r, (DriveVelocity){0, 0, -6}, false, 0.01f); assert(r.output.wz >= 0); }
    assert(r.reversing && r.output.wz == 0);
    step(&r, (DriveVelocity){0, 0, -6}, true, 0.01f); zero(r);
    step(&r, (DriveVelocity){0, 0, -6}, true, 0.01f); assert(fabsf(r.output.wz + 0.004f) < 0.000001f);
    const float invalid_dt[] = {-0.01f, 0.051f, NAN, INFINITY};
    for(unsigned i = 0; i < sizeof(invalid_dt) / sizeof(invalid_dt[0]); ++i) {
        assert(!manual_ramp_step(&r, forward, false, true, invalid_dt[i])); zero(r);
    }
    assert(!manual_ramp_step(&r, (DriveVelocity){NAN, 0, 0}, false, true, 0.01f)); zero(r);
    r.output.vy = INFINITY;
    assert(!manual_ramp_step(&r, forward, false, true, 0.01f)); zero(r);
    step(&r, forward, true, 0); zero(r);
    step(&r, forward, true, 0.01f); assert(fabsf(r.output.vx - 0.002f) < 0.000001f);
    /* Delayed wheel alignment must not accumulate a fast command for release. */
    for(unsigned i = 0; i < 500; ++i) {
        assert(manual_ramp_step(&r, forward, true, false, 0.05f));
        assert(r.output.vx <= 0.002001f);
    }
    step(&r, forward, true, 0.01f); assert(r.output.vx <= 0.004001f);
    assert(manual_ramp_step(&r, reverse, false, false, 0.05f));
    assert(r.reversing && r.output.vx == 0);
    for(unsigned i = 0; i < 20; ++i) {
        assert(manual_ramp_step(&r, reverse, false, false, 0.05f));
        assert(r.reversing && r.output.vx == 0);
    }
    assert(!manual_ramp_step(&r, forward, true, false, 0.051f)); zero(r);
    puts("MANUAL_RAMP_TEST_OK (vector/diagonal slew, real dt, gears, release, reversal feedback gate, rotation, reset and invalid inputs)");
    return 0;
}
