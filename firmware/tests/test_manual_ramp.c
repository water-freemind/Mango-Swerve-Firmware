#include "manual_ramp.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#undef assert
#define assert(c) do { if(!(c)) { fprintf(stderr, "CHECK failed: %s (%s:%d)\n", #c, __FILE__, __LINE__); exit(1); } } while(0)

static void step(ManualRamp* r, DriveVelocity target, bool stopped, float dt) {
    const DriveVelocity before = r->output, a = r->acceleration;
    assert(manual_ramp_step(r, target, stopped, true, dt));
    assert(hypotf(r->output.vx - before.vx, r->output.vy - before.vy) <= MANUAL_RAMP_LINEAR_DECEL * dt + 0.00003f);
    assert(fabsf(r->output.wz - before.wz) <= MANUAL_RAMP_ANGULAR_DECEL * dt + 0.00003f);
    assert(hypotf(r->acceleration.vx - a.vx, r->acceleration.vy - a.vy) <= MANUAL_RAMP_LINEAR_JERK * dt + 0.00001f);
    assert(fabsf(r->acceleration.wz - a.wz) <= MANUAL_RAMP_ANGULAR_JERK * dt + 0.00001f);
    assert(hypotf(r->output.vx, r->output.vy) <= 1.50003f);
    assert(fabsf(r->output.wz) <= 6.00003f);
}
static void zero(ManualRamp r) {
    assert(r.output.vx == 0 && r.output.vy == 0 && r.output.wz == 0);
    assert(r.acceleration.vx == 0 && r.acceleration.vy == 0 && r.acceleration.wz == 0);
    assert(r.last_motion.vx == 0 && r.last_motion.vy == 0 && r.last_motion.wz == 0 && !r.reversing);
}
static void run(ManualRamp* r, DriveVelocity target, int n) {
    for(int i=0;i<n;++i) step(r,target,false,.01f);
}
int main(void) {
    ManualRamp r = {0};
    const DriveVelocity forward = {1.5f,0,0}, reverse={-1.5f,0,0}, stop={0};
    /* Irregular dt preserves wall-clock trajectory, not invocation count. */
    ManualRamp regular = {0};
    for(unsigned i=0;i<400;++i) {
        step(&r,forward,false,.007f); step(&r,forward,false,.013f);
        step(&regular,forward,false,.01f); step(&regular,forward,false,.01f);
    }
    assert(fabsf(r.output.vx-regular.output.vx)<.00003f && r.output.vx==1.5f);
    run(&r,(DriveVelocity){.4f,0,0},500); assert(fabsf(r.output.vx-.4f)<.00001f);
    run(&r,stop,200); assert(r.output.vx==0);
    step(&r,reverse,false,.01f); assert(r.reversing && r.output.vx==0);
    step(&r,reverse,true,.01f); zero(r);
    step(&r,reverse,true,.01f); assert(r.output.vx<0 && r.output.vx>=-.00005f);
    run(&r,reverse,100); run(&r,forward,300);
    assert(r.reversing && r.output.vx==0);
    step(&r,forward,true,.01f); zero(r);
    /* Full translation and simultaneous full yaw no longer pass both maxima. */
    run(&r,(DriveVelocity){1.5f,0,6},900);
    assert(r.output.vx==1.5f && fabsf(r.output.wz-.2f)<.00001f);
    assert(hypotf(r.limited_target.vx,r.limited_target.vy)*fabsf(r.limited_target.wz)<=.30001f);
    manual_ramp_reset(&r);
    run(&r,(DriveVelocity){0,0,6},1600); assert(r.output.wz==6);
    run(&r,(DriveVelocity){0,0,-6},1200); assert(r.output.wz==0 && r.reversing);
    step(&r,(DriveVelocity){0,0,-6},true,.01f); zero(r);
    manual_ramp_reset(&r);
    run(&r,(DriveVelocity){1.06066017f,1.06066017f,0},900);
    assert(fabsf(r.output.vx-1.06066017f)<.00001f);
    run(&r,(DriveVelocity){0,1.5f,0},800);
    assert(r.output.vx==0 && r.output.vy==1.5f);
    run(&r,(DriveVelocity){0,-1.5f,0},800); assert(r.output.vy==0 && r.reversing);
    step(&r,(DriveVelocity){0,-1.5f,0},true,.01f); zero(r);
    /* Rapid retargeting while acceleration is cancelled: no jump, no runaway. */
    for(unsigned i=0;i<5000;++i) {
        const float x=(i/23)%2 ? .8f : .4f, y=(i/37)%2 ? .2f : 0;
        step(&r,(DriveVelocity){x,y,(i/53)%2 ? .2f:0},false,(i%2)? .007f:.013f);
    }
    run(&r,stop,1000); assert(r.output.vx==0 && r.output.vy==0 && r.output.wz==0);
    /* Every original acceleration phase can be interrupted without overshoot
       outside the speed gear envelope; ordinary stop reaches exact zero. */
    for(unsigned cut=1;cut<800;cut+=13) {
        manual_ramp_reset(&r); run(&r,forward,(int)cut);
        run(&r,stop,1000); assert(r.output.vx==0);
    }
    /* Verify the emitted command's finite-difference jerk, independently of
     * internal acceleration bookkeeping, including mixed-axis retargets. */
    manual_ramp_reset(&r);
    DriveVelocity avg_prev={0}; float previous_dt=0;
    for(unsigned i=0;i<12000;++i) {
        const float dt=i%2 ? .013f:.007f;
        DriveVelocity target=i<1800 ? (DriveVelocity){1.5f,0,0} : i<3500 ? (DriveVelocity){.4f,.3f,.2f} :
            i<6000 ? (DriveVelocity){0,0,6} : i<9000 ? (DriveVelocity){1.5f,0,6} : (DriveVelocity){0};
        const DriveVelocity prev=r.output;
        step(&r,target,false,dt);
        DriveVelocity avg={(r.output.vx-prev.vx)/dt,(r.output.vy-prev.vy)/dt,(r.output.wz-prev.wz)/dt};
        if(previous_dt>0) {
            float spacing=.5f*(dt+previous_dt);
            assert(hypotf(avg.vx-avg_prev.vx,avg.vy-avg_prev.vy)<=MANUAL_RAMP_LINEAR_JERK*spacing+.0003f);
            assert(fabsf(avg.wz-avg_prev.wz)<=MANUAL_RAMP_ANGULAR_JERK*spacing+.0005f);
        }
        avg_prev=avg; previous_dt=dt;
    }
    assert(r.output.vx==0 && r.output.vy==0 && r.output.wz==0);
    /* +/- one fast-gear stick count at 10ms must not prevent convergence. */
    manual_ramp_reset(&r);
    for(unsigned i=0;i<2000;++i) step(&r,(DriveVelocity){i%2 ? 1.003f:.997f,0,0},false,.01f);
    assert(fabsf(r.output.vx-1)<MANUAL_RAMP_LINEAR_RETARGET_BAND+.00001f);
    assert(r.output.vx>.99f && fabsf(r.acceleration.vx)<.00001f);
    manual_ramp_reset(&r);
    for(unsigned i=0;i<2000;++i) step(&r,(DriveVelocity){0,0,i%2 ? .512f:.488f},false,.01f);
    assert(fabsf(r.output.wz-.5f)<MANUAL_RAMP_ANGULAR_RETARGET_BAND+.00001f);
    assert(fabsf(r.acceleration.wz)<.00001f);
    const float invalid_dt[]={-.01f,.051f,NAN,INFINITY};
    for(unsigned i=0;i<4;++i) { assert(!manual_ramp_step(&r,forward,false,true,invalid_dt[i])); zero(r); }
    assert(!manual_ramp_step(NULL,stop,true,true,.01f));
    assert(!manual_ramp_step(&r,(DriveVelocity){NAN,0,0},false,true,.01f)); zero(r);
    assert(!manual_ramp_step(&r,(DriveVelocity){100,0,0},false,true,.01f)); zero(r);
    r.output.vy=INFINITY; assert(!manual_ramp_step(&r,forward,false,true,.01f)); zero(r);
    r.linear.ax=NAN; assert(!manual_ramp_step(&r,forward,false,true,.01f)); zero(r);
    step(&r,forward,true,0); zero(r);
    /* Closed gate emits only direction preview; not a velocity ramp. */
    for(unsigned i=0;i<500;++i) {
        assert(manual_ramp_step(&r,forward,true,false,.05f));
        assert(r.output.vx<=.002001f && r.output.vx>=.00199f);
        assert(r.acceleration.vx==0);
    }
    step(&r,forward,true,.01f); assert(r.output.vx<=.00205f);
    assert(manual_ramp_step(&r,reverse,false,false,.05f)); assert(r.reversing && r.output.vx==0);
    for(unsigned i=0;i<20;++i) { assert(manual_ramp_step(&r,reverse,false,false,.05f)); assert(r.reversing && r.output.vx==0); }
    manual_ramp_reset(&r); zero(r); /* protective stop is immediate, not profiled */
    puts("MANUAL_RAMP_TEST_OK (jerk, irregular dt, gears, reversal, vector/rotation, coupling, preview, invalidation)");
    return 0;
}
