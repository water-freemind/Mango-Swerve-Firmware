/* Command-generator comparison only: feedback uses the preceding command with
 * two zero-speed samples. This is not a physical plant or braking measurement. */
#ifdef USE_LEGACY_RAMP
#include "fixtures/manual_ramp_legacy.h"
#else
#include "manual_ramp.h"
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>

static DriveVelocity request(unsigned scenario, float t, bool* gate, bool* reset) {
    *gate=true; *reset=false;
    switch(scenario) {
    case 0: return t<10 ? (DriveVelocity){1.5f,0,0} : (DriveVelocity){0};
    case 1: return t<6 ? (DriveVelocity){.8f,0,0} : t<14 ? (DriveVelocity){-.8f,0,0} : (DriveVelocity){0};
    case 2: return t<9 ? (DriveVelocity){1.06066017f,1.06066017f,0} : (DriveVelocity){0};
    case 3: return t<4 ? (DriveVelocity){.8f,0,0} : t<11 ? (DriveVelocity){0,.8f,0} : (DriveVelocity){0};
    case 4: return t<9 ? (DriveVelocity){1.5f,0,0} : t<15 ? (DriveVelocity){.4f,0,0} : t<22 ? (DriveVelocity){.8f,0,0} : (DriveVelocity){0};
    case 5: return t<10 ? (DriveVelocity){1.5f,0,6} : (DriveVelocity){0};
    case 6: return t<17 ? (DriveVelocity){0,0,6} : t<30 ? (DriveVelocity){1.5f,0,6} : (DriveVelocity){0};
    case 7: *gate=t>=3 && !(t>=6 && t<7); return t<15 ? (DriveVelocity){.8f,0,0} : (DriveVelocity){0};
    case 8: return t<9 ? (DriveVelocity){1.5f,0,0} : (DriveVelocity){0};
    case 10: return t<15 ? (DriveVelocity){((unsigned)(t*100)%2) ? 1.003f:.997f,0,0} : (DriveVelocity){0};
    default: *reset=t>=5; return (DriveVelocity){.8f,0,0};
    }
}
int main(void) {
    const char* names[]={"step_release","reverse","diagonal","direction_change","gear_change","combined","spin_to_translation","alignment","jitter","protective_reset","stick_noise"};
    const float duration[]={20,22,18,20,30,22,44,22,20,8,22};
    puts("scenario,t_s,dt_s,drive_allowed,protective_reset,stationary,input_vx,input_vy,input_wz,limited_vx,limited_vy,limited_wz,cmd_vx,cmd_vy,cmd_wz,accel_vx,accel_vy,accel_wz,reversing");
    for(unsigned scene=0;scene<11;++scene) {
        ManualRamp r={0}; unsigned zeros=2, i=0; float t=0;
        while(t<duration[scene]) {
            const float dt=scene==8 ? ((i%2)? .013f:.007f) : .01f;
            bool gate, reset; DriveVelocity target=request(scene,t,&gate,&reset);
            const bool stationary=zeros>=2;
            if(reset) manual_ramp_reset(&r);
            else if(!manual_ramp_step(&r,target,stationary,gate,dt)) return 2;
            DriveVelocity limited=target, accel={0};
#ifndef USE_LEGACY_RAMP
            limited=r.limited_target; accel=r.acceleration;
#endif
            printf("%s,%.9f,%.9f,%u,%u,%u,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%u\n", names[scene],t+dt,dt,gate,reset,stationary,
                target.vx,target.vy,target.wz,limited.vx,limited.vy,limited.wz,r.output.vx,r.output.vy,r.output.wz,accel.vx,accel.vy,accel.wz,r.reversing);
            zeros=(r.output.vx==0 && r.output.vy==0 && r.output.wz==0) ? zeros+1 : 0;
            t+=dt; ++i;
        }
    }
    return 0;
}
