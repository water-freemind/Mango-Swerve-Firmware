#include "chassis_profile.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#define CHECK(e) do {if(!(e)){fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e);exit(1);}}while(0)
int main(void) {
    for(unsigned mode=0;mode<7;++mode) {
        const ChassisProfile* p=chassis_profile_for_mode(mode);
        CHECK(p && steer_wheel_model_valid(&p->model));
        /* Literal golden values protect the original mode-specific behavior. */
        CHECK(fabsf(p->model.length-.26572986916f)<1e-8f && fabsf(p->model.width-.26572986916f)<1e-8f);
        CHECK(fabsf(p->model.wheel_radius-(mode==6?.051f:.035276f))<1e-8f);
        CHECK(p->model.max_wheel_linear_speed==2.0f);
        CHECK(p->wheel_drive_ratio==1.0f);
        CHECK(p->span_source==CHASSIS_DIMENSION_LEGACY);
        CHECK(p->radius_source==(mode==6?CHASSIS_DIMENSION_USER_NOMINAL:CHASSIS_DIMENSION_LEGACY));
        CHECK(!p->axis_coordinates_confirmed && !p->loaded_radius_calibrated);
        CHECK(strcmp(p->name,mode==6?"NOMINAL102_LEGACY_SPAN":"LEGACY_GEOMETRY")==0);
    }
    CHECK(chassis_profile_for_mode(7)==NULL);
    CHECK(chassis_profile_for_mode((unsigned)-1)==NULL);
    puts("CHASSIS_PROFILE_OK");
    return 0;
}
