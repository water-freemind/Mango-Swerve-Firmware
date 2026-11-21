/* Included by verify_car_buttons.py after the extracted production functions. */
static const StraightInput ready={.stationary=true,.healthy=true};

static void enqueue(uint16_t seq,uint8_t op,uint32_t now) {
    Can3Frame frame={.len=8,.data={0xc1,op,(uint8_t)seq,(uint8_t)(seq>>8)},.rx_ms=now};
    frame.data[7]=car_checksum(frame.data);
    assert(car_receive(&frame,NULL));
}

static void command(uint16_t seq,uint8_t op,uint32_t now) {
    enqueue(seq,op,now);
    car_commands(now,ready);
}

static void direct_start_tests(void) {
    command(50003,CAR_STOP,21);
    reject_arm=false;
    unsigned origins=origin_calls, starts=go_calls;
    command(50004,CAR_START_NAV,22);
    assert(full_map.phase==MAP_RUNNING && !started_work && car_reply[4]==0);
    assert(origin_calls==origins+1 && go_calls==starts+1);
    command(50004,CAR_START_NAV,23);
    assert(origin_calls==origins+1 && go_calls==starts+1);
    command(50005,CAR_START_WORK,24);
    assert(car_reply[4]==4 && !map_work_enabled);

    /* A stop in the same receive batch wins over an older start retry. */
    enqueue(50006,CAR_STOP,25);
    enqueue(50004,CAR_START_NAV,25);
    car_commands(25,ready);
    assert(car_reply[1]==CAR_STOP && full_map.phase==MAP_IDLE);
    command(50004,CAR_START_NAV,26);
    assert(go_calls==starts+1 && full_map.phase==MAP_IDLE);

    work_ready=false;
    command(50007,CAR_START_WORK,27);
    assert(car_reply[4]==8 && origin_calls==origins+1 && !map_work_enabled);
    work_ready=true;
    command(50007,CAR_START_WORK,28);
    assert(go_calls==starts+1);

    control_source.mode=0;control_source.manual_armed=true;
    command(50008,CAR_START_NAV,29);
    assert(car_reply[4]==3 && go_calls==starts+1);
    control_source.manual_armed=false;control_source.mode=DRIVE_SOURCE_HOST;
    enqueue(50009,CAR_START_WORK,30);
    hardware_ready=false;
    car_commands(30,(StraightInput){.stationary=true,.healthy=false});
    hardware_ready=true;
    assert(car_reply[4]==10 && origin_calls==origins+1);
    enqueue(50010,CAR_START_WORK,31);
    car_commands(31,(StraightInput){.stationary=false,.healthy=true});
    assert(car_reply[4]==4 && go_calls==starts+1);
    enqueue(50011,CAR_START_WORK,32);
    car_commands(300,ready);
    assert(car_reply[4]==2 && origin_calls==origins+1);

    reject_origin=true;
    unsigned arms=arm_calls;
    command(50012,CAR_START_WORK,301);
    assert(car_reply[4]==9 && arm_calls==arms && !map_work_enabled);
    reject_origin=false;
    reject_arm=true;
    command(50013,CAR_START_WORK,302);
    assert(car_reply[4]==8 && go_calls==starts+1 && !map_work_enabled);
    reject_arm=false;
    reject_go=true;
    command(50014,CAR_START_WORK,303);
    assert(car_reply[4]==11 && full_map.phase==MAP_IDLE && !map_work_enabled);
    reject_go=false;

    command(30000,CAR_STATUS,304); /* One-way session synchronization after an app restart. */
    command(30001,CAR_START_WORK,305);
    assert(car_reply[4]==0 && full_map.phase==MAP_RUNNING && started_work && map_work_enabled);
    assert(go_calls==starts+3);
    command(30002,CAR_STOP,306);
    control_source.mode=0;
    rc_online=true;rc_state.rc_data.channel[REMOTE_CH_SWD]=REMOTE_SW_HIGH;
    assert(camera_swd_stop());
    command(30003,CAR_START_WORK,307);
    command(30004,CAR_ARM,308);
    command(30005,CAR_GO,309);
    assert(car_reply[4]==3 && full_map.phase==MAP_IDLE && !camera_map_owner && go_calls==starts+3);
    rc_online=false;
    assert(!camera_swd_stop());
    enqueue(30006,CAR_START_WORK,310);
    const StraightInput remote_offline={.stationary=true,.healthy=false};
    car_commands(310,remote_offline);
    assert(car_reply[4]==0 && full_map.phase==MAP_RUNNING && started_work && camera_map_owner);
    assert(map_drive_selected() && map_drive_input(remote_offline).healthy);
    hardware_ready=false;
    assert(!map_drive_input(remote_offline).healthy);
    hardware_ready=true;
    command(30007,CAR_STOP,311);
    assert(!camera_map_owner && !map_drive_selected() && !map_drive_input(remote_offline).healthy);
    test.phase=ST_RUNNING;
    command(30008,CAR_START_NAV,312);
    assert(car_reply[4]==3 && !camera_map_owner);
    test.phase=0;
    command(30009,CAR_START_NAV,313);
    assert(car_reply[4]==0 && full_map.phase==MAP_RUNNING && !started_work && camera_map_owner);
    puts("CAR_BUTTON_NO_REMOTE_OK explicit_camera_start_continues_hardware_gate_stop_and_owner_conflict");
}
int main(void) {
    control_source.mode=DRIVE_SOURCE_HOST;full_map.reason="TEST";
    command(10,CAR_NAV,0);assert(car_reply[4]==0 && !map_work_enabled);
    command(11,CAR_ORIGIN,1);assert(origin_calls==1 && car_reply[4]==0);
    command(11,CAR_ORIGIN,2);assert(origin_calls==1); /* Retry must not reset pose. */
    command(12,CAR_ARM,3);assert(arm_calls==1 && car_arm_token==42);
    command(13,CAR_GO,4);assert(go_calls==1 && car_reply[4]==0);
    command(13,CAR_GO,5);assert(go_calls==1); /* Lost ACK cannot start twice. */
    control_source.mode=0;
    command(14,CAR_STOP,6);assert(stop_calls==1 && output_stops==1 && car_reply[4]==0);
    command(13,CAR_GO,7);assert(go_calls==1 && stop_calls==1); /* Old GO after STOP. */
    control_source.manual_armed=true;
    command(15,CAR_WORK,8);assert(car_reply[4]==3);
    control_source.manual_armed=false;
    control_source.mode=DRIVE_SOURCE_HOST;
    full_map.phase=MAP_RUNNING;
    command(16,CAR_NAV,9);assert(car_reply[4]==4);
    full_map.phase=MAP_IDLE;g_station_work.hold=true;
    command(17,CAR_WORK,10);assert(car_reply[4]==6);
    g_station_work.hold=false;
    command(18,CAR_WORK,11);assert(car_reply[4]==0 && map_work_enabled);
    command(19,CAR_GO,12);assert(car_reply[4]==7 && go_calls==1);
    command(20,CAR_ARM,13);full_map.token++; /* Other controller replaces the arm token. */
    command(21,CAR_GO,14);assert(car_reply[4]==7 && go_calls==1);
    command(22,CAR_STOP,15);
    reject_arm=true;
    command(23,CAR_ARM,16);assert(car_reply[4]==8 && !car_arm_token);
    command(24,CAR_GO,17);assert(car_reply[4]==7 && go_calls==1);
    /* Query synchronizes sequence after camera reboot and never starts the map. */
    command(50000,CAR_STATUS,18);command(50001,CAR_NAV,19);assert(car_reply[4]==0);
    command(50002,CAR_ARM,20);assert(car_reply[4]==8);
    puts("CAR_BUTTON_GATES_DEDUP_TOKEN_STOP_AND_RESTART_OK");
    direct_start_tests();
    puts("CAR_BUTTON_DIRECT_START_GATES_DEDUP_STOP_AND_MODE_OK");
    return 0;
}
