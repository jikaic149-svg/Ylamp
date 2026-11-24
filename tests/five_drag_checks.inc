#include "five_drag.h"

static void test_drag_mode(void)
{
    FiveMIT_State x={0};
    FiveDrag_State mode={0};
    const float g[5]={0,1,.2f,.1f,.05f};
    for (unsigned i=0;i<5;++i) {
        x.raw_rad[i]=workspace_center_raw(i);
        x.software_deg[i]=(x.raw_rad[i]-g_joint_reference_config[i].zero_rad)/
            .01745329251994329577;
    }
    float out[5]={9,9,9,9,9};
    unsigned limited=99;
    assert(!FiveDrag_Sample(&mode,&x,g,0,out,&limited));
    assert(!FiveDrag_Enter(NULL,&x,0));
    x.velocity_rad_s[0]=NAN;
    assert(!FiveDrag_Enter(&mode,&x,0) && !mode.active);
    x.velocity_rad_s[0]=0;
    uint32_t now=UINT32_MAX-100;
    assert(FiveDrag_Enter(&mode,&x,now));
    assert(!FiveDrag_Enter(&mode,&x,now));
    assert(FiveDrag_Sample(&mode,&x,g,now,out,&limited));
    assert(!memcmp(out,g,sizeof g) && !limited);

    /* Drive zero codes contain a half-LSB velocity bias. Never turn that
     * into an autonomous friction kick, even after the entry ramp ends. */
    for (unsigned k=0;k<200;++k) {
        x.velocity_rad_s[0]=x.velocity_rad_s[1]=.0025f;
        x.velocity_rad_s[2]=x.velocity_rad_s[3]=x.velocity_rad_s[4]=.0074f;
        assert(FiveDrag_Sample(&mode,&x,g,now+=2,out,&limited));
        assert(!memcmp(out,g,sizeof g) && !limited);
    }
    float q[5],nominal[5],bending[5];
    for (unsigned i=0;i<5;++i) {
        q[i]=(float)((x.software_deg[i]+g_five_mit_hw[i].cad_offset_deg)*
            .01745329251994329577);
        assert(g_five_drag[i].total_max_nm<=g_five_mit_tracking[i].max_nm);
        if (i>=2) {
            assert(g_five_drag[i].fraction==0 && g_five_drag[i].damping_nm_s_rad==0 &&
                   g_five_drag[i].total_max_nm==g_five_mit_hw[i].model_max_nm);
        }
    }
    assert(FiveDynamics_GravityLoad(q,nominal,bending));
    for (int sign=1;sign>=-1;sign-=2) {
        for (unsigned i=0;i<5;++i) { x.velocity_rad_s[i]=sign*.08f; }
        if (sign<0) {
            assert(FiveDrag_Sample(&mode,&x,g,now+=2,out,&limited));
            /* Old positive filtered speed must not produce positive assist
             * into a newly negative movement. Only damping remains here. */
            for (unsigned i=0;i<2;++i) {
                const float expected=g[i]+g_five_drag[i].damping_nm_s_rad*
                    (.08f-g_five_drag[i].deadband_rad_s);
                assert(fabsf(out[i]-expected)<1e-6f);
            }
        }
        for (unsigned k=0;k<300;++k) {
            assert(FiveDrag_Sample(&mode,&x,g,now+=2,out,&limited));
        }
        assert(!limited);
        for (unsigned i=0;i<5;++i) {
            if (i>=2) { assert(out[i]==g[i]);continue; }
            const FiveDrag_Axis *a=&g_five_drag[i];
            const float velocity=sign*(.08f-a->deadband_rad_s);
            float full;
            assert(FiveMIT_SpeedFrictionTorque(i,velocity,nominal[i],bending[i],&full));
            const float assist=fmaxf(-a->assist_max_nm,fminf(a->assist_max_nm,a->fraction*full));
            const float expected=g[i]+assist-a->damping_nm_s_rad*velocity;
            assert(fabsf(out[i]-expected)<2e-6f && sign*(out[i]-g[i])>0);
            assert(fabsf(out[i]-g[i])<fabsf(full));
            assert(fabsf(out[i])<=a->total_max_nm);
        }
    }
    memset(x.velocity_rad_s,0,sizeof x.velocity_rad_s);
    assert(FiveDrag_Sample(&mode,&x,g,now+=2,out,&limited));
    assert(!memcmp(out,g,sizeof g)); /* No sign memory or trailing kick. */
    /* An equivalent raw drive branch must not change continuous travel logic. */
    x.raw_rad[0]+=6.2831853071795864769f;x.velocity_rad_s[0]=.08f;
    for (unsigned k=0;k<300;++k) {
        assert(FiveDrag_Sample(&mode,&x,g,now+=2,out,&limited));
    }
    assert(out[0]>g[0]);
    x.raw_rad[0]-=6.2831853071795864769f;
    /* Original ends: no outward assistance, no lockout of inward retreat. */
    x.raw_rad[0]=g_joint_reference_config[0].observed_lower_rad-.01f;
    x.software_deg[0]=(x.raw_rad[0]-g_joint_reference_config[0].zero_rad)/
        .01745329251994329577;
    x.velocity_rad_s[0]=-.08f;
    for (unsigned k=0;k<300;++k) {
        assert(FiveDrag_Sample(&mode,&x,g,now+=2,out,&limited));
    }
    assert(fabsf(out[0]-g[0]-g_five_drag[0].damping_nm_s_rad*
        (.08f-g_five_drag[0].deadband_rad_s))<1e-6f);
    x.velocity_rad_s[0]=.08f;
    for (unsigned k=0;k<300;++k) {
        assert(FiveDrag_Sample(&mode,&x,g,now+=2,out,&limited));
    }
    assert(out[0]>g[0]); /* Same real out-of-range reading, inward help works. */
    x.raw_rad[0]=workspace_center_raw(0);
    x.software_deg[0]=(x.raw_rad[0]-g_joint_reference_config[0].zero_rad)/
        .01745329251994329577;

    FiveDrag_State saved;memcpy(&saved,&mode,sizeof saved);
    const float sentinel[5]={4,4,4,4,4};memcpy(out,sentinel,sizeof out);limited=99;
    x.velocity_rad_s[3]=NAN;
    assert(!FiveDrag_Sample(&mode,&x,g,now+2,out,&limited));
    assert(!memcmp(&saved,&mode,sizeof mode) && !memcmp(out,sentinel,sizeof out) && limited==99);
    x.velocity_rad_s[3]=0;
    float invalid_g[5];memcpy(invalid_g,g,sizeof g);
    invalid_g[1]=g_five_mit_hw[1].model_max_nm+.1f;
    assert(!FiveDrag_Sample(&mode,&x,invalid_g,now+2,out,&limited));
    assert(!memcmp(&saved,&mode,sizeof mode) && !memcmp(out,sentinel,sizeof out));
    /* Numerical stress only, not a claim that such speed is physically allowed. */
    x.velocity_rad_s[0]=1000;
    assert(FiveDrag_Sample(&mode,&x,g,now+=2,out,&limited));
    assert((limited & 1) && fabsf(out[0])<=g_five_drag[0].total_max_nm);
    for (unsigned i=2;i<5;++i) { assert(out[i]==g[i]); }
    FiveDrag_Stop(&mode);
    assert(!mode.active && !FiveDrag_Sample(&mode,&x,g,now+2,out,&limited));
}

static void test_drag_control(void)
{
    test_drag_mode();
    DM_Motor_t *motors[5];five_setup(motors);
    assert(FiveControl_Init(motors,test_tick));
    assert(FiveControl_Drag()==BSP_CAN_NOT_READY);
    const FiveControl_Status *s=FiveControl_StatusNow();
    FiveControl_Release();FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_WAIT && !s->submitted);
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_GRAVITY && !s->fault);
    test_ipsr=1;assert(FiveControl_Drag()==BSP_CAN_NOT_READY);test_ipsr=0;
    /* A moving handover does not restore the deleted all-axis rest gate. */
    for (unsigned i=0;i<5;++i) {
        const uint16_t v=(uint16_t)lroundf((.08f/g_motor_can_config[i].vmax+1)*2047.5f);
        plant_sample_at_speed(i,1,workspace_center_raw(i),v);
    }
    assert(FiveControl_Drag()==BSP_CAN_OK);
    assert(FiveControl_Drag()==BSP_CAN_OK); /* Selecting the same idle is harmless. */
    for (unsigned k=0;k<200;++k) {
        for (unsigned i=0;i<5;++i) {
            const uint16_t v=(uint16_t)lroundf((.08f/g_motor_can_config[i].vmax+1)*2047.5f);
            plant_sample_at_speed(i,1,workspace_center_raw(i),v);
        }
        FiveControl_Tick(test_tick+=2);
        assert(s->phase==FIVE_DRAG && !s->fault && s->submitted==31 && !s->limited);
        assert_gravity_frames(s,0x1c);
    }
    for (unsigned i=0;i<2;++i) {
        const DM_Motor_MIT_Command_t pure={.torque_ff_nm=s->gravity_nm[i]};
        BSP_CAN_Frame_t frame;assert(DM_Motor_EncodeMIT(&g_motor_can_config[i],&pure,&frame)==BSP_CAN_OK);
        assert(memcmp(motor_last_mit[i+1],frame.data,8));
    }
    uint8_t running[2][8];
    for (unsigned i=0;i<2;++i) { memcpy(running[i],motor_last_mit[i+1],8); }
    FiveControl_Release();FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DRAG && !s->fault);
    for (unsigned i=0;i<2;++i) {
        assert(!memcmp(running[i],motor_last_mit[i+1],8)); /* No filter/ramp reset. */
    }
    /* Unassisted J4 stays at pure G even outside its recorded active travel;
     * it must not block J1/J2 manual assist or silently change its branch. */
    plant_sample_at(3,1,-184.f*.01745329251994329577f);
    FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DRAG && !s->fault);assert_gravity_frames(s,0x1c);
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    FiveControl_Tick(test_tick+=2);
#ifdef YLAMP_J3_REVERSE_TEST
    assert(FiveControl_ReverseJ3()==BSP_CAN_NOT_READY);
#endif
    assert(FiveControl_KeyPress()==BSP_CAN_OK);
    FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_IMPEDANCE && !s->fault);
    FiveControl_Release();FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DRAG && !s->fault);
    FiveControl_Gravity();FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_GRAVITY);assert_gravity_frames(s,31);
    FiveControl_Release();FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_GRAVITY);assert_gravity_frames(s,31);
    assert(FiveControl_Drag()==BSP_CAN_OK);
    double target[5];
    for (unsigned i=0;i<5;++i) {
        target[i]=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/
            .01745329251994329577;
    }
    target[1]+=.2;
    FiveMIT_Plan expected_plan;
    double from[5];
    for (unsigned i=0;i<5;++i) {
        from[i]=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/
            .01745329251994329577;
    }
    assert(FiveMIT_Prepare(&expected_plan,from,target,1));
    assert(FiveControl_Move(target,1000)==BSP_CAN_OK);
    FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_MOTION && !s->fault);
    FiveMIT_State measured={0};FiveMIT_Output expected_motion;
    for (unsigned i=0;i<5;++i) {
        measured.software_deg[i]=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/
            .01745329251994329577;
        measured.raw_rad[i]=s->feedback[i].position_rad;
        measured.velocity_rad_s[i]=s->feedback[i].velocity_rad_s;
    }
    assert(FiveMIT_SampleFeedback(&expected_plan,.002,&measured,s->gravity_nm,&expected_motion));
    for (unsigned i=0;i<5;++i) {
        BSP_CAN_Frame_t expected;
        assert(DM_Motor_EncodeMIT(&g_motor_can_config[i],&expected_motion.motor[i],&expected)==BSP_CAN_OK);
        assert(!memcmp(motor_last_mit[i+1],expected.data,8)); /* No second drag F. */
    }
    FiveControl_Release();FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DRAG && !s->fault);
    assert(FiveControl_Move(target,1000)==BSP_CAN_OK);
    for (unsigned k=0;k<500;++k) {
        for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
        FiveControl_Tick(test_tick+=2);
    }
    assert(s->phase==FIVE_POSITION && !s->fault);
    FiveControl_Release();FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DRAG && !s->fault);
    FiveControl_Gravity();FiveControl_Tick(test_tick+=2);
    assert(FiveControl_Drag()==BSP_CAN_OK);
    can_io[1].free_slots=0;
    FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DEGRADED && (s->fault & FIVE_TX) && s->submitted==0x1c);
    assert_gravity_frames(s,0x1c);
    assert(FiveControl_Drag()==BSP_CAN_NOT_READY);
    can_io[1].free_slots=8;
    FiveControl_Release();
    for (unsigned k=0;k<20;++k) {
        for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
        FiveControl_Tick(test_tick+=2);
        assert(s->phase==FIVE_DEGRADED && (s->fault & FIVE_TX));
        assert_gravity_frames(s,31); /* Healthy feedback does not restore drag after fault. */
    }
    assert(FiveControl_Acknowledge()==BSP_CAN_OK);
    FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DRAG && !s->fault);
    for (unsigned i=1;i<=5;++i) { assert(!motor_enable_calls[i] && !motor_disable_calls[i]); }
    puts("PASS: partial drag, persistent idle/pure G, KEY/USB/move/hold/explicit ACK, no double F or automatic fault recovery");
}
