static FiveMIT_State trajectory_center(void)
{
    FiveMIT_State x={0};
    for (unsigned i=0;i<5;++i) {
        x.raw_rad[i]=workspace_center_raw(i);
        x.software_deg[i]=(x.raw_rad[i]-g_joint_reference_config[i].zero_rad)/
            .01745329251994329577;
    }
    return x;
}
static void trajectory_stage(FiveTrajectory_State *mode,const FiveMIT_State *x,
                              float displacement,float velocity,float acceleration,uint32_t at)
{
    for (unsigned d=0;d<3;++d) {
        for (unsigned i=0;i<5;++i) {
            const float value=d==0 ? (float)x->software_deg[i]+(i==0 ? displacement : 0) :
                i==0 ? (d==1 ? velocity : acceleration) : 0;
            assert(FiveTrajectory_Set(mode,i,d,value));
        }
    }
    assert(FiveTrajectory_Append(mode,at));
}
static void test_trajectory_mode(void)
{
    static FiveTrajectory_State mode; /* Same bounded/static storage as MCU. */
    FiveMIT_State x=trajectory_center();
    FiveMIT_Reference r,before,after,saved_ref;
    const FiveTrajectory_Point template_first={.at_ms=2000,.position_deg={131,-1,19,116,14}};
    FiveTrajectory_Point template_next=template_first,relative;
    template_next.at_ms=2400;template_next.position_deg[0]+=2.f;
    template_next.velocity_rad_s[0]=.01f;template_next.acceleration_rad_s2[0]=.002f;
    assert(FiveTrajectory_RelativePoint(&relative,&template_next,&template_first,x.software_deg));
    for (unsigned i=0;i<5;++i) {
        assert(fabs(relative.position_deg[i]-(x.software_deg[i]+(i ? 0 : 2)))<1e-4);
        assert(relative.velocity_rad_s[i]==template_next.velocity_rad_s[i] &&
               relative.acceleration_rad_s2[i]==template_next.acceleration_rad_s2[i]);
    }
    assert(relative.at_ms==2400);
    FiveTrajectory_Point unchanged=relative;
    double invalid_origin[5]={NAN,0,0,0,0};
    assert(!FiveTrajectory_RelativePoint(&relative,&template_next,&template_first,invalid_origin) &&
           !memcmp(&relative,&unchanged,sizeof relative));
    assert(!FiveTrajectory_Sample(&mode,0,&r));
    assert(!FiveTrajectory_Begin(&mode,0) &&
           !FiveTrajectory_Begin(&mode,FIVE_TRAJECTORY_CAPACITY+1));
    assert(FiveTrajectory_Begin(&mode,3));
    assert(!FiveTrajectory_Set(&mode,5,0,0) && !FiveTrajectory_Set(&mode,0,3,0));
    assert(!FiveTrajectory_Set(&mode,0,0,NAN));
    assert(!FiveTrajectory_Append(&mode,500) && !FiveTrajectory_Start(&mode,0) &&
           !FiveTrajectory_PrepareStart(&mode,&x));
    trajectory_stage(&mode,&x,.5f,.01f,.004f,500);
    trajectory_stage(&mode,&x,1.f,.01f,-.003f,1000);
    trajectory_stage(&mode,&x,1.5f,0,0,1500);
    const uint32_t start=UINT32_MAX-1000;
    assert(FiveTrajectory_PrepareStart(&mode,&x));
    assert(FiveTrajectory_Start(&mode,start));
    const FiveTrajectory_Status status=FiveTrajectory_StatusNow(&mode);
    assert(status.active && !status.uploading && status.count==3 && status.duration_ms==1500);
    assert(status.start_ms==start && status.checksum!=0);
    static FiveTrajectory_State bundled;
    assert(FiveTrajectory_Begin(&bundled,3));
    for (unsigned n=0;n<3;++n) {
        FiveTrajectory_Point point={.at_ms=500U*(n+1)};
        for (unsigned i=0;i<5;++i) {
            point.position_deg[i]=(float)x.software_deg[i]+(i==0 ? .5f*(n+1) : 0.f);
        }
        point.velocity_rad_s[0]=n<2 ? .01f : 0.f;
        point.acceleration_rad_s2[0]=n==0 ? .004f : n==1 ? -.003f : 0.f;
        const FiveTrajectory_Status previous=FiveTrajectory_StatusNow(&bundled);
        FiveTrajectory_Point invalid=point;invalid.acceleration_rad_s2[4]=NAN;
        assert(!FiveTrajectory_AppendPoint(&bundled,&invalid));
        if (n==2) {
            invalid=point;invalid.velocity_rad_s[0]=.01f;
            assert(!FiveTrajectory_AppendPoint(&bundled,&invalid));
        }
        const FiveTrajectory_Status rejected=FiveTrajectory_StatusNow(&bundled);
        assert(rejected.count==previous.count && rejected.checksum==previous.checksum &&
               rejected.duration_ms==previous.duration_ms);
        assert(FiveTrajectory_AppendPoint(&bundled,&point));
    }
    assert(FiveTrajectory_PrepareStart(&bundled,&x) && FiveTrajectory_Start(&bundled,start));
    const FiveTrajectory_Status bundle=FiveTrajectory_StatusNow(&bundled);
    assert(bundle.checksum==status.checksum && !memcmp(bundle.knot_ms,status.knot_ms,sizeof bundle.knot_ms));
    for (uint32_t tick=0;tick<=1500;++tick) {
        assert(FiveTrajectory_Sample(&mode,tick,&r) && FiveTrajectory_Sample(&bundled,tick,&before));
        for (unsigned i=0;i<5;++i) {
            assert(r.position_deg[i]==before.position_deg[i] &&
                   r.velocity_rad_s[i]==before.velocity_rad_s[i] &&
                   r.acceleration_rad_s2[i]==before.acceleration_rad_s2[i]);
        }
    }
    assert(!FiveTrajectory_Begin(&mode,1) && !FiveTrajectory_Set(&mode,0,0,0));
    assert(FiveTrajectory_Sample(&mode,0,&r));
    for (unsigned i=0;i<5;++i) {
        assert(r.position_deg[i]==status.origin_deg[i]);
        assert(r.velocity_rad_s[i]==0 && r.acceleration_rad_s2[i]==0);
    }
    for (unsigned knot=500;knot<=1000;knot+=500) {
        assert(FiveTrajectory_Sample(&mode,knot-1,&before));
        assert(FiveTrajectory_Sample(&mode,knot,&r));
        assert(FiveTrajectory_Sample(&mode,knot+1,&after));
        assert(r.velocity_rad_s[0]==.01f); /* Not a rest-to-rest segment restart. */
        assert(fabsf(before.velocity_rad_s[0]-after.velocity_rad_s[0])<.0001f);
        assert(r.acceleration_rad_s2[0]==(knot==500 ? .004f : -.003f));
        /* Common q/v/a, finite jerk on both sides, no stop at the knot. */
        FiveMIT_Reference left2,right2;
        assert(FiveTrajectory_Sample(&mode,knot-2,&left2));
        assert(FiveTrajectory_Sample(&mode,knot+2,&right2));
        assert(fabsf(before.acceleration_rad_s2[0]-r.acceleration_rad_s2[0])<
            .75f*fabsf(left2.acceleration_rad_s2[0]-r.acceleration_rad_s2[0])+1e-8f);
        assert(fabsf(after.acceleration_rad_s2[0]-r.acceleration_rad_s2[0])<
            .75f*fabsf(right2.acceleration_rad_s2[0]-r.acceleration_rad_s2[0])+1e-8f);
        assert(fabs((after.position_deg[0]-before.position_deg[0])*.0174532925199433/.002-
            r.velocity_rad_s[0])<g_five_trajectory_limits[0].jerk_rad_s3*1e-6/6+1e-8);
    }
    assert(FiveTrajectory_Sample(&mode,1500,&r));
    assert(FiveTrajectory_Sample(&mode,UINT32_MAX,&after));
    assert(!memcmp(&r,&after,sizeof r) && r.velocity_rad_s[0]==0 &&
           r.acceleration_rad_s2[0]==0);
    FiveTrajectory_Cancel(&mode);memcpy(&saved_ref,&r,sizeof r);
    assert(!FiveTrajectory_Sample(&mode,100,&r) && !memcmp(&r,&saved_ref,sizeof r));
    assert(FiveTrajectory_Begin(&mode,3));
    trajectory_stage(&mode,&x,0,0,0,1000);
    assert(!FiveTrajectory_Append(&mode,1000) && !FiveTrajectory_Append(&mode,120001));
    for (unsigned d=0;d<3;++d) {
        for (unsigned i=0;i<5;++i) {
            assert(FiveTrajectory_Set(&mode,i,d,d==0 ? (float)x.software_deg[i] :
                d==1 && i==0 ? 1000.f : 0));
        }
    }
    assert(!FiveTrajectory_Append(&mode,2000) && FiveTrajectory_StatusNow(&mode).count==1);
    float q[5],g[5];
    for (unsigned i=0;i<5;++i) {
        q[i]=(float)((x.software_deg[i]+g_five_mit_hw[i].cad_offset_deg)*.0174532925199433);
        r.position_deg[i]=x.software_deg[i];r.velocity_rad_s[i]=r.acceleration_rad_s2[i]=0;
    }
    r.velocity_rad_s[0]=.01f;
    assert(FiveSupport_GravityModel(q,(float)(x.software_deg[2]*.0174532925199433),g));
    FiveMIT_Output out,saved;
    assert(FiveMIT_SampleReferenceFeedback(&r,1,&x,g,&out));
    float nominal[5],bending[5],friction;
    assert(FiveDynamics_GravityLoad(q,nominal,bending));
    assert(FiveMIT_SpeedFrictionTorque(0,.01f,nominal[0],bending[0],&friction));
    assert(friction>0);
    assert(fabsf(out.motor[0].torque_ff_nm-g[0]-.01f*g_five_mit_tracking[0].kd-friction)<1e-6f);
    for (unsigned i=0;i<5;++i) { assert(out.motor[i].kp==0 && out.motor[i].kd==0); }
    /* Every axis/direction: SAME ordinary-Move/Ruckig law, F exactly once.
     * Opposite measured velocity must not reverse its feedforward direction. */
    for (unsigned axis=0;axis<5;++axis) {
        for (int sign=-1;sign<=1;++sign) {
            for (unsigned i=0;i<5;++i) {
                r.velocity_rad_s[i]=i==axis ? sign*.08f : 0;
                x.velocity_rad_s[i]=-sign*.01f;
            }
            assert(FiveMIT_SampleReferenceFeedback(&r,1,&x,g,&out));
            float dynamic[5];
            assert(FiveDynamics_Inverse(q,x.velocity_rad_s,r.acceleration_rad_s2,dynamic));
            for (unsigned i=0;i<5;++i) {
                assert(FiveMIT_SpeedFrictionTorque(i,r.velocity_rad_s[i],
                    nominal[i],bending[i],&friction));
                const float pd=g_five_mit_tracking[i].kd*(r.velocity_rad_s[i]-x.velocity_rad_s[i]);
                assert(fabsf(out.motor[i].torque_ff_nm-dynamic[i]-g[i]+nominal[i]-pd-friction)<2e-6f);
                if (i!=axis || !sign) { assert(friction==0); }
                else { assert(sign*friction>0); }
            }
        }
    }
    /* Including F in the unchanged total budget rejects the whole batch. */
    memset(x.velocity_rad_s,0,sizeof x.velocity_rad_s);
    memset(r.velocity_rad_s,0,sizeof r.velocity_rad_s);
    r.velocity_rad_s[0]=.08f;
    float stressed[5];memcpy(stressed,g,sizeof stressed);
    stressed[0]=g_five_mit_tracking[0].max_nm-.08f*g_five_mit_tracking[0].kd-.01f;
    memcpy(&saved,&out,sizeof saved);
    assert(!FiveMIT_SampleReferenceFeedback(&r,1,&x,stressed,&out) &&
           !memcmp(&out,&saved,sizeof out));
    memcpy(&saved,&out,sizeof saved);r.velocity_rad_s[0]=INFINITY;
    assert(!FiveMIT_SampleReferenceFeedback(&r,1,&x,g,&out) && !memcmp(&out,&saved,sizeof out));
    /* A timing request is a minimum, not permission to exceed v/a/j. */
    FiveTrajectory_Cancel(&mode);
    assert(FiveTrajectory_Begin(&mode,1));
    trajectory_stage(&mode,&x,15,0,0,100);
    assert(FiveTrajectory_PrepareStart(&mode,&x));
    assert(FiveTrajectory_Start(&mode,0));
    const FiveTrajectory_Status retimed=FiveTrajectory_StatusNow(&mode);
    assert(retimed.duration_ms>100 && retimed.duration_ms%2==0);
    FiveMIT_Reference previous;
    assert(FiveTrajectory_Sample(&mode,0,&previous));
    for (uint32_t ms=2;ms<=retimed.duration_ms;ms+=2) {
        assert(FiveTrajectory_Sample(&mode,ms,&r));
        for (unsigned i=0;i<5;++i) {
            const FiveTrajectory_Limit limit=g_five_trajectory_limits[i];
            assert(fabs(r.velocity_rad_s[i])<=limit.velocity_rad_s+1e-6);
            assert(fabs(r.acceleration_rad_s2[i])<=limit.acceleration_rad_s2+1e-6);
            assert(fabs(r.acceleration_rad_s2[i]-previous.acceleration_rad_s2[i])<=
                limit.jerk_rad_s3*.002+1e-6);
        }
        previous=r;
    }
    FiveTrajectory_Cancel(&mode);
}
static void trajectory_control_stage(const FiveMIT_State *x,float displacement,uint32_t at)
{
    for (unsigned d=0;d<3;++d) {
        for (unsigned i=0;i<5;++i) {
            assert(FiveControl_TrajectorySet(i,d,d==0 ?
                (float)x->software_deg[i]+(i==0 ? displacement : 0) : 0)==BSP_CAN_OK);
        }
    }
    assert(FiveControl_TrajectoryAppend(at)==BSP_CAN_OK);
}
static void test_trajectory_control(void)
{
    test_trajectory_mode();
    DM_Motor_t *motors[5];five_setup(motors);
    assert(FiveControl_Init(motors,test_tick));
    assert(FiveControl_TrajectoryBegin(2)==BSP_CAN_NOT_READY);
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    FiveControl_Tick(test_tick+=2);
    const FiveControl_Status *s=FiveControl_StatusNow();
    assert(FiveControl_Drag()==BSP_CAN_OK);FiveControl_Tick(test_tick+=2);
    FiveMIT_State x=trajectory_center();
    assert(FiveControl_TrajectoryBegin(2)==BSP_CAN_OK);
    trajectory_control_stage(&x,.2f,1000);
    assert(s->phase==FIVE_DRAG && !FiveControl_TrajectoryStatus().active);
    FiveControl_Release();
    assert(FiveControl_TrajectoryStart()!=BSP_CAN_OK && !s->fault);
    assert(FiveControl_TrajectoryBegin(2)==BSP_CAN_OK);
    trajectory_control_stage(&x,.2f,1000);trajectory_control_stage(&x,.4f,2000);
    assert(FiveControl_TrajectoryStart()==BSP_CAN_OK);
    assert(FiveControl_TrajectoryStart()==BSP_CAN_NOT_READY);
    for (unsigned k=0;k<1000;++k) {
        for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
        FiveControl_Tick(test_tick+=2);
        assert(!s->fault && !s->limited && s->submitted==31);
        assert(s->phase==(k==999 ? FIVE_POSITION : FIVE_MOTION));
        for (unsigned i=1;i<=5;++i) {
            assert(((motor_last_mit[i][3]&15U)<<8 | motor_last_mit[i][4])==0);
        }
    }
    assert(FiveControl_KeyPress()==BSP_CAN_OK);FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DRAG && !FiveControl_TrajectoryStatus().active);
    assert(FiveControl_TrajectoryBegin(1)==BSP_CAN_OK);
    assert(FiveControl_Impedance()==BSP_CAN_OK);FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_IMPEDANCE && !FiveControl_TrajectoryStatus().uploading);
    FiveControl_Release();FiveControl_Tick(test_tick+=2);
    assert(FiveControl_TrajectoryBegin(1)==BSP_CAN_OK);
    trajectory_control_stage(&x,.2f,2000);
    assert(FiveControl_TrajectoryStart()==BSP_CAN_OK);
    plant_sample(0,0);FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DEGRADED && s->fault && !FiveControl_TrajectoryStatus().active);
    assert_gravity_frames(s,30);
    for (unsigned i=1;i<=5;++i) { assert(!motor_enable_calls[i] && !motor_disable_calls[i]); }
    puts("PASS: five-axis Ruckig through-states, feedback/hold, KEY/session/fault exits; no enable");
}

static void lookaround_tick(const float raw[5])
{
    for (unsigned i=0;i<5;++i) { plant_sample_at(i,1,raw[i]); }
    FiveControl_Tick(test_tick+=2);
    (void)LookaroundTask_Run(false);
}
static void test_lookaround_task(void)
{
    DM_Motor_t *motors[5];five_setup(motors);
    assert(FiveControl_Init(motors,test_tick));
    float raw[5];
    const float anchor[]={131,-1,19,116,14};
    for (unsigned i=0;i<5;++i) {
        raw[i]=g_joint_reference_config[i].zero_rad+anchor[i]*.0174532925199433f;
    }
    for (unsigned k=0;k<FIVE_TRAJECTORY_CAPACITY;++k) {
        lookaround_fixture[k]=(FiveTrajectory_Point){.at_ms=(k+1)*500};
        memcpy(lookaround_fixture[k].position_deg,anchor,sizeof anchor);
    }
    const FiveControl_Status *s=FiveControl_StatusNow();
    lookaround_tick(raw);
    assert(s->phase==FIVE_IDLE && !FiveControl_LocalTrajectoryOwned());
    assert(LookaroundTask_Run(true)==BSP_CAN_OK);
    lookaround_tick(raw);
    assert(s->phase==FIVE_DRAG);
    lookaround_loaded=false;
    assert(LookaroundTask_Run(true)==BSP_CAN_NOT_READY && !FiveControl_LocalTrajectoryOwned());
    lookaround_loaded=true;
    raw[0]+=.1f; /* Beyond the removed 3-degree window: use the captured origin. */
    lookaround_tick(raw);
    assert(LookaroundTask_Run(true)==BSP_CAN_OK);
    for (unsigned i=0;i<=FIVE_TRAJECTORY_CAPACITY;++i) { lookaround_tick(raw); }
    assert(FiveControl_LocalTrajectoryOwned());
    lookaround_tick(raw);
    assert(s->phase==FIVE_MOTION && !s->fault);
    assert(LookaroundTask_Run(true)==BSP_CAN_OK);
    lookaround_tick(raw);
    assert(s->phase==FIVE_IDLE);
    raw[0]-=.1f;
    lookaround_tick(raw);
    assert(LookaroundTask_Run(true)==BSP_CAN_OK);
    lookaround_tick(raw);
    assert(s->phase==FIVE_DRAG);
    assert(FiveControl_TrajectoryBegin(1)==BSP_CAN_OK);
    assert(LookaroundTask_Run(true)==BSP_CAN_NOT_READY);
    FiveControl_Release();
    assert(LookaroundTask_Run(true)==BSP_CAN_OK);
    FiveControl_Release(); /* USB disconnect must not cancel local preparation. */
    assert(FiveControl_TrajectoryStart()==BSP_CAN_NOT_READY);
    assert(!FiveControl_TrajectoryStatus().uploading);
    for (unsigned i=0;i<=FIVE_TRAJECTORY_CAPACITY;++i) { lookaround_tick(raw); }
    assert(FiveControl_LocalTrajectoryOwned());
    lookaround_tick(raw);
    assert(s->phase==FIVE_MOTION && !s->fault);
    FiveControl_Release();
    for (unsigned i=0;i<8010;++i) { lookaround_tick(raw); assert(!s->fault); }
    assert(s->phase==FIVE_IDLE && !FiveControl_LocalTrajectoryOwned());
    assert_gravity_frames(s,31);
    assert(LookaroundTask_Run(true)==BSP_CAN_OK);
    lookaround_tick(raw);
    /* Relative playback retains normal travel checks: no endpoint clipping. */
    const float original_raw=raw[0];
    raw[0]=g_joint_reference_config[0].observed_upper_rad-1.1f*.0174532925199433f;
    lookaround_fixture[1].position_deg[0]+=2.f;
    lookaround_tick(raw);
    assert(LookaroundTask_Run(true)==BSP_CAN_OK);
    for (unsigned i=0;i<=FIVE_TRAJECTORY_CAPACITY+1;++i) { lookaround_tick(raw); }
    assert(s->phase==FIVE_DRAG && !FiveControl_LocalTrajectoryOwned() && !s->fault);
    raw[0]=original_raw;lookaround_fixture[1].position_deg[0]-=2.f;
    lookaround_tick(raw);
    assert(LookaroundTask_Run(true)==BSP_CAN_OK);
    for (unsigned i=0;i<=FIVE_TRAJECTORY_CAPACITY+1;++i) { lookaround_tick(raw); }
    assert(s->phase==FIVE_MOTION && LookaroundTask_Run(true)==BSP_CAN_OK);
    lookaround_tick(raw);
    assert(s->phase==FIVE_IDLE && !FiveControl_LocalTrajectoryOwned());
    assert_gravity_frames(s,31);
    assert(LookaroundTask_Run(true)==BSP_CAN_OK);
    lookaround_tick(raw);
    assert(LookaroundTask_Run(true)==BSP_CAN_OK);
    for (unsigned i=0;i<=FIVE_TRAJECTORY_CAPACITY+1;++i) { lookaround_tick(raw); }
    assert(s->phase==FIVE_MOTION);
    plant_sample_at(0,0,raw[0]);
    FiveControl_Tick(test_tick+=2);
    (void)LookaroundTask_Run(false);
    assert(s->phase==FIVE_DEGRADED && s->fault && !FiveControl_LocalTrajectoryOwned());
    assert_gravity_frames(s,30);
    for (unsigned i=1;i<=5;++i) { assert(!motor_enable_calls[i] && !motor_disable_calls[i]); }
    puts("PASS: one local KEY action, no session takeover, normal/cancel IDLE and fault exit");
}
