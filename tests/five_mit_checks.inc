/* Reuse the same HAL fakes and driver as all other motor tests. */
#include "five_coordinates.h"
#include "five_motion.h"
static void live_workspace_pose(double q[5])
{
    const double cad[5]=FIVE_CAD_OFFSETS,table_cad[5]=FIVE_WORKSPACE_CAD_OFFSETS;
    for (unsigned i=0;i<5;++i) { q[i]+=table_cad[i]-cad[i]; }
}
static float workspace_center_raw(unsigned i)
{
    double center[5]={0};live_workspace_pose(center);
    return g_joint_reference_config[i].zero_rad+(float)(center[i]*.01745329251994329577);
}
static void five_setup(DM_Motor_t *motors[5])
{
    assert(BSP_CAN_Init(&hfdcan1)==BSP_CAN_OK && BSP_CAN_Init(&hfdcan2)==BSP_CAN_OK);
    for (int i=0;i<5;++i) {
        assert(DM_Motor_Init(&g_motor_can_config[i],&motors[i])==BSP_CAN_OK);
    }
}

static void test_gravity_formula(void)
{
    /* Independent Python potential-basis goldens at two recorded wrist poses.
     * Mathematical evaluation only, not workspace or hardware acceptance. */
    const double datum[2][5]={
        {-28.51236855812435,0,27.397681685578284,-180.8771105814127,-46.63179150156308},
        {-28.621651584844557,0,-60.314245012986056,-177.8827146681441,-71.11153099634728}
    };
    const double cosine[2]={-.00678242792057368,1.1415244366365485};
    const double sine[2]={-3.287249049659447,-2.214092015647563};
    const float zero[5]={0};
    for (unsigned pose=0;pose<2;++pose) {
        for (int degrees=-40;degrees<=40;degrees+=5) {
            float q[5],nominal[5],g[5];
            for (unsigned i=0;i<5;++i) {
                const double soft=datum[pose][i]*.0174532925199433-g_joint_reference_config[i].zero_rad;
                q[i]=(float)(soft+g_five_mit_hw[i].cad_offset_deg*.0174532925199433);
            }
            const float angle=(float)(degrees*.0174532925199433);
            q[1]=angle+(float)(g_five_mit_hw[1].cad_offset_deg*.0174532925199433);
            const float j3=q[2]-(float)(g_five_mit_hw[2].cad_offset_deg*.0174532925199433);
            assert(FiveDynamics_Inverse(q,zero,zero,nominal));
            assert(FiveSupport_GravityModel(q,j3,g));
            assert(fabs(g[1]-cosine[pose]*cos(angle)-sine[pose]*sin(angle))<2e-5);
            assert(g[0]==nominal[0] && g[3]==nominal[3] && g[4]==nominal[4]);
            assert(g[2]==nominal[2]*1.2774127431831357f-.010825531721891f*cosf(j3));
            const float previous=nominal[2]*1.18f-.01f*cosf(j3);
            assert(fabsf(g[2]-1.0825531721890982f*previous)<6e-7f);
            /* No turn folding in the input; the gravity math stays periodic. */
            q[3]-=6.28318530718f;
            float turned[5];assert(FiveSupport_GravityModel(q,j3,turned));
            for (unsigned i=0;i<5;++i) { assert(fabsf(turned[i]-g[i])<2e-5f); }
        }
    }
    float q[5]={0},out[5]={1,2,3,4,5},saved[5],delta[5][4]={{0}};
    assert(FiveDynamics_GravityResidual(q,delta,out));
    for (unsigned i=0;i<5;++i) { assert(out[i]==0); }
    for (unsigned i=0;i<5;++i) { out[i]=(float)i+1; }
    memcpy(saved,out,sizeof saved);
    delta[3][1]=NAN;
    assert(!FiveDynamics_GravityResidual(q,delta,out) && !memcmp(out,saved,sizeof saved));
    assert(!FiveDynamics_GravityResidual(NULL,delta,out));
    assert(!FiveDynamics_GravityResidual(q,NULL,out));
    assert(!FiveDynamics_GravityResidual(q,delta,NULL));
    assert(!FiveSupport_GravityModel(q,NAN,out) && !memcmp(out,saved,sizeof saved));
    assert(!FiveSupport_GravityModel(NULL,0,out) && !memcmp(out,saved,sizeof saved));
    assert(!FiveSupport_GravityModel(q,0,NULL));
    q[4]=INFINITY;
    assert(!FiveSupport_GravityModel(q,0,out) && !memcmp(out,saved,sizeof saved));
}

static void test_five_mit(void)
{
    test_gravity_formula();
    DM_Motor_t *motors[5];five_setup(motors);
    const float q[5]={.1f,.2f,-.7f,1.4f,-.1f}, v[5]={.5f,-.3f,.2f,1,-.4f};
    const float a[5]={-1.4f,3,.7f,.2f,4}, gold[5]={-.04815288f,.53445572f,-.39507084f,.01959310f,-.03791413f};
    float tau[5]; assert(FiveDynamics_Inverse(q,v,a,tau));
    for (int i=0;i<5;++i) { assert(fabsf(tau[i]-gold[i])<2e-5f); }
    double at[5]={0,27.363787522718127,0,-60.76971657601524,17.853385365324336};
    live_workspace_pose(at);
    const double shift=at[2];
    double band[2]; assert(FiveWorkspace_J3Band(at,band));
    const double travel_low=-69.319289-17.291374640739114+1;
    const double travel_high=132.748142-17.291374640739114-1;
    assert(fabs(band[0]-travel_low)<1e-11 && fabs(band[1]-travel_high)<1e-11);
    at[2]=band[1]+0.01; assert(!FiveWorkspace_J3Band(at,band));
    double boxes[][2][5]={
        {{0,-10,0,-30,-6},{0,10,0,30,6}},{{0,-34,0,-165,-15},{0,-30,0,-155,-12}},
        {{0,38,0,-3,-8},{0,44,0,5,-5}}};
    const double rounded[][2]={{-34.05,16.46},{-52.99,66.98},{-84.57,29.09}};
    for (int k=0;k<3;++k) {
        live_workspace_pose(boxes[k][0]);live_workspace_pose(boxes[k][1]);
        assert(FiveWorkspace_BoxJ3Band(boxes[k][0],boxes[k][1],band));
        assert(band[0]<=rounded[k][0]+shift && band[1]>=rounded[k][1]+shift);
        assert(!FiveWorkspace_BoxJ3Band(boxes[k][1],boxes[k][0],band));
        double left[5],right[5];
        memcpy(left,boxes[k][0],sizeof left);memcpy(right,boxes[k][1],sizeof right);
        left[2]=rounded[k][0]+shift;right[2]=rounded[k][1]+shift;
        assert(FiveWorkspace_Line(left,right));
        FiveMIT_Box box; FiveMIT_Plan box_plan;
        assert(FiveMIT_CheckBox(&box,boxes[k][0],boxes[k][1]));
        assert(FiveMIT_PrepareInBox(&box_plan,&box,boxes[k][0],boxes[k][1],30));
    }
    double anchor[5]={0,-28.73228985940499,-86.72787226950314,204.52846412226867,-4.8693157650418115};
    double end[5]={0,-30.73228985940499,-93.1805102842604,198.52846412226867,-5.6193157650418115};
    live_workspace_pose(anchor);live_workspace_pose(end);
    assert(FiveWorkspace_Line(anchor,end)); /* Positive raw J4 branch is now in range. */
    double inside[5]={0,-32,0,-160,-13};live_workspace_pose(inside);
    double outside[5];memcpy(outside,inside,sizeof outside);outside[2]=travel_low-.01;
    assert(FiveWorkspace_Line(inside,inside));
    assert(!FiveWorkspace_Line(outside,outside) && !FiveWorkspace_Line(inside,outside));
    /* Per-axis admission does not certify this negative J4 point's geometry. */
    double other[5]={0,-28.549226288298314,-94.09541149521296,-208.30512363628617,-0.47016172764041997};
    live_workspace_pose(other);
    assert(FiveWorkspace_Line(other,other) && FiveWorkspace_Line(end,other));
    double safe_anchor[5]={0,-32,0,-160,-13};
    double safe_end[5]={0,-32,2,-160,-13};
    live_workspace_pose(safe_anchor);live_workspace_pose(safe_end);
    assert(FiveWorkspace_Line(safe_anchor,safe_end));
    FiveMIT_State x={0};memcpy(x.software_deg,safe_anchor,sizeof safe_anchor);
    for (unsigned i=0;i<5;++i) {
        x.raw_rad[i]=g_joint_reference_config[i].zero_rad+(float)(safe_anchor[i]*.0174532925199433);
    }
    FiveMIT_Plan p;
    assert(!FiveMIT_Prepare(&p,safe_anchor,safe_end,0.0001));
    assert(FiveMIT_Prepare(&p,safe_anchor,safe_end,2));
    FiveMIT_Output batch;
    assert(FiveMIT_Sample(&p,0,&x,g_five_mit_sim,&batch) && batch.motor[3].position_rad==x.raw_rad[3]);
    assert(FiveMIT_Sample(&p,.1,&x,g_five_mit_sim,&batch) &&
           batch.motor[3].position_rad==x.raw_rad[3] && batch.motor[3].position_rad<0);
    assert(can_tx_calls==0);
    unsigned sent=99;
    assert(FiveMIT_SendSample(&p,.1,&x,g_five_mit_sim,NULL,motors,&sent)==BSP_CAN_OK && sent==5);
    for (int i=0;i<5;++i) {
        assert(batch.frame[i].id==(uint32_t)i+1 && !memcmp(motor_last_mit[i+1],batch.frame[i].data,8));
        assert(!motor_enable_calls[i+1] && !motor_disable_calls[i+1]);
    }
    assert(can_io[0].tx_calls==3 && can_io[1].tx_calls==2);
    can_io[0].free_slots=0;
    assert(FiveMIT_SendSample(&p,.1,&x,g_five_mit_sim,NULL,motors,&sent)==BSP_CAN_BUSY && sent==2);
    const unsigned prev=can_tx_calls;
    motors[4]=motors[3];
    assert(FiveMIT_SendSample(&p,.1,&x,g_five_mit_sim,NULL,motors,&sent)==BSP_CAN_INVALID_ARG && sent==0 && can_tx_calls==prev);
    FiveMIT_Axis profile[5];memcpy(profile,g_five_mit_sim,sizeof profile);
    profile[0].coulomb_nm=.10f;profile[0].viscous_nm_s_rad=.05f;
    profile[3].coulomb_nm=.12f;profile[3].viscous_nm_s_rad=.03f;
    for (int sign=-1;sign<=1;++sign) {
        x.velocity_rad_s[0]=(float)sign*.2f;
        x.velocity_rad_s[3]=(float)sign*.2f;
        FiveMIT_Output baseline, adjusted;
        assert(FiveMIT_Sample(&p,0,&x,g_five_mit_sim,&baseline) && FiveMIT_Sample(&p,0,&x,profile,&adjusted));
        assert(fabsf(adjusted.motor[0].torque_ff_nm-baseline.motor[0].torque_ff_nm-sign*.11f)<1e-5f);
        assert(adjusted.motor[1].torque_ff_nm==baseline.motor[1].torque_ff_nm);
        assert(fabsf(adjusted.motor[3].torque_ff_nm-baseline.motor[3].torque_ff_nm-sign*.126f)<1e-5f);
    }
    FiveMIT_Output saved=batch;
    memset(x.velocity_rad_s,0,sizeof x.velocity_rad_s);
    float cq[5],support_g[5],nominal_g[5];const float cv[5]={0};
    for (unsigned i=0;i<5;++i) {
        cq[i]=(float)((x.software_deg[i]+g_five_mit_hw[i].cad_offset_deg)*.0174532925199433);
    }
    assert(FiveDynamics_Inverse(cq,cv,cv,nominal_g));
    assert(FiveSupport_GravityModel(cq,(float)(x.software_deg[2]*.0174532925199433),support_g));
    FiveMIT_Output controlled,open_loop;
    assert(FiveMIT_Sample(&p,.1,&x,g_five_mit_hw,&open_loop));
    assert(FiveMIT_SampleFeedback(&p,.1,&x,support_g,&controlled));
    for (unsigned i=0;i<5;++i) {
        const DM_Motor_MIT_Command_t *cmd=&controlled.motor[i];
        float expected=open_loop.motor[i].torque_ff_nm+support_g[i]-nominal_g[i]+
            g_five_mit_tracking[i].kp*(cmd->position_rad-x.raw_rad[i])+
            g_five_mit_tracking[i].kd*cmd->velocity_rad_s;
        assert(cmd->kp==0 && cmd->kd==0 && fabsf(cmd->torque_ff_nm-expected)<1e-6f);
        assert(g_five_mit_tracking[i].max_nm>=g_five_mit_hw[i].model_max_nm &&
               g_five_mit_tracking[i].max_nm<g_motor_can_config[i].tmax);
    }
    FiveMIT_Output keep=controlled;
    support_g[0]=NAN;
    assert(!FiveMIT_SampleFeedback(&p,.1,&x,support_g,&controlled) &&
           !memcmp(&keep,&controlled,sizeof keep));
    support_g[0]=nominal_g[0];x.software_deg[0]+=60;
    assert(!FiveMIT_SampleFeedback(&p,.1,&x,support_g,&controlled));
    x.software_deg[0]-=60;
    x.raw_rad[4]=NAN;
    assert(!FiveMIT_Sample(&p,.1,&x,profile,&batch) && !memcmp(&saved,&batch,sizeof batch));
    at[0]=.0001; at[2]=0; assert(FiveMIT_Prepare(&p,at,at,1));
    assert(!FiveMIT_Sample(&p,0,&x,profile,&batch));
    double ya[5]={-200,-5,-5,-5,-5}, yb[5]={200,5,5,5,5};
    live_workspace_pose(ya);live_workspace_pose(yb);
    FiveMIT_Box yaw_box;
    assert(FiveMIT_CheckBox(&yaw_box,ya,yb) && FiveMIT_Prepare(&p,ya,yb,30));
    assert(FiveMIT_PrepareInBox(&p,&yaw_box,ya,yb,30));
    assert(FiveWorkspace_Line(yb,ya) && !FiveMIT_Prepare(&p,ya,yb,.001));
    /* Range/packet unit test only; this does not certify collision geometry. */
    FiveMIT_Plan yaw_sample={.seconds=30};
    memcpy(yaw_sample.start_deg,ya,sizeof ya);memcpy(yaw_sample.end_deg,yb,sizeof yb);
    FiveMIT_State yaw_state={0};
    memcpy(yaw_state.software_deg,ya,sizeof ya);
    for (int i=0;i<5;++i) {
        yaw_state.raw_rad[i]=g_joint_reference_config[i].zero_rad+(float)(ya[i]*.0174532925199433);
    }
    assert(FiveMIT_Sample(&yaw_sample,0,&yaw_state,g_five_mit_sim,&batch));
    assert(batch.motor[0].position_rad==yaw_state.raw_rad[0] && batch.motor[0].position_rad<-3.14f);
    for (int i=0;i<5;++i) {
        yaw_state.software_deg[i]=(ya[i]+yb[i])*.5;
        yaw_state.raw_rad[i]=workspace_center_raw((unsigned)i);
        yaw_state.velocity_rad_s[i]=(float)((yb[i]-ya[i])*.0174532925199433*1.875/30);
    }
    assert(FiveMIT_Sample(&yaw_sample,15,&yaw_state,g_five_mit_sim,&batch));
    assert(batch.motor[0].position_rad==yaw_state.raw_rad[0] &&
           fabsf(batch.motor[0].velocity_rad_s-.4363323f)<1e-6f);
    double qmid[5]={0};live_workspace_pose(qmid);
    assert(FiveWorkspace_Line(ya,qmid) && FiveWorkspace_Line(qmid,yb));
    double yp[5]={34.49,0,0,0,0};
    live_workspace_pose(yp);
    assert(FiveWorkspace_J3Band(yp,band));
    const double old_yaw_center[5]={34.49,0,0,0,0};
    assert(FiveWorkspace_J3Band(old_yaw_center,band)); /* Per-axis admission, not frozen geometry. */
    yb[0]=201;assert(!FiveWorkspace_Line(ya,yb));yb[0]=200;
    yb[3]=294.2975735;assert(!FiveWorkspace_Line(ya,yb));yb[3]=5;
    yb[4]=NAN;assert(!FiveWorkspace_Line(ya,yb));yb[4]=5;
    assert(!FiveMIT_CheckBox(&yaw_box,yb,ya));

    /* All five axes can leave the old small box, including nonzero J1.
     * J4 spans raw -179..349 (528 degrees after the retained 1-degree inset).
     * These are range checks ONLY, not a physical collision-free trajectory. */
    double lo[5]={-200,-34,-85,-234.7024265,-21};
    double hi[5]={200,45,114,293.2975735,21};
    assert(FiveWorkspace_Line(lo,hi) && FiveWorkspace_Line(hi,lo));
    assert(FiveMIT_CheckBox(&yaw_box,lo,hi));
    assert(FiveMIT_Prepare(&p,lo,hi,60));
    assert(fabs(p.end_deg[3]-p.start_deg[3]-528)<1e-10);
    assert(FiveMIT_PrepareInBox(&p,&yaw_box,hi,lo,60));
    assert(p.start_deg[3]==hi[3] && p.end_deg[3]==lo[3]);
    assert(!FiveMIT_Prepare(&p,lo,hi,.001) && p.seconds==0);
    assert(!FiveMIT_Prepare(&p,lo,hi,NAN));
    assert(!FiveMIT_CheckBox(&yaw_box,hi,lo) && !yaw_box.valid);

#define JOINT_REFERENCE(joint, zero, lower, upper, kind) {(lower)-(zero)+1.,(upper)-(zero)-1.},
    const double bounds[5][2]={
#include "joint_references.def"
    };
#undef JOINT_REFERENCE
    for (int i=0;i<5;++i) {
        double edge[5];memcpy(edge,lo,sizeof edge);
        edge[i]=bounds[i][0];assert(FiveWorkspace_Line(edge,hi));
        edge[i]-=.0001;assert(FiveWorkspace_Line(edge,hi)); /* Inward departure only. */
        assert(!FiveWorkspace_Line(hi,edge));
        edge[i]=bounds[i][0]-1.-FIVE_ENCODER_TOL_DEG-.0001;
        assert(!FiveWorkspace_Line(edge,hi));
        edge[i]=bounds[i][1];assert(FiveWorkspace_Line(lo,edge));
        edge[i]+=.0001;assert(!FiveWorkspace_Line(lo,edge));
        edge[i]=NAN;assert(!FiveWorkspace_Line(edge,hi));
        edge[i]=INFINITY;assert(!FiveWorkspace_Line(lo,edge));
        edge[i]=-INFINITY;assert(!FiveWorkspace_Line(edge,hi));
    }
    assert(!FiveWorkspace_Line(NULL,hi) && !FiveWorkspace_Line(lo,NULL));
    assert(!FiveWorkspace_J3Band(lo,NULL));
    double bad_box[5];memcpy(bad_box,hi,sizeof bad_box);bad_box[2]=lo[2]+1;
    band[0]=123;band[1]=456;
    assert(!FiveWorkspace_BoxJ3Band(lo,bad_box,band) && band[0]==123 && band[1]==456);
    const double endpoint_start[5]={60.106597,13.407484,-4.428592,-233.585141,-22.654712949};
    const double inward_end[5]={60.106597,13.407484,-4.428592,-233.585141,-10};
    assert(FiveMIT_Prepare(&p,endpoint_start,inward_end,8));
    /* True velocity plateau, analytic position integral, both signs/all five.
     * Numerical model checks only; not a physical speed/clearance acceptance. */
    double cruise_from[5],cruise_to[5];
    FiveMIT_State cruise_state={0};
    FiveMIT_Axis cruise_profile[5];memcpy(cruise_profile,g_five_mit_hw,sizeof cruise_profile);
    for (unsigned i=0;i<5;++i) {
        cruise_from[i]=(bounds[i][0]+bounds[i][1])*.5;
        cruise_state.software_deg[i]=cruise_from[i];
        cruise_state.raw_rad[i]=g_joint_reference_config[i].zero_rad+
            (float)(cruise_from[i]*.01745329251994329577);
        cruise_profile[i].model_max_nm=g_motor_can_config[i].tmax;
    }
    const double sample_times[]={0,.5,1,2,3,4,4.5,5};
    const double fractions[]={0,.01953125,.125,.375,.625,.875,.98046875,1};
    const double velocities[]={0,.04,.08,.08,.08,.08,.04,0};
    for (unsigned axis=0;axis<5;++axis) {
        for (int sign=-1;sign<=1;sign+=2) {
            memcpy(cruise_to,cruise_from,sizeof cruise_to);
            cruise_to[axis]+=sign*.08*4/.01745329251994329577;
            assert(FiveMIT_PrepareCruise(&p,cruise_from,cruise_to,5,1));
            for (unsigned k=0;k<sizeof sample_times/sizeof sample_times[0];++k) {
                assert(FiveMIT_Sample(&p,sample_times[k],&cruise_state,cruise_profile,&batch));
                for (unsigned i=0;i<5;++i) {
                    assert(fabs(batch.motor[i].velocity_rad_s-
                        (i==axis ? sign*velocities[k] : 0))<1e-7);
                    const double displacement=(i==axis ? sign*.08*4*fractions[k] : 0);
                    assert(fabs(batch.motor[i].position_rad-cruise_state.raw_rad[i]-displacement)<1e-6);
                }
            }
        }
    }
    assert(!FiveMIT_PrepareCruise(&p,cruise_from,cruise_to,5,0) && !p.seconds);
    assert(!FiveMIT_PrepareCruise(&p,cruise_from,cruise_to,5,2.5) && !p.seconds);
    assert(!FiveMIT_PrepareCruise(&p,cruise_from,cruise_to,5,NAN));
    assert(!FiveMIT_PrepareCruise(&p,cruise_from,cruise_to,NAN,1));
    assert(!FiveMIT_PrepareCruise(&p,NULL,cruise_to,5,1));
    assert(!FiveMIT_PrepareCruise(NULL,cruise_from,cruise_to,5,1));
    assert(!FiveMIT_PrepareCruise(&p,lo,hi,.1,.01)); /* Original protocol speed check. */
    cruise_to[0]=NAN;
    assert(!FiveMIT_PrepareCruise(&p,cruise_from,cruise_to,5,1));
    cruise_to[0]=cruise_from[0];
    assert(FiveMIT_Prepare(&p,cruise_from,cruise_to,5) && p.cruise_ramp_s==0);
    /* A speed tune must not retune the other four axes or ordinary moves.
     * Verify both directions, including continuation of the same endpoint
     * hold gains; formal G/effort rejection and raw feedback stay unchanged. */
    const FiveMIT_SpeedPI expected_speed[5]={
        {1.5f,20},{2.25f,15},{.75f,5},{.6f,3},{.5f,10}
    };
    const FiveMIT_SpeedFriction expected_friction[5]={
        {-.3529545390540462f,-.1094118938571156f,.256320566976014f,.016621367861598263f},
        {-.6264370238064128f,-.16367067413011038f,.3254983958136558f,.07914225028485657f},
        {-2.067608595539022f,0,0,0},
        {-2.0190656854614013f,-.08758386997159276f,0,0},
        {-2.006567438632663f,0,0,0}
    };
    /* These are amplitude knots, never motion-speed admission gates. */
    assert(g_five_speed_friction_range_rad_s[0]==.08f);
    assert(g_five_speed_friction_range_rad_s[1]==.8f);
    assert(g_five_speed_friction_knots_rad_s[0]==.16f);
    assert(g_five_speed_friction_knots_rad_s[1]==.48f);
    float untouched_friction=123;
    assert(!FiveMIT_SpeedFrictionTorque(5,.08f,0,0,&untouched_friction));
    assert(!FiveMIT_SpeedFrictionTorque(0,NAN,0,0,&untouched_friction));
    assert(!FiveMIT_SpeedFrictionTorque(0,.08f,NAN,0,&untouched_friction));
    assert(!FiveMIT_SpeedFrictionTorque(0,.08f,0,-1,&untouched_friction));
    assert(!FiveMIT_SpeedFrictionTorque(1,.08f,1e30f,0,&untouched_friction));
    assert(!FiveMIT_SpeedFrictionTorque(0,.08f,0,0,NULL));
    assert(untouched_friction==123);
    for (unsigned axis=0;axis<5;++axis) {
        assert(g_five_speed_pi[axis].kp==expected_speed[axis].kp);
        assert(g_five_speed_pi[axis].ki==expected_speed[axis].ki);
        assert(g_five_speed_friction[axis].log_amplitude_nm==expected_friction[axis].log_amplitude_nm);
        assert(g_five_speed_friction[axis].speed_exponent==expected_friction[axis].speed_exponent);
        assert(g_five_speed_friction[axis].hinge_016==expected_friction[axis].hinge_016);
        assert(g_five_speed_friction[axis].hinge_048==expected_friction[axis].hinge_048);
        assert(g_five_mit_hw[axis].coulomb_nm==0 && g_five_mit_hw[axis].viscous_nm_s_rad==0);
        const float velocities[]={0,.001f,.079999f,.08f,.080001f,.12f,.159999f,
            .16f,.160001f,.32f,.479999f,.48f,.480001f,.599999f,.6f,.600001f,
            .7f,.799999f,.8f,.800001f,1.2f};
        float last_knot=0;
        for (unsigned n=0;n<sizeof velocities/sizeof velocities[0];++n) {
            float positive,negative;
            assert(FiveMIT_SpeedFrictionTorque(axis,velocities[n],1,.2f,&positive));
            assert(FiveMIT_SpeedFrictionTorque(axis,-velocities[n],1,.2f,&negative));
            assert(isfinite(positive) && positive>=0 && fabsf(positive+negative)<1e-7f);
            if (!n) { assert(positive==0); }
            const float speed=fminf(.8f,fmaxf(.08f,velocities[n]));
            const FiveMIT_SpeedFriction *model=&expected_friction[axis];
            double log_amplitude=model->log_amplitude_nm+
                model->speed_exponent*log(speed/.08f)+
                model->hinge_016*fmax(0,log(speed/.16f))+
                model->hinge_048*fmax(0,log(speed/.48f));
            if (axis==1) {
                log_amplitude+=-.12727473763230213*.2f+.18466618285566616;
            }
            const float wanted=(float)(exp(log_amplitude)*tanh(velocities[n]/.02f));
            assert(fabsf(positive-wanted)<3e-7f);
            if (velocities[n]==.8f) { last_knot=positive; }
            if (velocities[n]>.8f) {
                assert(fabsf(positive-last_knot)<3e-7f);
            }
        }
        for (int sign=-1;sign<=1;sign+=2) {
            memcpy(cruise_to,cruise_from,sizeof cruise_to);
            cruise_to[axis]+=sign*.08*4/.01745329251994329577;
            assert(FiveMIT_PrepareCruise(&p,cruise_from,cruise_to,5,1));
            const double check_times[]={0,.2,.5,2,5};
            for (unsigned k=0;k<5;++k) {
                FiveMIT_State measured=cruise_state;
                FiveMIT_Output target,open,closed;
                assert(FiveMIT_Sample(&p,check_times[k],&measured,cruise_profile,&target));
                float local_q[5],local_g[5],local_nominal[5],local_load_g[5],local_bending[5];
                const float stopped[5]={0};
                for (unsigned i=0;i<5;++i) {
                    measured.raw_rad[i]=target.motor[i].position_rad-.002f;
                    measured.software_deg[i]=cruise_from[i]+
                        (measured.raw_rad[i]-cruise_state.raw_rad[i])/.01745329251994329577;
                    measured.velocity_rad_s[i]=-sign*.01f; /* Opposite reference sign. */
                    local_q[i]=(float)((measured.software_deg[i]+g_five_mit_hw[i].cad_offset_deg)*
                                      .01745329251994329577);
                }
                assert(FiveDynamics_Inverse(local_q,stopped,stopped,local_nominal));
                assert(FiveDynamics_GravityLoad(local_q,local_load_g,local_bending));
                for (unsigned j=0;j<5;++j) {
                    assert(fabsf(local_load_g[j]-local_nominal[j])<1e-6f);
                    assert(isfinite(local_bending[j]) && local_bending[j]>=0);
                }
                assert(FiveSupport_GravityModel(local_q,(float)(measured.software_deg[2]*
                    .01745329251994329577),local_g));
                assert(FiveMIT_Sample(&p,check_times[k],&measured,g_five_mit_hw,&open));
                assert(FiveMIT_SampleFeedback(&p,check_times[k],&measured,local_g,&closed));
                for (unsigned i=0;i<5;++i) {
                    const float position_gain=i==axis ? g_five_speed_pi[i].ki : g_five_mit_tracking[i].kp;
                    const float velocity_gain=i==axis ? g_five_speed_pi[i].kp : g_five_mit_tracking[i].kd;
                    const float feedback=position_gain*(open.motor[i].position_rad-measured.raw_rad[i])+
                        velocity_gain*(open.motor[i].velocity_rad_s-measured.velocity_rad_s[i]);
                    float friction=0;
                    if (i==axis) {
                        assert(FiveMIT_SpeedFrictionTorque(i,open.motor[i].velocity_rad_s,
                            local_nominal[i],local_bending[i],&friction));
                    }
                    const float expected=open.motor[i].torque_ff_nm+local_g[i]-local_nominal[i]+
                        (k ? feedback : 0)+friction; /* Original zero-PD entry. */
                    if (k==0 || k==4 || i!=axis) { assert(friction==0); }
                    else { assert(sign*friction>0); }
                    assert(fabsf(closed.motor[i].torque_ff_nm-expected)<2e-6f);
                    assert(closed.motor[i].kp==0 && closed.motor[i].kd==0);
                    assert(fabsf(closed.motor[i].torque_ff_nm)<=g_five_mit_tracking[i].max_nm);
                }
                if (k==3) {
                    /* The overlay is part of the original total budget, not
                     * added after validation or silently clipped. */
                    FiveMIT_Output preserved=closed;
                    float stressed_g[5];memcpy(stressed_g,local_g,sizeof stressed_g);
                    stressed_g[axis]+=sign*(g_five_mit_tracking[axis].max_nm+.01f)-
                        closed.motor[axis].torque_ff_nm;
                    assert(!FiveMIT_SampleFeedback(&p,check_times[k],&measured,stressed_g,&closed));
                    assert(!memcmp(&preserved,&closed,sizeof closed));
                }
            }
        }
    }
    puts("PASS: five-axis full travel without small-angle caps, invalid input, same-turn MIT and partial transport");
}

/* Local simulator pipe, deliberately not a serial/on-board command protocol. */
static void plant_sample_at_speed(unsigned i,unsigned state,float raw_rad,uint16_t vcode)
{
    const DM_Motor_Config_t *m=&g_motor_can_config[i];
    uint16_t pos=(uint16_t)lroundf((raw_rad/m->pmax+1.f)*32767.5f);
    uint8_t data[8]={(uint8_t)(state<<4 | m->can_id),(uint8_t)(pos>>8),(uint8_t)pos,
        (uint8_t)(vcode>>4),(uint8_t)((vcode<<4)|8),0,25,25};
    inject_motor_feedback(m->bus,m->master_id,data,sizeof data);
}
static void plant_sample_at(unsigned i,unsigned state,float raw_rad)
{
    plant_sample_at_speed(i,state,raw_rad,0x800);
}
static void plant_sample(unsigned i,unsigned state)
{
    plant_sample_at(i,state,workspace_center_raw(i));
}
static void assert_gravity_frames(const FiveControl_Status *s,unsigned mask)
{
    for (unsigned i=0;i<5;++i) {
        if (!(mask & (1U<<i))) { continue; }
        const DM_Motor_MIT_Command_t cmd={.torque_ff_nm=s->gravity_nm[i]};
        BSP_CAN_Frame_t expected;
        assert(DM_Motor_EncodeMIT(&g_motor_can_config[i],&cmd,&expected)==BSP_CAN_OK);
        assert(!memcmp(motor_last_mit[i+1],expected.data,8));
    }
}
static void test_smooth_control(void)
{
    assert(FIVE_PERIOD_MS==2U);
    assert(g_five_mit_tracking[2].kp==5 && g_five_mit_tracking[2].kd==.5f);
    assert(g_five_mit_tracking[3].kp==3 && g_five_mit_tracking[3].kd==.3f);
    const float doubled_active_caps[5]={4,10,3.6f,1.6f,.8f};
    const float unchanged_support_caps[5]={.4f,3,1.8f,.8f,.4f};
    for (unsigned i=0;i<5;++i) {
        assert(g_five_mit_tracking[i].max_nm==doubled_active_caps[i]);
        assert(g_five_mit_hw[i].model_max_nm==unchanged_support_caps[i]);
    }
    assert(g_five_tracking_ramp_s==.1f);
    DM_Motor_t *motors[5];five_setup(motors);
    assert(FiveControl_Init(motors,test_tick));
    const FiveControl_Status *s=FiveControl_StatusNow();
    const uint32_t start=test_tick;
    const unsigned initial_tx=can_tx_calls;
    for (unsigned ms=1;ms<=1000;++ms) {
        test_tick=start+ms;
        for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
        const uint32_t before=s->ticks;
        const unsigned sent=can_tx_calls;
        FiveControl_Tick(test_tick);
        assert(s->ticks==before+(ms%2==0));
        assert(can_tx_calls==sent+(ms%2==0 ? 5U : 0U));
    }
    assert(s->ticks==500 && can_tx_calls-initial_tx==2500);
    assert(s->phase==FIVE_GRAVITY && !s->fault && s->submitted==31);
    assert_gravity_frames(s,31);

    FiveMIT_State x={0};
    double from[5],to[5];
    float q[5],g[5],nominal[5];const float zero[5]={0};
    for (unsigned i=0;i<5;++i) {
        from[i]=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/.01745329251994329577;
        to[i]=from[i];
        x.software_deg[i]=from[i];x.raw_rad[i]=s->feedback[i].position_rad;
        q[i]=(float)((from[i]+g_five_mit_hw[i].cad_offset_deg)*.01745329251994329577);
    }
    to[1]+=.2;
    FiveMIT_Plan plan;assert(FiveMIT_Prepare(&plan,from,to,1));
    assert(FiveDynamics_Inverse(q,zero,zero,nominal));
    assert(FiveSupport_GravityModel(q,(float)(from[2]*.01745329251994329577),g));
    FiveMIT_Output active,bare;
    assert(FiveMIT_SampleFeedback(&plan,0,&x,g,&active));
    for (unsigned i=0;i<5;++i) {
        assert(fabsf(active.motor[i].torque_ff_nm-g[i])<2e-6f);
        x.velocity_rad_s[i]=.01f*(float)(i+1);
    }
    const double times[]={0,.025,.05,.075,.1,.2};
    const float blends[]={0,.103515625f,.5f,.896484375f,1,1};
    for (unsigned k=0;k<sizeof times/sizeof times[0];++k) {
        assert(FiveMIT_Sample(&plan,times[k],&x,g_five_mit_hw,&bare));
        assert(FiveMIT_SampleFeedback(&plan,times[k],&x,g,&active));
        for (unsigned i=0;i<5;++i) {
            const DM_Motor_MIT_Command_t *cmd=&active.motor[i];
            const float pd=g_five_mit_tracking[i].kp*(cmd->position_rad-x.raw_rad[i])+
                g_five_mit_tracking[i].kd*(cmd->velocity_rad_s-x.velocity_rad_s[i]);
            const float expected=bare.motor[i].torque_ff_nm+g[i]-nominal[i]+blends[k]*pd;
            assert(fabsf(cmd->torque_ff_nm-expected)<2e-6f && cmd->kp==0 && cmd->kd==0);
            assert(cmd->position_rad==bare.motor[i].position_rad &&
                   cmd->velocity_rad_s==bare.motor[i].velocity_rad_s);
        }
    }
    const FiveMIT_Output saved=active;
    x.software_deg[0]+=60;
    assert(!FiveMIT_SampleFeedback(&plan,.2,&x,g,&active) &&
           !memcmp(&saved,&active,sizeof saved)); /* Original effort budget, no clipping. */
    x.software_deg[0]-=60;x.velocity_rad_s[0]=NAN;
    assert(!FiveMIT_SampleFeedback(&plan,0,&x,g,&active) &&
           !memcmp(&saved,&active,sizeof saved)); /* Blend must not hide invalid feedback. */

    assert(FiveControl_Move(to,1000)==BSP_CAN_OK);
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_MOTION && !s->fault && s->submitted==31);
    FiveControl_Gravity();FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_GRAVITY);assert_gravity_frames(s,31);
    assert(FiveControl_Speed(0,.08f,5000,1000)==BSP_CAN_INVALID_ARG);
    assert(FiveControl_Speed(6,.08f,5000,1000)==BSP_CAN_INVALID_ARG);
    assert(FiveControl_Speed(1,NAN,5000,1000)==BSP_CAN_INVALID_ARG);
    assert(FiveControl_Speed(1,0,5000,1000)==BSP_CAN_INVALID_ARG);
    assert(FiveControl_Speed(1,.08f,5000,0)==BSP_CAN_INVALID_ARG);
    assert(FiveControl_Speed(1,.08f,5000,2500)==BSP_CAN_INVALID_ARG);
    assert(FiveControl_Speed(1,.08f,5000,UINT32_MAX)==BSP_CAN_INVALID_ARG);
    for (unsigned axis=0;axis<5;++axis) {
        for (int sign=-1;sign<=1;sign+=2) {
            /* Admission remains valid even if another axis is not stationary. */
            for (unsigned i=0;i<5;++i) {
                const uint16_t code=(uint16_t)lroundf((.06f/g_motor_can_config[i].vmax+1)*2047.5f);
                plant_sample_at_speed(i,1,workspace_center_raw(i),code);
            }
            FiveControl_Tick(test_tick+=2);
            assert(FiveControl_Speed(axis+1,sign*.08f,5000,1000)==BSP_CAN_OK);
            FiveControl_Tick(test_tick+=2);
            assert(s->phase==FIVE_MOTION && !s->fault && s->submitted==31);
            assert(FiveControl_Speed(axis+1,.08f,5000,1000)==BSP_CAN_NOT_READY);
            FiveControl_Gravity();FiveControl_Tick(test_tick+=2);
            assert(s->phase==FIVE_GRAVITY && !s->fault);
            assert_gravity_frames(s,31);
        }
    }
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    assert(FiveControl_Move(to,1000)==BSP_CAN_OK);
    can_io[1].free_slots=0;FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DEGRADED && (s->fault & FIVE_TX) && s->submitted==0x1c);
    assert_gravity_frames(s,0x1c); /* Fault G handover is immediate, not ramped. */
    for (unsigned i=1;i<=5;++i) { assert(!motor_enable_calls[i] && !motor_disable_calls[i]); }
    puts("PASS: 500 Hz cadence, quintic PD entry, unchanged G/budgets/raw feedback and immediate fault handover");
}
static void test_j3_read(const char *variant)
{
    DM_Motor_t *motors[5]; five_setup(motors);
    assert(FiveControl_Init(motors,test_tick));
    assert(FiveControl_ReadLegacyJ3()==BSP_CAN_NOT_READY);
    /* The real post-reflash condition: other drives remained enabled,
     * J3 has never replied, no position/G has been invented. */
    for (unsigned k=0;k<2;++k) {
        for (unsigned i=0;i<5;++i) { if (i!=2) { plant_sample(i,1); } }
        FiveControl_Tick(test_tick+=5);
    }
    const FiveControl_Status *s=FiveControl_StatusNow();
    assert(s->fresh==27 && s->enabled==27 && s->fault==FIVE_BAD_FEEDBACK &&
           !s->gravity_valid && !s->feedback[2].received_count);
    for (unsigned i=1;i<=5;++i) { assert(!motor_enable_calls[i]); }
    assert(FiveControl_ColdStart()==BSP_CAN_NOT_READY); /* Original gates unchanged. */
    test_ipsr=1; assert(FiveControl_ReadLegacyJ3()==BSP_CAN_NOT_READY); test_ipsr=0;
    assert(FiveControl_ReadLegacyJ3()==BSP_CAN_OK);
    assert(FiveControl_ReadLegacyJ3()==BSP_CAN_NOT_READY);
    const DM_Motor_MIT_Command_t neutral={0};
    BSP_CAN_Frame_t expected;
    assert(DM_Motor_EncodeMIT(&g_motor_can_config[2],&neutral,&expected)==BSP_CAN_OK);
    assert(motor_mit_calls[3]==1 && !memcmp(motor_last_mit[3],expected.data,8));
    if (!strcmp(variant,"cancel")) { FiveControl_Gravity(); }
    if (!strcmp(variant,"tx")) { can_tx_result=HAL_ERROR; }
    for (unsigned elapsed=5;elapsed<=FIVE_ENABLE_TIMEOUT_MS+25;elapsed+=5) {
        for (unsigned i=0;i<5;++i) { if (i!=2) { plant_sample(i,1); } }
        if (!strcmp(variant,"ok") && elapsed>=30) { plant_sample(2,1); }
        FiveControl_Tick(test_tick+=5);
        if (!strcmp(variant,"ok") && elapsed>=30) {
            assert(s->fresh==31 && s->enabled==31 && s->submitted==31 &&
                   s->gravity_valid && s->fault==FIVE_BAD_FEEDBACK);
            assert_gravity_frames(s,31);
        } else if (!strcmp(variant,"timeout") && elapsed>=20) {
            assert(!memcmp(motor_last_mit[3],expected.data,8));
        }
    }
    assert(!motor_query_calls[3] && !motor_disable_calls[3]);
    assert(motor_enable_calls[3]==(!strcmp(variant,"ok") || !strcmp(variant,"timeout") ? 1U : 0U));
    for (unsigned i=1;i<=5;++i) {
        if (i!=3) { assert(!motor_enable_calls[i] && !motor_disable_calls[i]); }
    }
    assert(FiveControl_ReadLegacyJ3()==BSP_CAN_NOT_READY); /* No auto/manual replay this boot. */
    if (!strcmp(variant,"ok")) {
        assert(FiveControl_Acknowledge()==BSP_CAN_OK); /* Separate explicit action only. */
        FiveControl_Tick(test_tick+=5);
        assert(s->phase==FIVE_DRAG && !s->fault && s->submitted==31);
    } else {
        assert(!s->gravity_valid && !s->feedback[2].received_count && !s->submitted);
        if (!strcmp(variant,"tx")) { assert(s->fault & FIVE_TX); }
    }
    printf("PASS: explicit legacy J3 read %s, zero preload, one FC, no 0xCC/other-axis enable/reset/clear/retry\n",variant);
}
/* Same order as the app: control tick, then one debounced key event. */
static BSP_CAN_Status_t key_level(bool pressed,unsigned state,const float raw[5])
{
    test_gpioa.IDR=pressed ? 0U : KEY__INPUT_Pin;
    BSP_CAN_Status_t result=BSP_CAN_NOT_READY;
    unsigned events=0;
    for (unsigned k=0;k<5;++k) {
        for (unsigned i=0;i<5;++i) { plant_sample_at(i,state,raw[i]); }
        FiveControl_Tick(test_tick+=5);
        if (BSP_KEY_Scan(test_tick)==BSP_KEY_EVENT_PRESSED) {
            ++events; result=FiveControl_KeyPress();
        }
    }
    assert(events<=1);
    return result;
}
static void test_key_modes(void)
{
    DM_Motor_t *m[5];five_setup(m);
    assert(BSP_KEY_Init(20)==HAL_OK && FiveControl_Init(m,test_tick));
    float raw[5];
    for (unsigned i=0;i<5;++i) { raw[i]=g_joint_reference_config[i].zero_rad; }
    const FiveControl_Status *s=FiveControl_StatusNow();
    (void)key_level(false,0,raw);
    assert(s->phase==FIVE_WAIT && !s->submitted);
    for (unsigned i=1;i<=5;++i) { assert(!motor_enable_calls[i]); }
    assert(key_level(true,0,raw)==BSP_CAN_OK); /* First press: cold start, not impedance. */
    assert(FiveControl_KeyPress()==BSP_CAN_NOT_READY); /* No duplicate during handshake. */
    for (unsigned k=0;k<2;++k) {
        for (unsigned i=0;i<5;++i) { plant_sample_at(i,0,raw[i]); }
        FiveControl_Tick(test_tick+=5);
    }
    (void)key_level(true,1,raw); /* Holding the key never toggles again. */
    assert(s->phase==FIVE_DRAG && s->enabled==31 && s->submitted==31 && !s->fault);
    assert_gravity_frames(s,31);
    for (unsigned i=1;i<=5;++i) { assert(motor_enable_calls[i]==1 && !motor_disable_calls[i]); }
    (void)key_level(false,1,raw);
    assert(key_level(true,1,raw)==BSP_CAN_OK);
    double anchor[5];
    for (unsigned i=0;i<5;++i) {
        anchor[i]=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/.01745329251994329577;
        raw[i]+=.04f;
        plant_sample_at_speed(i,1,raw[i],0x840);
    }
    FiveControl_Tick(test_tick+=5);
    assert(s->phase==FIVE_IMPEDANCE && !s->fault && s->submitted==31);
    for (unsigned i=0;i<5;++i) {
        const double angle=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/.01745329251994329577;
        const FiveSupport_Gains *gain=&g_five_support_gains[i];
        const double pd=gain->kp*(anchor[i]-angle)*.01745329251994329577-
            gain->kd*s->feedback[i].velocity_rad_s;
        const float room=g_five_mit_hw[i].model_max_nm-fabsf(s->gravity_nm[i]);
        const DM_Motor_MIT_Command_t cmd={
            .torque_ff_nm=s->gravity_nm[i]+fmaxf(-room,fminf(room,(float)pd))};
        BSP_CAN_Frame_t expected;
        assert(DM_Motor_EncodeMIT(&g_motor_can_config[i],&cmd,&expected)==BSP_CAN_OK);
        assert(!memcmp(motor_last_mit[i+1],expected.data,8));
    }
    (void)key_level(false,1,raw);
    assert(key_level(true,1,raw)==BSP_CAN_OK);
    (void)key_level(true,1,raw);
    assert(s->phase==FIVE_DRAG && !s->fault); assert_gravity_frames(s,31);
    (void)key_level(false,1,raw);
    assert(key_level(true,1,raw)==BSP_CAN_OK); /* Re-enter at the new pose, not the old center. */
    FiveControl_Tick(test_tick+=5);
    assert(s->phase==FIVE_IMPEDANCE && !s->fault);
    FiveMIT_State at={0}; FiveSupport_State mode={0};
    for (unsigned i=0;i<5;++i) {
        at.software_deg[i]=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/.01745329251994329577;
        at.velocity_rad_s[i]=s->feedback[i].velocity_rad_s;
    }
    float torque[5];assert(FiveSupport_EnterImpedance(&mode,&at));
    assert(FiveSupport_Sample(&mode,&at,s->gravity_nm,torque));
    for (unsigned i=0;i<5;++i) {
        const DM_Motor_MIT_Command_t cmd={.torque_ff_nm=torque[i]};
        BSP_CAN_Frame_t expected;
        assert(DM_Motor_EncodeMIT(&g_motor_can_config[i],&cmd,&expected)==BSP_CAN_OK);
        assert(!memcmp(motor_last_mit[i+1],expected.data,8));
    }
    can_io[1].free_slots=0;
    FiveControl_Tick(test_tick+=5);
    assert(s->phase==FIVE_DEGRADED && (s->fault & FIVE_TX) && s->submitted==0x1c);
    assert_gravity_frames(s,0x1c); /* Other bus gets G immediately, not stale impedance. */
    assert(FiveControl_KeyPress()==BSP_CAN_NOT_READY);
    can_io[1].free_slots=8;
    (void)key_level(false,1,raw);
    assert(FiveControl_Acknowledge()==BSP_CAN_OK);
    (void)key_level(false,1,raw);
    assert(s->phase==FIVE_DRAG); assert_gravity_frames(s,31);
    for (unsigned i=1;i<=5;++i) { assert(motor_enable_calls[i]==1 && !motor_disable_calls[i]); }
    puts("PASS: KEY explicit cold/drag/impedance/drag, fixed entry center, held key and fault handover");
}
static void test_motion_admission(void)
{
    DM_Motor_t *m[5];five_setup(m);
    assert(FiveControl_Init(m,test_tick));
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    FiveControl_Tick(test_tick+=5);
    const FiveControl_Status *s=FiveControl_StatusNow();
    double target[5]={60,0,0,0,0};
    live_workspace_pose(target);
    /* The retained zero and the measured +179-degree raw branch are both
     * inside the new travel. Neither needs a turn shift or a small-angle gate. */
    assert(s->phase==FIVE_GRAVITY && !s->fault && s->submitted==31);
    assert(FiveControl_Move(target,4000)==BSP_CAN_OK);
    FiveControl_Gravity();
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    plant_sample_at(3,1,179.1285411727544f*.01745329252f);
    FiveControl_Tick(test_tick+=5);
    assert(s->reference_rad[3]>g_joint_reference_config[3].observed_lower_rad &&
           s->reference_rad[3]<g_joint_reference_config[3].observed_upper_rad);
    for (unsigned i=0;i<5;++i) {
        target[i]=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/.01745329251994329577;
    }
    assert(FiveControl_Move(target,2000)==BSP_CAN_OK); /* No nonzero-J1/small-J4 gate. */
    FiveControl_Gravity(); /* Cancel in the host fixture; never send a physical target. */
    float raw[5];
    for (unsigned i=0;i<5;++i) { raw[i]=s->feedback[i].position_rad; }
    /* A moving test joint OR a moving companion must not veto the next
     * bounded trajectory. Exercise every joint in both velocity directions. */
    for (unsigned direction=0;direction<2;++direction) {
        for (unsigned moving=0;moving<5;++moving) {
            for (unsigned i=0;i<5;++i) {
                plant_sample_at_speed(i,1,raw[i],
                    i==moving ? (direction ? 0x7f4 : 0x80b) : 0x800);
            }
            FiveControl_Tick(test_tick+=5);
            assert(fabsf(s->feedback[moving].velocity_rad_s)>.04f);
            for (unsigned i=0;i<5;++i) {
                target[i]=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/.01745329251994329577;
            }
            target[1]+=.5;
            assert(FiveControl_Move(target,2000)==BSP_CAN_OK);
            FiveControl_Tick(test_tick+=5);
            assert(s->phase==FIVE_MOTION && !s->fault && s->submitted==31);
            FiveControl_Gravity();
        }
    }
    for (unsigned i=0;i<5;++i) { plant_sample_at_speed(i,1,raw[i],0x800); }
    FiveControl_Tick(test_tick+=5);
    target[0]=NAN;assert(FiveControl_Move(target,2000)==BSP_CAN_INVALID_ARG);
    target[0]=0;
    target[3]=(g_joint_reference_config[3].observed_upper_rad-
               g_joint_reference_config[3].zero_rad)/.01745329251994329577;
    assert(FiveControl_Move(target,2000)==BSP_CAN_INVALID_ARG);
    plant_sample_at(3,1,g_joint_reference_config[3].observed_upper_rad+.01745329252f);
    FiveControl_Tick(test_tick+=5);
    assert(FiveControl_Move(target,2000)==BSP_CAN_NOT_READY);
    assert(s->phase==FIVE_GRAVITY && !s->fault && s->submitted==31);
    /* A one-code endpoint discrepancy can only depart inward; no range fault
     * is hidden for a genuinely out-of-range target or further excursion. */
    plant_sample_at(3,1,179.1285411727544f*.01745329252f);
    plant_sample_at(4,1,g_joint_reference_config[4].observed_lower_rad);
    FiveControl_Tick(test_tick+=5);
    for (unsigned i=0;i<5;++i) {
        target[i]=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/.01745329251994329577;
    }
    target[4]=-10;
    assert(FiveControl_Move(target,8000)==BSP_CAN_OK);
    FiveControl_Tick(test_tick+=5);
    assert(s->phase==FIVE_MOTION && !s->fault && s->submitted==31);
    FiveControl_Gravity();
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    plant_sample_at(3,1,179.1285411727544f*.01745329252f);
    FiveControl_Tick(test_tick+=5);
    for (unsigned i=0;i<5;++i) {
        target[i]=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/.01745329251994329577;
    }
    target[4]+=2;assert(FiveControl_Move(target,2000)==BSP_CAN_OK);
    for (unsigned elapsed=5;elapsed<=4000;elapsed+=5) {
        for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
        plant_sample_at(3,1,179.1285411727544f*.01745329252f);
        FiveControl_Tick(test_tick+=5);
        if (elapsed==2000) { assert(s->phase==FIVE_POSITION && !s->fault); }
    }
    assert(s->phase==FIVE_POSITION && !s->fault && s->submitted==31);
    for (unsigned i=1;i<=5;++i) { assert(!motor_enable_calls[i] && !motor_disable_calls[i]); }
    puts("PASS: motion admission without small-angle/rest gates, all five signed velocities, raw travel retained");
}
#ifdef YLAMP_J3_REVERSE_TEST
static void test_j3_reverse(void)
{
    assert(g_five_j3_reverse.total_ms==600 && g_five_j3_reverse.ramp_ms==100 &&
           g_five_j3_reverse.max_nm==.5f);
    FiveSupport_State mode={0};
    float g[5]={.01f,.3f,.4f,.02f,-.06f}, torque[5];
    assert(!FiveSupport_EnterReverseJ3(&mode,0,NAN));
    assert(!FiveSupport_EnterReverseJ3(&mode,0,.5001f));
    assert(FiveSupport_EnterReverseJ3(&mode,UINT32_MAX-50U,g[2]));
    memcpy(torque,g,sizeof g);
    FiveSupport_ApplyReverseJ3(&mode,49U,torque); /* 100 ms across tick wrap. */
    assert(mode.reverse_j3 && torque[2]==-g[2]);
    for (unsigned i=0;i<5;++i) { if (i!=2) { assert(torque[i]==g[i]); } }
    memcpy(torque,g,sizeof g);
    FiveSupport_ApplyReverseJ3(&mode,549U,torque);
    assert(!mode.reverse_j3 && !memcmp(torque,g,sizeof g));
    assert(FiveSupport_EnterReverseJ3(&mode,0,g[2]));
    memcpy(torque,g,sizeof g);torque[2]=.5001f;
    FiveSupport_ApplyReverseJ3(&mode,100U,torque);
    assert(!mode.reverse_j3 && torque[2]==.5001f); /* Original G, not a clipped reverse. */

    DM_Motor_t *m[5];five_setup(m);
    assert(FiveControl_Init(m,test_tick));
    assert(FiveControl_ReverseJ3()==BSP_CAN_NOT_READY);
    /* Keep J3 support inside the unchanged reverse-test envelope. */
    const float raw[5]={-.923361659f,2.486647606f,.530982900f,3.704699993f,-1.008049369f};
    for (unsigned i=0;i<5;++i) { plant_sample_at(i,1,raw[i]); }
    FiveControl_Tick(test_tick+=5);
    const FiveControl_Status *s=FiveControl_StatusNow();
    assert(s->phase==FIVE_GRAVITY && !s->fault && fabsf(s->gravity_nm[2])<.5f);
    test_ipsr=1;assert(FiveControl_ReverseJ3()==BSP_CAN_NOT_READY);test_ipsr=0;
    assert(FiveControl_KeyPress()==BSP_CAN_OK);
    FiveControl_Tick(test_tick+=5);
    assert(FiveControl_ReverseJ3()==BSP_CAN_NOT_READY); /* No manual drag in a comparison. */
    FiveControl_Gravity();
    assert(FiveControl_ReverseJ3()==BSP_CAN_OK);
    assert(FiveControl_Drag()==BSP_CAN_NOT_READY); /* Before the phase-publishing tick too. */
    const double target[5]={0};
    assert(FiveControl_Move(target,2000)==BSP_CAN_NOT_READY);
    /* No USB servicing at all: the control owner must end the trial itself. */
    for (unsigned elapsed=5;elapsed<=600;elapsed+=5) {
        test_tick+=5;
        for (unsigned i=0;i<5;++i) { plant_sample_at(i,1,raw[i]); }
        FiveControl_Tick(test_tick);
        assert(!s->fault && s->submitted==31);
        assert_gravity_frames(s,31U & ~4U);
        if (elapsed<600) { assert(s->phase==FIVE_REVERSE_J3); }
        if (elapsed==300) { assert(FiveControl_ReverseJ3()==BSP_CAN_NOT_READY); }
        if ((elapsed>=100 && elapsed<=500) || elapsed==50 || elapsed==550) {
            const DM_Motor_MIT_Command_t cmd={
                .torque_ff_nm=(elapsed==50 || elapsed==550) ? 0.f : -s->gravity_nm[2]};
            BSP_CAN_Frame_t expected;
            assert(DM_Motor_EncodeMIT(&g_motor_can_config[2],&cmd,&expected)==BSP_CAN_OK);
            assert(!memcmp(motor_last_mit[3],expected.data,8));
        }
    }
    assert(s->phase==FIVE_GRAVITY);assert_gravity_frames(s,31);
    assert(FiveControl_ReverseJ3()==BSP_CAN_OK);
    for (unsigned i=0;i<5;++i) { plant_sample_at(i,1,raw[i]); }
    FiveControl_Tick(test_tick+=5);
    assert(FiveControl_KeyPress()==BSP_CAN_OK);
    FiveControl_Tick(test_tick+=5);
    assert(s->phase==FIVE_DRAG);assert_gravity_frames(s,31);
    FiveControl_Gravity();
    assert(FiveControl_ReverseJ3()==BSP_CAN_OK);
    can_io[1].free_slots=0;
    FiveControl_Tick(test_tick+=5);
    assert(s->phase==FIVE_DEGRADED && (s->fault & FIVE_TX) && s->submitted==0x1c);
    assert_gravity_frames(s,0x1c);
    assert(FiveControl_ReverseJ3()==BSP_CAN_NOT_READY);
    for (unsigned i=1;i<=5;++i) { assert(!motor_enable_calls[i] && !motor_disable_calls[i]); }
    puts("PASS: J3-only reverse frames, zero PD, ramps, local expiry/wrap, no extension/enable, KEY/fault exit");
}
#endif
static void test_control(void)
{
    DM_Motor_t *m[5]; five_setup(m);
    assert(FiveControl_Init(m,test_tick));
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    FiveControl_Tick(test_tick+=5);
    const FiveControl_Status *s=FiveControl_StatusNow();
    assert(s->gravity_valid && s->phase==FIVE_GRAVITY && s->fresh==31 && s->submitted==31);
    assert(fabsf(s->gravity_nm[1])+fabsf(s->gravity_nm[2])>.1f);
    /* Real packet equality, even after moving away from the initial pose
     * and reversing measured velocity. PD hidden inside torque_ff must fail. */
    const float offset_rad[5]={.08f,-.05f,.07f,.12f,-.09f};
    float initial_g2=s->gravity_nm[1], first_g[5];
    for (int pass=0;pass<2;++pass) {
        for (unsigned i=0;i<5;++i) {
            plant_sample_at_speed(i,1,g_joint_reference_config[i].zero_rad+offset_rad[i],
                                  pass ? 0x7c0 : 0x840);
        }
        FiveControl_Tick(test_tick+=5);
        assert(s->phase==FIVE_GRAVITY && !s->fault && !s->limited && s->submitted==31);
        assert(fabsf(initial_g2-s->gravity_nm[1])>.01f);
        float q[5],nominal[5],residual[5]; const float zero[5]={0};
        for (unsigned i=0;i<5;++i) {
            const double angle=(s->reference_rad[i]-g_joint_reference_config[i].zero_rad)/.01745329251994329577;
            q[i]=(float)((angle+g_five_mit_hw[i].cad_offset_deg)*.01745329251994329577);
        }
        assert(FiveDynamics_Inverse(q,zero,zero,nominal));
        assert(FiveDynamics_GravityResidual(q,g_five_j2_gravity_delta,residual));
        assert(g_five_j3_gravity_scale==1.2774127431831357f);
        assert(g_five_j3_gravity_cosine_nm==-.010825531721891f);
        for (unsigned i=0;i<5;++i) {
            const float angle=s->reference_rad[2]-g_joint_reference_config[2].zero_rad;
            const float expected_g=i==2 ? nominal[i]*1.2774127431831357f-.010825531721891f*cosf(angle) :
                nominal[i]+(i==1 ? residual[i] : 0);
            assert(fabsf(s->gravity_nm[i]-expected_g)<1e-6f);
            assert(g_five_mit_hw[i].kp==0 && g_five_mit_hw[i].kd==0);
            assert((pass ? -1.f : 1.f)*s->feedback[i].velocity_rad_s>.1f);
            const DM_Motor_MIT_Command_t only_g={.torque_ff_nm=s->gravity_nm[i]};
            BSP_CAN_Frame_t frame;
            assert(DM_Motor_EncodeMIT(&g_motor_can_config[i],&only_g,&frame)==BSP_CAN_OK);
            assert(!memcmp(motor_last_mit[i+1],frame.data,8));
            if (!pass) { first_g[i]=s->gravity_nm[i]; }
            else { assert(s->gravity_nm[i]==first_g[i]); }
        }
    }
    /* Real feedback/observe/frame path on both angular sides and across zero.
     * Keep the recorded upstream/wrist datum; the trim is not a new CAD zero. */
    const float datum[5]={4.600900802f,154.758084705f,0,-141.862468986f,-68.204561504f};
    const float angles[]={-10.f,-.1f,0.f,.1f,10.f,90.f};
    for (unsigned k=0;k<sizeof angles/sizeof angles[0];++k) {
        for (unsigned i=0;i<5;++i) {
            const float raw=i==2 ? g_joint_reference_config[2].zero_rad+angles[k]*.01745329252f :
                datum[i]*.01745329252f;
            plant_sample_at(i,1,raw);
        }
        FiveControl_Tick(test_tick+=5);
        assert(s->phase==FIVE_GRAVITY && !s->fault && !s->limited && s->submitted==31);
        float q[5],nominal[5],residual[5];const float zero[5]={0};
        for (unsigned i=0;i<5;++i) {
            q[i]=s->reference_rad[i]-g_joint_reference_config[i].zero_rad+
                g_five_mit_hw[i].cad_offset_deg*.01745329252f;
        }
        assert(FiveDynamics_Inverse(q,zero,zero,nominal));
        assert(FiveDynamics_GravityResidual(q,g_five_j2_gravity_delta,residual));
        const float angle=s->reference_rad[2]-g_joint_reference_config[2].zero_rad;
        const float old=nominal[2]*1.2774127431831357f,delta=-.010825531721891f*cosf(angle);
        assert(fabsf(delta)<=.010827f && fabsf(s->gravity_nm[2]-old-delta)<1e-6f);
        if (angles[k]==-10.f) { assert(old>0 && s->gravity_nm[2]>0 && s->gravity_nm[2]<old); }
        if (angles[k]==10.f) { assert(old<0 && s->gravity_nm[2]<old); }
        for (unsigned i=0;i<5;++i) {
            if (i!=2) {
                assert(fabsf(s->gravity_nm[i]-nominal[i]-(i==1 ? residual[i] : 0))<1e-6f);
            }
        }
        assert_gravity_frames(s,31);
    }
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    plant_sample_at(3,1,g_joint_reference_config[3].observed_upper_rad+.01745329252f);
    FiveControl_Tick(test_tick+=5);
    double goal[5]={1,0,0,0,0};live_workspace_pose(goal);
    assert(FiveControl_Move(goal,2000)==BSP_CAN_NOT_READY); /* Actual J4 feedback outside raw range. */
    FiveControl_Tick(test_tick+=5);
    assert(s->phase==FIVE_GRAVITY && s->submitted==31);
    can_io[1].free_slots=0; FiveControl_Tick(test_tick+=5);
    assert(s->phase==FIVE_DEGRADED && (s->fault & FIVE_TX) && s->submitted==0x1c);
    can_io[1].free_slots=8;
    uint32_t t=s->gravity_at; float g=s->gravity_nm[1];
    FiveControl_Tick(test_tick+=110);
    assert(s->fresh==0 && s->gravity_at==t && s->gravity_nm[1]==g && s->submitted==31);
    assert(FiveControl_Move(goal,2000)==BSP_CAN_NOT_READY);
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    FiveControl_Tick(test_tick+=5);
    assert(s->phase==FIVE_DEGRADED && s->submitted==31);
    assert(FiveControl_Acknowledge()==BSP_CAN_OK);
    FiveControl_Tick(test_tick+=5); assert(s->phase==FIVE_DRAG);
    for (unsigned i=1;i<=5;++i) { assert(!motor_enable_calls[i] && !motor_disable_calls[i]); }
}
static void test_cold(bool silent)
{
    DM_Motor_t *m[5]; five_setup(m);
    assert(FiveControl_Init(m,test_tick));
    float raw[5];
    for (unsigned i=0;i<5;++i) { raw[i]=g_joint_reference_config[i].zero_rad; }
    if (silent) {
        /* Recorded raw angles, not commanded positions. J3 has no pre-enable sample. */
        raw[0]=-54.24f*.01745329252f; raw[1]=148.33f*.01745329252f;
        raw[3]=97.30f*.01745329252f; raw[4]=-80.051f*.01745329252f;
    }
    for (unsigned i=0;i<5;++i) { if(i!=2) { plant_sample_at(i,0,raw[i]); } }
    FiveControl_Tick(test_tick+=5);
    const FiveControl_Status *s=FiveControl_StatusNow();
    assert(s->fresh==27 && !s->submitted && !s->gravity_valid && s->phase==FIVE_WAIT);
    if (silent) { assert(s->reference_rad[4]<g_joint_reference_config[4].observed_lower_rad); }
    assert(FiveControl_ColdStart()==BSP_CAN_OK && motor_mit_calls[3]==1 && !motor_enable_calls[3]);
    if (silent) {
        for (unsigned j=0;j<4;++j) { FiveControl_Tick(test_tick+=5); }
        for (unsigned i=1;i<=5;++i) { assert(motor_enable_calls[i]==(unsigned)(i==3)); }
    }
    for (unsigned i=0;i<5;++i) { plant_sample_at(i,(unsigned)(silent && i==2),raw[i]); }
    FiveControl_Tick(test_tick+=5); assert(s->gravity_valid && s->submitted==31);
    FiveControl_Tick(test_tick+=5);
    for(unsigned i=1;i<=5;++i) { assert(motor_enable_calls[i]==1 && !motor_disable_calls[i]); }
    assert(s->phase!=FIVE_GRAVITY);
    for(unsigned i=0;i<5;++i) { plant_sample_at(i,1,raw[i]); }
    FiveControl_Tick(test_tick+=5);
    assert(s->fault==0 && s->phase==FIVE_DRAG && s->enabled==31 && s->submitted==31);
    assert(FiveControl_ColdStart()==BSP_CAN_NOT_READY);
    if (silent) {
        const double target[5]={0};
        for (unsigned k=0;k<4;++k) {
            for (unsigned i=0;i<5;++i) { plant_sample_at(i,1,raw[i]); }
            FiveControl_Tick(test_tick+=5);
            assert(s->phase==FIVE_DRAG && s->submitted==31 && !s->fault);
        }
        assert(FiveControl_Move(target,10000)==BSP_CAN_NOT_READY);
        assert(FiveControl_Acknowledge()==BSP_CAN_OK);
        for (unsigned i=1;i<=5;++i) { assert(motor_enable_calls[i]==1 && !motor_disable_calls[i]); }
    }
}
static int five_wire_stream(void)
{
    DM_Motor_t *motors[5];five_setup(motors);
    FiveMIT_Plan plan={0};
    char cmd;
    while (scanf(" %c",&cmd)==1) {
        if (cmd=='P') {
            double a[5],b[5],sec;
            for (int i=0;i<5;++i) { if (scanf("%lf",&a[i])!=1) {return 2;} }
            for (int i=0;i<5;++i) { if (scanf("%lf",&b[i])!=1) {return 2;} }
            if (scanf("%lf",&sec)!=1) {return 2;}
            puts(FiveMIT_Prepare(&plan,a,b,sec)?"OK":"REJECT");
        } else if (cmd=='L') {
            float q[5],g[5],b[5];
            for (unsigned i=0;i<5;++i) {if (scanf("%f",&q[i])!=1) {return 2;}}
            if (!FiveDynamics_GravityLoad(q,g,b)) {puts("REJECT");}
            else {
                for (unsigned i=0;i<5;++i) {printf("%.9g %.9g%c",g[i],b[i],i==4?'\n':' ');}
            }
        } else if (cmd=='F') {
            unsigned axis;float v,g,b,torque;
            if (scanf("%u%f%f%f",&axis,&v,&g,&b)!=4) {return 2;}
            if (FiveMIT_SpeedFrictionTorque(axis,v,g,b,&torque)) {printf("%.9g\n",torque);}
            else {puts("REJECT");}
        } else if (cmd=='S') {
            double t;FiveMIT_State x;unsigned sent;
            if (scanf("%lf",&t)!=1) {return 2;}
            for (int i=0;i<5;++i) { if (scanf("%lf",&x.software_deg[i])!=1) {return 2;} }
            for (int i=0;i<5;++i) { if (scanf("%f",&x.raw_rad[i])!=1) {return 2;} }
            for (int i=0;i<5;++i) { if (scanf("%f",&x.velocity_rad_s[i])!=1) {return 2;} }
            if (FiveMIT_SendSample(&plan,t,&x,g_five_mit_sim,NULL,motors,&sent)!=BSP_CAN_OK) {puts("REJECT");}
            else {
                for (int i=1;i<=5;++i) {
                    for (int j=0;j<8;++j) {printf("%02x",(unsigned)motor_last_mit[i][j]);}
                    putchar(i==5?'\n':' ');
                }
            }
        } else { return 2; }
        fflush(stdout);
    }
    return 0;
}
