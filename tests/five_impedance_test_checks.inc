static void test_impedance_cycle_mode(void)
{
    FiveMIT_State x={0};FiveSupport_State mode={0},saved;
    for (unsigned i=0;i<5;++i) { x.raw_rad[i]=.1f*(i+1); }
    assert(!FiveSupport_StartTest(&mode,1,1,0));
    assert(FiveSupport_TestDuration()==24000);
    assert(FiveSupport_EnterImpedance(&mode,&x));
    memcpy(&saved,&mode,sizeof saved);
    for (unsigned joint=0;joint<=6;joint+=6) {
        assert(!FiveSupport_StartTest(&mode,joint,1,0));
        assert(!memcmp(&mode,&saved,sizeof mode));
    }
    assert(!FiveSupport_StartTest(&mode,1,NAN,0));
    assert(!FiveSupport_StartTest(&mode,1,0,0));
    assert(!FiveSupport_StartTest(&mode,1,60.01f,0));
    const uint32_t start=UINT32_MAX-500;
    for (int sign=-1;sign<=1;sign+=2) {
        assert(FiveSupport_StartTest(&mode,4,30.f*sign,start));
        assert(!FiveSupport_StartTest(&mode,4,(float)sign,start+1));
        const unsigned times[]={0,2000,4000,5999,6000,7999,12000,14000,16000,17999,18000,23999,24000};
        const float ratios[]={0,.5f,1,1,0,0,0,-.5f,-1,-1,0,0,0};
        for (unsigned k=0;k<sizeof times/sizeof times[0];++k) {
            FiveSupport_TestTick(&mode,start+times[k]);
            assert(fabsf(mode.anchor_raw_rad[3]-x.raw_rad[3]-
                sign*ratios[k]*30.f*.0174532925199433f)<2e-7f);
            for (unsigned i=0;i<5;++i) {
                if (i!=3) { assert(mode.anchor_raw_rad[i]==x.raw_rad[i]); }
            }
            assert(mode.impedance && mode.test_started_ms==start);
        }
        assert(!mode.test_joint && mode.anchor_raw_rad[3]==x.raw_rad[3]);
    }
    assert(FiveSupport_StartTest(&mode,4,1,start));
    FiveSupport_TestTick(&mode,start+500);
    FiveSupport_Gravity(&mode);
    assert(!mode.impedance && !mode.test_joint);
    float stopped=mode.anchor_raw_rad[3];
    FiveSupport_TestTick(&mode,start+2000);
    assert(mode.anchor_raw_rad[3]==stopped); /* Never resume a cancelled cycle. */
}
static void test_remote_impedance(void)
{
    test_impedance_cycle_mode();
    DM_Motor_t *motors[5];five_setup(motors);
    assert(FiveControl_Init(motors,test_tick));
    assert(FiveControl_Impedance()==BSP_CAN_NOT_READY);
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    FiveControl_Tick(test_tick+=2);
    const FiveControl_Status *s=FiveControl_StatusNow();
    assert(s->phase==FIVE_IDLE && !s->fault);
    test_ipsr=1;assert(FiveControl_Impedance()==BSP_CAN_NOT_READY);test_ipsr=0;
    assert(FiveControl_ImpedanceTest(4,1)==BSP_CAN_NOT_READY);
    assert(FiveControl_Impedance()==BSP_CAN_OK);
    FiveControl_Tick(test_tick+=2);
    float center[5];memcpy(center,s->impedance_target_raw_rad,sizeof center);
    assert(s->phase==FIVE_IMPEDANCE && !s->impedance_test_joint);
    assert(FiveControl_ImpedanceTest(0,1)==BSP_CAN_INVALID_ARG);
    assert(FiveControl_ImpedanceTest(6,1)==BSP_CAN_INVALID_ARG);
    assert(FiveControl_ImpedanceTest(4,NAN)==BSP_CAN_INVALID_ARG);
    assert(FiveControl_ImpedanceTest(4,61)==BSP_CAN_INVALID_ARG);
    assert(FiveControl_ImpedanceTest(4,30)==BSP_CAN_OK);
    const uint32_t started=s->impedance_test_started_ms;
    assert(s->impedance_test_joint==4);
    assert(FiveControl_ImpedanceTest(4,1)==BSP_CAN_NOT_READY);
    assert(FiveControl_Impedance()==BSP_CAN_OK && s->impedance_test_started_ms==started);
    double target[5]={0};
    assert(FiveControl_Move(target,1000)==BSP_CAN_NOT_READY);
    for (unsigned elapsed=2;elapsed<=24000;elapsed+=2) {
        for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
        FiveControl_Tick(test_tick+=2);
        assert(s->phase==FIVE_IMPEDANCE && s->submitted==31 && !s->fault);
        assert(s->impedance_test_started_ms==started);
        for (unsigned i=0;i<5;++i) {
            if (i!=3) { assert(s->impedance_target_raw_rad[i]==center[i]); }
            assert(((motor_last_mit[i+1][3]&15U)<<8 | motor_last_mit[i+1][4])>0);
        }
        if (elapsed==4000 || elapsed==16000) {
            const float sign=elapsed==4000 ? 1.f : -1.f;
            assert(fabsf(s->impedance_target_raw_rad[3]-center[3]-
                sign*30.f*.0174532925199433f)<2e-7f);
        }
        if (elapsed==6000 || elapsed==18000) {
            assert(s->impedance_target_raw_rad[3]==center[3]); /* Step, not a ramp. */
        }
    }
    assert(!s->impedance_test_joint && !memcmp(center,s->impedance_target_raw_rad,sizeof center));
    FiveControl_Release();FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DRAG && !s->fault);
    assert(FiveControl_Impedance()==BSP_CAN_OK);
    assert(FiveControl_ImpedanceTest(4,1)==BSP_CAN_OK);
    assert(FiveControl_KeyPress()==BSP_CAN_OK);
    FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DRAG && !s->impedance_test_joint);
    /* Native manual hold has no new test angle gate, even outside J4 travel.
     * An automated cycle must retain the old trajectory admission instead. */
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    plant_sample_at(3,1,351.f*.0174532925199433f);
    FiveControl_Tick(test_tick+=2);
    assert(FiveControl_Impedance()==BSP_CAN_OK);
    FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_IMPEDANCE && !s->fault);
    assert(FiveControl_ImpedanceTest(4,1)!=BSP_CAN_OK && !s->fault);
    FiveControl_Release();
    for (unsigned i=0;i<5;++i) { plant_sample(i,1); }
    FiveControl_Tick(test_tick+=2);
    assert(FiveControl_Impedance()==BSP_CAN_OK);
    assert(FiveControl_ImpedanceTest(4,1)==BSP_CAN_OK);
    plant_sample(0,0);FiveControl_Tick(test_tick+=2);
    assert(s->phase==FIVE_DEGRADED && s->fault && !s->impedance_test_joint);
    assert(FiveControl_Impedance()==BSP_CAN_NOT_READY);
    assert_gravity_frames(s,30);
    for (unsigned i=1;i<=5;++i) { assert(!motor_enable_calls[i] && !motor_disable_calls[i]); }
    puts("PASS: remote native hold/cycle, fixed center, wrap, travel, KEY/session/fault cancellation; no enable");
}
