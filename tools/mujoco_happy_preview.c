/* Simulation-only player. No serial, CAN, probe, enable or firmware calls.
 * q/v/a share one C2 quintic trajectory in CAD radians. MuJoCo native PID owns
 * PD; the existing C model supplies accepted G + nominal M*a+C feedforward.
 * This is NOT the MCU active-control or sampled KEY-yielding implementation.
 */
#include <windows.h>
#include <GL/gl.h>
#include <mujoco/mujoco.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <io.h>
#include <fcntl.h>
#include <stdint.h>
#include "five_dynamics.h"
#include "five_support.h"

int yi_model(const double *, const double *, double *, double *);

#define WIDTH 960
#define HEIGHT 720
#define DEG (180.0/3.14159265358979323846)

static mjModel *model;
static mjData *data;
static const mjtNum *keys;
static int nkeys, quit, paused, restart, drag_x, drag_y;
static double duration;
static double response_scale=1;
static int expressive;
static mjvCamera camera;
static mjvOption option;
static mjvScene scene;
static mjrContext context;
static HWND window;
static HDC dc;
static HGLRC gl_context;

typedef struct { double time,q[5],v[5]; } State;
static State *replay_states;
static uint32_t replay_count;

/* One gaze, one upward release, two diminishing head-led responses.
 * Unlike v1, expressive components overlap; not every joint stops at a knot. */
static void shape(double t,double begin,double length,int pulse,double out[3])
{
    double s=(t-begin)/length;
    if (s<=0 || s>=1) {
        out[0]=pulse?0:(s>=1?1:0); out[1]=out[2]=0; return;
    }
    if (pulse) {
        double sn=sin(3.14159265358979323846*s),cs=cos(3.14159265358979323846*s);
        out[0]=sn*sn*sn*sn;
        out[1]=4*3.14159265358979323846*sn*sn*sn*cs/length;
        out[2]=4*3.14159265358979323846*3.14159265358979323846*
            (3*sn*sn*cs*cs-sn*sn*sn*sn)/(length*length);
    } else {
        out[0]=s*s*s*(10+s*(-15+6*s));
        out[1]=30*s*s*(1-s)*(1-s)/length;
        out[2]=60*s*(1-s)*(1-2*s)/(length*length);
    }
}

static void reference(double t, double q[5], double v[5], double a[5])
{
    if (expressive) {
        const double gaze[5]={14,-1,-2,0,6};
        const double perk[5]={0,-13,-15,0,7};
        const double relax[5]={0,1,2,3,-1};
        const double response[5]={0,2.0,3.2,0,2.7};
        const double lag[5]={0,.12,.08,0,0};
        for (int j=0;j<5;++j) {
            double g[3],p[3],r[3],b1[3],b2[3];
            shape(t,.35,.55,0,g);
            shape(t,1.15+lag[j],.55,0,p);
            shape(t,3.25+lag[j],.7,0,r);
            shape(t,1.90+lag[j],.58,1,b1);
            shape(t,2.51+lag[j],.64,1,b2);
            q[j]=keys[j+1]+(gaze[j]*g[0]+perk[j]*p[0]+relax[j]*r[0]+
                response_scale*response[j]*(b1[0]+.55*b2[0]))/DEG;
            v[j]=(gaze[j]*g[1]+perk[j]*p[1]+relax[j]*r[1]+
                response_scale*response[j]*(b1[1]+.55*b2[1]))/DEG;
            a[j]=(gaze[j]*g[2]+perk[j]*p[2]+relax[j]*r[2]+
                response_scale*response[j]*(b1[2]+.55*b2[2]))/DEG;
        }
        return;
    }
    int k=0;
    while (k<nkeys-2 && t>keys[(k+1)*6]) ++k;
    double dt=keys[(k+1)*6]-keys[k*6];
    double s=fmax(0,fmin(1,(t-keys[k*6])/dt));
    double h=s*s*s*(10+s*(-15+6*s));
    double dh=30*s*s*(1-s)*(1-s)/dt;
    double ddh=60*s*(1-s)*(1-2*s)/(dt*dt);
    for (int j=0;j<5;++j) {
        double delta=keys[(k+1)*6+1+j]-keys[k*6+1+j];
        q[j]=keys[k*6+1+j]+delta*h;
        v[j]=delta*dh;
        a[j]=delta*ddh;
    }
}

static int control(void)
{
    double q[5],v[5],a[5],g[5],unused[5];
    float measured_q[5],measured_v[5],accel[5],inverse[5],nominal_g[5],bend[5];
    reference(data->time,q,v,a);
    if (!yi_model(data->qpos,data->qvel,g,unused)) return 0;
    for (int j=0;j<5;++j) {
        measured_q[j]=(float)data->qpos[j];
        measured_v[j]=(float)data->qvel[j];
        accel[j]=(float)a[j];
    }
    if (!FiveDynamics_Inverse(measured_q,measured_v,accel,inverse) ||
        !FiveDynamics_GravityLoad(measured_q,nominal_g,bend)) return 0;
    for (int j=0;j<5;++j) {
        int adr=model->actuator_ctrladr[j];
        data->ctrl[adr]=q[j];
        data->ctrl[adr+1]=v[j];
        data->ctrl[adr+2]=g[j]+inverse[j]-nominal_g[j];
    }
    return 1;
}

static void reset(void)
{
    mj_resetData(model,data);
    for (int j=0;j<5;++j) data->qpos[j]=keys[1+j];
    control();
    mj_forward(model,data);
}

static const char *phase(double t)
{
    if (expressive) {
        if (t<.35) return "Notice";
        if (t<1.15) return "Look at you";
        if (t<1.90) return "Brighten up";
        if (t<3.25) return "Little delight";
        if (t<4.1) return "Relax";
        return "Stay connected";
    }
    if (t<.5) return "Ready";
    if (t<1.4) return "Perk up";
    if (t<4.8) return "Playful sway";
    if (t<6.35) return "Happy nod";
    if (t<7.2) return "Soft return";
    return "Rest";
}

static LRESULT CALLBACK events(HWND w,UINT msg,WPARAM wp,LPARAM lp)
{
    switch (msg) {
    case WM_CLOSE: quit=1; return 0;
    case WM_KEYDOWN:
        if (wp==VK_ESCAPE) quit=1;
        if (wp==VK_SPACE) paused=!paused;
        if (wp=='R') restart=1;
        return 0;
    case WM_LBUTTONDOWN:
        drag_x=(short)LOWORD(lp); drag_y=(short)HIWORD(lp);
        SetCapture(w); return 0;
    case WM_LBUTTONUP: ReleaseCapture(); return 0;
    case WM_MOUSEMOVE:
        if (wp&MK_LBUTTON) {
            int x=(short)LOWORD(lp),y=(short)HIWORD(lp);
            camera.azimuth-=(x-drag_x)*.35;
            camera.elevation=fmax(-85,fmin(10,camera.elevation-(y-drag_y)*.25));
            drag_x=x; drag_y=y;
        }
        return 0;
    case WM_MOUSEWHEEL:
        camera.distance*=GET_WHEEL_DELTA_WPARAM(wp)>0?.9:1.1;
        return 0;
    default: return DefWindowProc(w,msg,wp,lp);
    }
}

static int graphics(int visible)
{
    WNDCLASSA wc={0};
    wc.style=CS_OWNDC; wc.lpfnWndProc=events;
    wc.hInstance=GetModuleHandle(NULL); wc.lpszClassName="YlampHappyPreview";
    wc.hCursor=LoadCursor(NULL,IDC_ARROW);
    if (!RegisterClassA(&wc)) return 0;
    RECT r={0,0,WIDTH,HEIGHT};
    AdjustWindowRect(&r,WS_OVERLAPPEDWINDOW,FALSE);
    char title[128];
    snprintf(title,sizeof(title),"Ylamp | HAPPY v2 x%.1f | MuJoCo simulation ONLY",
        response_scale);
    window=CreateWindowA(wc.lpszClassName,title,
        WS_OVERLAPPEDWINDOW,CW_USEDEFAULT,CW_USEDEFAULT,
        r.right-r.left,r.bottom-r.top,NULL,NULL,wc.hInstance,NULL);
    if (!window) return 0;
    dc=GetDC(window);
    PIXELFORMATDESCRIPTOR p={0};
    p.nSize=sizeof(p); p.nVersion=1;
    p.dwFlags=PFD_DRAW_TO_WINDOW|PFD_SUPPORT_OPENGL|PFD_DOUBLEBUFFER;
    p.iPixelType=PFD_TYPE_RGBA; p.cColorBits=24; p.cDepthBits=24;
    int format=ChoosePixelFormat(dc,&p);
    if (!format || !SetPixelFormat(dc,format,&p)) return 0;
    gl_context=wglCreateContext(dc);
    if (!gl_context || !wglMakeCurrent(dc,gl_context)) return 0;
    mjv_defaultCamera(&camera); mjv_defaultOption(&option);
    mjv_defaultScene(&scene); mjr_defaultContext(&context);
    mjv_makeScene(model,&scene,2000);
    mjr_makeContext(model,&context,mjFONTSCALE_150);
    option.geomgroup[3]=0; /* Hide collision proxies, retain contact physics. */
    camera.lookat[0]=0; camera.lookat[1]=0; camera.lookat[2]=.23;
    camera.distance=.88; camera.azimuth=135; camera.elevation=-18;
    if (visible) { ShowWindow(window,SW_SHOW); SetForegroundWindow(window); }
    return 1;
}

static void render(int record)
{
    RECT r; GetClientRect(window,&r);
    mjrRect viewport={0,0,record?WIDTH:r.right,record?HEIGHT:r.bottom};
    mjv_updateScene(model,data,&option,NULL,&camera,mjCAT_ALL,&scene);
    mjr_render(viewport,&scene,&context);
    char left[160],right[160];
    snprintf(left,sizeof(left),"YLAMP / HAPPY v%d x%.1f\nSIMULATION ONLY\n%s",
        expressive?2:1,response_scale,phase(data->time));
    snprintf(right,sizeof(right),"%04.1f / %.1f s\n5 axes / MIT PD + model FF\n%s",
        data->time,duration,record?"Motion study - not hardware validation":
        "SPACE pause | R replay | drag orbit | wheel zoom");
    mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,viewport,left,NULL,&context);
    mjr_overlay(mjFONT_NORMAL,mjGRID_BOTTOMLEFT,viewport,right,NULL,&context);
    if (record) {
        static unsigned char pixels[WIDTH*HEIGHT*3];
        mjr_readPixels(pixels,NULL,viewport,&context);
        if (fwrite(pixels,1,sizeof(pixels),stdout)!=sizeof(pixels)) quit=1;
    } else SwapBuffers(dc);
}

typedef struct {
    double error[5],effort[5],velocity[5],accel[5],qmin[5],qmax[5];
    double penetration;
    int contacts,warnings,range_violations,clamp_steps,steps,finite;
    char deepest_pair[180];
} Audit;

static void sample(Audit *out)
{
    double q[5],v[5],a[5];
    reference(data->time,q,v,a);
    ++out->steps;
    for (int j=0;j<5;++j) {
        out->error[j]=fmax(out->error[j],fabs(q[j]-data->qpos[j])*DEG);
        out->effort[j]=fmax(out->effort[j],fabs(data->qfrc_actuator[j]));
        out->velocity[j]=fmax(out->velocity[j],fabs(data->qvel[j]));
        out->accel[j]=fmax(out->accel[j],fabs(a[j]));
        out->qmin[j]=fmin(out->qmin[j],data->qpos[j]*DEG);
        out->qmax[j]=fmax(out->qmax[j],data->qpos[j]*DEG);
        if (!isfinite(data->qpos[j]) || !isfinite(data->qvel[j]) ||
            !isfinite(data->qfrc_actuator[j])) out->finite=0;
        if (data->qpos[j]<model->jnt_range[j*2] ||
            data->qpos[j]>model->jnt_range[j*2+1]) ++out->range_violations;
        if (fabs(data->qfrc_actuator[j])>=model->actuator_forcerange[j*2+1]-1e-6)
            ++out->clamp_steps;
    }
    out->contacts=out->contacts>data->ncon?out->contacts:data->ncon;
    for (int k=0;k<data->ncon;++k) {
        if (-data->contact[k].dist>out->penetration) {
            out->penetration=-data->contact[k].dist;
            snprintf(out->deepest_pair,sizeof(out->deepest_pair),"%s / %s",
                mj_id2name(model,mjOBJ_GEOM,data->contact[k].geom[0]),
                mj_id2name(model,mjOBJ_GEOM,data->contact[k].geom[1]));
        }
    }
    int warnings=0;
    for (int k=0;k<mjNWARNING;++k) warnings+=data->warning[k].number;
    out->warnings=out->warnings>warnings?out->warnings:warnings;
}

static void array(FILE *f,const char *name,const double *values)
{
    fprintf(f,"  \"%s\": [",name);
    for (int j=0;j<5;++j) fprintf(f,"%s%.9g",j?", ":"",values[j]);
    fprintf(f,"],\n");
}

static int report(const char *path,const Audit *out)
{
    FILE *f=fopen(path,"wx"); /* Never replace existing audit evidence. */
    if (!f) { fprintf(stderr,"Cannot create report: %s\n",path); return 0; }
    fprintf(f,"{\n  \"simulation_only\": true,\n  \"trajectory\": \"%s, not trained\",\n",
        expressive?"head-led C2 overlapping happy v2":"C2 quintic happy v1");
    fprintf(f,"  \"response_amplitude_scale\": %.9g,\n",response_scale);
    fprintf(f,"  \"controller\": \"native MIT PD + accepted G + nominal M*a+C; not MCU active/KEY yielding\",\n");
    fprintf(f,"  \"plant\": \"nominal CAD inertia/armature/viscous damping, original SDF contacts; nonlinear friction not executed\",\n");
    fprintf(f,"  \"actuator_caps_nm\": [4,10,3.6,1.6,0.8],\n");
    fprintf(f,"  \"duration_s\": %.9g,\n  \"steps\": %d,\n",duration,out->steps);
    array(f,"max_tracking_error_deg",out->error);
    array(f,"peak_actuator_nm",out->effort);
    array(f,"peak_actual_velocity_rad_s",out->velocity);
    array(f,"peak_reference_accel_rad_s2",out->accel);
    array(f,"actual_min_CAD_deg",out->qmin);
    array(f,"actual_max_CAD_deg",out->qmax);
    fprintf(f,"  \"finite\": %s,\n  \"warnings\": %d,\n  \"range_violations\": %d,\n",
        out->finite?"true":"false",out->warnings,out->range_violations);
    fprintf(f,"  \"max_contacts\": %d,\n  \"max_penetration_m\": %.9g,\n  \"deepest_pair\": \"%s\",\n",
        out->contacts,out->penetration,out->deepest_pair);
    fprintf(f,"  \"axis_steps_at_simulation_clamp\": %d\n}\n",out->clamp_steps);
    int ok=!ferror(f); if (fclose(f)) ok=0;
    return ok;
}

static int trajectory_test(void)
{
    double eps=1e-5;
    for (double t=0;t<=duration;t+=.007) {
        double q[5],v[5],a[5],qm[5],vm[5],am[5],qp[5],vp[5],ap[5];
        reference(t,q,v,a);
        reference(t-eps,qm,vm,am); reference(t+eps,qp,vp,ap);
        for (int j=0;j<5;++j) {
            if (fabs((qp[j]-qm[j])/(2*eps)-v[j])>1e-5 ||
                fabs((vp[j]-vm[j])/(2*eps)-a[j])>1e-3 ||
                q[j]<=model->jnt_range[j*2] || q[j]>=model->jnt_range[j*2+1])
                return 0;
        }
    }
    for (int k=0;k<nkeys && !expressive;++k) {
        double q[5],v[5],a[5];
        reference(keys[k*6],q,v,a);
        for (int j=0;j<5;++j)
            if (fabs(q[j]-keys[k*6+1+j])>1e-10 || fabs(v[j])>1e-9 || fabs(a[j])>1e-8)
                return 0;
    }
    double q[5],v[5],a[5];
    reference(duration,q,v,a);
    for (int j=0;j<5;++j) {
        if (fabs(v[j])+fabs(a[j])>1e-9) return 0;
        if (expressive && fabs(q[j]-keys[(nkeys-1)*6+1+j])>1e-9) return 0;
    }
    return 1;
}

static void save_state(FILE *f)
{
    State state; state.time=data->time;
    memcpy(state.q,data->qpos,sizeof(state.q));
    memcpy(state.v,data->qvel,sizeof(state.v));
    if (fwrite(&state,sizeof(state),1,f)!=1) {
        fprintf(stderr,"Cannot save simulation state\n"); exit(5);
    }
}

static int load_replay(const char *path)
{
    FILE *f=fopen(path,"rb");
    if (!f) return 0;
    uint32_t header[4];
    int ok=fread(header,sizeof(header),1,f)==1 && header[0]==0x594C4832 &&
        header[1]==1 && header[2]>1 && header[2]<1000000 && header[3]==sizeof(State);
    if (!ok) { fclose(f); return 0; }
    replay_count=header[2]; replay_states=calloc(replay_count,sizeof(State));
    if (!replay_states) { fclose(f); return 0; }
    ok=fread(replay_states,sizeof(State),replay_count,f)==replay_count && fgetc(f)==EOF;
    fclose(f);
    for (uint32_t k=0;ok && k<replay_count;++k) {
        State *s=replay_states+k;
        ok=isfinite(s->time) && (k==0?s->time==0:s->time>replay_states[k-1].time);
        for (int j=0;ok && j<5;++j)
            ok=isfinite(s->q[j]) && isfinite(s->v[j]) &&
                s->q[j]>=model->jnt_range[2*j] && s->q[j]<=model->jnt_range[2*j+1];
    }
    return ok && fabs(replay_states[replay_count-1].time-duration)<1e-6;
}

static void replay(double t)
{
    uint32_t lo=0,hi=replay_count-1;
    while (hi-lo>1) {
        uint32_t mid=(lo+hi)/2;
        if (replay_states[mid].time<t) lo=mid; else hi=mid;
    }
    State *s=replay_states+lo,*e=replay_states+hi;
    double w=fmax(0,fmin(1,(t-s->time)/(e->time-s->time)));
    data->time=t;
    for (int j=0;j<5;++j) {
        data->qpos[j]=s->q[j]+w*(e->q[j]-s->q[j]);
        data->qvel[j]=s->v[j]+w*(e->v[j]-s->v[j]);
    }
    control(); mj_forward(model,data);
}

int main(int argc,char **argv)
{
    if (argc<3) {
        fprintf(stderr,"Usage: preview --check model.xml report.json states.bin\n"
                       "       preview --view|--record model.xml states.bin\n"
                       "       preview --reference model.xml reference.csv\n"); return 2;
    }
    int view=!strcmp(argv[1],"--view"),record=!strcmp(argv[1],"--record");
    int export_reference=!strcmp(argv[1],"--reference");
    if ((!view && !record && !export_reference && strcmp(argv[1],"--check")) ||
        ((view || record || export_reference)?argc!=4:argc!=5))
        return 2;
    char error[1024]={0};
    model=mj_loadXML(argv[2],NULL,error,sizeof(error));
    if (!model) { fprintf(stderr,"%s\n",error); return 2; }
    int id=mj_name2id(model,mjOBJ_NUMERIC,"happy_keyframes");
    if (model->nq!=5 || model->nv!=5 || model->nu!=15 || model->nactuator!=5 ||
        id<0 || model->numeric_size[id]%6 || model->numeric_size[id]<12) return 2;
    keys=model->numeric_data+model->numeric_adr[id];
    nkeys=model->numeric_size[id]/6; duration=keys[(nkeys-1)*6];
    int style=mj_name2id(model,mjOBJ_NUMERIC,"happy_style");
    expressive=style>=0 && model->numeric_data[model->numeric_adr[style]]==2;
    int amplitude=mj_name2id(model,mjOBJ_NUMERIC,"happy_response_scale");
    if (amplitude>=0) {
        if (model->numeric_size[amplitude]!=1) return 2;
        response_scale=model->numeric_data[model->numeric_adr[amplitude]];
        if (!isfinite(response_scale) || response_scale<=0) return 2;
    }
    for (int k=1;k<nkeys;++k) if (keys[k*6]<=keys[(k-1)*6]) return 2;
    /* Values are linked directly from the frozen firmware config, not an old
       XML snapshot. Native simulation clamp is not a real-drive guarantee. */
    for (int j=0;j<5;++j) {
        model->actuator_biasprm[j*mjNBIAS+1]=-g_five_support_gains[j].kp;
        model->actuator_biasprm[j*mjNBIAS+2]=-g_five_support_gains[j].kd;
        model->actuator_forcerange[j*2]=-g_five_mit_tracking[j].max_nm;
        model->actuator_forcerange[j*2+1]=g_five_mit_tracking[j].max_nm;
    }
    if (!trajectory_test()) { fprintf(stderr,"Trajectory self-test failed\n"); return 3; }
    if (export_reference) {
        FILE *f=fopen(argv[3],"wx");
        if (!f) return 5;
        fprintf(f,"time_s");
        for (int field=0;field<3;++field)
            for (int j=0;j<5;++j)
                fprintf(f,",%s_J%d",field==0?"CAD_deg":field==1?"velocity_rad_s":"accel_rad_s2",j+1);
        fprintf(f,"\n");
        for (int k=0;k<=(int)lround(duration/.002);++k) {
            double q[5],v[5],a[5];
            reference(k*.002,q,v,a); fprintf(f,"%.9g",k*.002);
            for (int j=0;j<5;++j) fprintf(f,",%.12g",q[j]*DEG);
            for (int j=0;j<5;++j) fprintf(f,",%.12g",v[j]);
            for (int j=0;j<5;++j) fprintf(f,",%.12g",a[j]);
            fprintf(f,"\n");
        }
        int ok=!ferror(f); if (fclose(f)) ok=0;
        mj_deleteModel(model);
        return ok?0:5;
    }
    data=mj_makeData(model); if (!data) return 2;
    reset();
    if ((view || record) && !load_replay(argv[3])) {
        fprintf(stderr,"Invalid simulation-only state replay\n"); return 3;
    }
    if ((view || record) && !graphics(view)) {
        fprintf(stderr,"Win32 OpenGL initialization failed\n"); return 4;
    }
    Audit audit={0}; audit.finite=1;
    for (int j=0;j<5;++j) audit.qmin[j]=audit.qmax[j]=data->qpos[j]*DEG;
    if (view) {
        LARGE_INTEGER frequency,last,now;
        QueryPerformanceFrequency(&frequency); QueryPerformanceCounter(&last);
        double target=0;
        while (!quit) {
            MSG msg;
            while (PeekMessage(&msg,NULL,0,0,PM_REMOVE)) {
                TranslateMessage(&msg); DispatchMessage(&msg);
            }
            QueryPerformanceCounter(&now);
            double elapsed=(double)(now.QuadPart-last.QuadPart)/frequency.QuadPart;
            last=now;
            if (restart) { target=0; restart=0; paused=0; }
            if (!paused) target+=fmin(elapsed,.05);
            target=fmin(target,duration); /* Hold the final gaze, no loop jump. */
            replay(target);
            render(0); Sleep(5);
        }
    } else if (record) {
        _setmode(_fileno(stdout),_O_BINARY);
        int frames=(int)lround(duration*30);
        for (int frame=0;frame<frames && !quit;++frame) {
            replay((frame+1)/30.0); render(1);
        }
    } else {
        FILE *trace=fopen(argv[4],"wbx");
        if (!trace) return 5;
        uint32_t header[4]={0x594C4832,1,(uint32_t)lround(duration/model->opt.timestep)+1,
            (uint32_t)sizeof(State)};
        if (fwrite(header,sizeof(header),1,trace)!=1) return 5;
        save_state(trace);
        int frames=(int)lround(duration*30);
        sample(&audit);
        for (int frame=0;frame<frames && !quit;++frame) {
            double target=(frame+1)/30.0;
            while (data->time+model->opt.timestep<=target+1e-10) {
                if (!control()) { audit.finite=0; quit=1; break; }
                mj_step(model,data); sample(&audit); save_state(trace);
                if (!audit.finite) { quit=1; break; }
            }
        }
        if (fclose(trace)) return 5;
        if (!report(argv[3],&audit)) return 5;
        if (!audit.finite || audit.warnings || audit.range_violations) return 6;
    }
    free(replay_states);
    if (view || record) {
        mjr_freeContext(&context); mjv_freeScene(&scene);
        wglMakeCurrent(NULL,NULL); wglDeleteContext(gl_context);
        ReleaseDC(window,dc); DestroyWindow(window);
    }
    mj_deleteData(data); mj_deleteModel(model);
    return 0;
}
