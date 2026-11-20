#include "five_dynamics.h"
#include <math.h>
#include <string.h>

typedef struct { float x, y, z; } Vec;
typedef struct { Vec row[3]; } Mat;
typedef struct {
    Vec origin, com;
    Mat fixed, inertia;
    float mass, armature;
} Link;
#include "five_scene.inc"

static Vec add(Vec a, Vec b) { return (Vec){a.x+b.x,a.y+b.y,a.z+b.z}; }
static Vec scale(Vec a, float s) { return (Vec){a.x*s,a.y*s,a.z*s}; }
static float dot(Vec a, Vec b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
static Vec cross(Vec a, Vec b)
{
    return (Vec){a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
static Vec rotate(const Mat *m, Vec v)
{
    return (Vec){dot(m->row[0],v),dot(m->row[1],v),dot(m->row[2],v)};
}
static Vec inverse_rotate(const Mat *m, Vec v)
{
    return add(add(scale(m->row[0],v.x),scale(m->row[1],v.y)),scale(m->row[2],v.z));
}
static Vec rigid_accel(Vec alpha, Vec omega, Vec r)
{
    return add(cross(alpha,r),cross(omega,cross(omega,r)));
}

bool FiveDynamics_Inverse(const float q[5], const float v[5], const float a[5], float tau[5])
{
    if (!q || !v || !a || !tau) { return false; }
    Mat rotation[5];
    Vec force[5], moment[5], omega={0}, alpha={0}, acc={0,-9.81f,0};
    float result[5];
    for (int i=0; i<5; ++i) {
        if (!isfinite(q[i]) || !isfinite(v[i]) || !isfinite(a[i])) { return false; }
        const Link *l=&links[i];
        const float c=cosf(q[i]), s=sinf(q[i]);
        for (int j=0; j<3; ++j) {  /* parent R child = fixed * Ry(-q) */
            const Vec r=l->fixed.row[j];
            rotation[i].row[j]=(Vec){r.x*c+r.z*s,r.y,-r.x*s+r.z*c};
        }
        acc=inverse_rotate(&rotation[i],add(acc,rigid_accel(alpha,omega,l->origin)));
        omega=inverse_rotate(&rotation[i],omega);
        const Vec rate={0,-v[i],0}, accel={0,-a[i],0};
        alpha=add(inverse_rotate(&rotation[i],alpha),add(accel,cross(omega,rate)));
        omega=add(omega,rate);
        force[i]=scale(add(acc,rigid_accel(alpha,omega,l->com)),l->mass);
        moment[i]=add(cross(l->com,force[i]),add(rotate(&l->inertia,alpha),
                          cross(omega,rotate(&l->inertia,omega))));
    }
    for (int i=4; i>=0; --i) {
        result[i]=-moment[i].y+links[i].armature*a[i];
        if (!isfinite(result[i])) { return false; }
        if (i) {
            const Vec f=rotate(&rotation[i],force[i]);
            force[i-1]=add(force[i-1],f);
            moment[i-1]=add(moment[i-1],add(rotate(&rotation[i],moment[i]),cross(links[i].origin,f)));
        }
    }
    memcpy(tau,result,sizeof result);
    return true;
}

bool FiveDynamics_GravityResidual(const float q[5],const float delta[5][4],float tau[5])
{
    if (!q || !delta || !tau) { return false; }
    Mat rotation[5];
    Vec force[5],moment[5],acc={0,-9.81f,0};
    float result[5];
    for (int i=0;i<5;++i) {
        if (!isfinite(q[i])) { return false; }
        for (int k=0;k<4;++k) {
            if (!isfinite(delta[i][k])) { return false; }
        }
        const float c=cosf(q[i]),s=sinf(q[i]);
        for (int j=0;j<3;++j) {
            const Vec r=links[i].fixed.row[j];
            rotation[i].row[j]=(Vec){r.x*c+r.z*s,r.y,-r.x*s+r.z*c};
        }
        acc=inverse_rotate(&rotation[i],acc);
        const Vec first_moment={delta[i][1],delta[i][2],delta[i][3]};
        force[i]=scale(acc,delta[i][0]);
        moment[i]=cross(first_moment,acc);
    }
    for (int i=4;i>=0;--i) {
        result[i]=-moment[i].y;
        if (!isfinite(result[i])) { return false; }
        if (i) {
            const Vec f=rotate(&rotation[i],force[i]);
            force[i-1]=add(force[i-1],f);
            moment[i-1]=add(moment[i-1],add(rotate(&rotation[i],moment[i]),cross(links[i].origin,f)));
        }
    }
    memcpy(tau,result,sizeof result);
    return true;
}
