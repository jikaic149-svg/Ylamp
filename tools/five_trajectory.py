"""Bounded Ruckig state-point payload/reference; no hardware or implicit execution."""
import ctypes as ct
import math
import os
from pathlib import Path
import re
import struct

FIELDS = ("position_deg", "velocity_rad_s", "acceleration_rad_s2")
CAPACITY = 32
DEFAULT_NATIVE = Path(__file__).resolve().parents[1]/"build/v12-lookaround-relative-host-20261127/libfive_ruckig_native.dll"


def reference_friction(q_cad_rad, velocity_rad_s):
    """Current TOTAL F once, for offline active-demand checks, not static truth.

    Same reference sign/load proxies as production C. Independent NumPy
    evaluation is checked against that C by the stutter reproduction harness.
    """
    import numpy as np
    from five_friction_map import load_features
    from five_identify import HW, SCENE
    q, v = np.asarray(q_cad_rad, float), np.asarray(velocity_rad_s, float)
    scalar = q.ndim == 1
    q, v = np.atleast_2d(q), np.atleast_2d(v)
    if q.shape != v.shape or q.shape[1] != 5 or not np.isfinite(q).all() or not np.isfinite(v).all():
        raise ValueError("Expected matching finite N by 5 CAD positions and reference velocities")
    text = (Path(__file__).resolve().parents[1]/"config/five_mit.c").read_text(encoding="utf-8")
    def table(name, shape):
        body = re.search(r"\b"+name+r"(?:\[[^\]]+\])+\s*=\s*\{(.*?)\};", text, re.S)
        if body is None:
            raise ValueError("Missing friction table: "+name)
        values = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", body[1])
        return np.asarray(values, dtype=np.float32).astype(float).reshape(shape)
    cfg = table("g_five_speed_friction", (5, 4))
    lo, hi = table("g_five_speed_friction_range_rad_s", (2,))
    k16, k48 = table("g_five_speed_friction_knots_rad_s", (2,))
    load = table("g_five_speed_friction_j2", (2,))
    z, reaches, nominal_g = load_features(q)
    bend = z[:, :, 2]*np.array([SCENE[i:, -2].sum()*9.81 for i in range(5)])*reaches
    speed = np.clip(abs(v), lo, hi)
    loga = (cfg[:, 0]+cfg[:, 1]*np.log(speed/lo)+
            cfg[:, 2]*np.maximum(0, np.log(speed/k16))+
            cfg[:, 3]*np.maximum(0, np.log(speed/k48)))
    loga[:, 1] += load[0]*bend[:, 1]+load[1]*abs(nominal_g[:, 1])
    result = np.exp(loga)*np.tanh(v/HW[:, 4])
    if not np.isfinite(result).all():
        raise ValueError("Nonfinite friction demand")
    return result[0] if scalar else result


def normalize_points(points):
    points = list(points)
    if not 1 <= len(points) <= CAPACITY:
        raise ValueError(f"Expected 1..{CAPACITY} complete future Ruckig states")
    checked, previous = [], 0
    for point in points:
        at = point["at_ms"]
        if (not isinstance(at, int) or isinstance(at, bool) or
                not previous+2 <= at <= 120000):
            raise ValueError("Knot milliseconds must increase within the original duration")
        row = dict(at_ms=at)
        for name in FIELDS:
            values = [float(v) for v in point[name]]
            if len(values) != 5 or not all(map(math.isfinite, values)):
                raise ValueError("Expected five finite q/v/a components")
            try:
                row[name] = list(struct.unpack("<5f", struct.pack("<5f", *values)))
            except (OverflowError, struct.error) as exc:
                raise ValueError("Knot does not fit finite float32") from exc
            if not all(map(math.isfinite,row[name])):
                raise ValueError("Knot overflows finite float32")
        checked.append(row)
        previous = at
    if previous < 100 or any(v != 0 for name in FIELDS[1:] for v in checked[-1][name]):
        raise ValueError("Original minimum duration and zero final v/a are required")
    return checked


def pack_point(point):
    """Canonical 64-byte LE point; caller passes an already normalized knot."""
    return struct.pack("<I15f", point["at_ms"], *(v for name in FIELDS for v in point[name]))

def relative_points(points, origin_deg):
    """Same one-time KEY rebase as production; not a moving anchor or clamp."""
    points=normalize_points(points)
    origin=[float(v) for v in origin_deg]
    if len(origin)!=5 or not all(map(math.isfinite,origin)):
        raise ValueError("Expected five finite captured origin degrees")
    first=points[0]["position_deg"]
    return normalize_points([dict(p,position_deg=[q+(x-base) for q,x,base in
        zip(origin,p["position_deg"],first)]) for p in points])


def trajectory_hash(points):
    value = 2166136261
    for point in normalize_points(points):
        for byte in pack_point(point):
            value = ((value ^ byte)*16777619) & 0xffffffff
    return value


class _Point(ct.Structure):
    _fields_ = [("at_ms",ct.c_uint32)]+[(name,ct.c_float*5) for name in FIELDS]


class RuckigTrajectory:
    """Native SAME Mode/config/Ruckig as MCU. Quantized only to its 1ms clock.

    One caller-owned profile set per object, no shared mutable planner. Building
    this library never enables a motor; a plan object never opens a USB/SWD link.
    """
    def __init__(self, origin_deg, points, library=DEFAULT_NATIVE, origin_velocity_rad_s=None):
        import numpy as np
        self.np = np
        self.points = normalize_points(points)
        origin = np.asarray(origin_deg, float)
        if origin.shape != (5,) or not np.isfinite(origin).all():
            raise ValueError("Expected finite captured origin")
        velocity=np.zeros(5) if origin_velocity_rad_s is None else np.asarray(origin_velocity_rad_s,float)
        if velocity.shape!=(5,) or not np.isfinite(velocity).all():
            raise ValueError("Expected five finite captured velocities")
        self.library = Path(library).resolve()
        self._dll_directory = os.add_dll_directory("C:/msys64/ucrt64/bin") if os.name=="nt" else None
        self.dll = ct.CDLL(str(self.library))
        self.dll.FiveRuckig_StateBytes.restype = ct.c_size_t
        self.dll.FiveRuckig_Version.restype = ct.c_uint32
        if self.dll.FiveRuckig_Version()!=0x594c5302:
            raise ValueError("Native trajectory ABI mismatch")
        dp = ct.POINTER(ct.c_double);up = ct.POINTER(ct.c_uint32)
        self.dll.FiveRuckig_Prepare.argtypes = [ct.c_void_p,dp,dp,ct.POINTER(_Point),
                                               ct.c_uint,up,up]
        self.dll.FiveRuckig_Prepare.restype = ct.c_bool
        self.dll.FiveRuckig_SampleBatch.argtypes = [ct.c_void_p,up,ct.c_size_t,dp,dp,dp]
        self.dll.FiveRuckig_SampleBatch.restype = ct.c_bool
        self.dll.FiveRuckig_Limits.argtypes = [dp]
        self.dll.FiveRuckig_Limits.restype = None
        self.state = (ct.c_double*((self.dll.FiveRuckig_StateBytes()+7)//8))()
        data = (_Point*len(self.points))()
        for target,p in zip(data,self.points):
            target.at_ms=p["at_ms"]
            for name in FIELDS:
                getattr(target,name)[:]=p[name]
        duration=ct.c_uint32();knots=(ct.c_uint32*CAPACITY)()
        if not self.dll.FiveRuckig_Prepare(self.state,(ct.c_double*5)(*origin),
                (ct.c_double*5)(*velocity),data,len(data),ct.byref(duration),knots):
            raise ValueError("Ruckig state/limits/travel infeasible; no motion")
        self.duration_ms=duration.value
        self.knot_ms=list(knots)[:len(data)]
        self.times=np.r_[0,self.knot_ms]*.001
        self.limits=np.empty((5,3),float)
        self.dll.FiveRuckig_Limits(self.limits.ctypes.data_as(dp))

    def sample(self, elapsed_s):
        np = self.np
        scalar = np.ndim(elapsed_s) == 0
        elapsed = np.atleast_1d(np.asarray(elapsed_s, float))
        if elapsed.ndim!=1 or not np.isfinite(elapsed).all():
            raise ValueError("Expected finite one-dimensional trajectory times")
        ms=np.ascontiguousarray(np.rint(np.clip(elapsed,0,self.times[-1])*1000),dtype=np.uint32)
        q,v,a=[np.empty((len(ms),5),float) for _ in range(3)]
        dp=ct.POINTER(ct.c_double)
        if not self.dll.FiveRuckig_SampleBatch(self.state,ms.ctypes.data_as(ct.POINTER(ct.c_uint32)),
                len(ms),*(x.ctypes.data_as(dp) for x in (q,v,a))):
            raise ValueError("Native Ruckig sample rejected")
        return (q[0], v[0], a[0]) if scalar else (q, v, a)
