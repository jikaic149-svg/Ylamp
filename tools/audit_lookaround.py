"""Finite offline preset audit using the same production native Ruckig path."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
from five_trajectory import RuckigTrajectory, normalize_points, relative_points, reference_friction, trajectory_hash
from five_happy_motion import Geometry, CAPS
from five_identify import HW
from five_gravity_residual_report import support_gravity
from five_inertia_identify import nominal_torque


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    for name in ("preset","firmware","native","report"):
        cli.add_argument("--"+name,type=Path,required=True)
    cli.add_argument("--origin",type=float,nargs=5,help="Explicit software-degree KEY pose for this finite audit")
    args=cli.parse_args()
    template=normalize_points(json.loads(args.preset.read_text(encoding="utf-8"))["points"])
    anchor=np.array(args.origin if args.origin else template[0]["position_deg"])
    points=relative_points(template,anchor)
    curve=RuckigTrajectory(anchor,points,library=args.native)
    q,v,a=curve.sample(np.arange(0,curve.duration_ms+1,2)*.001)
    assert np.all(np.max(abs(v),axis=0)<=curve.limits[:,0]+1e-5)
    assert np.all(np.max(abs(a),axis=0)<=curve.limits[:,1]+1e-5)
    assert np.all(np.max(abs(np.diff(a,axis=0)/.002),axis=0)<=curve.limits[:,2]+1e-3)
    def torque(q,v,a):
        cad=np.deg2rad(q+HW[:,6]);zero=np.zeros_like(q);g=support_gravity(cad,clip=False)
        demand=nominal_torque(cad,v,a)-nominal_torque(cad,zero,zero)+g+reference_friction(cad,v)
        assert np.all(np.max(abs(g),axis=0)<=HW[:,5])
        assert np.all(np.max(abs(demand),axis=0)<=CAPS)
        return np.max(abs(demand),axis=0)
    peaks=torque(q,v,a);entry_peak=np.zeros(5);entries=[];accepted=0
    for signs in itertools.product((-1,1),repeat=5):
        sign=np.array(signs)
        for origin,velocity in ((anchor+sign*2.95,None),(anchor+sign*2.5,sign*.02)):
            entry=RuckigTrajectory(origin,relative_points(template,origin),library=args.native,origin_velocity_rad_s=velocity)
            eq,ev,ea=entry.sample(np.linspace(0,entry.times[1],101))
            accepted+=1;entry_peak=np.maximum(entry_peak,torque(eq,ev,ea))
            entries.extend(eq[::25]+HW[:,6])
    with Geometry() as geometry:
        nominal=geometry.check([[x.tolist(),x.tolist()] for x in q[::100]+HW[:,6]])
        approach=geometry.check([[x.tolist(),x.tolist()] for x in entries])
        model=geometry.source
    if nominal["witness"] is not None or approach["witness"] is not None:
        raise ValueError("Finite CAD sample has a collision witness")
    report=dict(firmware_sha256=hashlib.sha256(args.firmware.read_bytes()).hexdigest(),
        preset_sha256=hashlib.sha256(args.preset.read_bytes()).hexdigest(),
        native_library_sha256=hashlib.sha256(args.native.read_bytes()).hexdigest(),
        count=len(points),template_fnv1a32=trajectory_hash(template),rebased_fnv1a32=trajectory_hash(points),
        captured_origin_deg=anchor.tolist(),nominal_duration_ms=curve.duration_ms,
        reference_samples=len(q),model=model,nominal_CAD=nominal,entry_CAD=approach,
        sampled_entry_cases=64,rebased_entry_cases=accepted,
        nominal_zero_tracking_error_peak_nm=peaks.tolist(),entry_zero_tracking_error_peak_nm=entry_peak.tolist(),
        continuous_collision_proof=False,hardware_motion_sent=False)
    with args.report.open("x",encoding="utf-8") as output:
        json.dump(report,output,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2))


if __name__=="__main__":
    main()
