"""Close the firmware's actual quantized MIT packets around the pinned local
MuJoCo model. No hardware. Requires the project's existing simulation tools."""
import argparse
import json
import subprocess
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from export_five_profile import ROOT, SCENE_SHA, pinned


def decode(word, axis):
    b = bytes.fromhex(word)
    p, v, kp = int.from_bytes(b[:2], "big"), b[2]*16+(b[3]>>4), (b[3]&15)*256+b[4]
    kd, torque = b[5]*16+(b[6]>>4), (b[6]&15)*256+b[7]
    vmax, tmax = (10, 28) if axis < 2 else (30, 10)
    return ((2*p/65535-1)*12.5, (2*v/4095-1)*vmax, (2*torque/4095-1)*tmax,
            kp*500/4095, kd*5/4095)


def request(proc, cmd, *arrays):
    line = " ".join(format(float(i), ".17g") for i in np.concatenate(arrays))
    proc.stdin.write(cmd+" "+line+"\n")
    proc.stdin.flush()
    result = proc.stdout.readline().strip()
    if not result or result == "REJECT":
        raise RuntimeError(f"{cmd} rejected: {line} -> {result!r}")
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sim-dir", required=True, type=Path)
    p.add_argument("--exe", required=True, type=Path)
    p.add_argument("--output", type=Path, help="Save a new simulation report (will not overwrite)")
    args = p.parse_args()
    sys.path.insert(0, str(ROOT / "tools"))
    from dm_monitor import JOINT_REFERENCES
    from five_identify import HW, WORKSPACE_CAD
    sys.path.insert(0, str(args.sim_dir.resolve()))
    import prepare
    import tune_mit
    root = ET.fromstring(pinned(prepare.SCENE, SCENE_SHA))
    dll = prepare.native_library()
    expected = np.array(tune_mit.gains(root))
    cad = np.radians(HW[:, 6])
    raw0 = np.array([JOINT_REFERENCES[j].zero_rad for j in range(1, 6)])
    zero, span = np.zeros(5), np.array([0, -3, 4, -8, 2])
    center = WORKSPACE_CAD-HW[:, 6]
    yaw_a, yaw_b = np.array([30., 0., 0., 0., 0.]), np.array([36., -1.5, 2., -3., 1.])
    dt = float(root.find("option").get("timestep"))
    result = []
    with subprocess.Popen([str(args.exe.resolve()), "five-wire"], text=True, bufsize=1,
                          stdin=subprocess.PIPE, stdout=subprocess.PIPE) as proc:
        try:
            for old_start, old_goal in ((zero, span), (zero, -span), (yaw_a, yaw_b), (yaw_b, yaw_a)):
                start, goal = old_start+center, old_goal+center
                T = 2
                delta = goal-start
                assert request(proc, "P", start, goal, [T]) == "OK"
                def send(t, software, v):
                    words = request(proc, "S", [t], software, raw0+np.radians(software), v).split()
                    assert len(words)==5
                    return np.array([decode(w,j) for j,w in enumerate(words)])
                first = send(0,start,zero)
                gains = first[:,3:]
                assert np.all(abs(expected-gains)<=[500/4095+1e-9,5/4095+1e-9])
                sim = tune_mit.Simulation(dll,root,gains.tolist())
                try:
                    sim.q[:] = cad+np.radians(start)
                    dll.mj_setState(sim.model,sim.data,sim.q,1<<1)
                    dll.mj_forward(sim.model,sim.data)
                    err = np.zeros(5)
                    peak_j1_rate = 0.
                    steps = round((T+.4)/dt)
                    for n in range(steps):
                        dll.mj_getState(sim.model,sim.data,sim.q,1<<1)
                        dll.mj_getState(sim.model,sim.data,sim.v,1<<2)
                        peak_j1_rate = max(peak_j1_rate, abs(sim.v[0]))
                        wire = send(min(n*dt,T), np.degrees(np.array(sim.q)-cad), list(sim.v))
                        assert np.array_equal(gains,wire[:,3:])
                        sim.command(cad+wire[:,0]-raw0,wire[:,1],wire[:,2])
                        dll.mj_step(sim.model,sim.data)
                        dll.mj_getState(sim.model,sim.data,sim.q,1<<1)
                        t = min((n+1)*dt/T,1)
                        actual = np.degrees(np.array(sim.q)-cad)
                        err = np.maximum(err,abs(actual-(start+delta*t**3*(10-15*t+6*t*t))))
                    assert np.all(err<.20), err
                    if delta[0]:
                        assert peak_j1_rate > abs(np.radians(delta[0]))/T
                    result.append(dict(start_deg=start.tolist(), goal_deg=goal.tolist(), frames=steps*5,
                                       peak_error_deg=err.tolist(), final_error_deg=(actual-goal).tolist(),
                                       peak_abs_j1_rate_rad_s=peak_j1_rate))
                finally:
                    sim.close()
        finally:
            proc.stdin.close()
        if proc.wait()!=0:
            raise RuntimeError("HAL-fake transport failed")
    report = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        with args.output.open("x", encoding="utf-8") as f:
            f.write(report+"\n")
    print(report)


if __name__=="__main__":
    main()
