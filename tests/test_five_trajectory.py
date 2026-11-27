"""No hardware: Ruckig through-states, finite payload, complete upload, one commit."""
import math
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import Mock

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from dm_serial import encode, TRAJ_BEGIN, TRAJ_POINT, TRAJ_START, MOTION_QUERY, TRAJ_ACK, TRAJ_RESULT, MOTION_STATE
from five_control import Board, CONTROL_VERSION, TRAJECTORY_VERSION, MOTION_BUNDLE_VERSION, RPCRejected, ReplyTimeout
from five_trajectory import normalize_points, pack_point, trajectory_hash, RuckigTrajectory, reference_friction
from five_identify import REFS
from usb_monitor import Stream


def points():
    return [dict(at_ms=1000,position_deg=[1,2,3,4,5],
                 velocity_rad_s=[.01]*5,acceleration_rad_s2=[.002]*5),
            dict(at_ms=2000,position_deg=[2,3,4,5,6],
                 velocity_rad_s=[0]*5,acceleration_rad_s2=[0]*5)]


def motion_state(prefix, plan_id, start_id, checksum, flags=2):
    data = bytearray(180)
    data[:12] = prefix
    data[12:21] = bytes((0, 2, 31, 31, 31, 0, flags, 2, 2))
    struct.pack_into("<8I", data, 24, 1235, 0, 1, plan_id, start_id, checksum, 1234, 2000)
    struct.pack_into("<5f5f5f2I", data, 56, *range(15), 1000, 2000)
    return bytes(data)


class MotionBundleTests(unittest.TestCase):
    def board(self):
        board = Board(Mock())
        board.latest = {"tick_ms": 1234}
        board.at = math.inf  # Fixed fresh clock; no serial device needed.
        board.motion_bundle = True
        return board

    def test_connect_capability_fallback_requires_explicit_unsupported(self):
        for reply, supported, error in (
                (MOTION_BUNDLE_VERSION, True, None),
                (RPCRejected("field247", 1), False, None),
                (RPCRejected("field247", 3), False, RPCRejected),
                (ReplyTimeout(), False, ReplyTimeout),
                (0x594c4201, True, None),
                (0x594c4203, False, ValueError)):
            with self.subTest(reply=reply):
                board = self.board()
                board.motion_bundle = False
                board.exchange = Mock()
                board.rpc = Mock(side_effect=[CONTROL_VERSION, reply])
                if error is None:
                    board.connect()
                else:
                    with self.assertRaises(error):
                        board.connect()
                self.assertEqual(board.motion_bundle, supported)

    def test_upload_uses_one_request_per_point_and_checks_whole_hash(self):
        knots = normalize_points(points())
        checksum = trajectory_hash(knots)
        for stored in (checksum, checksum ^ 1):
            with self.subTest(stored=stored):
                board = self.board()
                def ack(kind, payload, expected, prefix):
                    self.assertEqual(expected, TRAJ_ACK)
                    self.assertEqual(prefix, payload[:12])
                    self.assertEqual(struct.unpack_from("<I", payload, 12)[0], 1234)
                    if kind == TRAJ_BEGIN:
                        self.assertEqual(payload[16:], b"\x02")
                        count = 0
                    else:
                        self.assertEqual(kind, TRAJ_POINT)
                        plan, index = struct.unpack_from("<IB", payload, 16)
                        self.assertEqual(plan, 1)
                        self.assertEqual(payload[21:], pack_point(knots[index]))
                        count = index+1
                    return prefix+struct.pack("<4B2I", 0, count, 2, 1, 1, stored)
                board.exchange = Mock(side_effect=ack)
                if stored == checksum:
                    self.assertEqual(board.trajectory_upload(points()), knots)
                else:
                    with self.assertRaisesRegex(ValueError, "hash"):
                        board.trajectory_upload(points())
                    with self.assertRaisesRegex(ValueError, "No verified"):
                        board.trajectory_start()
                self.assertEqual([call.args[0] for call in board.exchange.call_args_list],
                                 [TRAJ_BEGIN, TRAJ_POINT, TRAJ_POINT])

    def test_status_one_query_and_does_not_advance_write_id(self):
        board = self.board()
        board.ident = 6
        def exchange(kind, payload, expected, prefix):
            self.assertEqual((kind, expected, payload, len(payload)),
                             (MOTION_QUERY, MOTION_STATE, prefix, 12))
            return motion_state(prefix, 4, 6, 123)
        board.exchange = Mock(side_effect=exchange)
        status = board.status()
        self.assertEqual(status["tick_ms"], 1235)
        self.assertEqual(status["model_gravity_nm"], list(range(5)))
        self.assertIs(status["telemetry"], board.latest)
        self.assertEqual((board.ident, board.query_ident), (6, 1))
        board.exchange.assert_called_once()

    def test_start_once_direct_or_lost_reply(self):
        for mode in ("direct", "lost", "wrong_start_id", "wrong_plan_id", "wrong_hash",
                     "inactive", "no_session", "partial_write"):
            with self.subTest(mode=mode):
                board = self.board()
                board.ident, board._trajectory_plan = 9, (4, 0x12345678)
                def exchange(kind, payload, expected, prefix):
                    if kind == TRAJ_START:
                        self.assertEqual(expected, TRAJ_RESULT)
                        self.assertEqual(struct.unpack_from("<IIII", payload, 8),
                                         (10, 1234, 4, 0x12345678))
                        if mode == "partial_write":
                            raise OSError("partial write")
                        if mode != "direct":
                            raise ReplyTimeout()
                    else:
                        self.assertEqual((kind, expected, payload), (MOTION_QUERY, MOTION_STATE, prefix))
                    if mode == "no_session":
                        return prefix+b"\x03"+bytes(167)
                    return motion_state(prefix, 5 if mode == "wrong_plan_id" else 4,
                                        11 if mode == "wrong_start_id" else 10,
                                        0 if mode == "wrong_hash" else 0x12345678,
                                        1 if mode == "inactive" else 2)
                board.exchange = Mock(side_effect=exchange)
                if mode in ("direct", "lost"):
                    path = board.trajectory_start()
                    self.assertEqual((path["start_ms"], path["knot_ms"]), (1234, [1000, 2000]))
                    self.assertEqual(path["origin_velocity_rad_s"], list(range(10, 15)))
                else:
                    with self.assertRaises((RuntimeError, OSError)):
                        board.trajectory_start()
                with self.assertRaisesRegex(ValueError, "No verified"):
                    board.trajectory_start()
                expected = [TRAJ_START] if mode in ("direct", "partial_write") else [TRAJ_START, MOTION_QUERY]
                self.assertEqual([call.args[0] for call in board.exchange.call_args_list], expected)
                self.assertEqual(board.ident, 10)

    def test_stream_stitches_split_and_rejects_bad_crc_and_size(self):
        stream = Stream()
        reply = motion_state(b"abcdefgh"+struct.pack("<I", 9), 4, 9, 123)
        frame = encode(MOTION_STATE, reply)
        corrupt = frame[:-1]+bytes((frame[-1]^1,))
        self.assertEqual(list(stream.messages(corrupt+frame[:64])), [])
        self.assertEqual(list(stream.messages(frame[64:128])), [])
        self.assertEqual(list(stream.messages(frame[128:])), [(MOTION_STATE, reply)])
        with self.assertRaises(ValueError):
            encode(MOTION_STATE, reply[:-1])
        wrong_size = frame[:4]+bytes((179,))+frame[5:]
        self.assertEqual(list(stream.messages(wrong_size+frame)), [(MOTION_STATE, reply)])


class TrajectoryTests(unittest.TestCase):
    def test_total_friction_reference_sign_zero_and_shape(self):
        q=np.radians([117,36,-84,113,14])
        velocity=np.array([.08,-.16,.32,-.48,.8])
        positive=reference_friction(q,velocity)
        np.testing.assert_allclose(reference_friction(q,-velocity),-positive,atol=1e-12)
        np.testing.assert_array_equal(reference_friction(q,np.zeros(5)),np.zeros(5))
        self.assertTrue(np.all(positive*velocity>0))
        batch=reference_friction(np.tile(q,(2,1)),np.vstack((velocity,-velocity)))
        np.testing.assert_allclose(batch,np.vstack((positive,-positive)),atol=1e-12)
        with self.assertRaises(ValueError):reference_friction(q,[0]*4)
        with self.assertRaises(ValueError):reference_friction(q,[np.nan]*5)

    def test_complete_upload_never_commits_or_enables(self):
        knots=normalize_points(points());count=0
        def rpc(field,scope=0,value=0):
            nonlocal count
            if field==248:return TRAJECTORY_VERSION
            if field==33:
                count+=1;return count
            if field==36:return 2|(2<<8)|(1<<16)
            if field==38:return 2000
            if field==39:return trajectory_hash(knots)
            return 0
        board=Board(Mock());board.rpc=Mock(side_effect=rpc)
        self.assertEqual(board.trajectory_upload(knots),knots)
        fields=[call.args[0] for call in board.rpc.call_args_list]
        self.assertEqual(fields.count(34),1)
        self.assertEqual(fields.count(33),2)
        self.assertEqual(sum(fields.count(i) for i in (30,31,32)),30)
        self.assertNotIn(35,fields)
        self.assertTrue(set(fields)<={248,34,30,31,32,33,36,37,38,39})

    def test_invalid_payload_is_rejected_before_any_io(self):
        for bad in ([],points()*40,
                    [dict(points()[0],position_deg=[math.nan]*5)],
                    [dict(points()[1],velocity_rad_s=[.1]*5)]):
            board=Board(Mock());board.rpc=Mock()
            with self.assertRaises(ValueError):board.trajectory_upload(bad)
            board.rpc.assert_not_called()

    def test_hash_mismatch_never_commits(self):
        board=Board(Mock())
        def rpc(field,**kwargs):
            if field==248:return TRAJECTORY_VERSION
            if field==33:return 1 if kwargs["value"]==1000 else 2
            if field==36:return 2|(2<<8)|(1<<16)
            return 0
        board.rpc=Mock(side_effect=rpc)
        with self.assertRaisesRegex(ValueError,"hash"):board.trajectory_upload(points())
        self.assertNotIn(35,[call.args[0] for call in board.rpc.call_args_list])

    def test_commit_is_one_request_and_reads_real_origin(self):
        board=Board(Mock())
        def rpc(field,*args,**kwargs):
            return {36:2|(2<<8)|(1<<17),37:1234,38:2000,39:99}.get(field,0)
        board.rpc=Mock(side_effect=rpc)
        result=board.trajectory_start()
        self.assertTrue(result["active"]);self.assertEqual(result["start_ms"],1234)
        self.assertEqual([call.args[0] for call in board.rpc.call_args_list].count(35),1)
        board.rpc=Mock(side_effect=TimeoutError("unknown outcome"))
        with self.assertRaises(TimeoutError):board.trajectory_start()
        self.assertEqual(board.rpc.call_count,1)  # Never replay.

    def test_reference_common_qva_and_nonzero_knot_velocity(self):
        origin=np.rad2deg(np.mean(REFS[:,1:],axis=1)-REFS[:,0])
        knots=normalize_points([dict(p,position_deg=(origin+index+1).tolist())
                                for index,p in enumerate(points())])
        curve=RuckigTrajectory(origin,knots)
        for index,at in enumerate(curve.times[1:]):
            q,v,a=curve.sample(at)
            np.testing.assert_allclose(q,knots[index]["position_deg"],atol=1e-12)
            np.testing.assert_allclose(v,knots[index]["velocity_rad_s"],atol=1e-12)
            np.testing.assert_allclose(a,knots[index]["acceleration_rad_s2"],atol=1e-12)
            if index==0:
                left=curve.sample(at-.001);right=curve.sample(at+.001)
                self.assertLess(np.max(abs(left[1]-right[1])),.005)
                self.assertTrue(np.all(v>.009))
        self.assertTrue(np.array_equal(curve.sample(20)[1],np.zeros(5)))

    def test_retiming_and_finite_jerk_bounds(self):
        origin=np.rad2deg(np.mean(REFS[:,1:],axis=1)-REFS[:,0])
        knot=dict(at_ms=100,position_deg=(origin+15).tolist(),
                  velocity_rad_s=[0]*5,acceleration_rad_s2=[0]*5)
        curve=RuckigTrajectory(origin,[knot])
        self.assertGreater(curve.duration_ms,100)
        self.assertEqual(curve.duration_ms%2,0)
        times=np.arange(0,curve.duration_ms+1,2)*.001
        _,v,a=curve.sample(times)
        self.assertTrue(np.all(np.max(abs(v),axis=0)<=curve.limits[:,0]+1e-6))
        self.assertTrue(np.all(np.max(abs(a),axis=0)<=curve.limits[:,1]+1e-6))
        self.assertTrue(np.all(np.max(abs(np.diff(a,axis=0)),axis=0)/.002<=curve.limits[:,2]+1e-3))

    def test_multiple_caller_owned_profiles_do_not_alias(self):
        origin=np.rad2deg(np.mean(REFS[:,1:],axis=1)-REFS[:,0])
        def curve(delta):
            return RuckigTrajectory(origin,[dict(at_ms=1000,
                position_deg=(origin+delta).tolist(),velocity_rad_s=[0]*5,
                acceleration_rad_s2=[0]*5)])
        first=curve(1)
        expected=first.sample(.5)[0].copy()
        second=curve(2)
        np.testing.assert_array_equal(first.sample(.5)[0],expected)
        self.assertGreater(np.max(abs(second.sample(.5)[0]-expected)),.1)

    def test_start_captures_velocity_and_rejects_segment_interior_overshoot(self):
        origin=np.rad2deg(np.mean(REFS[:,1:],axis=1)-REFS[:,0])
        knot=dict(at_ms=1000,position_deg=(origin+2).tolist(),
                  velocity_rad_s=[0]*5,acceleration_rad_s2=[0]*5)
        curve=RuckigTrajectory(origin,[knot],origin_velocity_rad_s=[.02]*5)
        np.testing.assert_allclose(curve.sample(0)[1],[.02]*5,atol=1e-8)
        # Both endpoints remain in the original target inset, but the current
        # outward velocity needs more stopping travel than remains available.
        upper=np.rad2deg(REFS[:,2]-REFS[:,0])-1
        origin[0]=upper[0]-.001
        target=origin.copy();target[0]-=.01
        knot=dict(knot,position_deg=target.tolist())
        with self.assertRaisesRegex(ValueError,"infeasible"):
            RuckigTrajectory(origin,[knot],origin_velocity_rad_s=[.5,0,0,0,0])


if __name__=="__main__":
    unittest.main()
