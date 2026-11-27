"""No hardware: denser native Ruckig preset and independent storage protocol."""
import json
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import Mock

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from dm_serial import (encode, PRESET_BEGIN, PRESET_POINT, PRESET_SAVE, PRESET_QUERY,
                       PRESET_ACK, PRESET_STATE, PRESET_DATA, MOTION_STATE)
from five_control import Board, ReplyTimeout, decode_motion
from five_trajectory import RuckigTrajectory, normalize_points, trajectory_hash, relative_points
from lookaround_preset import DEFAULT, decode, provision
from usb_monitor import Stream


class LookaroundTests(unittest.TestCase):
    def test_current_origin_keeps_shape_and_returns_to_key_pose(self):
        template=normalize_points(json.loads(DEFAULT.read_text(encoding="utf-8"))["points"])
        origin=[129.6334797296,7.2000439136,9.6473177687,114.8362983645,3.9014450350]
        points=relative_points(template,origin)
        np.testing.assert_allclose(points[0]["position_deg"],origin,atol=8e-6)
        np.testing.assert_allclose(points[-1]["position_deg"],origin,atol=8e-6)
        for before,after in zip(template,points):
            self.assertEqual(before["at_ms"],after["at_ms"])
            self.assertEqual(before["velocity_rad_s"],after["velocity_rad_s"])
            self.assertEqual(before["acceleration_rad_s2"],after["acceleration_rad_s2"])
            np.testing.assert_allclose(np.array(after["position_deg"])-points[0]["position_deg"],
                np.array(before["position_deg"])-template[0]["position_deg"],atol=1.6e-5)
        curve=RuckigTrajectory(origin,points)
        self.assertEqual(curve.duration_ms,16000)
        q,v,a=curve.sample(np.arange(0,16001,2)*.001)
        np.testing.assert_allclose(q[0],origin,atol=8e-6)
        np.testing.assert_allclose(q[-1],origin,atol=8e-6)
        self.assertTrue(np.all(np.max(abs(v),axis=0)<=curve.limits[:,0]+1e-5))
        self.assertTrue(np.all(np.max(abs(a),axis=0)<=curve.limits[:,1]+1e-5))

    def test_dense_native_profiles(self):
        points=normalize_points(json.loads(DEFAULT.read_text(encoding="utf-8"))["points"])
        self.assertEqual(len(points),32)
        curve=RuckigTrajectory(points[0]["position_deg"],points)
        q,v,a=curve.sample(np.arange(0,curve.duration_ms+1,2)*.001)
        self.assertTrue(np.isfinite(q).all())
        self.assertTrue(np.all(np.max(abs(v),axis=0)<=curve.limits[:,0]+1e-5))
        self.assertTrue(np.all(np.max(abs(a),axis=0)<=curve.limits[:,1]+1e-5))
        self.assertTrue(np.all(np.max(abs(np.diff(a,axis=0)/.002),axis=0)<=curve.limits[:,2]+1e-3))
        for index,at in enumerate(curve.knot_ms[:-1]):
            _,actual_v,actual_a=curve.sample(at*.001)
            np.testing.assert_allclose(actual_v,points[index]["velocity_rad_s"],atol=1e-8)
            np.testing.assert_allclose(actual_a,points[index]["acceleration_rad_s2"],atol=1e-8)
        np.testing.assert_allclose(q[-1],points[-1]["position_deg"],atol=1e-7)

    def test_large_motion_state_and_storage_stream(self):
        payload=bytearray(244);payload[12:21]=bytes((0,2,31,31,31,0,2,32,32))
        struct.pack_into("<32I",payload,116,*range(32))
        _,path=decode_motion(payload)
        self.assertEqual(path["knot_ms"],list(range(32)))
        for kind,length in ((MOTION_STATE,244),(PRESET_ACK,40),(PRESET_STATE,40),(PRESET_DATA,244)):
            data=bytes(length);frame=encode(kind,data);stream=Stream()
            self.assertEqual(list(stream.messages(frame[:64])),[] if len(frame)>64 else [(kind,data)])
            if len(frame)>64:
                self.assertEqual(list(stream.messages(frame[64:])),[(kind,data)])

    def test_save_is_not_motion_and_missing_ack_only_queries(self):
        points=normalize_points(json.loads(DEFAULT.read_text(encoding="utf-8"))["points"])
        checksum=trajectory_hash(points)
        for lost in (False,True):
            board=Board(Mock());board.latest={"tick_ms":1000};board.at=float("inf")
            board.rpc=Mock(return_value=0x594c5001)
            board.pump=Mock()
            count=0;saved=False;preset_id=0
            def exchange(kind,payload,expected,prefix):
                nonlocal count,saved,preset_id
                flags=1|4;seq=7;loaded=0
                if kind==PRESET_QUERY:
                    flags=1|2 if saved else 1;seq=8 if saved else 7;loaded=32 if saved else 0
                elif kind==PRESET_BEGIN:
                    preset_id=struct.unpack_from("<I",payload,8)[0]
                    self.assertEqual(payload[16],32);self.assertEqual(expected,PRESET_ACK)
                elif kind==PRESET_POINT:
                    self.assertEqual(payload[20],count);count+=1
                elif kind==PRESET_SAVE:
                    self.assertEqual(count,32);self.assertEqual(struct.unpack_from("<I",payload,20)[0],checksum)
                    saved=True;flags=1|8
                    if lost: raise ReplyTimeout()
                else: self.fail("Unexpected motion command")
                reply=prefix+struct.pack("<4B6I",0,count,32,loaded,flags,seq,
                    checksum if saved else 0,checksum,preset_id,0)
                return reply
            board.exchange=Mock(side_effect=exchange)
            result=provision(board,points)
            self.assertEqual((result["sequence"],result["checksum"]),(8,checksum))
            sent=[call.args[0] for call in board.exchange.call_args_list]
            self.assertEqual(sent.count(PRESET_SAVE),1)
            self.assertEqual(sent.count(PRESET_POINT),32)
            self.assertEqual(sent[-1],PRESET_QUERY)


if __name__=="__main__":
    unittest.main()
