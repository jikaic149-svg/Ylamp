"""Generate/provision one W25Q64 preset. Storage commands never start motion."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import time
import zlib

from dm_serial import (PRESET_BEGIN, PRESET_POINT, PRESET_SAVE, PRESET_QUERY, PRESET_ACK,
                       PRESET_STATE, PRESET_READ, PRESET_DATA)
from five_control import Board, RPCRejected, ReplyTimeout
from five_trajectory import normalize_points, pack_point, trajectory_hash
from usb_monitor import serial_port

VERSION = 0x594c5001
DEFAULT = Path(__file__).resolve().parents[1]/"config/lookaround.json"
ANCHOR = [131, -1, 19, 116, 14]


def from_csv(path):
    """Retain one original 8s cycle, 1.5x duration, 2s entry/rest tail.

    The 30 interior samples are eight original 30Hz frames apart, not
    interpolation between the old ten knots. Retain original q/v/a; initial
    rest overrides the moving CSV seam, final rest is Ruckig-planned.
    """
    with Path(path).open(encoding="utf-8-sig", newline="") as source:
        rows = list(csv.DictReader(source))
    if len(rows) < 241 or any(abs(float(rows[i]["time_s"])-i/30) > 1e-6 for i in range(241)):
        raise ValueError("Expected the original 30Hz CSV with a complete first 8s cycle")
    home = [float(rows[0][f"J{j}_position_deg"]) for j in range(1, 6)]
    points = [dict(at_ms=2000, position_deg=ANCHOR, velocity_rad_s=[0]*5, acceleration_rad_s2=[0]*5)]
    for frame in range(8, 241, 8):
        row = rows[frame]
        points.append(dict(at_ms=2000+frame*50,
            position_deg=[ANCHOR[j-1]+float(row[f"J{j}_position_deg"])-home[j-1] for j in range(1, 6)],
            velocity_rad_s=[math.radians(float(row[f"J{j}_velocity_deg_s"]))/1.5 for j in range(1, 6)],
            acceleration_rad_s2=[math.radians(float(row[f"J{j}_acceleration_deg_s2"]))/2.25
                                 for j in range(1, 6)]))
    points.append(dict(at_ms=16000, position_deg=ANCHOR, velocity_rad_s=[0]*5, acceleration_rad_s2=[0]*5))
    return normalize_points(points)


def decode(reply):
    if len(reply) != 40:
        raise ValueError("Invalid preset status length")
    rc, staged, expected, count = reply[12:16]
    if rc:
        raise RPCRejected("external preset", rc)
    flags, sequence, checksum, pending_hash, preset_id, errors = struct.unpack_from("<6I", reply, 16)
    if not staged <= expected <= 32 or count > 32:
        raise ValueError("Invalid preset counts")
    return dict(flags=flags, sequence=sequence, checksum=checksum, staged_checksum=pending_hash,
                preset_id=preset_id, count=count, staged_count=staged, expected=expected, errors=errors)


def query(board):
    board.query_ident += 1
    prefix = board.session + struct.pack("<I", board.query_ident)
    return decode(board.exchange(PRESET_QUERY, prefix, PRESET_STATE, prefix))

def read_region(board):
    image = bytearray()
    deadline = time.monotonic()+12
    while len(image)<8192:
        board.query_ident += 1
        prefix = board.session+struct.pack("<I", board.query_ident)
        reply = board.exchange(PRESET_READ,prefix+struct.pack("<I",len(image)),PRESET_DATA,prefix)
        if reply[12] == 3:  # BSP_CAN_BUSY: only the read is repeated.
            if time.monotonic()>=deadline:
                raise TimeoutError("Preset backup chip ownership unavailable; no write")
            board.pump()
            continue
        if reply[12]:
            raise RPCRejected("preset partition backup",reply[12])
        n = min(224,8192-len(image))
        if len(reply)!=244 or reply[13]!=n or struct.unpack_from("<I",reply,16)[0]!=len(image):
            raise ValueError("Preset backup offset/length mismatch")
        image.extend(reply[20:20+n])
    return bytes(image)


def verify_region(image,points,state):
    expected = b"".join(pack_point(p) for p in normalize_points(points))
    for slot in (0,1):
        b=image[slot*4096:(slot+1)*4096]
        magic,version,sequence,count,length,crc,fnv = struct.unpack_from("<7I",b)
        if (magic,version,sequence,count,length,fnv)!=(0x594c5031,1,state["sequence"],
                len(points),len(expected),trajectory_hash(points)):
            continue
        if (struct.unpack_from("<I",b,60)[0]!=0x434f4d54 or
                struct.unpack_from("<I",b,56)[0]!=zlib.crc32(b[:56]) or
                crc!=zlib.crc32(b[64:64+length]) or b[64:64+length]!=expected):
            raise ValueError("Complete external flash readback differs from uploaded preset")
        return slot
    raise ValueError("Committed external preset absent from complete readback")


def provision(board, points):
    points = normalize_points(points)
    if board.rpc(246) != VERSION:
        raise ValueError("Firmware lacks W25Q64 preset support; no fallback to motion commands")
    before = query(board)
    header = board.write_prefix()
    preset_id = board.ident

    def write(kind, prefix, body):
        return decode(board.exchange(kind, prefix+body, PRESET_ACK, prefix[:12]))

    state = write(PRESET_BEGIN, header, bytes([len(points)]))
    for index, point in enumerate(points):
        state = write(PRESET_POINT, board.write_prefix(), struct.pack("<IB", preset_id, index)+pack_point(point))
        if (state["preset_id"], state["staged_count"], state["expected"]) != (preset_id, index+1, len(points)):
            raise ValueError("Preset upload identity/count mismatch; not saved")
    checksum = trajectory_hash(points)
    if state["staged_checksum"] != checksum:
        raise ValueError("Preset upload checksum mismatch; not saved")
    # An absent SAVE ACK is an unknown outcome: QUERY only, never replay SAVE.
    try:
        write(PRESET_SAVE, board.write_prefix(), struct.pack("<II", preset_id, checksum))
    except ReplyTimeout:
        pass
    deadline = time.monotonic()+12
    while time.monotonic() < deadline:
        state = query(board)
        if state["errors"] or state["flags"] & (16 | 64):
            raise RuntimeError(f"Preset persistence failed: {state}")
        if not state["flags"] & 8:
            if (state["sequence"] == (before["sequence"]+1) & 0xffffffff and
                    state["count"] == len(points) and state["checksum"] == checksum and state["flags"] & 2):
                return state
            raise RuntimeError(f"Preset commit not confirmed; no replay: {state}")
        board.pump()
    raise TimeoutError("Preset commit still unknown; query explicitly, do not repeat save")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest="action", required=True)
    generate = actions.add_parser("generate")
    generate.add_argument("--csv", type=Path, required=True)
    generate.add_argument("--output", type=Path, required=True)
    for name in ("status", "save"):
        sub = actions.add_parser(name)
        sub.add_argument("--port", required=True)
        if name == "save":
            sub.add_argument("--preset", type=Path, default=DEFAULT)
            sub.add_argument("--backup", type=Path, required=True)
            sub.add_argument("--readback", type=Path, required=True)
    args = parser.parse_args()
    if args.action == "generate":
        points = from_csv(args.csv)
        record = dict(format="ylamp-lookaround-v1", source_sha256=hashlib.sha256(args.csv.read_bytes()).hexdigest(),
                      time_scale=1.5, playback_origin="current_position_at_key", points=points)
        with args.output.open("x", encoding="utf-8") as output:
            json.dump(record, output, indent=2, allow_nan=False)
            output.write("\n")
        print(f"{len(points)} points, FNV1a32 {trajectory_hash(points):08x}; no hardware accessed")
        return
    points = normalize_points(json.loads(args.preset.read_text(encoding="utf-8"))["points"]) if args.action == "save" else None
    with serial_port(args.port) as port:
        board = Board(port)
        try:
            board.connect()
            if board.rpc(246) != VERSION:
                raise ValueError("Firmware lacks matching external preset support")
            if points:
                if args.backup.exists() or args.readback.exists():
                    raise ValueError("Use fresh backup/readback paths; no write")
                before = read_region(board)
                with args.backup.open("xb") as output:
                    output.write(before)
            result = provision(board, points) if points else query(board)
            if points:
                after = read_region(board)
                with args.readback.open("xb") as output:
                    output.write(after)
                result["slot"]=verify_region(after,points,result)
                result["backup_sha256"]=hashlib.sha256(before).hexdigest()
                result["readback_sha256"]=hashlib.sha256(after).hexdigest()
            print(json.dumps(result, indent=2))
        finally:
            port.dtr = False


if __name__ == "__main__":
    main()
