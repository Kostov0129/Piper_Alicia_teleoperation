"""Capture a reversible Alicia-DL / PiPER joint-space pairing anchor.

Read-only: this program never enables motors, calibrates firmware, or moves arms.
"""

import argparse
import json
import math
import statistics
import time
from pathlib import Path


def joints_piper(arm):
    msg = arm.GetArmJointMsgs()
    if msg.time_stamp <= 0 or msg.Hz <= 0:
        raise RuntimeError("PiPER 关节反馈尚未更新")
    return [getattr(msg.joint_state, f"joint_{i}") / 1000 for i in range(1, 7)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leader-port", default="/dev/ttyACM0")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("pair_anchor.json"))
    args = parser.parse_args()

    from alicia_d_sdk import create_robot
    from piper_sdk import C_PiperInterface_V2

    leader = create_robot(port=args.leader_port, auto_connect=False)
    piper = C_PiperInterface_V2(can_name=args.can, judge_flag=False, start_sdk_joint_limit=True)
    try:
        if not leader.connect():
            raise RuntimeError("Alicia 未连接")
        piper.ConnectPort()
        time.sleep(0.2)
        firmware = piper.GetPiperFirmwareVersion()
        if not firmware or firmware == -0x4AF:
            raise RuntimeError("PiPER 固件版本无法读取")

        samples = []
        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            state = leader.get_robot_state("joint_gripper", timeout=1.0)
            if state is None or len(state.angles) != 6:
                raise RuntimeError("Alicia 未返回完整关节数据")
            leader_deg = [math.degrees(float(x)) for x in state.angles]
            follower_deg = joints_piper(piper)
            samples.append((leader_deg, follower_deg, state.gripper, state.run_status_text))
            time.sleep(0.1)

        if len(samples) < 5:
            raise RuntimeError("采样不足；未保存配对起点")
        lead = list(zip(*(row[0] for row in samples)))
        follow = list(zip(*(row[1] for row in samples)))
        max_span = max(max(v) - min(v) for v in lead + follow)
        if max_span > 2.0:
            raise RuntimeError(f"采样期间关节移动了 {max_span:.1f}°，请保持两臂静止后重试")

        result = {
            "description": "软件配对起点；不修改任何硬件零点；轴向和比例仍需逐轴校验",
            "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "leader_port": args.leader_port,
            "leader_version": leader.get_robot_state("version"),
            "piper_can": args.can,
            "piper_firmware": str(firmware),
            "leader_joints_deg": [round(statistics.median(v), 3) for v in lead],
            "piper_joints_deg": [round(statistics.median(v), 3) for v in follow],
            "leader_gripper_0_1000": round(statistics.median(row[2] for row in samples), 1),
            "leader_status": samples[-1][3],
            "max_sample_motion_deg": round(max_span, 3),
            "axis_direction_validated": [False] * 6,
        }
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print(f"已保存：{args.output}")
    finally:
        try:
            piper.DisconnectPort()
        finally:
            leader.disconnect()


if __name__ == "__main__":
    main()
