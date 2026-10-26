"""Read-only Alicia-DL / PiPER connection and joint diagnostics.

This program never enables motors or sends motion commands.
"""

import argparse
import glob
import json
import math
import time


def leader(port: str, seconds: float) -> None:
    from alicia_d_sdk import create_robot

    robot = create_robot(port=port, auto_connect=False)
    try:
        if not robot.connect() or not robot.is_connected():
            raise RuntimeError(f"Alicia 未连接：{port}")
        print("Alicia version:", robot.get_robot_state("version"), flush=True)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            state = robot.get_robot_state("joint_gripper", timeout=1.0)
            if state is None or len(state.angles) != 6:
                raise RuntimeError("Alicia 无完整六轴反馈")
            payload = {
                "joints_deg": [round(math.degrees(x), 2) for x in state.angles],
                "gripper_0_1000": state.gripper,
                "status": state.run_status_text,
                "timestamp": state.timestamp,
            }
            print("Alicia:", json.dumps(payload, ensure_ascii=False), flush=True)
            time.sleep(0.25)
    finally:
        robot.disconnect()


def piper(can_port: str, protocol: str, seconds: float) -> None:
    from piper_sdk import C_PiperInterface_V2

    if protocol == "v3":
        try:
            from piper_sdk import C_PiperInterface_V3 as cls
        except ImportError as exc:
            raise RuntimeError("当前 piper_sdk 不含 V3 接口，请升级 SDK") from exc
    else:
        cls = C_PiperInterface_V2
    arm = cls(can_name=can_port, judge_flag=False, start_sdk_joint_limit=True)
    arm.ConnectPort()
    try:
        time.sleep(0.2)
        print("PiPER firmware:", arm.GetPiperFirmwareVersion(), flush=True)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = arm.GetArmJointMsgs()
            status = arm.GetArmStatus()
            if (msg.time_stamp <= 0 or msg.Hz <= 0
                    or time.time() - msg.time_stamp > 0.5):
                raise RuntimeError("PiPER 六轴反馈缺失或过期")
            if status.time_stamp <= 0 or time.time() - status.time_stamp > 0.5:
                raise RuntimeError("PiPER 状态反馈缺失或过期")
            payload = {
                "joints_deg": [round(getattr(msg.joint_state, f"joint_{i}") / 1000, 2) for i in range(1, 7)],
                "feedback_timestamp": msg.time_stamp,
                "feedback_hz": msg.Hz,
                "arm_status": str(status.arm_status),
            }
            print("PiPER:", json.dumps(payload, ensure_ascii=False), flush=True)
            time.sleep(0.25)
    finally:
        arm.DisconnectPort()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("device", choices=["list", "leader", "piper"])
    parser.add_argument("--port", default="/dev/ttyACM0", help="Alicia USB 串口")
    parser.add_argument("--can", default="can0", help="PiPER CAN 接口")
    parser.add_argument("--protocol", choices=["v2", "v3"], default="v2")
    parser.add_argument("--seconds", type=float, default=3)
    args = parser.parse_args()
    try:
        if args.device == "list":
            print("串口:", glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            print("CAN:", glob.glob("/sys/class/net/can*"))
        elif args.device == "leader":
            leader(args.port, args.seconds)
        else:
            if not glob.glob(f"/sys/class/net/{args.can}"):
                raise RuntimeError(f"没有 {args.can}；先接入并启用 PiPER USB-CAN 设备")
            piper(args.can, args.protocol, args.seconds)
    except Exception as exc:
        raise SystemExit(f"诊断未通过：{exc}") from None


if __name__ == "__main__":
    main()
