"""Reversible, bounded Alicia-DL -> PiPER joint teleop trial.

Default mode is read-only preview. Live mode is deliberately short and bounded.
Gripper following requires --with-gripper. No firmware zeroing or return-to-home.
"""

import argparse
import json
import math
import time
from pathlib import Path

SIGNS = (1, -1, -1, -1, -1, -1)
# Conservative intersection of the installed SDK limits and the official demo.
LIMITS = ((-145, 145), (5, 175), (-165, -5), (-95, 95), (-65, 65), (-95, 95))
MAX_OFFSET_DEG = 40.0
MAX_RATE_DEG_S = 45.0
MAX_TRACKING_ERROR_DEG = 8.0
MAX_COMMAND_LEAD_DEG = 3.0
SMOOTHING_TIME_S = 0.12
DEADBAND_DEG = 0.15
SPEED_PERCENT = 100
GAIN = 0.5


def mapped_target(leader, anchor, shoulder_gain=GAIN, joint_gains=None,
                  max_offset_deg=MAX_OFFSET_DEG):
    result = []
    for i in range(6):
        center = anchor["piper_joints_deg"][i]
        gain = joint_gains[i] if joint_gains is not None else (
            SIGNS[i] * (shoulder_gain if i == 1 else GAIN))
        requested = center + gain * (leader[i] - anchor["leader_joints_deg"][i])
        low = max(LIMITS[i][0], center - max_offset_deg)
        high = min(LIMITS[i][1], center + max_offset_deg)
        result.append(max(low, min(high, requested)))
    return result


def validate_target(target, anchor, max_offset_deg=MAX_OFFSET_DEG):
    for i, value in enumerate(target):
        low, high = LIMITS[i]
        if not math.isfinite(value) or value < low-1e-6 or value > high+1e-6:
            raise RuntimeError(f"J{i+1} 目标 {value:.2f}° 超过保守限位 [{low}, {high}]°")
        if abs(value - anchor["piper_joints_deg"][i]) > max_offset_deg+1e-6:
            raise RuntimeError(f"J{i+1} 请求超过起点 ±{max_offset_deg}° 试验范围")


def read_leader(leader):
    state = leader.get_robot_state("joint_gripper", timeout=0.5)
    if state is None or len(state.angles) != 6:
        raise RuntimeError("Alicia 关节反馈丢失")
    return [math.degrees(float(x)) for x in state.angles]


def read_leader_with_gripper(leader):
    state = leader.get_robot_state("joint_gripper", timeout=0.5)
    if state is None or len(state.angles) != 6:
        raise RuntimeError("Alicia 关节或夹爪反馈丢失")
    return [math.degrees(float(x)) for x in state.angles], float(state.gripper)


def read_piper(piper):
    msg = piper.GetArmJointMsgs()
    if msg.time_stamp <= 0 or msg.Hz <= 0 or time.time() - msg.time_stamp > 0.5:
        raise RuntimeError("PiPER 关节反馈过期")
    return [getattr(msg.joint_state, f"joint_{i}") / 1000 for i in range(1, 7)]


def send_joints(piper, joints):
    # PiPER JointCtrl unit is 0.001 degree.
    piper.JointCtrl(*(round(x * 1000) for x in joints))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--anchor", type=Path, default=Path(__file__).with_name("teleop_anchor.json"))
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--live", action="store_true", help="Switch PiPER to low-speed joint control")
    ap.add_argument("--with-gripper", action="store_true", help="Also follow Alicia trigger over PiPER's 0–70 mm gripper range")
    ap.add_argument("--shoulder-gain", type=float, default=GAIN,
                    help="J2 angle mapping; 0.5 default, up to 1.0 for a bounded reach test")
    ap.add_argument("--mapping", type=Path, help="Experimental six-axis mapping JSON")
    args = ap.parse_args()
    if not 1 <= args.seconds <= 90:
        ap.error("试验时长必须为 1–90 秒")
    if not 0.5 <= args.shoulder_gain <= 1.0:
        ap.error("J2 映射增益必须在 0.5–1.0 之间")
    anchor = json.loads(args.anchor.read_text())
    mapping = json.loads(args.mapping.read_text()) if args.mapping else None
    joint_gains = None
    max_offset_deg = MAX_OFFSET_DEG
    max_rate_deg_s = MAX_RATE_DEG_S
    speed_percent = SPEED_PERCENT
    if mapping is not None:
        joint_gains = mapping["joint_slopes"]
        if len(joint_gains) != 6 or any(
            not math.isfinite(x) or x * sign <= 0 or abs(x) > 1.5
            for x, sign in zip(joint_gains, SIGNS)
        ):
            raise RuntimeError("标定映射的方向或比例超出试验范围")
        max_offset_deg = float(mapping["max_offset_deg"])
        max_rate_deg_s = float(mapping["max_rate_deg_s"])
        speed_percent = int(mapping["speed_percent"])
        if not (0 < max_offset_deg <= 40 and 0 < max_rate_deg_s <= 45
                and 1 <= speed_percent <= 100):
            raise RuntimeError("标定映射中的试验边界无效")
    if anchor["max_sample_motion_deg"] > 2:
        raise RuntimeError("起点采样不稳定")
    validate_target(anchor["piper_joints_deg"], anchor, max_offset_deg)

    from alicia_d_sdk import create_robot
    from piper_sdk import C_PiperInterface_V2
    from gripper_trial import (MAX_OPEN_MM, MAX_RATE_MM_S, MAX_COMMAND_LEAD_MM,
                               DEADBAND_MM, gripper_mm, send_gripper, target_mm)
    leader = create_robot(port=anchor["leader_port"], auto_connect=False)
    piper = C_PiperInterface_V2(can_name=anchor["piper_can"], judge_flag=False,
                                start_sdk_joint_limit=True,
                                start_sdk_gripper_limit=args.with_gripper)
    sent = False
    grip_sent = False
    try:
        if not leader.connect():
            raise RuntimeError("Alicia 连接失败")
        piper.ConnectPort()
        time.sleep(0.3)
        actual = read_piper(piper)
        if args.with_gripper:
            lead, leader_grip = read_leader_with_gripper(leader)
            grip_actual = gripper_mm(piper)
            grip_target = target_mm(leader_grip)
            if piper.GetSDKGripperRangeParam()[1] < 0.07:
                raise RuntimeError("PiPER SDK 夹爪行程不足 70 mm")
            if abs(grip_actual - grip_target) > 3.0:
                raise RuntimeError(f"夹爪起点不一致：PiPER {grip_actual:.1f} mm，Alicia目标 {grip_target:.1f} mm；拒绝启动")
        else:
            lead = read_leader(leader)
        for i in range(6):
            if abs(actual[i] - anchor["piper_joints_deg"][i]) > 2.0:
                raise RuntimeError(f"PiPER J{i+1} 偏离起点超过 2°，请重记起点")
            if abs(lead[i] - anchor["leader_joints_deg"][i]) > 3.0:
                raise RuntimeError(f"Alicia J{i+1} 偏离起点超过 3°，请重记起点")
        status = piper.GetArmStatus()
        if status.time_stamp <= 0 or time.time() - status.time_stamp > 0.5 or status.arm_status.arm_status != 0:
            raise RuntimeError(f"PiPER 状态异常：{status.arm_status}")
        print(f"起点检查通过；关节比例 {joint_gains if joint_gains is not None else '默认 J2=' + str(args.shoulder_gain)}；目标相对起点最多 {max_offset_deg:.0f}°；速率最多 {max_rate_deg_s:.0f}°/s；PiPER 速度 {speed_percent}%", flush=True)
        if not args.live:
            print("只读预览：PiPER 不会运动", flush=True)
        else:
            # At the stationary start pose, command a hold before accepting leader motion.
            sent = True  # From this point, always attempt to hold measured pose on exit.
            piper.EnableArm(7)
            time.sleep(0.3)
            if not all(piper.GetArmEnableStatus()):
                raise RuntimeError("PiPER 关节未全部使能")
            piper.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
            send_joints(piper, actual)
            time.sleep(0.3)
            post = read_piper(piper)
            if max(abs(x-y) for x,y in zip(post,actual)) > 2.0:
                raise RuntimeError("切换控制模式时位置变化超过 2°")
            if args.with_gripper:
                send_gripper(piper, grip_actual)
                grip_sent = True
                print("整臂＋夹爪跟随开始；可以移动 Alicia 和扳机，Ctrl+C 结束", flush=True)
            else:
                print("低速试验开始；仅小幅移动 Alicia，Ctrl+C 结束", flush=True)

        command = actual[:]
        filtered = actual[:]
        grip_command = grip_actual if args.with_gripper else None
        end = time.monotonic() + args.seconds
        previous = time.monotonic()
        last_print = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            dt = min(now - previous, 0.2)
            previous = now
            if args.with_gripper:
                lead, leader_grip = read_leader_with_gripper(leader)
                grip_actual = gripper_mm(piper)
            else:
                lead = read_leader(leader)
            actual = read_piper(piper)
            target = mapped_target(lead, anchor, args.shoulder_gain,
                                   joint_gains, max_offset_deg)
            validate_target(target, anchor, max_offset_deg)
            if args.live and max(abs(x-y) for x,y in zip(command,actual)) > MAX_TRACKING_ERROR_DEG:
                raise RuntimeError("跟踪误差超过 8°")
            alpha = dt / (SMOOTHING_TIME_S + dt) if dt > 0 else 0.0
            filtered = [f + alpha*(t-f) for f,t in zip(filtered,target)]
            next_command = []
            for i, (c,t) in enumerate(zip(command,filtered)):
                if abs(t-c) < DEADBAND_DEG:
                    step = 0.0
                else:
                    step = max(-max_rate_deg_s*dt, min(max_rate_deg_s*dt,t-c))
                value = c + step
                value = max(actual[i]-MAX_COMMAND_LEAD_DEG,
                            min(actual[i]+MAX_COMMAND_LEAD_DEG,value))
                next_command.append(value)
            command = next_command
            if args.live:
                send_joints(piper, command)
            if args.with_gripper:
                grip_target = target_mm(leader_grip)
                grip_delta = grip_target - grip_command
                grip_step = 0.0 if abs(grip_delta) < DEADBAND_MM else max(
                    -MAX_RATE_MM_S*dt, min(MAX_RATE_MM_S*dt, grip_delta))
                grip_command = max(grip_actual-MAX_COMMAND_LEAD_MM,
                                   min(grip_actual+MAX_COMMAND_LEAD_MM,
                                       grip_command+grip_step))
                if args.live:
                    send_gripper(piper, grip_command)
            if now-last_print >= 0.2:
                print("Alicia", [round(x,1) for x in lead], "PiPER", [round(x,1) for x in actual],
                      "目标", [round(x,1) for x in target],
                      *([f"夹爪 {grip_actual:.1f}/{grip_target:.1f} mm"] if args.with_gripper else []),
                      flush=True)
                last_print = now
            time.sleep(0.03)
    finally:
        if grip_sent:
            try:
                send_gripper(piper, gripper_mm(piper))
                print("夹爪已停止跟随并保持当前开口", flush=True)
            except Exception as exc:
                print(f"无法确认夹爪保持：{exc}；请在现场使用 PiPER 急停", flush=True)
        if sent:
            try:
                # Hold at the measured pose. Do not return to the starting pose or disable torque.
                send_joints(piper, read_piper(piper))
                print("已停止跟随并保持当前位置", flush=True)
            except Exception as exc:
                print(f"无法确认停止保持：{exc}；请在现场使用 PiPER 急停", flush=True)
        try:
            piper.DisconnectPort()
        finally:
            leader.disconnect()


if __name__ == "__main__":
    main()
