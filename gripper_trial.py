"""Bounded Alicia trigger -> PiPER gripper trial; no hardware zeroing.

Start with the Alicia trigger released and the PiPER gripper empty.
PiPER opens gradually to its SDK-configured 70 mm maximum before following.
This program does not command the arm joints.
"""

import argparse
import time

MAX_OPEN_MM = 70.0
MAX_RATE_MM_S = 30.0
MAX_COMMAND_LEAD_MM = 5.0
DEADBAND_MM = 0.5
EFFORT_MILLI_NM = 500


def target_mm(leader_value):
    return MAX_OPEN_MM * max(0.0, min(1000.0, leader_value)) / 1000.0


def gripper_mm(piper):
    msg = piper.GetArmGripperMsgs()
    if msg.time_stamp <= 0 or msg.Hz <= 0 or time.time() - msg.time_stamp > 0.5:
        raise RuntimeError("PiPER 夹爪反馈过期")
    state = msg.gripper_state
    if state.foc_status.driver_error_status or state.foc_status.driver_overcurrent:
        raise RuntimeError("PiPER 夹爪驱动异常")
    return state.grippers_angle / 1000.0


def send_gripper(piper, mm):
    piper.GripperCtrl(round(mm * 1000), EFFORT_MILLI_NM, 0x01, 0x00)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--live", action="store_true")
    args = ap.parse_args()
    if not 1 <= args.seconds <= 60:
        ap.error("时长必须为 1–60 秒")

    from alicia_d_sdk import create_robot
    from piper_sdk import C_PiperInterface_V2
    leader = create_robot(port="/dev/ttyACM0", auto_connect=False)
    piper = C_PiperInterface_V2(can_name="can0", judge_flag=False,
                                start_sdk_joint_limit=True, start_sdk_gripper_limit=True)
    sent = False
    try:
        if not leader.connect():
            raise RuntimeError("Alicia 连接失败")
        piper.ConnectPort()
        time.sleep(0.3)
        state = leader.get_robot_state("joint_gripper", timeout=0.5)
        if state is None:
            raise RuntimeError("Alicia 夹爪反馈丢失")
        leader_start = float(state.gripper)
        piper_start = gripper_mm(piper)
        print(f"起点：Alicia={leader_start:.1f}/1000，PiPER={piper_start:.2f} mm", flush=True)
        if leader_start < 850 or not 0 <= piper_start <= MAX_OPEN_MM:
            raise RuntimeError("请先完全松开 Alicia 扳机；PiPER 夹爪反馈须在 0–70 mm")
        if piper.GetSDKGripperRangeParam()[1] < 0.07:
            raise RuntimeError("PiPER SDK 的夹爪行程上限小于 70 mm")
        if not args.live:
            print("只读预览；PiPER 夹爪不会运动", flush=True)
        else:
            sent = True
            print("先将空夹爪缓慢张开至 70 mm；此阶段请保持 Alicia 扳机松开", flush=True)
            command = piper_start
            align_end = time.monotonic() + 8.0
            previous = time.monotonic()
            while time.monotonic() < align_end:
                now = time.monotonic()
                dt = min(0.2, now-previous)
                previous = now
                actual = gripper_mm(piper)
                if actual >= 67.0:
                    break
                command = min(MAX_OPEN_MM, command + MAX_RATE_MM_S*dt,
                              actual + MAX_COMMAND_LEAD_MM)
                send_gripper(piper, command)
                time.sleep(0.03)
            else:
                raise RuntimeError("PiPER 夹爪未能在 8 秒内张开到至少 67 mm")
            print(f"夹爪已张开到 {gripper_mm(piper):.1f} mm；现在可按下／松开 Alicia 扳机", flush=True)
        command = gripper_mm(piper)
        previous = time.monotonic()
        end = previous + args.seconds
        last_print = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            dt = min(0.2, now - previous)
            previous = now
            state = leader.get_robot_state("joint_gripper", timeout=0.5)
            if state is None:
                raise RuntimeError("Alicia 夹爪反馈丢失")
            actual = gripper_mm(piper)
            target = target_mm(float(state.gripper))
            delta = target-command
            step = 0.0 if abs(delta) < DEADBAND_MM else max(-MAX_RATE_MM_S*dt, min(MAX_RATE_MM_S*dt, delta))
            command = max(actual-MAX_COMMAND_LEAD_MM,
                          min(actual+MAX_COMMAND_LEAD_MM,command+step))
            if args.live:
                send_gripper(piper, command)
            if now-last_print >= 0.2:
                print(f"Alicia {state.gripper:.0f}/1000; PiPER {actual:.1f} mm; 目标 {target:.1f} mm", flush=True)
                last_print = now
            time.sleep(0.03)
    finally:
        if sent:
            try:
                send_gripper(piper, gripper_mm(piper))
                print("夹爪跟随已停止，保持当前开口", flush=True)
            except Exception as exc:
                print(f"无法确认夹爪保持：{exc}；请使用 PiPER 急停", flush=True)
        try:
            piper.DisconnectPort()
        finally:
            leader.disconnect()


if __name__ == "__main__":
    main()
