"""Read-only live joint feedback for the local Alicia–PiPER interface."""

import argparse
import json
import math
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    from alicia_d_sdk import create_robot
    from piper_sdk import C_PiperInterface_V2

    leader = create_robot(port=args.port, auto_connect=False)
    piper = C_PiperInterface_V2(can_name=args.can, judge_flag=False,
                                start_sdk_joint_limit=True)
    try:
        if not leader.connect():
            raise RuntimeError("Alicia 连接失败")
        piper.ConnectPort()
        time.sleep(0.3)
        while True:
            state = leader.get_robot_state("joint_gripper", timeout=0.5)
            msg = piper.GetArmJointMsgs()
            if (state is None or len(state.angles) != 6 or msg.time_stamp <= 0
                    or msg.Hz <= 0 or time.time() - msg.time_stamp > 0.5):
                raise RuntimeError("关节反馈中断或过期")
            payload = {
                "timestamp": time.time(),
                "leader": [math.degrees(float(x)) for x in state.angles],
                "piper": [getattr(msg.joint_state, f"joint_{i}") / 1000
                          for i in range(1, 7)],
            }
            temporary = args.output.with_suffix(".tmp")
            temporary.write_text(json.dumps(payload))
            temporary.replace(args.output)
            time.sleep(0.2)
    finally:
        try:
            piper.DisconnectPort()
        finally:
            leader.disconnect()


if __name__ == "__main__":
    main()
