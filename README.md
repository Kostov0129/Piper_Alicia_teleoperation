# Piper + Alicia 本机遥操界面

本机浏览器界面，用 Alicia-DL 示教臂控制 PiPER 单臂和夹爪。界面可**分别检测两臂连接**，然后记录当前软件起点、只读预检、启动和退出短时遥操，并显示六轴反馈和日志。只监听 `127.0.0.1`，不修改硬件零点。

## 安装与打开

在连接两臂的**本机**运行（建议 Python 3.12）：

```bash
git clone https://github.com/Kostov0129/Piper_Alicia_teleoperation.git
cd Piper_Alicia_teleoperation
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
env -u PYTHONPATH .venv/bin/python interface.py
```

然后在同一台电脑的浏览器打开 `http://127.0.0.1:8769/`。Alicia 默认串口 `/dev/ttyACM0`，PiPER 默认 CAN 接口 `can0`；起点记录界面可填写实际设备名。PiPER 当前使用 V2 SDK，已在 S-V1.8-5 固件上测试。

先分别点击“检测 Alicia 示教臂”和“检测 PiPER”。再把两臂摆到舒适的对应姿态、空夹并停稳，点击“记录软件起点”。只读预检核对关节位置、状态和夹爪开度；若任一项不合适，会拒绝启动。遥操最长 90 秒，到时自动停止并保持 PiPER 当前姿态。网页“退出遥操”会让控制程序停止并尝试保持当前位置，**不是硬件急停**；现场需要有人监护并可使用 PiPER 实体急停。程序运行后 PiPER 可能保持 CAN 控制与电机使能，要手动搬动时先按厂家流程切回示教模式。

## 映射与范围

六轴增量映射取自六组人工匹配姿态，配置见 `matched_pose_trial_mapping_wider.json`。当前相对起点范围 ±40°、PiPER 速度档位 100%、最大指令速率 45°/秒，另有绝对关节限位、反馈过期、跟踪误差和指令超前量保护。它是**试验映射**，尚未用独立姿态验证末端精度，不适合无人监护或直接用于采摘。

本机软件起点和界面运行日志保存在 `runtime/`；SDK 还可能生成 `logs/`。两者均由 `.gitignore` 排除，不会上传到 GitHub。每次更换姿态或重启试验都应重新记录起点；预检通过后才能启动一次遥操，再次启动需重新预检。
