"""Local browser interface for bounded Alicia-DL -> PiPER teleoperation.

The server never imports hardware SDKs or drives joints itself. It runs the
existing read-only anchor capture and guarded teleop controller as children.
"""

import argparse
import hashlib
import json
import math
import os
import re
import secrets
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent
RUNTIME = ROOT / "runtime"
ANCHOR = RUNTIME / "current_anchor.json"
MAPPING = ROOT / "matched_pose_trial_mapping_wider.json"
HTML = ROOT / "index.html"
TELEMETRY = RUNTIME / "joints.json"


def joint_ranges(anchor):
    from teleop_trial import LIMITS
    if not anchor:
        return {"leader": None, "piper": LIMITS}
    mapping = json.loads(MAPPING.read_text())
    offset = float(mapping["max_offset_deg"])
    gains = mapping["joint_slopes"]
    piper = []
    leader = []
    for i, (center, gain) in enumerate(zip(anchor["piper_joints_deg"], gains)):
        low = max(LIMITS[i][0], center - offset)
        high = min(LIMITS[i][1], center + offset)
        piper.append([low, high])
        values = [anchor["leader_joints_deg"][i] + (bound - center) / gain
                  for bound in (low, high)]
        leader.append([min(values), max(values)])
    return {"leader": leader, "piper": piper}


def read_telemetry():
    try:
        data = json.loads(TELEMETRY.read_text())
        if time.time() - data["timestamp"] > 1.5:
            return None
        if any(len(data[name]) != 6 or not all(math.isfinite(x) for x in data[name])
               for name in ("leader", "piper")):
            return None
        return data
    except (OSError, ValueError, KeyError, TypeError):
        return None


def tail(path: Path, limit=36):
    if not path or not path.exists():
        return []
    # The bounded child runs produce small logs; keep the HTTP response compact.
    return path.read_text(errors="replace").splitlines()[-limit:]


def fingerprint(gripper: bool):
    if not ANCHOR.exists():
        raise ValueError("请先记录当前软件起点")
    data = ANCHOR.read_bytes() + MAPPING.read_bytes() + bytes([gripper])
    return hashlib.sha256(data).hexdigest()


class Controller:
    def __init__(self):
        RUNTIME.mkdir(exist_ok=True)
        self.lock = threading.RLock()
        self.process = None
        self.log_handle = None
        self.log_path = None
        self.kind = "idle"
        self.result = "尚未操作"
        self.preview_fingerprint = None
        self.preview_time = 0.0
        self.gripper = True
        self.connections = {"leader": None, "piper": None}
        self.checked_at = {"leader": None, "piper": None}
        self.monitor = None
        self.monitor_retry_after = 0.0
        self.port = "/dev/ttyACM0"
        self.can = "can0"

    def _stop_monitor(self):
        if self.monitor and self.monitor.poll() is None:
            os.killpg(self.monitor.pid, signal.SIGINT)
            try:
                self.monitor.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.monitor.pid, signal.SIGTERM)
                self.monitor.wait(timeout=2)
        self.monitor = None
        TELEMETRY.unlink(missing_ok=True)

    def _ensure_monitor(self):
        if self.process is not None or not all(self.connections.values()):
            return
        if self.monitor and self.monitor.poll() is None:
            return
        if time.monotonic() < self.monitor_retry_after:
            return
        self.monitor_retry_after = time.monotonic() + 8
        with (RUNTIME / "joint-monitor.log").open("ab") as log:
            self.monitor = subprocess.Popen(
                [sys.executable, "joint_monitor.py", "--port", self.port,
                 "--can", self.can, "--output", str(TELEMETRY)],
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )

    def _finish(self):
        if self.process is None or self.process.poll() is None:
            return
        code = self.process.returncode
        if self.log_handle:
            self.log_handle.close()
            self.log_handle = None
        if self.kind in ("detect_leader", "detect_piper"):
            device = self.kind.split("_", 1)[1]
            self.connections[device] = code == 0
            self.checked_at[device] = time.strftime("%H:%M:%S")
            self.result = ("Alicia" if device == "leader" else "PiPER") + (
                " 连接与反馈正常" if code == 0 else " 检测未通过；查看日志")
        elif self.kind in ("capture", "prepare_capture") and code == 0:
            pending = RUNTIME / "anchor_pending.json"
            if not pending.exists():
                self.result = "起点记录未生成；查看运行记录"
                self.process = None
                self.kind = "idle"
                return
            pending.replace(ANCHOR)
            self.preview_fingerprint = None
            self.connections = {"leader": True, "piper": True}
            self.checked_at = {"leader": time.strftime("%H:%M:%S"),
                               "piper": time.strftime("%H:%M:%S")}
            if self.kind == "prepare_capture":
                self.process = None
                self.kind = "idle"
                self._launch("preview", self._preview_command(self.gripper), self.gripper)
                self.result = "起点已记录，正在只读检查"
                return
            self.result = "软件起点已保存；下一步做只读预检"
        elif self.kind == "preview" and code == 0:
            self.preview_fingerprint = fingerprint(self.gripper)
            self.preview_time = time.monotonic()
            self.result = "只读预检通过；可以启动有界遥操"
        elif self.kind == "live" and code == 0:
            self.result = "遥操正常结束；PiPER 保持当前位置"
        elif code == -signal.SIGINT:
            self.result = "已请求停止；请核对机械臂状态"
        else:
            self.result = f"{self.kind} 未完成（退出码 {code}）；查看下方日志"
        self.process = None
        self.kind = "idle"
        TELEMETRY.unlink(missing_ok=True)

    def _launch(self, kind: str, command, gripper=True):
        self._finish()
        if self.process is not None:
            raise ValueError("已有操作正在运行；请先等待或停止")
        self._stop_monitor()
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.log_path = RUNTIME / f"{stamp}-{kind}.log"
        self.log_handle = self.log_path.open("wb")
        env = os.environ.copy()
        env.pop("PYTHONPATH", None)
        self.process = subprocess.Popen(
            [sys.executable, *command], cwd=ROOT, env=env,
            stdout=self.log_handle, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.kind = kind
        self.gripper = gripper
        self.result = f"{kind} 运行中"

    @staticmethod
    def _preview_command(gripper):
        command = ["teleop_trial.py", "--anchor", str(ANCHOR), "--mapping",
                   str(MAPPING), "--seconds", "2", "--telemetry-file", str(TELEMETRY)]
        if gripper:
            command.append("--with-gripper")
        return command

    def start(self, kind, body):
        with self.lock:
            self._finish()
            if self.process is not None:
                raise ValueError("已有操作正在运行")
            if kind in ("capture", "prepare", "detect_leader", "detect_piper"):
                port = body.get("port", "/dev/ttyACM0")
                can = body.get("can", "can0")
                if not re.fullmatch(r"/dev/tty(?:ACM|USB)[0-9]{1,3}", port):
                    raise ValueError("串口格式应为 /dev/ttyACM0 或 /dev/ttyUSB0")
                if not re.fullmatch(r"can[0-9]{1,2}", can):
                    raise ValueError("CAN 名称应为 can0 等")
                self.port, self.can = port, can
                if kind == "detect_leader":
                    self._launch(kind, ["probe.py", "leader", "--port", port,
                                        "--seconds", "1"])
                    return
                if kind == "detect_piper":
                    self._launch(kind, ["probe.py", "piper", "--can", can,
                                        "--protocol", "v2", "--seconds", "1"])
                    return
                self.preview_fingerprint = None
                gripper = body.get("gripper", True)
                if not isinstance(gripper, bool):
                    raise ValueError("夹爪选项无效")
                if kind == "prepare" and not all(self.connections.values()):
                    raise ValueError("请先检测两臂连接")
                (RUNTIME / "anchor_pending.json").unlink(missing_ok=True)
                self._launch("prepare_capture" if kind == "prepare" else "capture",
                             ["capture_pair_anchor.py", "--leader-port", port,
                              "--can", can, "--output", str(RUNTIME / "anchor_pending.json")], gripper)
                return
            if kind not in ("preview", "live"):
                raise ValueError("未知操作")
            gripper = body.get("gripper", True)
            if not isinstance(gripper, bool):
                raise ValueError("夹爪选项无效")
            key = fingerprint(gripper)
            if kind == "live":
                if not all(self.connections.values()):
                    raise ValueError("请先分别检测 Alicia 和 PiPER 连接")
                if body.get("confirmed") is not True:
                    raise ValueError("请先确认有人现场监护并能使用硬件急停")
                if self.preview_fingerprint != key or time.monotonic() - self.preview_time > 120:
                    raise ValueError("请先以相同的起点、映射和夹爪选项重新通过只读预检")
                seconds = body.get("seconds", 30)
                if type(seconds) is not int or not 5 <= seconds <= 90:
                    raise ValueError("遥操时长应为 5–90 秒")
                # A successful preview is single-use; repeat it before another live run.
                self.preview_fingerprint = None
            else:
                seconds = 2
            command = ["teleop_trial.py", "--anchor", str(ANCHOR), "--mapping", str(MAPPING),
                       "--seconds", str(seconds), "--telemetry-file", str(TELEMETRY)]
            if gripper:
                command.append("--with-gripper")
            if kind == "live":
                command.append("--live")
            self._launch(kind, command, gripper)

    def stop(self):
        with self.lock:
            self._finish()
            if self.process is None:
                return
            os.killpg(self.process.pid, signal.SIGINT)
            self.result = "正在停止并保持 PiPER 当前姿态；这不是硬件急停"

    def status(self):
        with self.lock:
            self._finish()
            self._ensure_monitor()
            anchor = None
            if ANCHOR.exists():
                try:
                    data = json.loads(ANCHOR.read_text())
                    anchor = {
                        "time": data.get("captured_at"),
                        "leader": data.get("leader_joints_deg"),
                        "piper": data.get("piper_joints_deg"),
                    }
                except (OSError, ValueError):
                    self.result = "起点文件损坏；请重新记录"
            lines = tail(self.log_path)
            ranges = joint_ranges(data if anchor else None)
            return {
                "busy": self.process is not None,
                "kind": self.kind,
                "result": self.result,
                "anchor": anchor,
                "connections": self.connections.copy(),
                "checked_at": self.checked_at.copy(),
                "preview_ready": bool(self.preview_fingerprint and
                                      time.monotonic() - self.preview_time <= 120),
                "telemetry": read_telemetry(),
                "joint_ranges": ranges,
                "log": lines,
                "log_file": str(self.log_path) if self.log_path else None,
            }


def make_handler(controller, token):
    class Handler(BaseHTTPRequestHandler):
        def _json(self, status, data):
            raw = json.dumps(data, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _local_host(self):
            return self.headers.get("Host", "").split(":")[0] in ("127.0.0.1", "localhost")

        def do_GET(self):
            if not self._local_host():
                return self._json(403, {"error": "仅允许本机访问"})
            path = urlsplit(self.path).path
            if path == "/api/status":
                return self._json(200, controller.status())
            if path == "/":
                raw = HTML.read_text().replace("__TOKEN__", token).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                return self.wfile.write(raw)
            self._json(404, {"error": "页面不存在"})

        def do_POST(self):
            if not self._local_host() or self.headers.get("X-Local-Token") != token:
                return self._json(403, {"error": "本机请求验证失败"})
            path = urlsplit(self.path).path
            if path not in ("/api/capture", "/api/prepare", "/api/detect_leader", "/api/detect_piper",
                            "/api/preview", "/api/live", "/api/stop"):
                return self._json(404, {"error": "未知操作"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size > 4096:
                    raise ValueError("请求过大")
                body = json.loads(self.rfile.read(size) or b"{}")
                if not isinstance(body, dict):
                    raise ValueError("请求格式错误")
                if path == "/api/stop":
                    controller.stop()
                else:
                    controller.start(path.rsplit("/", 1)[1], body)
                self._json(200, controller.status())
            except (ValueError, OSError) as exc:
                self._json(400, {"error": str(exc)})

        def log_message(self, fmt, *args):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8769)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("端口必须在 1024–65535")
    controller = Controller()
    token = secrets.token_urlsafe(32)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(controller, token))
    server.daemon_threads = True
    print(f"本机界面：http://127.0.0.1:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("界面已退出", flush=True)
    finally:
        controller.stop()
        controller._stop_monitor()
        if controller.process:
            try:
                controller.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(controller.process.pid, signal.SIGTERM)
        server.server_close()


if __name__ == "__main__":
    main()
