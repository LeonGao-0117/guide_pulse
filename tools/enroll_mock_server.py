#!/usr/bin/env python3
"""enroll 注册面本地 Mock（本地联调 / 集成测试用，勿用于生产）。

实现 RobotPolice 协议 v0.10.0 的 Robot 注册面（仅 Happy Path + 400）：
  POST /v1/robots/enroll   {"serial_number": "..."}
  -> 200 {"robot_id", "username", "password", "expires_at"}

语义对齐真实 Server（D-019/P-025）：
  - 注册表以 SN 为唯一锚点；首次 enroll 分配可读递增 robot_id
    （robot-001 起三位补零），同 SN 恒返回同一 robot_id；
  - username 即 robot_id；密码每次 enroll 轮换（随机 64 hex）；
  - serial_number 非法返回 400 invalid_request。
（限速 429 / 吊销 403 / 未登记 403 等错误分支由单元测试覆盖，
 本 Mock 不实现。）

用法：
  python3 tools/enroll_mock_server.py                    # 默认 127.0.0.1:18080
  python3 tools/enroll_mock_server.py --port 18081

无第三方依赖。Ctrl+C 退出。
"""

import argparse
import json
import re
import secrets
import sys
from datetime import datetime, timedelta, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SERIAL_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{5,63}$")
PASSWORD_TTL_DAYS = 30

# SN -> robot_id 注册表（进程内存，重启后重新发号，仅本地测试语义）
registry: dict[str, str] = {}
registry_order: list[str] = []


class EnrollHandler(BaseHTTPRequestHandler):
    def do_POST(self):  # noqa: N802（http.server 接口命名）
        if self.path != "/v1/robots/enroll":
            self._error(404, "not_found", f"unknown path: {self.path}")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(length).decode("utf-8")) if length else {}
        except (ValueError, UnicodeDecodeError):
            self._error(400, "invalid_request", "请求体不是合法 JSON")
            return
        if not isinstance(body, dict) or set(body) != {"serial_number"}:
            self._error(400, "invalid_request", "请求体必须只含 serial_number 字段")
            return
        serial = body["serial_number"]
        if not isinstance(serial, str) or not SERIAL_RE.fullmatch(serial):
            self._error(400, "invalid_request", "serial_number 格式非法")
            return

        # SN 唯一锚点 + 发号：首个见到的新 SN 分配下一个可读递增 ID
        if serial not in registry:
            registry[serial] = f"robot-{len(registry_order) + 1:03d}"
            registry_order.append(serial)
        robot_id = registry[serial]

        response = {
            "robot_id": robot_id,
            "username": robot_id,  # 协议：username 即 robot_id
            "password": secrets.token_hex(32),  # 每次 enroll 轮换
            "expires_at": (datetime.now(timezone.utc) +
                           timedelta(days=PASSWORD_TTL_DAYS)).isoformat(),
        }
        data = json.dumps(response).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)
        # 密码不打印（对齐真实施工规范）
        print(f"[enroll-mock] SN={serial} -> robot_id={robot_id}（密码已轮换）",
              flush=True)

    def _error(self, status: int, code: str, message: str) -> None:
        data = json.dumps({
            "code": code,
            "message": message,
            "request_id": "00000000-0000-7000-8000-000000000000",
        }).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)
        print(f"[enroll-mock] {status} {code}: {message}", file=sys.stderr, flush=True)

    def log_message(self, fmt, *args):  # 静默默认访问日志，走自定义打印
        pass


def main():
    parser = argparse.ArgumentParser(description="RobotPolice enroll 注册面本地 Mock")
    parser.add_argument("--address", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18080)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.address, args.port), EnrollHandler)
    print(f"[enroll-mock] listening on http://{args.address}:{args.port}/v1/robots/enroll",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[enroll-mock] 退出")


if __name__ == "__main__":
    main()
