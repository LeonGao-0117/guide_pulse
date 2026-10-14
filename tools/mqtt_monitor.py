#!/usr/bin/env python3
"""guide_pulse MQTT 消息监视器：实时打印 broker 上的相关消息。

默认读取 src/guide_pulse/config/guide_pulse.yaml 里的 broker/主题前缀，
订阅 {prefix}/{robot_id}/# 全部上下行消息（presence/heartbeat/下行指令/回执）。
注意：协议 v0.9.0+ 后机器人凭据由 enroll 动态换取，本工具只适用于
本地匿名 broker 联调；robot_id 用 --robot 指定（enroll 分配的 ID）。

用法：
  python3 tools/mqtt_monitor.py                 # 监视 robot-001 的全部主题
  python3 tools/mqtt_monitor.py --all-robots    # 监视 prefix 下全部机器人
  python3 tools/mqtt_monitor.py --broker tcp://127.0.0.1:1884 --robot robot-002
  python3 tools/mqtt_monitor.py --topic '#'     # 监视 broker 全部主题（慎用）

Ctrl+C 退出。
"""

import argparse
import datetime
import json
import os
import sys
from urllib.parse import urlparse

# 自动加载随工程分发的 paho-mqtt
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEPS_DIR = os.path.join(SCRIPT_DIR, "deps")
if os.path.isdir(DEPS_DIR):
    sys.path.insert(0, DEPS_DIR)

import paho.mqtt.client as mqtt  # noqa: E402
import yaml  # noqa: E402


def load_config_defaults():
    """从工程配置里取默认 broker/前缀（找不到则返回空默认）。

    协议 v0.9.0+ 后机器人凭据由 enroll 动态换取，配置里不再有静态账号；
    --robot 默认 robot-001 适配本地 Mock（enroll_mock_server 首个 SN 的分配）。
    """
    candidates = [
        os.path.join(SCRIPT_DIR, "..", "src", "guide_pulse", "config", "guide_pulse.yaml"),
        os.path.join(SCRIPT_DIR, "..", "..", "share", "guide_pulse", "config", "guide_pulse.yaml"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            params = yaml.safe_load(open(path))["tts_bridge"]["ros__parameters"]
            return {
                "broker": params.get("broker_uri", "tcp://localhost:11883"),
                "prefix": params.get("topic_prefix", "robotPolice/v1/robots"),
            }
    return {"broker": "tcp://localhost:11883", "prefix": "robotPolice/v1/robots"}


def main():
    # 重定向/管道场景下保持逐行实时输出
    sys.stdout.reconfigure(line_buffering=True)

    defaults = load_config_defaults()
    parser = argparse.ArgumentParser(description="实时打印 guide_pulse 相关 MQTT 消息")
    parser.add_argument("--broker", default=defaults["broker"], help="broker 地址")
    parser.add_argument("--username", default="",
                        help="用户名（本地匿名 broker 无需；真实后端联调请用 "
                             "RobotPolice 侧运维/admin 通道）")
    parser.add_argument("--password", default="", help="密码（同上）")
    parser.add_argument("--robot", default="robot-001",
                        help="机器人 ID（enroll 分配的 robot_id，本地 Mock 默认 robot-001）")
    parser.add_argument("--prefix", default=defaults["prefix"], help="主题前缀（默认取配置）")
    parser.add_argument("--all-robots", action="store_true",
                        help=f"订阅 {defaults['prefix']}/# 全部机器人")
    parser.add_argument("--topic", help="自定义订阅主题（如 '#'，慎用），覆盖默认")
    args = parser.parse_args()

    if args.topic:
        topics = [(args.topic, 0)]
    elif args.all_robots:
        topics = [(f"{args.prefix}/#", 0)]
    else:
        topics = [(f"{args.prefix}/{args.robot}/#", 0)]

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="tts-monitor")
    if args.username:
        client.username_pw_set(args.username, args.password)

    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            for topic, qos in topics:
                client.subscribe(topic, qos)
                print(f"[监视] 已订阅 {topic}  （Ctrl+C 退出）")
        else:
            print(f"[监视] 连接失败: {reason_code}", file=sys.stderr)
            sys.exit(1)

    def on_message(client, userdata, msg):
        stamp = datetime.datetime.now().strftime("%H:%M:%S")
        payload = msg.payload.decode("utf-8", "replace")
        try:
            payload = json.dumps(json.loads(payload), ensure_ascii=False, separators=(",", ":"))
        except (ValueError, TypeError):
            pass  # 非 JSON 原样输出
        print(f"[{stamp}] {msg.topic}\n  {payload}")

    client.on_connect = on_connect
    client.on_message = on_message

    url = urlparse(args.broker)
    client.connect(url.hostname, url.port or 11883, 30)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n[监视] 退出")


if __name__ == "__main__":
    main()
