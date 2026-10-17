#!/usr/bin/env python3
"""后端模拟器：模拟后端（APP 经 Server）通过 MQTT 向机器人下发展厅讲解指令
（迎宾 scene / 导航+汽车讲解 navigate / 地图重定位 relocate / 循环播报 tts /
停止 tts.stop / 停止导航 navigate.stop），并监听 tts.result / relocate.result
回执。

协议基线：RobotCar 协议 v0.5.0（tts 播报列表）+ v0.6.0（tts.result 回执）
+ v0.16.0（navigate）+ v0.17.0（scene 迎宾）+ v0.20.0（navigate/scene 可选
随动播报 texts）+ v0.21.0（relocate 地图重定位，2026-10-17 接入）
+ v0.23.0（navigate.stop）。scene / navigate / tts.stop / navigate.stop
无回执（契约）。

用法（先确保 guide_bridge 已在运行）：
  # 迎宾（本地默认词；协议 scene=welcome，v0.17.0）
  python3 backend_simulator.py welcome
  python3 backend_simulator.py welcome --text "欢迎光临" --text "请随我来"
  # 迎宾 + 二维码（v0.22.0 可选 qrcode_png_base64，脸屏常驻 5 分钟）
  python3 backend_simulator.py welcome --qrcode /opt/liwei/QRCode.png

  # 导航 + 汽车讲解（到达点位后按序播讲解词）
  python3 backend_simulator.py navigate 3号展台 --text "这是我们的主打车型" --text "续航一千公里"
  python3 backend_simulator.py navigate 充电桩            # 只导航，到达不播报

  # 地图重定位（机器人已被摆到点位上；回 relocate.result，缺省播
  # 「{点位名}点重定位成功」）
  python3 backend_simulator.py relocate A

  # 点位映射表全量同步（v0.25.0，navigate/relocate 的 point_name 经表
  # 解析为航点 code；回 point_map.result）
  python3 backend_simulator.py point-map "A=比亚迪ATTO 1" "B=腾势D9"

  # 循环播报（多条文本按序循环，v0.5.0）
  python3 backend_simulator.py play "第一条" "第二条" --interval 5

  # 停止一切播报并取消导航（tts.stop 无回执）
  python3 backend_simulator.py stop

  # 只取消当前导航（v0.23.0 navigate.stop 无回执；无导航时 no-op）
  python3 backend_simulator.py nav-stop

依赖：paho-mqtt（已随工程分发 deps/，自动加载，无需额外安装）。
"""

import argparse
import json
import os
import sys
import threading
import time
import uuid
from urllib.parse import urlparse

# 自动加载随工程分发的 paho-mqtt
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEPS_DIR = os.path.join(SCRIPT_DIR, "deps")
if os.path.isdir(DEPS_DIR):
    sys.path.insert(0, DEPS_DIR)

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[错误] 缺少 paho-mqtt，请先执行: pip3 install --target deps paho-mqtt", file=sys.stderr)
    sys.exit(1)

try:
    import yaml
except ImportError:
    yaml = None


def load_config_defaults():
    """从工程配置取默认 broker/robot_id。

    协议 v0.9.0+ 后 MQTT 凭据由机器人侧 enroll 动态换取（不落配置），
    本工具面向本地匿名 mosquitto 联调，不再读取 username/password。
    """
    if yaml is None:
        return {}
    candidates = [
        os.path.join(SCRIPT_DIR, "..", "src", "guide_pulse", "config", "guide_pulse.yaml"),
        os.path.join(SCRIPT_DIR, "..", "..", "share", "guide_pulse", "config", "guide_pulse.yaml"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            params = yaml.safe_load(open(path))["guide_bridge"]["ros__parameters"]
            return {
                "broker": params.get("broker_uri", "tcp://localhost:11883"),
                # robot_id 由 Server enroll 分配；本地 Mock（enroll_mock_server
                # 首个 SN）分配 robot-001
                "robot": "robot-001",
            }
    return {}


def rfc3339(epoch_seconds):
    """Unix 秒 -> RFC3339 UTC（毫秒精度）。"""
    ms = int(round(epoch_seconds * 1000))
    seconds, millis = divmod(ms, 1000)
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(seconds)) + f".{millis:03d}Z"


def uuid7():
    """UUIDv7：48bit 毫秒时间戳 + ver7 + variant + 随机（契约要求）。"""
    import secrets
    ts = int(time.time() * 1000)
    bb = bytearray(secrets.token_bytes(16))
    bb[0:6] = ts.to_bytes(6, "big")
    bb[6] = (bb[6] & 0x0F) | 0x70  # version 7
    bb[8] = (bb[8] & 0x3F) | 0x80  # variant 10xx
    h = bb.hex()
    return f"{h[0:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:32]}"


def main():
    config_defaults = load_config_defaults()
    parser = argparse.ArgumentParser(
        description="展厅讲解后端模拟器（scene 迎宾 / navigate 导航+汽车讲解 / tts 循环播报）")
    parser.add_argument("--broker", default=os.environ.get("GUIDE_PULSE_BROKER",
                        config_defaults.get("broker", "tcp://localhost:11883")),
                        help="MQTT broker 地址（默认取工程配置）")
    parser.add_argument("--robot", default=config_defaults.get("robot", "robot-001"),
                        help="目标 robot_id（enroll 分配的 ID，本地 Mock 默认 robot-001）")
    parser.add_argument("--prefix", default="robotCar/v1/robots",
                        help="主题前缀（RobotCar Server 固定值）")
    parser.add_argument("--username", default="",
                        help="MQTT 用户名（本地匿名 broker 无需）")
    parser.add_argument("--password", default="", help="MQTT 密码（同上）")
    sub = parser.add_subparsers(dest="command", required=True)

    p_welcome = sub.add_parser("welcome", help="下发迎宾指令（down/commands type=scene）")
    p_welcome.add_argument("--text", dest="texts", action="append", default=[],
                           help="迎宾词（可重复多次=多条；缺省用机器人本地默认词）")
    p_welcome.add_argument(
        "--qrcode", default="",
        help="迎宾二维码 PNG 路径（v0.22.0：base64 后随指令下发，脸屏常驻显示）")
    p_welcome.add_argument("--expires", type=int, default=60, help="指令有效期秒（默认 60）")

    p_nav = sub.add_parser("navigate", help="下发导航+讲解指令（type=navigate）")
    p_nav.add_argument("point", help="点位名（1..64 个 Unicode 标量，如「3号展台」）")
    p_nav.add_argument("--text", dest="texts", action="append", default=[],
                       help="讲解词（可重复多次=多条；缺省到达后不播报）")
    p_nav.add_argument("--expires", type=int, default=60, help="指令有效期秒（默认 60）")

    p_relocate = sub.add_parser(
        "relocate", help="下发地图重定位指令（type=relocate，回 relocate.result）")
    p_relocate.add_argument("point", help="点位名（重定位到该航点，机器人不动）")
    p_relocate.add_argument("--text", dest="texts", action="append", default=[],
                            help="重定位成功后的播报词（可重复多条；缺省播"
                                 "「{点位名}点重定位成功」）")
    p_relocate.add_argument("--expires", type=int, default=60, help="指令有效期秒（默认 60）")
    p_relocate.add_argument("--wait-result", type=float, default=3.0,
                            help="发布后等待 relocate.result 回执的秒数（默认 3，0 不等待）")

    p_pointmap = sub.add_parser(
        "point-map", help="下发点位映射表全量同步（type=point_map，v0.25.0，"
                          "回 point_map.result）")
    p_pointmap.add_argument(
        "entries", nargs="+", metavar="CODE=NAME",
        help='映射条目（1..256 条，全量替换机器人本地表），如 "A=比亚迪ATTO 1"')
    p_pointmap.add_argument("--expires", type=int, default=120, help="指令有效期秒（默认 120）")
    p_pointmap.add_argument("--wait-result", type=float, default=3.0,
                            help="发布后等待 point_map.result 回执的秒数（默认 3，0 不等待）")

    p_play = sub.add_parser("play", help="下发列表循环播报指令（type=tts，有回执）")
    p_play.add_argument("texts", nargs="+",
                        help="播报列表文本（1..20 条，空格分隔多条；允许单条）")
    p_play.add_argument("--mode", choices=["sequence", "shuffle"], default="sequence",
                        help="播报模式（默认 sequence 按序循环）")
    p_play.add_argument("--interval", type=int, default=30,
                        help="每两条内容之间的播报间隔秒（契约恒必填，默认 30）")
    p_play.add_argument("--expires", type=int, default=120, help="指令有效期秒（默认 120）")
    p_play.add_argument("--wait-result", type=float, default=2.0,
                        help="发布后等待 tts.result 回执的秒数（默认 2，0 不等待）")

    p_stop = sub.add_parser("stop", help="下发停止指令（type=tts.stop，无回执）")
    p_stop.add_argument("--expires", type=int, default=120, help="指令有效期秒（默认 120）")

    p_navstop = sub.add_parser("nav-stop", help="下发停止导航指令（type=navigate.stop，无回执）")
    p_navstop.add_argument("--expires", type=int, default=120, help="指令有效期秒（默认 120）")

    args = parser.parse_args()

    prefix = args.prefix.rstrip("/")
    down_topic = f"{prefix}/{args.robot}/down/commands"
    up_topic = f"{prefix}/{args.robot}/up/commands"

    receipts = []
    got_receipt = threading.Event()

    def on_message(client, userdata, msg):
        try:
            root = json.loads(msg.payload.decode("utf-8", "replace"))
        except ValueError:
            return
        if not root.get("type", "").endswith(".result"):
            return
        payload = root.get("payload", {})
        receipts.append((payload.get("operation_id"), root.get("event_id"),
                         root.get("type"), payload.get("outcome"),
                         payload.get("reason", "")))
        got_receipt.set()
        reason = f" reason={payload['reason']}" if payload.get("reason") else ""
        print(f"[后端] 收到回执 type={root.get('type')} event_id={root.get('event_id')} "
              f"outcome={payload.get('outcome')}{reason}")

    # 兼容 paho-mqtt 1.x 与 2.x 的构造方式
    client_id = f"guide-pulse-backend-{uuid.uuid4().hex[:8]}"
    try:
        client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
            protocol=mqtt.MQTTv5,
        )
    except (AttributeError, TypeError):
        client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv5)
    if args.username:
        client.username_pw_set(args.username, args.password)

    def on_connect(client, userdata, flags, reason_code, properties):
        if int(getattr(reason_code, "value", reason_code)) == 0:
            print(f"[后端] 已连接 broker: {args.broker}")
            client.subscribe(up_topic, qos=1)  # 回执面（v0.6.0 tts.result）
        else:
            print(f"[后端] 连接失败 reason={reason_code}", file=sys.stderr)
            sys.exit(1)

    client.on_connect = on_connect
    client.on_message = on_message
    url = urlparse(args.broker if "://" in args.broker else f"tcp://{args.broker}")
    client.connect(url.hostname, url.port or 11883, keepalive=30)
    client.loop_start()
    time.sleep(0.3)

    def build_event(event_type, payload, expires=None):
        """契约 command-event 信封（严格 8 字段）。"""
        now = time.time()
        ttl = expires if expires is not None else args.expires
        return {
            "spec_version": "1.0",
            "event_id": uuid7(),
            "robot_id": args.robot,
            "direction": "down",
            "type": event_type,
            "occurred_at": rfc3339(now),
            "expires_at": rfc3339(now + ttl),
            "payload": payload,
        }

    operation_id = uuid7()
    if args.command == "welcome":
        payload = {"operation_id": operation_id, "scene": "welcome"}
        if args.texts:
            payload["texts"] = args.texts  # v0.20.0 可选随动播报
        qrcode_desc = "无"
        if args.qrcode:
            with open(args.qrcode, "rb") as f:
                import base64
                payload["qrcode_png_base64"] = base64.b64encode(f.read()).decode()
            qrcode_desc = f"{args.qrcode}({len(payload['qrcode_png_base64'])}B base64)"
        event = build_event("scene", payload, expires=args.expires)
        desc = f"迎宾 scene=welcome 迎宾词={len(args.texts)}条（0=本地默认） 二维码={qrcode_desc}"
    elif args.command == "navigate":
        payload = {"operation_id": operation_id, "point_name": args.point}
        if args.texts:
            payload["texts"] = args.texts
        event = build_event("navigate", payload, expires=args.expires)
        desc = f"导航 point={args.point} 讲解词={len(args.texts)}条（0=到达不播报）"
    elif args.command == "relocate":
        payload = {"operation_id": operation_id, "point_name": args.point}
        if args.texts:
            payload["texts"] = args.texts
        event = build_event("relocate", payload, expires=args.expires)
        desc = f"重定位 point={args.point} 播报词={len(args.texts)}条（0=本地默认）"
    elif args.command == "point-map":
        points = []
        for entry in args.entries:
            code, sep, name = entry.partition("=")
            if not sep or not code or not name:
                parser.error(f'映射条目格式应为 CODE=NAME: "{entry}"')
            points.append({"code": code, "name": name})
        payload = {"operation_id": operation_id, "points": points}
        event = build_event("point_map", payload, expires=args.expires)
        desc = f"点位映射表同步 {len(points)} 条（全量替换）"
    elif args.command == "play":
        event = build_event("tts", {
            "operation_id": operation_id,
            "texts": args.texts,
            "mode": args.mode,
            "interval_seconds": args.interval,
        })
        desc = f"循环播报 mode={args.mode} interval={args.interval}s 条数={len(args.texts)}"
    elif args.command == "stop":
        event = build_event("tts.stop", {"operation_id": operation_id})
        desc = "stop"
    else:  # nav-stop
        event = build_event("navigate.stop", {"operation_id": operation_id})
        desc = "nav-stop"

    info = client.publish(down_topic, json.dumps(event, ensure_ascii=False), qos=1)
    info.wait_for_publish(timeout=5)
    if not info.is_published():
        print("[后端] 发布失败：消息未确认送达 broker（检查认证/网络）", file=sys.stderr)
        sys.exit(1)
    print(f"[后端] 已下发 {desc} 指令: operation_id={operation_id} topic={down_topic}")

    # v0.6.0：tts 必须回执；relocate（2026-10-17 接入）回一次性最终结果
    # relocate.result；point_map（v0.25.0）回 point_map.result。
    # scene/navigate/tts.stop/navigate.stop 不在本工具等待
    expects_receipt = args.command in ("play", "relocate", "point-map")
    if expects_receipt and args.wait_result > 0:
        deadline = time.time() + args.wait_result
        while time.time() < deadline and not any(
                r[0] == operation_id for r in receipts):
            time.sleep(0.1)
        if not any(r[0] == operation_id for r in receipts):
            print("[后端] 等待回执超时（未收到 *.result）", file=sys.stderr)
    elif not expects_receipt:
        print("[后端] 该指令无回执（发后即忘）；消息观察请用 mqtt_monitor.py")

    client.loop_stop()
    client.disconnect()
    # 非零退出码便于脚本断言回执缺失场景
    if expects_receipt and args.wait_result > 0 and not any(
            r[0] == operation_id for r in receipts):
        sys.exit(2)


if __name__ == "__main__":
    main()
