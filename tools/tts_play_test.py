#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TTS 播放测试工具：直接调用机器人本体 SDK 的 /aimdk_5Fmsgs/srv/PlayTts
服务播放一段文本，打印 SDK 回执，用于验证 TTS 链路与试听发音效果。

用法：
    python3 tts_play_test.py                    # 播放内置默认话术（粤语自我介绍）
    python3 tts_play_test.py "你好，世界"        # 播放指定文本
    python3 tts_play_test.py --stop             # 最高优先级抢占，停止当前播报

依赖环境（Jetson 本体已具备）：
    source /opt/ros/humble/setup.bash
    source /opt/liwei/guide_pulse/install/setup.bash   # 提供 aimdk_msgs
"""

import argparse
import sys
import uuid

import rclpy
from rclpy.node import Node

try:
    from aimdk_msgs.srv import PlayTts
except ImportError:
    # 环境未 source 时给出明确指引（aimdk_msgs 来自 guide_pulse 的 install 目录）
    print(
        "错误：找不到 aimdk_msgs。请先加载 ROS 环境后再运行：\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source /opt/liwei/guide_pulse/install/setup.bash",
        file=sys.stderr,
    )
    sys.exit(2)

# PlayTts 服务名（SDK 固定，勿改）
TTS_SERVICE = "/aimdk_5Fmsgs/srv/PlayTts"

# TtsPriorityLevel 取值（见 aimdk_msgs/interaction/msg/TtsPriorityLevel.msg）
PRIORITY_INTERACTION_L6 = 6   # 交互响应层，业务播报默认档
PRIORITY_SAFETY_L10 = 10      # 生命安全层，停止抢占用

# 默认测试话术：小T 粤语自我介绍（2026-10-19 TTS 播放测试）
DEFAULT_TEXT = (
    "我係比亚迪品牌体验中心嘅机器人导览员小T，身高102cm，"
    "主要负责带你逛展厅、介绍比亚迪同腾势嘅展车，"
    "按你嘅用车需求推荐车型，解答品牌、技术相关问题，"
    "有需要随时叫我就得啦。"
)

# 停止抢占文本：SSML 静音段（与 guide_pulse/tts_demo 停止策略一致；
# 勿用纯标点——任务无法完结会把仲裁通道卡住 30-60 秒）
STOP_TEXT = '<speak><break time="300ms"/></speak>'


def main() -> int:
    parser = argparse.ArgumentParser(description="机器人本体 TTS 播放测试")
    parser.add_argument("text", nargs="?", default=DEFAULT_TEXT, help="要播放的文本（默认：粤语自我介绍）")
    parser.add_argument("--stop", action="store_true", help="不播新文本，抢占停止当前播报")
    parser.add_argument("--timeout", type=float, default=10.0, help="服务调用超时秒数（默认 10）")
    args = parser.parse_args()

    rclpy.init()
    node = Node("tts_play_test")
    cli = node.create_client(PlayTts, TTS_SERVICE)

    # 等待服务上线
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"TTS 服务不可达：{TTS_SERVICE}（机器人 SDK 是否已启动？）")
        rclpy.shutdown()
        return 2

    req = PlayTts.Request()
    req.header.header.stamp = node.get_clock().now().to_msg()

    if args.stop:
        # 停止：最高优先级 + 最大加权 + 打断 + 静音文本，立即掐断当前播报
        req.tts_req.text = STOP_TEXT
        req.tts_req.priority_level.value = PRIORITY_SAFETY_L10
        req.tts_req.priority_weight = 99
        req.tts_req.is_interrupted = True
        req.tts_req.trace_id = f"stop-{uuid.uuid4().hex[:8]}"
        node.get_logger().info("发送停止抢占请求 ...")
    else:
        req.tts_req.text = args.text
        req.tts_req.priority_level.value = PRIORITY_INTERACTION_L6
        req.tts_req.priority_weight = 0
        req.tts_req.is_interrupted = True
        req.tts_req.trace_id = f"test-{uuid.uuid4().hex[:8]}"
        node.get_logger().info(f"发送播放请求（{len(args.text)} 字）：{args.text}")

    req.tts_req.domain = "tts_play_test"

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)

    if not future.done():
        node.get_logger().error(f"服务调用超时（{args.timeout}s），未收到回执")
        rclpy.shutdown()
        return 3

    resp = future.result()
    code = resp.header.status.value
    ok = code == 0 and resp.tts_resp.is_success
    node.get_logger().info(
        f"回执：header.code={code} message={resp.header.message!r} "
        f"is_success={resp.tts_resp.is_success} "
        f"error_message={resp.tts_resp.error_message!r} "
        f"estimated_duration_ms={resp.tts_resp.estimated_duration}"
    )

    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
