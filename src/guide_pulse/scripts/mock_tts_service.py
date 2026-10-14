#!/usr/bin/env python3
"""Mock SDK TTS 服务：在本机模拟机器人固件提供的 /aimdk_5Fmsgs/srv/PlayTts 服务。

用途：没有真机时配合 guide_pulse 做端到端联调——桥接节点照常调用 SDK 服务名，
本脚本扮演服务端，打印收到的每一个 PlayTts 请求（文本/优先级/打断等），
并按预设返回成功或失败回执。

用法（需先 source 工作区 install/setup.bash 以获得 aimdk_msgs）：
  python3 mock_tts_service.py                 # 全部返回成功
  python3 mock_tts_service.py --fail          # 全部返回业务失败（测失败路径）
  python3 mock_tts_service.py --service /xxx  # 自定义服务名

注意：真机上机器人固件本身提供该服务，请勿运行本脚本，避免服务名冲突。
"""

import argparse

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import CommonState, TtsPriorityLevel
from aimdk_msgs.srv import PlayTts

# 优先级档位名称，用于日志可读性
LEVEL_NAMES = {
    TtsPriorityLevel.UNKNOWN: "UNKNOWN",
    TtsPriorityLevel.BACKGROUND_L1: "BACKGROUND_L1",
    TtsPriorityLevel.SERVICE_L2: "SERVICE_L2",
    TtsPriorityLevel.MISSION_L4: "MISSION_L4",
    TtsPriorityLevel.INTERACTION_L6: "INTERACTION_L6",
    TtsPriorityLevel.WARNING_L8: "WARNING_L8",
    TtsPriorityLevel.SAFETY_L10: "SAFETY_L10",
}


class MockTtsService(Node):
    """模拟 SDK PlayTts 服务端。"""

    def __init__(self, fail_all: bool, delay_seconds: float, service_name: str):
        super().__init__("mock_tts_service")
        self._fail_all = fail_all
        self._delay = delay_seconds
        self._count = 0
        self._srv = self.create_service(PlayTts, service_name, self._handle)
        self.get_logger().info(
            f"Mock PlayTts 服务已就绪 ({service_name})，"
            f"fail_all={fail_all} delay={delay_seconds}s"
        )

    def _handle(self, request, response):
        self._count += 1
        req = request.tts_req
        level_name = LEVEL_NAMES.get(req.priority_level.value, f"L{req.priority_level.value}")
        self.get_logger().info(
            f"[#{self._count}] 收到 PlayTts: text='{req.text}' priority={level_name} "
            f"weight={req.priority_weight} interrupt={req.is_interrupted} "
            f"domain='{req.domain}' trace_id='{req.trace_id}'"
        )

        # 可选延迟，模拟流式合成/播报耗时
        if self._delay > 0:
            import time
            time.sleep(self._delay)

        ok = not self._fail_all
        response.header.header.stamp = self.get_clock().now().to_msg()
        response.header.header.code = 0 if ok else 1
        response.header.status.value = CommonState.SUCCESS if ok else CommonState.FAILURE
        response.header.message = "mock ok" if ok else "mock rejected"
        response.tts_resp.text = req.text
        response.tts_resp.priority_level = req.priority_level
        response.tts_resp.priority_weight = req.priority_weight
        response.tts_resp.domain = req.domain
        response.tts_resp.trace_id = req.trace_id
        response.tts_resp.is_success = ok
        response.tts_resp.error_message = "" if ok else "mock synthesis failure"
        response.tts_resp.estimated_duration = 0  # SDK 当前版本固定返回 0
        return response


def main():
    parser = argparse.ArgumentParser(description="Mock SDK PlayTts 服务")
    parser.add_argument("--fail", action="store_true", help="所有请求返回业务失败")
    parser.add_argument("--delay", type=float, default=0.0, help="处理延迟（秒），模拟播报耗时")
    parser.add_argument("--service", default="/aimdk_5Fmsgs/srv/PlayTts", help="服务名")
    args = parser.parse_args()

    rclpy.init()
    node = MockTtsService(args.fail, args.delay, args.service)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
