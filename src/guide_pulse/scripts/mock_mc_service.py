#!/usr/bin/env python3
"""Mock 厂家运控服务：模拟机器人本体固件的 SetMcAction/GetMcAction 服务。

用途：没有真机时配合 guide_pulse 做端到端联调——迎宾指令（scene=welcome）
到达后桥接节点会直调这两个服务确保人形形态，本脚本扮演运控服务端，
按简化的动作转移状态机推演并打印收到的每一条请求：

  初始状态：QUADRUPED_STAND_DEFAULT（狗站立，迎宾最典型起点）
  · 收到 SetMcAction(X)：切到动作 X；受理后 --transition-ms 毫秒内
    GetMcAction 回「切换中(200)」（模拟动作执行耗时），之后回
    RUNNING(100)。特别地 QUADRUPED_TO_BIPED_ROTATE 完成后自动落到
    BIPED_LOCOMOTION_WBC（模拟运控的「桥接动作」自动后继，SDK 文档行为）
  · 收到 GetMcAction：回当前动作 + 按上述规则回状态
  · --reject-next：下一条 SetMcAction 回业务拒绝（reason=5 低电量，
    测失败路径：迎宾变形失败不阻断播报）

真机语义参考 t1_slam_nav_ws 的 stand_if_down.py / mc_velocity_bridge.py
（转移表与回执口径在那边真机校准过；本 Mock 只做联调推演用）。

用法（需先 source 工作区 install/setup.bash 以获得 aimdk_msgs）：
  python3 mock_mc_service.py                        # 狗站立起步，全成功
  python3 mock_mc_service.py --start PASSIVE_DEFAULT  # 趴姿起步（四步起立）
  python3 mock_mc_service.py --transition-ms 2000   # 动作受理后 2s 内回切换中
  python3 mock_mc_service.py --reject-next           # 首条动作被拒
  python3 mock_mc_service.py --get /xxx --set /yyy   # 自定义服务名

注意：真机上这两个服务由运控固件（SoC0）提供，请勿运行本脚本，
避免服务名冲突。
"""

import argparse

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import CommonState
from aimdk_msgs.srv import GetMcAction, SetMcAction

# McActionStatus.value
STATUS_RUNNING = 100
STATUS_TRANSITION = 200

# 变形动作发出后的自动后继（运控自己落到人形运动模式，勿手动发）
AUTO_NEXT = {"QUADRUPED_TO_BIPED_ROTATE": "BIPED_LOCOMOTION_WBC"}


class MockMcService(Node):
    """模拟运控 SetMcAction/GetMcAction 服务端（简化状态机）。"""

    def __init__(
        self, start_action: str, get_name: str, set_name: str,
        transition_ms: int
    ):
        super().__init__("mock_mc_service")
        self._action = start_action
        self._reject_next = False
        self._set_count = 0
        self._get_count = 0
        self._transition_ms = transition_ms / 1000.0
        # 动作切换截止时刻：time.monotonic() 之后才算 RUNNING
        #（初始 0 = 启动即稳定）
        self._transition_until = 0.0
        self._set_srv = self.create_service(SetMcAction, set_name, self._on_set)
        self._get_srv = self.create_service(GetMcAction, get_name, self._on_get)
        self.get_logger().info(
            f"Mock 运控服务已就绪: {set_name} / {get_name}，"
            f"当前动作={start_action} transition={transition_ms}ms"
        )

    def _on_set(self, request, response):
        self._set_count += 1
        action = request.command.action_desc
        self.get_logger().info(
            f"[set#{self._set_count}] 收到 SetMcAction: action='{action}' "
            f"source='{request.source}'"
        )
        resp = response.response
        resp.header.stamp = self.get_clock().now().to_msg()
        if self._reject_next:
            self._reject_next = False
            self.get_logger().warn(f"[set#{self._set_count}] 模拟拒绝（低电量）")
            resp.header.code = 1
            resp.state.value = CommonState.FAILURE
            resp.state.reason = 5  # REASON_LOW_BATTERY
            return response
        resp.header.code = 0
        resp.state.value = CommonState.SUCCESS
        resp.state.reason = 0
        self._action = AUTO_NEXT.get(action, action)
        import time
        self._transition_until = time.monotonic() + self._transition_ms
        self.get_logger().info(
            f"[set#{self._set_count}] 动作受理，当前动作 -> {self._action}"
            + (f"（{self._transition_ms:.1f}s 内报切换中）" if self._transition_ms else "")
        )
        return response

    def _on_get(self, _request, response):
        import time
        self._get_count += 1
        response.header.stamp = self.get_clock().now().to_msg()
        response.info.action_desc = self._action
        response.info.source = "mock"
        # 切换窗口内报「切换中」，之后报「运行中」——真机上 action_desc 在
        # 切换开始时就已变成目标名，只有 status 能区分稳定与否
        response.info.status.value = (
            STATUS_TRANSITION if time.monotonic() < self._transition_until
            else STATUS_RUNNING
        )
        response.info.switch_ready = True
        return response


def main():
    parser = argparse.ArgumentParser(description="Mock 厂家运控（SetMcAction/GetMcAction）服务")
    parser.add_argument(
        "--start", default="QUADRUPED_STAND_DEFAULT",
        help="初始动作（默认狗站立；测趴姿起立可给 PASSIVE_DEFAULT）")
    parser.add_argument(
        "--transition-ms", type=int, default=0,
        help="动作受理后这么多毫秒内 GetMcAction 回「切换中(200)」"
             "（模拟动作执行耗时/真机 2003 拒绝场景）")
    parser.add_argument(
        "--reject-next", action="store_true",
        help="下一条 SetMcAction 回业务拒绝（reason=5 低电量）")
    parser.add_argument(
        "--get", default="/mock_mc/GetMcAction", help="GetMcAction 服务名")
    parser.add_argument(
        "--set", default="/mock_mc/SetMcAction", help="SetMcAction 服务名")
    args = parser.parse_args()

    rclpy.init()
    node = MockMcService(args.start, args.get, args.set, args.transition_ms)
    if args.reject_next:
        node._reject_next = True  # noqa: SLF001 - 测试脚本直接注入
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
