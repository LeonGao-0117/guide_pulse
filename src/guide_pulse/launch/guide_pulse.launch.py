# ============================================================================
# guide_pulse 启动文件（ROS2 节点方式，colcon build 后使用）
#
# 所有运行参数以 config/guide_pulse.yaml 为准；robot_id 由 Server 在首次
# enroll 时分配，不作为启动参数。命令行仅用于临时覆盖与开关 mock：
#   ros2 launch guide_pulse guide_pulse.launch.py                       # 全按 yaml
#   ros2 launch guide_pulse guide_pulse.launch.py broker_uri:=tcp://127.0.0.1:1884
#   ros2 launch guide_pulse guide_pulse.launch.py mock_tts:=true        # 本地联调
#
# SN 来源（批量部署零手工配置）：
#   serial_number 参数/yaml > GUIDE_PULSE_SERIAL_NUMBER 环境变量 >
#   本地 /robot/info/sn > SSH 运控板读取（run@10.1.1.100，可用
#   GUIDE_PULSE_MC_SSH 覆盖；默认密码方式认证 GUIDE_PULSE_MC_PASSWORD=1，
#   置空回落 ssh-copy-id 免密钥）。
# ============================================================================
import os
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SN_FILE = "/robot/info/sn"


def _trim(value):
    return value.strip() if isinstance(value, str) else ""


def _sn_over_ssh(target):
    """SSH 读运控板 SN：默认密码方式（sshpass，板间内网出厂密码 1），
    密码显式置空或无 sshpass 时回落免密钥 BatchMode。"""
    password = os.environ.get("GUIDE_PULSE_MC_PASSWORD", "1")
    if password:
        try:
            result = subprocess.run(
                ["sshpass", "-e", "ssh", "-o", "StrictHostKeyChecking=accept-new",
                 "-o", "ConnectTimeout=3", target, "cat", SN_FILE],
                capture_output=True, text=True, timeout=6, check=True,
                env={**os.environ, "SSHPASS": password})
            return _trim(result.stdout.splitlines()[0] if result.stdout else "")
        except (OSError, subprocess.SubprocessError, IndexError):
            return ""
    try:
        result = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
             target, "cat", SN_FILE],
            capture_output=True, text=True, timeout=6, check=True)
        return _trim(result.stdout.splitlines()[0] if result.stdout else "")
    except (OSError, subprocess.SubprocessError, IndexError):
        return ""


def _resolve_serial_number(context):
    """按 参数 > 环境变量 > 本地文件 > SSH 运控板 解析 SN。

    只在三级静态来源都为空时才尝试 SSH（单次、3s 超时、免交互，
    失败仅告警交由节点报缺配置错误——手动联调路径的兜底）。
    """
    sn = _trim(LaunchConfiguration("serial_number").perform(context))
    if sn:
        return sn, "launch 参数"
    sn = _trim(os.environ.get("GUIDE_PULSE_SERIAL_NUMBER", ""))
    if sn:
        return sn, "环境变量 GUIDE_PULSE_SERIAL_NUMBER"
    try:
        with open(SN_FILE, "r", encoding="ascii") as f:
            sn = _trim(f.readline())
        if sn:
            return sn, f"本地文件 {SN_FILE}"
    except OSError:
        pass
    target = os.environ.get("GUIDE_PULSE_MC_SSH", "run@10.1.1.100")
    sn = _sn_over_ssh(target)
    if sn:
        return sn, f"运控板 {target}"
    return "", None


def _launch_setup(context):
    config = PathJoinSubstitution(
        [FindPackageShare("guide_pulse"), "config", "guide_pulse.yaml"]
    )

    # 仅命令行显式传入的参数才覆盖 yaml（默认值为空 = 不覆盖）
    overrides = {}
    for key in ("broker_uri", "enroll_origin", "serial_number",
                "mock_nav_seconds"):
        value = LaunchConfiguration(key).perform(context)
        if value:
            overrides[key] = int(value) if key == "mock_nav_seconds" else value
    if "serial_number" not in overrides:
        sn, source = _resolve_serial_number(context)
        if sn:
            overrides["serial_number"] = sn
            print(f"[guide_pulse.launch] serial_number={sn}（取自 {source}）")

    # 桥接节点：enroll 换凭据 -> MQTT 下行指令 -> scene 迎宾 / navigate
    # 导航+讲解 / tts 循环播报 -> tts.result 回执
    guide_bridge = Node(
        package="guide_pulse",
        executable="guide_bridge",
        name="guide_bridge",
        output="screen",
        parameters=[config, overrides],
    )

    # Mock SDK TTS 服务（可选）：进程方式运行包内脚本
    mock_tts_node = ExecuteProcess(
        cmd=[sys.executable,
             PathJoinSubstitution([FindPackageShare("guide_pulse"), "..", "..",
                                   "lib", "guide_pulse", "mock_tts_service.py"])],
        output="screen",
        condition=IfCondition(LaunchConfiguration("mock_tts")),
    )

    return [guide_bridge, mock_tts_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "broker_uri", default_value="",
            description="临时覆盖 yaml 中的 MQTT broker 地址（默认空=用 yaml）",
        ),
        DeclareLaunchArgument(
            "enroll_origin", default_value="",
            description="临时覆盖 yaml 中的 enroll 注册面地址（默认空=用 yaml）",
        ),
        DeclareLaunchArgument(
            "serial_number", default_value="",
            description="临时覆盖序列号（默认空=自动获取：环境变量/板端文件/SSH 运控板）",
        ),
        DeclareLaunchArgument(
            "mock_nav_seconds", default_value="",
            description="临时覆盖 Mock 导航模拟秒数（默认空=用 yaml；联调演示用）",
        ),
        DeclareLaunchArgument(
            "mock_tts", default_value="false",
            description="true 时额外启动 Mock SDK PlayTts 服务（仅本地联调，真机勿开）",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
