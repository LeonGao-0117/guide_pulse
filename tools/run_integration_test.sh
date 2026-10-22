#!/usr/bin/env bash
# ============================================================================
# guide_pulse 端到端集成测试（无需真机）
#
# 链路：enroll_mock_server(注册面) <- guide_bridge(enroll 换凭据)
#       backend_simulator(scene/navigate/relocate/tts/tts.stop/navigate.stop)
#       -> mosquitto(1884)
#       -> guide_bridge -> mock_tts_service(SDK PlayTts Mock) + MockNavExecutor
#                     \-> mock_mc_service(运控 SetMcAction/GetMcAction Mock)
# 契约：RobotCar 协议 v0.9.0/v0.10.0（enroll）+ v0.5.0/v0.6.0（tts 循环播报/
#       回执）+ v0.17.0（scene 迎宾）+ v0.16.0/v0.20.0（navigate 导航+随动
#       播报）+ v0.23.0（navigate.stop）+ v0.24.0（navigate/scene 一次性
#       结果回执）+ v0.21.0（relocate 地图重定位，2026-10-17 接入）。
#       断言依据 Mock 服务日志、桥接日志
#       与 simulator 收到的回执。
#
# 前置：已执行 scripts/build.sh（工作区 install/ 就绪）
# 用法：bash tools/run_integration_test.sh
# ============================================================================
set -euo pipefail

GUIDE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${GUIDE_TEST_MQTT_PORT:-1884}"
ENROLL_PORT="${GUIDE_TEST_ENROLL_PORT:-18080}"
WORK_DIR="$(mktemp -d /tmp/guide_pulse_test.XXXXXX)"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  sleep 1
  # 兜底：正常退出路径若卡住，强杀避免进程泄漏
  for pid in "${PIDS[@]:-}"; do
    kill -9 "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  rm -rf "$WORK_DIR"
}
trap cleanup EXIT

log() { echo -e "\033[1;32m[测试]\033[0m $*"; }
fail() { echo -e "\033[1;31m[失败]\033[0m $*" >&2; exit 1; }

wait_log() {  # wait_log <pattern> <file> <timeout_seconds>
  local pattern="$1" file="$2" timeout="${3:-10}"
  for _ in $(seq 1 $((timeout * 5))); do
    grep -q "$pattern" "$file" 2>/dev/null && return 0
    sleep 0.2
  done
  return 1
}

wait_count() {  # wait_count <pattern> <file> <want_at_least> <timeout_seconds>
  local pattern="$1" file="$2" want="$3" timeout="${4:-10}"
  for _ in $(seq 1 $((timeout * 5))); do
    [ "$(grep -c "$pattern" "$file" 2>/dev/null || true)" -ge "$want" ] && return 0
    sleep 0.2
  done
  return 1
}

# ---------------------------------------------------------------------------
# 0. 前置检查
# ---------------------------------------------------------------------------
[ -f "${GUIDE_DIR}/install/setup.bash" ] || fail "未找到 install/setup.bash，请先执行 bash scripts/build.sh"
command -v mosquitto >/dev/null || fail "缺少 mosquitto"

# ---------------------------------------------------------------------------
# 1. 启动本地 mosquitto（独立端口，避免影响系统服务）
# ---------------------------------------------------------------------------
cat > "${WORK_DIR}/mosquitto.conf" <<EOF
listener ${PORT} 127.0.0.1
allow_anonymous true
EOF
mosquitto -c "${WORK_DIR}/mosquitto.conf" -d
MOSQ_PID=$(pgrep -f "mosquitto -c ${WORK_DIR}/mosquitto.conf" | head -1 || true)
[ -n "$MOSQ_PID" ] && PIDS+=("$MOSQ_PID")
sleep 1

# mosquitto_pub/sub 命令行工具本机可能未安装，统一用 python paho（工程自带 deps）
mqtt_pub() {  # topic payload [qos]
  local qos="${3:-1}"
  PYTHONPATH="${GUIDE_DIR}/tools/deps" python3 - "$1" "$2" "$qos" "$PORT" <<'PYEOF'
import sys
import paho.mqtt.client as mqtt
topic, payload, qos, port = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
c = mqtt.Client(client_id="it-pub")
c.connect("127.0.0.1", port, 30)
c.loop_start()
c.publish(topic, payload, qos=qos).wait_for_publish(timeout=5)
c.loop_stop()
c.disconnect()
PYEOF
}
log "mosquitto 已启动 (port=${PORT})"

# ---------------------------------------------------------------------------
# 1b. 启动 enroll 注册面 Mock（协议 v0.9.0/v0.10.0：SN 换 robot_id+凭据）。
#     本地 broker 为匿名 mosquitto，凭据不校验，但 robot_id 由 Mock 发号：
#     首个登记的 SN（SIM-guide-it）分配 robot-001
# ---------------------------------------------------------------------------
python3 "${GUIDE_DIR}/tools/enroll_mock_server.py" --port "${ENROLL_PORT}" \
  > "${WORK_DIR}/enroll.log" 2>&1 &
ENROLL_PID=$!
PIDS+=("$ENROLL_PID")
wait_log "listening on" "${WORK_DIR}/enroll.log" 10 || fail "enroll Mock 未启动"
log "enroll Mock 已启动 (port=${ENROLL_PORT})"

# ---------------------------------------------------------------------------
# 2. source 工作区，启动 Mock SDK TTS 服务与桥接节点
# ---------------------------------------------------------------------------
set +u
source "${GUIDE_DIR}/install/setup.bash"
set -u

# 安装到包内的 Mock TTS 服务。使用独立服务名而非 SDK 真名：真机上机器人固件
# 会提供同名真服务并抢先应答，导致 Mock 收不到请求、断言误判失败。
MOCK_SERVICE="/guide_pulse_integration/mock_play_tts"
PYTHONUNBUFFERED=1 "${GUIDE_DIR}/install/guide_pulse/lib/guide_pulse/mock_tts_service.py" \
  --service "${MOCK_SERVICE}" > "${WORK_DIR}/mock_tts.log" 2>&1 &
MOCK_PID=$!
PIDS+=("$MOCK_PID")

# Mock 厂家运控服务（SetMcAction/GetMcAction，独立服务名防真机同名真服务
# 抢先应答）：迎宾指令到达后桥接应直调它确保人形形态，用例 2a 断言完整
# 转移序列（狗站立 -> 行走态 -> 变形 -> 自动落位 BIPED_LOCOMOTION_WBC）。
# transition 窗口 3s：用例 2c 注入「坐下」后迎宾链路必须落在窗口内才查
# 得到「切换中」。迎宾新增引导语先行播完（2026-10-17）才变形，播完等待
# 的兜底估制约 ~1s：桥接启动参数把估算压小（tts_speech_ms_per_char=10
# + suppress_tail_lead_ms=300）保证 ensure_biped 落在窗口内（估算值只
# 影响兜底超时/抑制尾部，不影响任何断言语义）
MOCK_MC_GET="/guide_pulse_integration/mock_mc_get"
MOCK_MC_SET="/guide_pulse_integration/mock_mc_set"
PYTHONUNBUFFERED=1 "${GUIDE_DIR}/install/guide_pulse/lib/guide_pulse/mock_mc_service.py" \
  --get "${MOCK_MC_GET}" --set "${MOCK_MC_SET}" --transition-ms 3000 \
  > "${WORK_DIR}/mock_mc.log" 2>&1 &
MOCK_MC_PID=$!
PIDS+=("$MOCK_MC_PID")

# 桥接节点：enroll 换凭据（SN SIM-guide-it -> robot-001）后连本地 mosquitto；
# mock_nav_seconds=2 模拟行走耗时（导航替换用例需要「在途」窗口）；
# SN 经 serial_number_file 自动获取（模拟真机从板端文件读取）。
# 默认迎宾词由测试显式传入：现场部署文案（代码默认值/yaml）随时会改，
# 断言不跟部署走（2026-10-16 比亚迪文案改动曾让用例 2a 误报失败）
DEFAULT_WELCOME="欢迎光临！我是展厅讲解机器人，很高兴为您服务。"
echo -n "SIM-guide-it" > "${WORK_DIR}/sn"
"${GUIDE_DIR}/install/guide_pulse/lib/guide_pulse/guide_bridge" --ros-args \
  -p broker_uri:="tcp://127.0.0.1:${PORT}" \
  -p enroll_origin:="http://127.0.0.1:${ENROLL_PORT}" \
  -p serial_number_file:="${WORK_DIR}/sn" \
  -p tts_service_name:="${MOCK_SERVICE}" \
  -p get_action_service:="${MOCK_MC_GET}" \
  -p set_action_service:="${MOCK_MC_SET}" \
  -p tts_wait_timeout_seconds:=10 \
  -p heartbeat_interval_seconds:=5 \
  -p mock_nav_seconds:=6 \
  -p mock_relocate_hold_polls:=16 \
  -p transform_wait_timeout_seconds:=40 \
  -p tts_speech_ms_per_char:=10 \
  -p suppress_tail_lead_ms:=300 \
  -p point_map_file:="${WORK_DIR}/point_map.json" \
  > "${WORK_DIR}/bridge.log" 2>&1 &
BRIDGE_PID=$!
PIDS+=("$BRIDGE_PID")

wait_log "从文件 .* 读取 serial_number=SIM-guide-it" "${WORK_DIR}/bridge.log" 15 || {
  cat "${WORK_DIR}/bridge.log" || true
  fail "桥接节点未从 SN 文件自动获取序列号"
}
wait_log "enroll 成功: robot_id=robot-001" "${WORK_DIR}/bridge.log" 15 || {
  cat "${WORK_DIR}/bridge.log" || true
  fail "桥接节点 enroll 失败"
}
wait_log "已订阅下行主题: robotCar/v1/robots/robot-001/down/commands" "${WORK_DIR}/bridge.log" 15 || {
  cat "${WORK_DIR}/bridge.log" || true
  fail "桥接节点未完成订阅（robotCar 前缀）"
}
log "guide_bridge 已就绪 (pid=${BRIDGE_PID}, robot_id=robot-001)"

# 等待 Mock 服务完成 DDS 发现（过早下发指令会被桥接按「服务不可用」丢弃）
wait_log "Mock PlayTts 服务已就绪" "${WORK_DIR}/mock_tts.log" 15 || {
  cat "${WORK_DIR}/mock_tts.log" || true
  fail "Mock TTS 服务未就绪"
}
log "Mock TTS 服务已就绪 (service=${MOCK_SERVICE})"

wait_log "Mock 运控服务已就绪" "${WORK_DIR}/mock_mc.log" 15 || {
  cat "${WORK_DIR}/mock_mc.log" || true
  fail "Mock 运控服务未就绪"
}
log "Mock 运控服务已就绪 (get=${MOCK_MC_GET} set=${MOCK_MC_SET})"
SIM="python3 ${GUIDE_DIR}/tools/backend_simulator.py --broker tcp://127.0.0.1:${PORT} --robot robot-001"

# ---------------------------------------------------------------------------
# 3. 用例 1：tts 循环播报基线 -> Mock 收到请求 + tts.result accepted
# ---------------------------------------------------------------------------
$SIM play "循环播报基线" > "${WORK_DIR}/sim1.log" 2>&1
wait_log "text='循环播报基线'" "${WORK_DIR}/mock_tts.log" 10 || fail "Mock 未收到 tts 首播请求"
# v0.23.0（d245fab）播报提档 level 7（权重 700>600，播报期间唤醒/L6 业务被
# 仲裁丢弃）。注意断言写 L7 而非 SYSTEM_L7：内置 aimdk_msgs（SDK v0.9.4.4）
# 的 TtsPriorityLevel 枚举只有 1/2/4/6/8/10 常量，Mock 对 7 走兜底渲染 "L7"
grep -q "priority=L7 weight=0" "${WORK_DIR}/mock_tts.log" || fail "播报优先级不是 L7（weight=0）"
wait_log "outcome=accepted" "${WORK_DIR}/sim1.log" 5 || {
  cat "${WORK_DIR}/sim1.log" || true
  fail "未收到 tts.result accepted 回执"
}
log "用例 1 通过：tts 送达 Mock（L7）且回执 accepted"
# 单业务占用（2026-10-16）：tts 受理后进入播报占用窗口，窗口内新业务被拒。
# 用 stop 显式释放（tts.stop 是解除占用的唯一指令），再进入迎宾用例
$SIM stop > /dev/null 2>&1

# ---------------------------------------------------------------------------
# 4. 用例 2：迎宾（scene=welcome）
#    2a. 未带 texts -> 迎宾词强制取映射表迎宾点（welcome_point_code=Y）
#        条目的 text（2026-10-19：本地默认迎宾词已移除），单轮播完不循环。
#        先推含 Y 播放词的初始表（1 条；2d 起的重表不含 Y 不影响本组）
# ---------------------------------------------------------------------------
$SIM point-map "Y=迎宾点=${DEFAULT_WELCOME}" > /dev/null 2>&1
wait_log "点位映射表同步: 1 条（含播放词 1 条）" "${WORK_DIR}/bridge.log" 5 \
  || fail "初始映射表（Y 迎宾词）未被同步"
$SIM welcome > /dev/null 2>&1
wait_log "迎宾开始" "${WORK_DIR}/bridge.log" 5 || fail "未见迎宾开始日志"
wait_log "迎宾词取自映射表迎宾点 Y" "${WORK_DIR}/bridge.log" 5 \
  || fail "迎宾词未取自映射表迎宾点"
# 迎宾引导语（2026-10-17 现场需求）：先播 TTS「哈喽哈喽！电池满格，
# 热情也满格」，播完再变形，变形完成后才播迎宾词——三段顺序都要断言
wait_log "text='哈喽哈喽！电池满格，热情也满格'" "${WORK_DIR}/mock_tts.log" 5 \
  || fail "未播迎宾引导语（welcome_lead_text）"
wait_log "迎宾引导语播完（.*），开始变形" "${WORK_DIR}/bridge.log" 10 \
  || fail "未见引导语播完日志"
# 迎宾变形：scene=welcome 应在引导语播完后直调运控确保人形，且
# **变形完成后才开播**（2026-10-15 需求：先变形、后人声）。Mock 从狗站立
# 起步，按转移表应依次发 行走态 -> 变形（ROTATE 后自动落位 BIPED_LOCOMOTION_WBC）
wait_log "迎宾变形进行中，变形完成后开始播报" "${WORK_DIR}/bridge.log" 10 || fail "未见变形等待日志"
wait_log "action='QUADRUPED_LOCOMOTION_DEFAULT'" "${WORK_DIR}/mock_mc.log" 10 || fail "迎宾变形未先进入四足行走态"
wait_log "action='QUADRUPED_TO_BIPED_ROTATE'" "${WORK_DIR}/mock_mc.log" 20 || fail "迎宾变形未发变形动作"
# 顺序断言（同一日志文件行号比较）：引导语播完 -> 首条变形动作 -> 变形
# 流程结束开播迎宾词，2026-10-17 时序需求（先播 TTS 再变形再说迎宾词）
LEAD_DONE_LINE=$(grep -n "迎宾引导语播完" "${WORK_DIR}/bridge.log" | head -1 | cut -d: -f1)
FIRST_MOTION_LINE=$(grep -n "迎宾变形: 发动作" "${WORK_DIR}/bridge.log" | head -1 | cut -d: -f1)
[ -n "${LEAD_DONE_LINE}" ] && [ -n "${FIRST_MOTION_LINE}" ] && \
  [ "${LEAD_DONE_LINE}" -lt "${FIRST_MOTION_LINE}" ] || {
  fail "引导语未先于变形动作播完（lead=${LEAD_DONE_LINE} motion=${FIRST_MOTION_LINE}）"
}
wait_log "迎宾变形完成: 已切到 BIPED_LOCOMOTION_WBC" "${WORK_DIR}/bridge.log" 30 || fail "未见迎宾变形完成日志"
wait_log "迎宾变形流程结束，开始迎宾播报" "${WORK_DIR}/bridge.log" 5 || fail "未见变形后开播日志"
wait_log "text='${DEFAULT_WELCOME}'" "${WORK_DIR}/mock_tts.log" 10 || fail "变形完成后未播映射表迎宾词"
# 顺序硬断言：bridge.log 中「变形结束」必须先于「迎宾播报会话开始」
WAIT_LINE=$(grep -n "迎宾变形流程结束，开始迎宾播报" "${WORK_DIR}/bridge.log" | head -1 | cut -d: -f1)
SPEAK_LINE=$(grep -n "播报会话开始: 迎宾:" "${WORK_DIR}/bridge.log" | head -1 | cut -d: -f1)
[ -n "$WAIT_LINE" ] && [ -n "$SPEAK_LINE" ] && [ "$SPEAK_LINE" -gt "$WAIT_LINE" ] \
  || fail "播报未等到变形完成之后（变形结束行=${WAIT_LINE} 播报开始行=${SPEAK_LINE}）"
sleep 9  # 迎宾词 22 字 speech~5s + interval 2s：若误循环，~7s 处会出现第二次
WELCOME_COUNT=$(grep -c "text='${DEFAULT_WELCOME}'" "${WORK_DIR}/mock_tts.log" || true)
[ "$WELCOME_COUNT" -eq 1 ] || fail "迎宾词应单轮播完不循环（播了 ${WELCOME_COUNT} 次）"
wait_log "单轮播完，会话结束" "${WORK_DIR}/bridge.log" 5 || fail "迎宾会话未按单轮结束"
# v0.24.0（P-038）：迎宾流程主体完成回 scene.result success
wait_log '"outcome":"success".*"type":"scene.result"' "${WORK_DIR}/bridge.log" 5 \
  || fail "迎宾完成未回 scene.result success"
log "用例 2a 通过：变形完成（行走态->ROTATE->WBC）后才开播，映射表迎宾词一轮结束"

# 2b. 带 texts -> App 指定迎宾词优先；再次迎宾应再次查形态——此时已是
# 人形（Mock 状态 BIPED_LOCOMOTION_WBC），应零动作直接确认（幂等）。
# 先 stop 释放 2a 迎宾占用窗口（单业务占用模型，窗口内新迎宾会被拒）
$SIM stop > /dev/null 2>&1
$SIM welcome --text "指定迎宾词甲" --text "指定迎宾词乙" > /dev/null 2>&1
wait_log "text='指定迎宾词甲'" "${WORK_DIR}/mock_tts.log" 10 || fail "Mock 未收到指定迎宾词甲"
wait_log "text='指定迎宾词乙'" "${WORK_DIR}/mock_tts.log" 15 || fail "Mock 未收到指定迎宾词乙（按序播第二条）"
wait_log "已是 BIPED_LOCOMOTION_WBC 且运行中，无需切换" "${WORK_DIR}/bridge.log" 10 || fail "第二次迎宾未做形态确认"
SET_COUNT=$(grep -c "收到 SetMcAction" "${WORK_DIR}/mock_mc.log" || true)
[ "$SET_COUNT" -eq 2 ] || fail "已是人形后再次迎宾不应再发动作（SetMcAction 共 ${SET_COUNT} 次，期望 2）"
sleep 5
COUNT_A=$(grep -c "text='指定迎宾词甲'" "${WORK_DIR}/mock_tts.log" || true)
[ "$COUNT_A" -eq 1 ] || fail "指定迎宾词不应循环（甲 ${COUNT_A} 次）"
# 优先级负断言：指令带 texts 时不得再播映射表迎宾词（指令 texts >
# 表 text；2a 播过一次 DEFAULT_WELCOME，之后不应再出现）
WELCOME_COUNT_2B=$(grep -c "text='${DEFAULT_WELCOME}'" "${WORK_DIR}/mock_tts.log" || true)
[ "${WELCOME_COUNT_2B}" -eq 1 ] || fail "指令 texts 应优先于映射表迎宾词（迎宾词播了 ${WELCOME_COUNT_2B} 次，期望仅 2a 的 1 次）"
log "用例 2b 通过：迎宾 texts 优先且按序单轮（甲→乙）；已人形时零动作确认"

# 2c. 动作切换中迎宾到达 -> 等稳定后再变形（复现真机 2026-10-15 故障：
#     App 点坐下 417ms 后迎宾到达，发 STAND 被运控 2003 拒 state=-1008）
# 先 stop 释放 2b 迎宾占用窗口：单业务占用模型下窗口内新迎宾会被拒
#（曾以 sleep 10 盲等窗口过期，链路快慢不均时间歇失败；stop 确定释放）
$SIM stop > /dev/null 2>&1
sleep 1
# 注入：向 Mock 运控发「坐下」（受理后 transition-ms 内报切换中 200），随即迎宾
python3 - "${MOCK_MC_SET}" <<'PYEOF' > /dev/null 2>&1
import sys
import rclpy
from aimdk_msgs.srv import SetMcAction
rclpy.init()
node = rclpy.create_node("it_mc_inject")
cli = node.create_client(SetMcAction, sys.argv[1])
cli.wait_for_service(timeout_sec=5)
req = SetMcAction.Request()
req.source = "integration_test"
req.command.action_desc = "QUADRUPED_SIT_DOWN_DEFAULT"
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
node.destroy_node()
rclpy.shutdown()
PYEOF
$SIM welcome > /dev/null 2>&1
wait_log "尚未稳定，等它稳定" "${WORK_DIR}/bridge.log" 10 || fail "切换中状态未进入等待稳定分支"
wait_count "收到 SetMcAction" "${WORK_DIR}/mock_mc.log" 5 30 || fail "等稳定后未走完坐下->人形变形（应再发 3 条动作）"
wait_count "迎宾变形完成: 已切到 BIPED_LOCOMOTION_WBC" "${WORK_DIR}/bridge.log" 2 30 || fail "切换中起点的迎宾变形未完成"
log "用例 2c 通过：动作切换中迎宾到达 -> 等稳定 -> 坐下->站立->变形成功"

# ---------------------------------------------------------------------------
# 4b. 点位映射表同步（v0.25.0 P-039）：navigate/relocate 的 point_name
#     唯一权威数据源（2026-10-17 现场需求：表空/未命中直接拒绝）——先推表，
#     用例 3 起的导航/重定位才有数据。code=同名使「导航开始: point=<原名>」
#     断言原样成立（日志打的是解析后的航点 code）；A/B 供 relocate 直配。
#     落盘已隔离到 WORK_DIR（launch 参数），不动真机持久化映射表
# ---------------------------------------------------------------------------
$SIM point-map "3号展台=3号展台" "A展台=A展台" "B展台=B展台" \
  "C展台=C展台" "D展台=D展台" "A=A点" "B=B点" > /dev/null 2>&1
wait_log "点位映射表同步: 7 条" "${WORK_DIR}/bridge.log" 5 \
  || fail "点位映射表未被全量替换（后续导航/重定位无数据）"
log "用例 2d 通过：点位映射表全量同步落盘"

# ---------------------------------------------------------------------------
# 4b-2. 点位映射表播放词（v0.28.0 P-042）：条目可选 text 随表落盘持久
#     （断电不丢的行为内核 = 整表原子写文件）；到达播报优先级第 2 级
#     验证——不带 texts 的 navigate 到达播表内 text（第 1 级=指令 texts
#     见用例 3，第 3 级=本地默认词 yaml 未配此处不验）。重推 8 条表 =
#     原 7 条（code=同名，后续用例断言不变）+ E展台 带播放词
# ---------------------------------------------------------------------------
$SIM point-map "3号展台=3号展台" "A展台=A展台" "B展台=B展台" \
  "C展台=C展台" "D展台=D展台" "A=A点" "B=B点" \
  "E展台=E展台=E展台播放词：由映射表下发" > /dev/null 2>&1
wait_log "点位映射表同步: 8 条（含播放词 1 条）" "${WORK_DIR}/bridge.log" 5 \
  || fail "带播放词的映射表未被全量替换"
wait_log "映射: E展台 -> E展台（含播放词）" "${WORK_DIR}/bridge.log" 5 \
  || fail "条目日志未标注播放词"
grep -q '"text": "E展台播放词：由映射表下发"' "${WORK_DIR}/point_map.json" \
  || fail "播放词未随表落盘到 point_map.json"
# stop 释放 2c 迎宾占用窗口（推表不占业务，迎宾窗口仍在），再导航
$SIM stop > /dev/null 2>&1
sleep 1
$SIM navigate E展台 > /dev/null 2>&1
wait_log "导航到达: E展台，未携带讲解词，播点位映射表播放词" "${WORK_DIR}/bridge.log" 10 \
  || fail "到达未播映射表播放词（优先级第 2 级失效）"
wait_log '"outcome":"success".*"type":"navigate.result"' "${WORK_DIR}/bridge.log" 5 \
  || fail "E展台导航未回 navigate.result success"
wait_log "text='E展台播放词：由映射表下发'" "${WORK_DIR}/mock_tts.log" 10 \
  || fail "映射表播放词未送达 TTS"
$SIM stop > /dev/null 2>&1
log "用例 2e 通过：播放词随表落盘 + 到达播报（指令 texts > 表 text）"

# ---------------------------------------------------------------------------
# 5. 用例 3：汽车讲解（navigate：mock 导航 2s -> 到达 -> 讲解词按序一轮）
# ---------------------------------------------------------------------------
# stop 释放 2c 迎宾占用窗口（单业务占用模型，窗口内新导航会被拒）
$SIM stop > /dev/null 2>&1
sleep 1
$SIM navigate 3号展台 --text "讲解第一句车型概述" --text "讲解第二句续航参数" > /dev/null 2>&1
wait_log "导航开始: point=3号展台" "${WORK_DIR}/bridge.log" 5 || fail "未见导航开始日志"
wait_log "导航到达: 3号展台，开始汽车讲解" "${WORK_DIR}/bridge.log" 10 || fail "Mock 导航未到达"
# v0.24.0（P-038）：到达即回 navigate.result success（讲解词播报与否不影响结果）
wait_log '"outcome":"success".*"type":"navigate.result"' "${WORK_DIR}/bridge.log" 5 \
  || fail "导航到达未回 navigate.result success"
wait_log "text='讲解第一句车型概述'" "${WORK_DIR}/mock_tts.log" 10 || fail "未播讲解词第一句"
wait_log "text='讲解第二句续航参数'" "${WORK_DIR}/mock_tts.log" 20 || fail "未播讲解词第二句"
sleep 6  # 第二句 speech~3s + interval 2s：若误循环会出现第二轮
COUNT_S1=$(grep -c "text='讲解第一句车型概述'" "${WORK_DIR}/mock_tts.log" || true)
COUNT_S2=$(grep -c "text='讲解第二句续航参数'" "${WORK_DIR}/mock_tts.log" || true)
[ "$COUNT_S1" -eq 1 ] || fail "讲解词不应循环（第一句 ${COUNT_S1} 次）"
[ "$COUNT_S2" -eq 1 ] || fail "讲解词不应循环（第二句 ${COUNT_S2} 次）"
log "用例 3 通过：导航到达后讲解词按序播一轮（不循环）"

# ---------------------------------------------------------------------------
# 6. 用例 4：讲解途中 tts.stop -> 掐断语音 + 终止会话（本地策略：放弃讲解）
# ---------------------------------------------------------------------------
# stop 释放用例 3 讲解占用窗口（单业务占用模型，窗口内新 tts 会被拒）
$SIM stop > /dev/null 2>&1
$SIM play "循环播报中等待停止" "第二条循环内容" --interval 1 --wait-result 0 > /dev/null 2>&1
wait_log "text='循环播报中等待停止'" "${WORK_DIR}/mock_tts.log" 10 || fail "循环播报未开始"
$SIM stop > /dev/null 2>&1
wait_log "priority=SAFETY_L10" "${WORK_DIR}/mock_tts.log" 10 || fail "tts.stop 未触发 SAFETY_L10 抢占"
wait_log "停止播报" "${WORK_DIR}/bridge.log" 5 || fail "未见停止播报日志"
sleep 4
COUNT_LOOP=$(grep -c "text='循环播报中等待停止'" "${WORK_DIR}/mock_tts.log" || true)
COUNT_STOP_NOW=$((COUNT_LOOP + $(grep -c "text='第二条循环内容'" "${WORK_DIR}/mock_tts.log" || true)))
sleep 4
COUNT_LOOP2=$(grep -c "text='循环播报中等待停止'" "${WORK_DIR}/mock_tts.log" || true)
COUNT_AFTER=$((COUNT_LOOP2 + $(grep -c "text='第二条循环内容'" "${WORK_DIR}/mock_tts.log" || true)))
[ "$COUNT_AFTER" -eq "$COUNT_STOP_NOW" ] || fail "tts.stop 后仍在重播（${COUNT_STOP_NOW} -> ${COUNT_AFTER}）"
log "用例 4 通过：tts.stop 掐断语音并终止循环"

# ---------------------------------------------------------------------------
# 7. 用例 5：导览互斥（单业务占用：A 在途时发 B -> B 被拒，A 正常完成）
# ---------------------------------------------------------------------------
# stop 释放用例 4 残留占用（用例 4 自身以 stop 收尾，此处幂等兜底）
$SIM stop > /dev/null 2>&1
$SIM navigate A展台 --text "A展台讲解词正常播出" > /dev/null 2>&1
wait_log "导航开始: point=A展台" "${WORK_DIR}/bridge.log" 5 || fail "A 导航未开始"
sleep 3  # A 在途（mock 6s 行走中；窗口须覆盖 simulator 启动+发布耗时）
# 导览进行中下发新导航：单业务占用模型下必须被拒（v0.24.0 回
# navigate.result failed/busy，App 可感知稍后重试）
$SIM navigate B展台 --text "B展台讲解词不应播出" > /dev/null 2>&1
wait_log "业务互斥: 导览进行中，拒绝 type=navigate" "${WORK_DIR}/bridge.log" 5 \
  || fail "导览进行中未拒绝新导航（单业务占用失效）"
wait_log '"outcome":"failed","reason":"busy".*"type":"navigate.result"' \
  "${WORK_DIR}/bridge.log" 5 || fail "忙碌拒绝未回 navigate.result failed/busy"
grep -q "导航开始: point=B展台" "${WORK_DIR}/bridge.log" && fail "被拒绝的 B 导航不应启动"
# A 不受影响：正常到达并讲解（旧替换语义已废除，A 是唯一业务）
wait_log "导航到达: A展台，开始汽车讲解" "${WORK_DIR}/bridge.log" 15 || fail "A 导航未到达"
wait_log "text='A展台讲解词正常播出'" "${WORK_DIR}/mock_tts.log" 10 || fail "A 讲解词未播"
sleep 3  # B 若未被正确拒绝将在此前到达（其下发时刻 + 6s），兜底窗口
grep -q "导航到达: B展台" "${WORK_DIR}/bridge.log" && fail "被拒绝的 B 导航不应到达"
grep -q "text='B展台讲解词不应播出'" "${WORK_DIR}/mock_tts.log" && fail "被拒绝的 B 讲解词不应播出"
log "用例 5 通过：导览进行中新导航被拒（单业务占用），A 正常到达讲解"

# ---------------------------------------------------------------------------
# 7b. 用例 5b：navigate.stop 取消导航（v0.23.0：在途取消 + 无导航 no-op）
# ---------------------------------------------------------------------------
# stop 释放用例 5 A 讲解占用窗口（单业务占用模型，窗口内新导航会被拒）
$SIM stop > /dev/null 2>&1
sleep 1
$SIM navigate C展台 --text "C展台讲解词不应播出" > /dev/null 2>&1
wait_log "导航开始: point=C展台" "${WORK_DIR}/bridge.log" 5 || fail "C 导航未开始"
sleep 2  # 在途（mock 6s 行走中）
$SIM nav-stop > /dev/null 2>&1
wait_log "停止导航: point=C展台" "${WORK_DIR}/bridge.log" 5 || fail "navigate.stop 未取消在途导航"
# v0.24.0（P-038）：被停止的导航回 navigate.result failed/aborted
wait_log '"outcome":"failed","reason":"aborted".*"type":"navigate.result"' \
  "${WORK_DIR}/bridge.log" 5 || fail "被取消的 C 导航未回 failed/aborted"
sleep 8  # 若取消失效，mock 6s 导航会在此前到达
grep -q "导航到达: C展台" "${WORK_DIR}/bridge.log" && fail "被取消的 C 导航不应到达"
grep -q "text='C展台讲解词不应播出'" "${WORK_DIR}/mock_tts.log" && fail "被取消的 C 讲解词不应播出"
# 无导航时 navigate.stop = no-op（不掐语音、不影响业务）
$SIM nav-stop > /dev/null 2>&1
wait_log "停止导航: 无导航在途 no-op" "${WORK_DIR}/bridge.log" 5 || fail "无导航时 navigate.stop 应 no-op"
log "用例 5b 通过：navigate.stop 取消在途导航（讲解不播），无导航时 no-op"

# ---------------------------------------------------------------------------
# 7c. 用例 5c：地图重定位（relocate，2026-10-17 接入：终态=导航模块验收）
#     下发 relocate A -> 受理 -> 等待验收（mock hold 8 次 poll ≈ 2s）->
#     回 relocate.result success -> 播报「A点 重定位 成功」
# ---------------------------------------------------------------------------
$SIM relocate A --wait-result 8 > "${WORK_DIR}/sim5c.log" 2>&1
wait_log "MQTT 下行重定位已受理: 航点「A」" "${WORK_DIR}/bridge.log" 5 \
  || fail "relocate 未被受理（Mock set_pose 应即时成功）"
wait_log "等待导航模块验收" "${WORK_DIR}/bridge.log" 5 || fail "未见等待验收日志"
wait_log "MQTT 下行重定位验收通过" "${WORK_DIR}/bridge.log" 8 \
  || fail "Mock 验收轮询未在 hold 窗口后给出结果"
wait_log "text='A点 重定位 成功'" "${WORK_DIR}/mock_tts.log" 10 \
  || fail "验收通过后未播报默认话术「A点 重定位 成功」"
wait_log "type=relocate.result" "${WORK_DIR}/sim5c.log" 5 \
  || { cat "${WORK_DIR}/sim5c.log" || true; fail "后端未收到 relocate.result 回执"; }
grep -q "outcome=success" "${WORK_DIR}/sim5c.log" || fail "relocate 回执不是 success"
$SIM stop > /dev/null 2>&1 || true
log "用例 5c 通过：relocate 受理 -> 等验收 -> 回 success -> 播报「A点 重定位 成功」"

# 7d. 用例 5d：重定位等待期新业务被拒（不可打断业务）+ 语音提示
#     「我正在重定位，请稍后」；重定位本体不受影响照常终态。
#     relocate 用 --wait-result 0（发后即忘）：若等回执会阻塞到重定位
#     终态（业务已释放），拒绝窗口就错过了。用航点 B 区别于 5c 的 A，
#     日志断言不与 5c 串扰；hold=16（~4.2s 窗口）须覆盖「sleep 1 +
#     simulator python 冷启动 ~1.6s」的打点开销
$SIM relocate B --wait-result 0 > /dev/null 2>&1
wait_log "MQTT 下行重定位已受理: 航点「B」" "${WORK_DIR}/bridge.log" 5 \
  || fail "第二次 relocate 未被受理"
sleep 1  # 处于等待验收窗口
$SIM navigate X展台 --text "重定位中被拒的讲解词不应播出" > /dev/null 2>&1
wait_log "业务互斥: 重定位进行中，拒绝 type=navigate" "${WORK_DIR}/bridge.log" 5 \
  || fail "重定位等待期未拒绝新导航（不可打断业务失效）"
wait_log '"outcome":"failed","reason":"busy".*"type":"navigate.result"' \
  "${WORK_DIR}/bridge.log" 5 || fail "被拒导航未回 navigate.result failed/busy"
wait_log "text='我正在重定位，请稍后'" "${WORK_DIR}/mock_tts.log" 5 \
  || fail "重定位中拒绝新业务未播报忙碌提示"
grep -q "导航开始: point=X展台" "${WORK_DIR}/bridge.log" && fail "被拒的 X 导航不应启动"
grep -q "text='重定位中被拒的讲解词不应播出'" "${WORK_DIR}/mock_tts.log" \
  && fail "被拒的 X 讲解词不应播出"
# 重定位本体不受影响：等待窗口过后照常验收终态（B 的验收/回执行为 5d 独有）
wait_log "MQTT 下行重定位验收通过: 航点「B」" "${WORK_DIR}/bridge.log" 10 \
  || fail "被拒业务影响了重定位本体（未照常验收终态）"
wait_count '"outcome":"success".*"type":"relocate.result"' "${WORK_DIR}/bridge.log" 2 5 \
  || fail "重定位终态未回 relocate.result success（累计应 2 条：5c+5d）"
$SIM stop > /dev/null 2>&1 || true
log "用例 5d 通过：重定位等待期新导航被拒（failed/busy）+ 播报「我正在重定位，请稍后」"

# 7e. 用例 5e：导航在途时 relocate 被互斥拒绝 + 播报「我正在导航，请稍后」
$SIM navigate D展台 > /dev/null 2>&1
wait_log "导航开始: point=D展台" "${WORK_DIR}/bridge.log" 5 || fail "D 导航未开始"
sleep 3  # 在途（mock 6s 行走中）
$SIM relocate A --wait-result 5 > "${WORK_DIR}/sim5e.log" 2>&1
wait_log "业务互斥: 导览进行中，拒绝 type=relocate" "${WORK_DIR}/bridge.log" 5 \
  || fail "导航在途时 relocate 未被互斥拒绝"
wait_log "type=relocate.result" "${WORK_DIR}/sim5e.log" 5 \
  || { cat "${WORK_DIR}/sim5e.log" || true; fail "导航在途 relocate 未回 relocate.result"; }
grep -q "reason=busy" "${WORK_DIR}/sim5e.log" || fail "导航在途 relocate 回执应 failed/busy"
wait_log "text='我正在导航，请稍后'" "${WORK_DIR}/mock_tts.log" 5 \
  || fail "导航在途拒绝重定位未播报忙碌提示"
# 收尾：取消 D 导航（tts.stop 停一切），确保后续用例不受导览占用影响
$SIM stop > /dev/null 2>&1 || true
log "用例 5e 通过：导航在途 relocate 被拒回 failed/busy + 播报「我正在导航，请稍后」"

# ---------------------------------------------------------------------------
# 8. 用例 6：健壮性（垃圾报文 / 残留 loop 字段回 invalid_request /
#    重复 operation_id 幂等 / 过期 navigate 丢弃不导航）
# ---------------------------------------------------------------------------
# stop 释放用例 5 A 讲解占用窗口（单业务占用模型，窗口内新 tts 会被拒，
# 幂等用例的第一条 tts 需要真实受理才能断言「播 1 次 + 重发同一回执」）
$SIM stop > /dev/null 2>&1
mqtt_pub "robotCar/v1/robots/robot-001/down/commands" 'not a json payload'

LEGACY_EVENT=$(PYTHONPATH="${GUIDE_DIR}/tools" python3 - <<'PYEOF'
import json, time
from backend_simulator import uuid7, rfc3339
now = time.time()
print(json.dumps({
    "spec_version": "1.0", "event_id": uuid7(), "robot_id": "robot-001",
    "direction": "down", "type": "tts",
    "occurred_at": rfc3339(now), "expires_at": rfc3339(now + 120),
    "payload": {"operation_id": "0199aaaa-bbbb-7ccc-8ddd-eeeeffff0010",
                "texts": ["残留loop"], "mode": "sequence",
                "interval_seconds": 30, "loop": True},
}, ensure_ascii=False))
PYEOF
)
mqtt_pub "robotCar/v1/robots/robot-001/down/commands" "$LEGACY_EVENT"
wait_log '"outcome":"rejected","reason":"invalid_request"' "${WORK_DIR}/bridge.log" 5 \
  || fail "残留 loop 字段未回 invalid_request 回执"
grep -q "text='残留loop'" "${WORK_DIR}/mock_tts.log" && fail "残留 loop 的非法载荷被误播报"

DUP_EVENT=$(PYTHONPATH="${GUIDE_DIR}/tools" python3 - <<'PYEOF'
import json, time
from backend_simulator import uuid7, rfc3339
now = time.time()
print(json.dumps({
    "spec_version": "1.0", "event_id": uuid7(), "robot_id": "robot-001",
    "direction": "down", "type": "tts",
    "occurred_at": rfc3339(now), "expires_at": rfc3339(now + 120),
    "payload": {"operation_id": "0199aaaa-bbbb-7ccc-8ddd-eeeeffff0001",
                "texts": ["幂等测试"], "mode": "sequence", "interval_seconds": 30},
}, ensure_ascii=False))
PYEOF
)
for i in 1 2; do
  mqtt_pub "robotCar/v1/robots/robot-001/down/commands" "$DUP_EVENT"
  sleep 0.3
done
sleep 1
kill -0 "$BRIDGE_PID" 2>/dev/null || fail "桥接节点在异常报文后崩溃"
wait_log "校验失败" "${WORK_DIR}/bridge.log" 5 || fail "垃圾报文未被拦截"
COUNT=$(grep -c "text='幂等测试'" "${WORK_DIR}/mock_tts.log" || true)
[ "$COUNT" -eq 1 ] || fail "重复 operation_id 去重失败（播报 ${COUNT} 次，期望 1 次）"
wait_log "重发同一回执" "${WORK_DIR}/bridge.log" 5 \
  || fail "重复指令未重发同一 tts.result 回执（同 event_id）"
$SIM stop > /dev/null 2>&1 || true

# 过期 navigate：回 navigate.result rejected/command_expired（v0.24.0 起
# 有回执面），不得触发导航
EXPIRED_NAV=$(PYTHONPATH="${GUIDE_DIR}/tools" python3 - <<'PYEOF'
import json, time
from backend_simulator import uuid7, rfc3339
now = time.time()
print(json.dumps({
    "spec_version": "1.0", "event_id": uuid7(), "robot_id": "robot-001",
    "direction": "down", "type": "navigate",
    "occurred_at": rfc3339(now - 600), "expires_at": rfc3339(now - 540),
    "payload": {"operation_id": "0199aaaa-bbbb-7ccc-8ddd-eeeeffff0003",
                "point_name": "过期点位"},
}, ensure_ascii=False))
PYEOF
)
mqtt_pub "robotCar/v1/robots/robot-001/down/commands" "$EXPIRED_NAV"
wait_log "指令已过期: type=navigate" "${WORK_DIR}/bridge.log" 5 \
  || fail "过期 navigate 未被拒绝"
wait_log '"outcome":"rejected","reason":"command_expired".*"type":"navigate.result"' \
  "${WORK_DIR}/bridge.log" 5 || fail "过期 navigate 未回 rejected/command_expired"
grep -q "导航开始: point=过期点位" "${WORK_DIR}/bridge.log" && fail "过期 navigate 误触发导航"
log "用例 6 通过：垃圾/残留 loop 报文拦截回执；重复指令幂等重发同一回执；过期 navigate 丢弃"

# ---------------------------------------------------------------------------
# 9. 用例 7：异常后正常指令仍可处理（节点无残留故障）
# ---------------------------------------------------------------------------
$SIM play "异常恢复后播报" > "${WORK_DIR}/sim7.log" 2>&1
wait_log "text='异常恢复后播报'" "${WORK_DIR}/mock_tts.log" 10 || fail "异常恢复失败"
wait_log "outcome=accepted" "${WORK_DIR}/sim7.log" 5 || fail "异常恢复后未收到 accepted 回执"
$SIM stop > /dev/null 2>&1 || true
log "用例 7 通过：异常后恢复正常并回执 accepted"

# ---------------------------------------------------------------------------
# 10. 清理并汇总
# ---------------------------------------------------------------------------
log "全部集成测试通过 ✅"
log "--- Mock TTS 服务日志（最后 10 行）---"
tail -10 "${WORK_DIR}/mock_tts.log" || true
log "--- 桥接日志（最后 10 行）---"
tail -10 "${WORK_DIR}/bridge.log" || true
