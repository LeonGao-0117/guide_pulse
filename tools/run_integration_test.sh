#!/usr/bin/env bash
# ============================================================================
# guide_pulse 端到端集成测试（无需真机）
#
# 链路：enroll_mock_server(注册面) <- guide_bridge(enroll 换凭据)
#       backend_simulator(scene/navigate/tts/tts.stop) -> mosquitto(1884)
#       -> guide_bridge -> mock_tts_service(SDK PlayTts Mock) + MockNavExecutor
# 契约：RobotCar 协议 v0.9.0/v0.10.0（enroll）+ v0.5.0/v0.6.0（tts 循环播报/
#       回执）+ v0.17.0（scene 迎宾）+ v0.16.0/v0.20.0（navigate 导航+随动
#       播报）。断言依据 Mock 服务日志、桥接日志与 simulator 收到的回执。
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

# 桥接节点：enroll 换凭据（SN SIM-guide-it -> robot-001）后连本地 mosquitto；
# mock_nav_seconds=2 模拟行走耗时（导航替换用例需要「在途」窗口）；
# SN 经 serial_number_file 自动获取（模拟真机从板端文件读取）
echo -n "SIM-guide-it" > "${WORK_DIR}/sn"
"${GUIDE_DIR}/install/guide_pulse/lib/guide_pulse/guide_bridge" --ros-args \
  -p broker_uri:="tcp://127.0.0.1:${PORT}" \
  -p enroll_origin:="http://127.0.0.1:${ENROLL_PORT}" \
  -p serial_number_file:="${WORK_DIR}/sn" \
  -p tts_service_name:="${MOCK_SERVICE}" \
  -p tts_wait_timeout_seconds:=10 \
  -p heartbeat_interval_seconds:=5 \
  -p mock_nav_seconds:=6 \
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

SIM="python3 ${GUIDE_DIR}/tools/backend_simulator.py --broker tcp://127.0.0.1:${PORT} --robot robot-001"
DEFAULT_WELCOME="欢迎光临！我是展厅讲解机器人，很高兴为您服务。"

# ---------------------------------------------------------------------------
# 3. 用例 1：tts 循环播报基线 -> Mock 收到请求 + tts.result accepted
# ---------------------------------------------------------------------------
$SIM play "循环播报基线" > "${WORK_DIR}/sim1.log" 2>&1
wait_log "text='循环播报基线'" "${WORK_DIR}/mock_tts.log" 10 || fail "Mock 未收到 tts 首播请求"
grep -q "priority=INTERACTION_L6" "${WORK_DIR}/mock_tts.log" || fail "播报优先级不是 INTERACTION_L6"
wait_log "outcome=accepted" "${WORK_DIR}/sim1.log" 5 || {
  cat "${WORK_DIR}/sim1.log" || true
  fail "未收到 tts.result accepted 回执"
}
log "用例 1 通过：tts 送达 Mock（INTERACTION_L6）且回执 accepted"

# ---------------------------------------------------------------------------
# 4. 用例 2：迎宾（scene=welcome）
#    2a. 未带 texts -> 本地默认迎宾词，单轮播完不循环
# ---------------------------------------------------------------------------
$SIM welcome > /dev/null 2>&1
wait_log "迎宾开始" "${WORK_DIR}/bridge.log" 5 || fail "未见迎宾开始日志"
wait_log "text='${DEFAULT_WELCOME}'" "${WORK_DIR}/mock_tts.log" 10 || fail "Mock 未收到默认迎宾词"
sleep 9  # 迎宾词 22 字 speech~5s + interval 2s：若误循环，~7s 处会出现第二次
WELCOME_COUNT=$(grep -c "text='${DEFAULT_WELCOME}'" "${WORK_DIR}/mock_tts.log" || true)
[ "$WELCOME_COUNT" -eq 1 ] || fail "迎宾词应单轮播完不循环（播了 ${WELCOME_COUNT} 次）"
wait_log "单轮播完，会话结束" "${WORK_DIR}/bridge.log" 5 || fail "迎宾会话未按单轮结束"
log "用例 2a 通过：迎宾播本地默认词一轮结束（不循环）"

# 2b. 带 texts -> App 指定迎宾词优先
$SIM welcome --text "指定迎宾词甲" --text "指定迎宾词乙" > /dev/null 2>&1
wait_log "text='指定迎宾词甲'" "${WORK_DIR}/mock_tts.log" 10 || fail "Mock 未收到指定迎宾词甲"
wait_log "text='指定迎宾词乙'" "${WORK_DIR}/mock_tts.log" 15 || fail "Mock 未收到指定迎宾词乙（按序播第二条）"
sleep 5
COUNT_A=$(grep -c "text='指定迎宾词甲'" "${WORK_DIR}/mock_tts.log" || true)
[ "$COUNT_A" -eq 1 ] || fail "指定迎宾词不应循环（甲 ${COUNT_A} 次）"
log "用例 2b 通过：迎宾 texts 优先且按序单轮（甲→乙）"

# ---------------------------------------------------------------------------
# 5. 用例 3：汽车讲解（navigate：mock 导航 2s -> 到达 -> 讲解词按序一轮）
# ---------------------------------------------------------------------------
$SIM navigate 3号展台 --text "讲解第一句车型概述" --text "讲解第二句续航参数" > /dev/null 2>&1
wait_log "导航开始: point=3号展台" "${WORK_DIR}/bridge.log" 5 || fail "未见导航开始日志"
wait_log "导航到达: 3号展台，开始汽车讲解" "${WORK_DIR}/bridge.log" 10 || fail "Mock 导航未到达"
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
# 7. 用例 5：导航替换（A 在途时发 B -> 只有 B 到达并讲解）
# ---------------------------------------------------------------------------
$SIM navigate A展台 --text "A展台讲解词不应播出" > /dev/null 2>&1
wait_log "导航开始: point=A展台" "${WORK_DIR}/bridge.log" 5 || fail "A 导航未开始"
sleep 3  # A 在途（mock 6s 行走中；窗口须覆盖 simulator 启动+发布耗时）
$SIM navigate B展台 --text "B展台讲解词正常播出" > /dev/null 2>&1
wait_log "导航开始: point=B展台" "${WORK_DIR}/bridge.log" 5 || fail "B 导航未开始"
wait_log "导航到达: B展台，开始汽车讲解" "${WORK_DIR}/bridge.log" 15 || fail "B 导航未到达"
wait_log "text='B展台讲解词正常播出'" "${WORK_DIR}/mock_tts.log" 10 || fail "B 讲解词未播"
sleep 3  # A 若未正确作废将在发起后 ~6s 到达（B 到达断言已越过该时刻），兜底窗口
grep -q "导航到达: A展台" "${WORK_DIR}/bridge.log" && fail "被替换的 A 导航不应到达"
grep -q "text='A展台讲解词不应播出'" "${WORK_DIR}/mock_tts.log" && fail "被替换的 A 讲解词不应播出"
log "用例 5 通过：新导航替换旧导航（A 作废，B 到达讲解）"

# ---------------------------------------------------------------------------
# 8. 用例 6：健壮性（垃圾报文 / 残留 loop 字段回 invalid_request /
#    重复 operation_id 幂等 / 过期 navigate 丢弃不导航）
# ---------------------------------------------------------------------------
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

# 过期 navigate：直接丢弃（无回执通道），不得触发导航
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
wait_log "指令已过期丢弃: type=navigate" "${WORK_DIR}/bridge.log" 5 \
  || fail "过期 navigate 未被丢弃"
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
