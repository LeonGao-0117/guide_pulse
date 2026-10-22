#!/usr/bin/env bash
# ============================================================================
# 导航途中随机播报（navigate_chatter，2026-10-21）专项冒烟测试：
# mock 导航拉长到 25s、chatter 间隔压到 5~8s，验证两个场景：
#  场景 1 普通带客导航：受理后安排（普通池）→ 途中 >=2 条安抚语、
#        不连续重复 → 到达讲解照常 → 到达后无新增
#  场景 2 前往迎宾点（App 下发 navigate 迎宾点）：迎宾引导语 + 迎宾池
#        途中播报（就位/迎接类话术，不在普通池中）→ 到达后无新增
# ============================================================================
set -euo pipefail
GUIDE_DIR="/opt/liwei/guide_pulse"
PORT=1885
ENROLL_PORT=18085
WORK_DIR="$(mktemp -d /tmp/chatter_test.XXXXXX)"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]:-}"; do kill -9 "$pid" 2>/dev/null || true; done
  # CHATTER_KEEP=1 时保留工作目录（失败排查用）
  [ "${CHATTER_KEEP:-0}" = "1" ] || rm -rf "$WORK_DIR"
}
trap cleanup EXIT

log() { echo -e "\033[1;32m[chatter测试]\033[0m $*"; }
fail() { keep_note; echo -e "\033[1;31m[chatter失败]\033[0m $*" >&2; exit 1; }
keep_note() { echo "[chatter调试] 日志目录保留于: ${WORK_DIR}"; }
wait_log() {
  local pattern="$1" file="$2" timeout="${3:-10}"
  for _ in $(seq 1 $((timeout * 5))); do
    grep -aq "$pattern" "$file" 2>/dev/null && return 0
    sleep 0.2
  done
  return 1
}
chatter_count() {
  grep -ac "trace_id='navigate_chatter'" "${WORK_DIR}/mock_tts.log" 2>/dev/null || true
}

# 1. 本地 mosquitto + enroll Mock
cat > "${WORK_DIR}/mosq.conf" <<EOF
listener ${PORT} 127.0.0.1
allow_anonymous true
EOF
mosquitto -c "${WORK_DIR}/mosq.conf" -d
PIDS+=("$(pgrep -f "mosquitto -c ${WORK_DIR}/mosq.conf" | head -1)")
sleep 1
python3 "${GUIDE_DIR}/tools/enroll_mock_server.py" --port "${ENROLL_PORT}" \
  > "${WORK_DIR}/enroll.log" 2>&1 &
PIDS+=("$!")
wait_log "listening on" "${WORK_DIR}/enroll.log" 10 || fail "enroll Mock 未启动"

# 2. Mock TTS / 运控 + 桥接（chatter 播完静默 5s——现场真实配置，mock 导航 25s）
set +u; source "${GUIDE_DIR}/install/setup.bash"; set -u
PYTHONUNBUFFERED=1 "${GUIDE_DIR}/install/guide_pulse/lib/guide_pulse/mock_tts_service.py" \
  --service /guide_pulse_chatter/mock_play_tts > "${WORK_DIR}/mock_tts.log" 2>&1 &
PIDS+=("$!")
PYTHONUNBUFFERED=1 "${GUIDE_DIR}/install/guide_pulse/lib/guide_pulse/mock_mc_service.py" \
  --get /guide_pulse_chatter/mock_mc_get --set /guide_pulse_chatter/mock_mc_set \
  --transition-ms 3000 > "${WORK_DIR}/mock_mc.log" 2>&1 &
PIDS+=("$!")
echo -n "SIM-guide-it" > "${WORK_DIR}/sn"
"${GUIDE_DIR}/install/guide_pulse/lib/guide_pulse/guide_bridge" --ros-args \
  -p broker_uri:="tcp://127.0.0.1:${PORT}" \
  -p enroll_origin:="http://127.0.0.1:${ENROLL_PORT}" \
  -p serial_number_file:="${WORK_DIR}/sn" \
  -p tts_service_name:=/guide_pulse_chatter/mock_play_tts \
  -p get_action_service:=/guide_pulse_chatter/mock_mc_get \
  -p set_action_service:=/guide_pulse_chatter/mock_mc_set \
  -p tts_wait_timeout_seconds:=10 \
  -p mock_nav_seconds:=25 \
  -p navigate_chatter_silence_seconds:=5 \
  -p tts_speech_ms_per_char:=10 \
  -p suppress_tail_lead_ms:=300 \
  -p point_map_file:="${WORK_DIR}/point_map.json" \
  > "${WORK_DIR}/bridge.log" 2>&1 &
PIDS+=("$!")
wait_log "已订阅下行主题: robotCar/v1/robots/robot-001/down/commands" "${WORK_DIR}/bridge.log" 20 \
  || { cat "${WORK_DIR}/bridge.log"; fail "桥接未就绪"; }
wait_log "Mock PlayTts 服务已就绪" "${WORK_DIR}/mock_tts.log" 15 || fail "Mock TTS 未就绪"
wait_log "Mock 运控服务已就绪" "${WORK_DIR}/mock_mc.log" 15 || fail "Mock 运控未就绪"

SIM="python3 ${GUIDE_DIR}/tools/backend_simulator.py --broker tcp://127.0.0.1:${PORT} --robot robot-001"
chatter_texts_log() {  # 全部途中播报语料（时序）
  grep -a "trace_id='navigate_chatter'" "${WORK_DIR}/mock_tts.log" | \
    sed -E "s/.*text='([^']*)'.*/\1/"
}

# ================= 场景 1：普通带客导航 =================
$SIM point-map "A展台=A展台" > /dev/null 2>&1
log "场景1: 下发普通导航（mock 25s，chatter 播完静默 5s）"
$SIM navigate "A展台" --text "到达后的讲解词内容" > "${WORK_DIR}/sim1.log" 2>&1
wait_log "导航途中随机播报已安排: 普通池" "${WORK_DIR}/bridge.log" 10 || fail "受理后未安排途中播报（普通池）"
for _ in $(seq 1 200); do
  [ "$(chatter_count)" -ge 2 ] && break
  sleep 0.2
done
# 25s 导航 / 每轮（播报估算 ~7s + 静默 5s）≈ 12s：约 2~3 条；>=2 为合理下限
[ "$(chatter_count)" -ge 2 ] || { cat "${WORK_DIR}/bridge.log"; fail "途中播报不足 2 条（实际 $(chatter_count) 条）"; }
log "场景1: 途中播报 $(chatter_count) 条"
# 相邻两条收单时间差须 >= 静默 5s + 播报余量（间隔含播报时长语义的
# 直接验证；旧 3~4s 连发模式差值仅 3~4s）
min_gap=$(grep -a "trace_id='navigate_chatter'" "${WORK_DIR}/mock_tts.log" | \
  grep -oE '\[17[0-9]{8}\.[0-9]+' | tr -d '[' | awk 'NR>1 && $1-prev<gap {gap=$1-prev} {prev=$1} END{print gap+0}' gap=6.0)
awk -v g="$min_gap" 'BEGIN{exit !(g>=6.0)}' \
  || fail "相邻两句间隔仅 ${min_gap}s（应 >=6s：播报时长+静默 5s）"
chatter_texts_log > "${WORK_DIR}/scene1_texts.txt"
dup=$(awk 'prev==$0{print} {prev=$0}' "${WORK_DIR}/scene1_texts.txt" | wc -l)
[ "$dup" -eq 0 ] || fail "场景1 存在连续重复的途中播报"
# 途中话术必须都来自普通池（不含「迎宾」类话术）
if grep -q "迎宾" "${WORK_DIR}/scene1_texts.txt"; then
  fail "场景1 途中播报混入迎宾话术: $(cat "${WORK_DIR}/scene1_texts.txt" | tr '\n' '/')"
fi
wait_log "导航到达: A展台，开始汽车讲解" "${WORK_DIR}/bridge.log" 40 || { cat "${WORK_DIR}/bridge.log"; fail "场景1 导航未到达或未开始讲解"; }
wait_log "text='到达后的讲解词内容'" "${WORK_DIR}/mock_tts.log" 10 || fail "场景1 到达讲解词未播"
at_arrival=$(chatter_count)
sleep 3
[ "$(chatter_count)" -eq "$at_arrival" ] || fail "场景1 到达后仍在途中播报（${at_arrival} -> $(chatter_count)）"
log "场景1 通过: 到达讲解正常且到达后无新增"

# ================= 场景 2：前往迎宾点 =================
$SIM stop > /dev/null 2>&1   # 确定释放业务占用
sleep 2
$SIM point-map "Y=迎宾点" > /dev/null 2>&1
log "场景2: 下发去迎宾点导航"
scene2_base=$(chatter_count)
$SIM navigate "迎宾点" > "${WORK_DIR}/sim2.log" 2>&1
# 迎宾引导语（好嘞，马上前去迎宾点）+ 迎宾池安排
wait_log "导航受理成功，播引导语: 好嘞，马上前去迎宾点（迎宾点）" "${WORK_DIR}/bridge.log" 10 \
  || { cat "${WORK_DIR}/bridge.log"; fail "场景2 未播迎宾引导语"; }
wait_log "导航途中随机播报已安排: 迎宾池" "${WORK_DIR}/bridge.log" 10 \
  || { cat "${WORK_DIR}/bridge.log"; fail "场景2 未安排迎宾池途中播报"; }
wait_log "导航途中随机播报（前往迎宾点）" "${WORK_DIR}/bridge.log" 30 \
  || { cat "${WORK_DIR}/bridge.log"; fail "场景2 途中未播迎宾话术"; }
# 锚定本次到达（Y 点未配播放词，到达日志带「未携带讲解词」尾巴），防止
# 匹配到场景 1 的旧「导航到达」行造成假阳性提前收尾
wait_log "导航到达: Y" "${WORK_DIR}/bridge.log" 40 \
  || { cat "${WORK_DIR}/bridge.log"; fail "场景2 导航未到达"; }
# 到达后再取全量快照（导航期间持续播报，取早了会漏计）
chatter_texts_log | tail -n +$((scene2_base + 1)) > "${WORK_DIR}/scene2_texts.txt"
[ -s "${WORK_DIR}/scene2_texts.txt" ] || fail "场景2 无途中播报语料"
# 25s 导航 / 每轮约 12s：约 2~3 条，>=2 为下限
[ "$(wc -l < "${WORK_DIR}/scene2_texts.txt")" -ge 2 ] \
  || fail "场景2 途中播报仅 $(wc -l < "${WORK_DIR}/scene2_texts.txt") 条（应 >=2）"
if grep -qE "跟我|跟紧|不要着急|不用着急" "${WORK_DIR}/scene2_texts.txt"; then
  fail "场景2 途中播报混入带客话术: $(cat "${WORK_DIR}/scene2_texts.txt" | tr '\n' '/')"
fi
log "场景2: 途中播报 $(wc -l < "${WORK_DIR}/scene2_texts.txt") 条（迎宾话术）"
at_arrival2=$(chatter_count)
sleep 3
[ "$(chatter_count)" -eq "$at_arrival2" ] || fail "场景2 到达后仍在途中播报"
log "场景2 通过: 到达后无新增途中播报"

echo "--- 场景1 途中话术（时序） ---"; cat "${WORK_DIR}/scene1_texts.txt"
echo "--- 场景2 途中话术（时序） ---"; cat "${WORK_DIR}/scene2_texts.txt"
log "全部通过"
