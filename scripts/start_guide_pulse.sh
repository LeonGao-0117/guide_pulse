#!/usr/bin/env bash
# process_manager 开机入口：等价于 source install/setup.bash 后
#   ros2 launch guide_pulse guide_pulse.launch.py serial_number:=<SN>
# SN 自动解析（批量部署零手工配置）：
#   ROBOT_SERIAL_NUMBER 环境变量 > 本地 /robot/info/sn > SSH 运控板读取
# （运控板 cat /robot/info/sn；默认密码方式认证，见下方 MC_SSH_PASSWORD）。
# robot_id 由 Server 在首次 enroll 时分配（协议 v0.9.0+），不再是本工程配置。
# 安装位置: /robot/software/guide_pulse/bin/start_guide_pulse
#
# 由 EM 注入的环境变量（见 t1_v2d0_soc0_config.yaml）:
#   LOG_PATH                日志目录
#   LOG_KEEP_DAYS           启动日志 / ROS 日志保留天数（默认 14；0=不删）
#   GUIDE_PULSE_WS             工作区（默认 /opt/guide_pulse）
#   ROBOT_SERIAL_NUMBER     本机序列号显式覆盖（默认空=自动解析；须运维在
#                           Server 侧预登记后 enroll 才会成功）
#   GUIDE_PULSE_MC_SSH         运控板 SSH 目标（默认 run@10.1.1.100）
#   GUIDE_PULSE_SN_FETCH_RETRIES/FETCH_INTERVAL SN 跨板获取重试次数/间隔
#                           （默认 6 次 / 10s：等运控板完成开机）
#   HOME                    普通用户家目录（sudo: false 时显式设为 /home/run）
#   WAIT_TTS_SERVICE_SECONDS 等 SDK PlayTts 服务的上限秒数（默认 0=不等：
#                           ros2 CLI 探测经 daemon 冷发现，板上常超时且无意义；
#                           guide_bridge 自身"服务未就绪不崩溃、每3s按需重试"。
#                           >0 时启动前最多等 N 秒，超时也会照常 launch）
#   TTS_SERVICE_NAME        探测的 SDK 服务名（默认 /aimdk_5Fmsgs/srv/PlayTts）
#   GUIDE_PULSE_BROKER_URI     可选，临时覆盖 yaml 里的 MQTT broker 地址
#   GUIDE_PULSE_ENROLL_ORIGIN  可选，临时覆盖 yaml 里的 enroll 注册面地址
#   GUIDE_PULSE_DDS_PROFILES   可选，guide_pulse 专用 FastDDS 配置。默认不设：
#                           guide_pulse 只有一个 ROS participant，板端全局配置
#                           预留的 200MB SHM 可承受，且沿用板端传输才能保证
#                           与 SDK / 官方模块互通（robot_com 的 UDP loopback
#                           白名单不含其它板卡，勿直接套用）
#
# 日志（LOG_PATH 下）:
#   latest.log / stdout.log / stderr.log  → 本次 runs/<stamp>.<pid>.log
#   current.run                           → run_id、pid、serial、文件路径
#   runs/                                 → 每次启动一份，按 LOG_KEEP_DAYS 清理
#   ros/                                  → ROS_LOG_DIR（launch 自己的文件）
set -euo pipefail

LOG_PATH="${LOG_PATH:-/robot/persist/log/guide_pulse}"
LOG_KEEP_DAYS="${LOG_KEEP_DAYS:-14}"
WORKSPACE="${GUIDE_PULSE_WS:-/opt/guide_pulse}"
ROBOT_SERIAL_NUMBER="${ROBOT_SERIAL_NUMBER:-}"
if [[ -n "${ROBOT_SERIAL_NUMBER}" && ! "${ROBOT_SERIAL_NUMBER}" =~ ^[A-Za-z0-9][A-Za-z0-9._:-]{5,63}$ ]]; then
  echo "ROBOT_SERIAL_NUMBER 格式非法（需 6..64 位，首字符字母数字，其后仅限 [A-Za-z0-9._:-]）: ${ROBOT_SERIAL_NUMBER}" >&2
  exit 1
fi
WAIT_TTS_SERVICE_SECONDS="${WAIT_TTS_SERVICE_SECONDS:-0}"
TTS_SERVICE_NAME="${TTS_SERVICE_NAME:-/aimdk_5Fmsgs/srv/PlayTts}"
GUIDE_PULSE_BROKER_URI="${GUIDE_PULSE_BROKER_URI:-}"
GUIDE_PULSE_ENROLL_ORIGIN="${GUIDE_PULSE_ENROLL_ORIGIN:-}"
GUIDE_PULSE_DDS_PROFILES="${GUIDE_PULSE_DDS_PROFILES:-}"
HOME="${HOME:-/home/run}"
export HOME
export PATH="${HOME}/.local/bin:${PATH}"
export PYTHONPATH="${HOME}/.local/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
export PYTHONUNBUFFERED=1
export RCUTILS_COLORIZED_OUTPUT=0
export RCUTILS_LOGGING_USE_STDOUT=1

# guide_pulse 是可牺牲的播报演示；MC/device_manager 不可牺牲。提高本进程树
# 的 OOM badness，内核缺内存时优先杀 guide_pulse，而不是全局长时间回收拖垮运控。
if [[ -w /proc/self/oom_score_adj ]]; then
  if ! echo 500 > /proc/self/oom_score_adj 2>/dev/null; then
    echo "WARN: 无法设置 guide_pulse oom_score_adj=500" >&2
  fi
fi

RUN_STAMP="$(date '+%Y-%m-%d_%H-%M-%S')"
RUN_ID="$(date '+%Y%m%d-%H%M%S')-${$}"
export GUIDE_PULSE_RUN_ID="${RUN_ID}"

RUNS_DIR="${LOG_PATH}/runs"
ROS_LOG_DIR="${LOG_PATH}/ros"
export ROS_LOG_DIR

mkdir -p "${LOG_PATH}" "${RUNS_DIR}" "${ROS_LOG_DIR}"

cleanup_old_logs() {
  local keep_days="${LOG_KEEP_DAYS}"
  if ! [[ "${keep_days}" =~ ^[0-9]+$ ]] || (( keep_days <= 0 )); then
    return 0
  fi
  find "${RUNS_DIR}" -type f -name '*.log' -mtime "+${keep_days}" -delete 2>/dev/null || true
  find "${ROS_LOG_DIR}" -mindepth 1 -maxdepth 1 -mtime "+${keep_days}" -exec rm -rf {} + 2>/dev/null || true
}

if [[ -f "${LOG_PATH}/stdout.log" && ! -L "${LOG_PATH}/stdout.log" ]]; then
  mv "${LOG_PATH}/stdout.log" "${RUNS_DIR}/legacy-stdout-${RUN_STAMP}.log" || true
fi
if [[ -f "${LOG_PATH}/stderr.log" && ! -L "${LOG_PATH}/stderr.log" ]]; then
  mv "${LOG_PATH}/stderr.log" "${RUNS_DIR}/legacy-stderr-${RUN_STAMP}.log" || true
fi

cleanup_old_logs

RUN_LOG="${RUNS_DIR}/${RUN_STAMP}.${$}.log"
touch "${RUN_LOG}"
ln -sfn "runs/${RUN_STAMP}.${$}.log" "${LOG_PATH}/latest.log"
ln -sfn "runs/${RUN_STAMP}.${$}.log" "${LOG_PATH}/stdout.log"
ln -sfn "runs/${RUN_STAMP}.${$}.log" "${LOG_PATH}/stderr.log"

cat >"${LOG_PATH}/current.run" <<EOF
run_id=${RUN_ID}
pid=${$}
serial=${ROBOT_SERIAL_NUMBER:-<yaml>}
started=$(date -Iseconds)
log=${RUN_LOG}
workspace=${WORKSPACE}
ros_log_dir=${ROS_LOG_DIR}
EOF

# EM 无 TTY：只写文件。手动在终端跑时同时打到屏幕，方便盯现场。
if [[ -t 1 ]]; then
  exec > >(tee -a "${RUN_LOG}") 2>&1
else
  exec >>"${RUN_LOG}" 2>&1
fi

stamp() { date '+%Y-%m-%d %H:%M:%S'; }
log() { printf '%s [guide_pulse] [%s] %s\n' "$(stamp)" "${RUN_ID}" "$*"; }

# launch 被杀后子进程可能被 init 收养；仅 SIGINT 不够。与 start_robot_com 相同。
# 模式精确匹配本工程进程：不波及 robot_com 的 speaker/text_announcer。
# 节点模式锚定工作区绝对路径开头（^）：旁观 shell 的 cmdline 以 /bin/bash 等
# 开头，即使命令行里写着同样的模式串也不会被 pkill 误杀。
stop_previous_stack() {
  local launch_pat='humble/bin/ros2 launch guide_pulse'
  local node_pat="^${WORKSPACE}/install/guide_pulse/lib/guide_pulse/(guide_bridge|mock_tts_service)"
  if ! pgrep -f "${launch_pat}" >/dev/null 2>&1 && ! pgrep -f "${node_pat}" >/dev/null 2>&1; then
    return 0
  fi
  log "正在停止旧的 guide_pulse 进程"
  pkill -INT -f "${launch_pat}" || true
  sleep 1
  pkill -TERM -f "${launch_pat}" || true
  sleep 1
  pkill -KILL -f "${node_pat}" || true
  pkill -KILL -f "${launch_pat}" || true
  sleep 1
}

tts_service_ready() {
  ros2 service list 2>/dev/null | grep -Fxq "${TTS_SERVICE_NAME}"
}

wait_for_tts_service() {
  local limit="${WAIT_TTS_SERVICE_SECONDS}"
  if ! [[ "${limit}" =~ ^[0-9]+$ ]]; then
    log "WAIT_TTS_SERVICE_SECONDS=${limit} invalid, launching anyway"
    return 0
  fi
  if (( limit == 0 )); then
    return 0  # 默认不等（WAIT_TTS_SERVICE_SECONDS=0），静默返回
  fi
  log "wait up to ${limit}s for SDK service ${TTS_SERVICE_NAME}"
  local started="${SECONDS}"
  while (( SECONDS - started < limit )); do
    if tts_service_ready; then
      log "SDK TTS service ready after $((SECONDS - started))s"
      return 0
    fi
    sleep 0.5
  done
  log "TTS service wait timeout ${limit}s (${TTS_SERVICE_NAME} not visible), launching anyway"
}

log "start workspace=${WORKSPACE} serial=<待解析> uid=$(id -u) pid=${$}"
log "trace run_id=${RUN_ID} log=${RUN_LOG}"

INSTALL_SETUP="${WORKSPACE}/install/setup.bash"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  log "ERROR: 未找到 /opt/ros/humble/setup.bash"
  exit 1
fi
if [[ ! -f "${INSTALL_SETUP}" ]]; then
  log "ERROR: 未找到 ${INSTALL_SETUP}，请先在工作区执行 bash scripts/build.sh"
  exit 1
fi

# 沿用板端 DDS：与 SDK / 官方模块同一套传输。guide_pulse 单 participant，
# 不需要 robot_com 那份 UDP loopback 覆盖（见文件头 GUIDE_PULSE_DDS_PROFILES 说明）。
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -n "${DDS_CFG_PREFIX:-}" ]]; then
  if [[ -f "${DDS_CFG_PREFIX}/ros_dds_configuration_all.xml" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_CFG_PREFIX}/ros_dds_configuration_all.xml"
  elif [[ -f "${DDS_CFG_PREFIX}/ros_dds_configuration.xml" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_CFG_PREFIX}/ros_dds_configuration.xml"
  fi
fi
if [[ -n "${GUIDE_PULSE_DDS_PROFILES}" ]]; then
  if [[ ! -f "${GUIDE_PULSE_DDS_PROFILES}" ]]; then
    log "ERROR: GUIDE_PULSE_DDS_PROFILES 不存在: ${GUIDE_PULSE_DDS_PROFILES}"
    exit 1
  fi
  export FASTRTPS_DEFAULT_PROFILES_FILE="${GUIDE_PULSE_DDS_PROFILES}"
  export FASTDDS_DEFAULT_PROFILES_FILE="${GUIDE_PULSE_DDS_PROFILES}"
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "${INSTALL_SETUP}"
set -u

# 先停旧进程再进入工作区
stop_previous_stack
cd "${WORKSPACE}"
wait_for_tts_service

LAUNCH_PID=""
cleanup() {
  trap - SIGTERM SIGINT
  log "received stop, shutting down launch tree"
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
  fi
  pkill -TERM -f 'humble/bin/ros2 launch guide_pulse' 2>/dev/null || true
  sleep 1
  pkill -KILL -f "^${WORKSPACE}/install/guide_pulse/lib/guide_pulse/(guide_bridge|mock_tts_service)" 2>/dev/null || true
  pkill -KILL -f 'humble/bin/ros2 launch guide_pulse' 2>/dev/null || true
  if [[ -n "${LAUNCH_PID}" ]]; then
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  log "stopped"
  exit 0
}
trap cleanup SIGTERM SIGINT

# ---------------------------------------------------------------------------
# SN 自动解析（批量部署：同一份配置可刷任意机器，无需逐台写 SN）
# 优先级：ROBOT_SERIAL_NUMBER 环境变量 > 本地 /robot/info/sn 文件
#         > SSH 运控板读取（GUIDE_PULSE_MC_SSH，默认 run@10.1.1.100）
# SN 由制造商写入运控板（cat /robot/info/sn）；Jetson 本地的
# /robot/info/sn 是否落地取决于固件版本，读到就用、读不到跨板取。
# SSH 认证：默认密码方式（sshpass + GUIDE_PULSE_MC_PASSWORD，板间内网出厂
# 密码 1，零配置批量部署）；置空密码回落免密钥（ssh-copy-id）。
# 开机时运控板可能尚未就绪：按 GUIDE_PULSE_SN_FETCH_RETRIES 次重试（默认 6，
# 间隔 GUIDE_PULSE_SN_FETCH_INTERVAL 秒，默认 10）。
# ---------------------------------------------------------------------------
MC_SSH_TARGET="${GUIDE_PULSE_MC_SSH:-run@10.1.1.100}"
# 运控板 SSH 密码（板间内网链路，出厂密码 1）。置 GUIDE_PULSE_MC_PASSWORD=""
# 可禁用密码方式，回落免密钥（ssh-copy-id）认证。
MC_SSH_PASSWORD="${GUIDE_PULSE_MC_PASSWORD-1}"
SN_FETCH_RETRIES="${GUIDE_PULSE_SN_FETCH_RETRIES:-6}"
SN_FETCH_INTERVAL="${GUIDE_PULSE_SN_FETCH_INTERVAL:-10}"

trim_sn() { printf '%s' "$1" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//'; }

sn_over_ssh() {
  # 密码方式：sshpass 经 SSHPASS 环境变量取密码（不进 argv）；
  # accept-new 首连自动接受运控板主机指纹（批量部署零交互）
  if [[ -n "${MC_SSH_PASSWORD}" ]] && command -v sshpass >/dev/null 2>&1; then
    SSHPASS="${MC_SSH_PASSWORD}" sshpass -e ssh \
      -o StrictHostKeyChecking=accept-new -o ConnectTimeout=3 \
      "${MC_SSH_TARGET}" cat /robot/info/sn 2>/dev/null || true
    return 0
  fi
  # 免密钥方式（BatchMode：无密钥时立即失败，绝不交互式卡密码提示）
  ssh -o BatchMode=yes -o ConnectTimeout=3 "${MC_SSH_TARGET}" \
    cat /robot/info/sn 2>/dev/null || true
}

resolve_serial_number() {
  if [[ -n "${ROBOT_SERIAL_NUMBER}" ]]; then
    log "serial 取自环境变量 ROBOT_SERIAL_NUMBER: ${ROBOT_SERIAL_NUMBER}"
    return 0
  fi
  if [[ -r /robot/info/sn ]]; then
    local sn
    sn="$(trim_sn "$(cat /robot/info/sn)")"
    if [[ -n "${sn}" ]]; then
      ROBOT_SERIAL_NUMBER="${sn}"
      log "serial 取自本地文件 /robot/info/sn: ${sn}"
      return 0
    fi
  fi
  local attempt=1 sn=""
  while (( attempt <= SN_FETCH_RETRIES )); do
    sn="$(trim_sn "$(sn_over_ssh)")"
    if [[ -n "${sn}" ]]; then
      ROBOT_SERIAL_NUMBER="${sn}"
      log "serial 取自运控板 ${MC_SSH_TARGET}: ${sn}（第 ${attempt} 次尝试）"
      return 0
    fi
    log "serial 获取失败（${attempt}/${SN_FETCH_RETRIES}；运控板未就绪或认证失败），${SN_FETCH_INTERVAL}s 后重试"
    sleep "${SN_FETCH_INTERVAL}"
    attempt=$((attempt + 1))
  done
  return 1
}

if ! resolve_serial_number; then
  log "ERROR: 无法获取 serial_number（环境变量/本地文件/SSH 均失败），退出"
  exit 1
fi
if [[ ! "${ROBOT_SERIAL_NUMBER}" =~ ^[A-Za-z0-9][A-Za-z0-9._:-]{5,63}$ ]]; then
  log "ERROR: 解析到的 serial_number 格式非法: ${ROBOT_SERIAL_NUMBER}"
  exit 1
fi

log "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
LAUNCH_ARGS=("serial_number:=${ROBOT_SERIAL_NUMBER}")
if [[ -n "${GUIDE_PULSE_BROKER_URI}" ]]; then
  LAUNCH_ARGS+=("broker_uri:=${GUIDE_PULSE_BROKER_URI}")
  log "broker override ${GUIDE_PULSE_BROKER_URI}"
fi
if [[ -n "${GUIDE_PULSE_ENROLL_ORIGIN}" ]]; then
  LAUNCH_ARGS+=("enroll_origin:=${GUIDE_PULSE_ENROLL_ORIGIN}")
  log "enroll_origin override ${GUIDE_PULSE_ENROLL_ORIGIN}"
fi
log "launch guide_pulse guide_pulse.launch.py ${LAUNCH_ARGS[*]}"

ros2 launch guide_pulse guide_pulse.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

set +e
wait "${LAUNCH_PID}"
status=$?
set -e
trap - SIGTERM SIGINT
log "launch exited status=${status}"
exit "${status}"
