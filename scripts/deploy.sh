#!/usr/bin/env bash
# ============================================================================
# guide_pulse 一键部署：在目标机（已装 ROS 2 Humble 的机器人/Ubuntu 22.04，
# x86_64 或 aarch64）安装依赖并编译工作区。
# 不安装 ROS 2；目标机必须是 Humble，否则立即退出。
#
# 新环境使用步骤：
#   1. 把 guide_pulse/ 整个目录拷到目标机（SDK 消息包 aimdk_msgs 已内置
#      于 src/aimdk_msgs，无需另行拷贝外部 SDK 目录）
#   2. chmod +x scripts/deploy.sh && ./scripts/deploy.sh
#   3. source install/setup.bash && ros2 launch guide_pulse guide_pulse.launch.py
#
# 常用参数见 --help。脚本可重复执行。
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE}"

REQUIRED_ROS_DISTRO="humble"
ROS_SETUP="/opt/ros/${REQUIRED_ROS_DISTRO}/setup.bash"
CONFIG_YAML="${WORKSPACE}/src/guide_pulse/config/guide_pulse.yaml"

CLEAN=0
BUILD_ONLY=0
JOBS=""
COLCON_WORKERS=""
JOBS_SET=0
WORKERS_SET=0

log()  { printf '\033[1;32m[deploy]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[deploy]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[deploy]\033[0m %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'
用法: ./scripts/deploy.sh [选项]

目标环境: 已安装 ROS 2 Humble 的 Ubuntu（x86_64 / aarch64）。
本脚本不安装 ROS 2；若当前不是 Humble，立即退出。
编译并行按 nproc 与内存自动收紧（约每路 2G）。
SDK 消息包 aimdk_msgs 已内置 src/aimdk_msgs（来源 sdk_t1-v0.9.4.4），
无需外部 SDK 目录。

选项:
  --build-only       只 colcon 编译，不装依赖
  --clean            删除 build/ install/ log/ 后全量编译（不装依赖）
  --jobs N           每包编译器 -j
  --workers N        同时编译的 ROS 包数（colcon --parallel-workers）
  -h, --help         显示本说明

部署完成后:
  source install/setup.bash
  ros2 launch guide_pulse guide_pulse.launch.py
  python3 tools/mqtt_monitor.py          # 实时观察 MQTT 消息
  bash tools/run_integration_test.sh     # 本地自检（自带 broker+mock）
EOF
}

require_positive_int() {
  local name="$1" value="$2"
  [[ "${value}" =~ ^[1-9][0-9]*$ ]] || die "${name} 必须是正整数，收到: ${value:-<空>}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-only) BUILD_ONLY=1; shift ;;
    --clean) CLEAN=1; shift ;;
    --jobs)
      [[ $# -ge 2 ]] || die "缺少 --jobs 参数"
      require_positive_int "--jobs" "$2"; JOBS="$2"; JOBS_SET=1; shift 2 ;;
    --jobs=*)
      require_positive_int "--jobs" "${1#*=}"; JOBS="${1#*=}"; JOBS_SET=1; shift ;;
    --workers)
      [[ $# -ge 2 ]] || die "缺少 --workers 参数"
      require_positive_int "--workers" "$2"; COLCON_WORKERS="$2"; WORKERS_SET=1; shift 2 ;;
    --workers=*)
      require_positive_int "--workers" "${1#*=}"; COLCON_WORKERS="${1#*=}"; WORKERS_SET=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "未知参数: $1（见 --help）" ;;
  esac
done

as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    die "需要 root 权限执行: $*"
  fi
}

host_arch() { uname -m; }

# 同时跑的编译器数量：min(nproc, 总内存/每路预算)。ARM 小板内存紧张，
# paho/aimdk_msgs 编译峰值高，超并行会被 OOM 杀掉 cc1plus。
# 实测单路 cc1plus 峰值几百 M，每路预算默认 1.25G；内存富余的机器可用
# 环境变量 TTS_MEM_PER_JOB_GIB 放宽（如 TTS_MEM_PER_JOB_GIB=0.8）。
cpu_jobs() {
  local n mem_kb mem_jobs per_job_gib
  n="$(nproc 2>/dev/null || echo 2)"
  [[ "${n}" =~ ^[1-9][0-9]*$ ]] || n=2
  mem_kb="$(awk '/^MemTotal:/ {print $2; exit}' /proc/meminfo 2>/dev/null || echo 0)"
  [[ "${mem_kb}" =~ ^[0-9]+$ ]] || mem_kb=0
  per_job_gib="${TTS_MEM_PER_JOB_GIB:-1.25}"
  mem_jobs="$(awk -v m="${mem_kb}" -v p="${per_job_gib}" \
    'BEGIN { printf "%d", int(m / (p * 1048576)) }')"
  [[ "${mem_jobs}" -ge 1 ]] || mem_jobs=1
  if [[ "${n}" -lt "${mem_jobs}" ]]; then echo "${n}"; else echo "${mem_jobs}"; fi
}

set_build_parallelism() {
  local total auto_jobs auto_workers
  total="$(cpu_jobs)"
  if [[ "${total}" -le 2 ]]; then
    auto_workers=1; auto_jobs="${total}"
  else
    auto_workers=2; auto_jobs=$(((total + 1) / 2))
  fi
  [[ "${WORKERS_SET}" -eq 0 ]] && COLCON_WORKERS="${auto_workers}"
  [[ "${JOBS_SET}" -eq 0 ]] && JOBS="${auto_jobs}"
  if [[ $((JOBS * COLCON_WORKERS)) -gt "${total}" ]]; then
    warn "并行 -j${JOBS} × ${COLCON_WORKERS} 包超过本机估算 ${total}，可能 OOM"
  fi
}

_strip_prefix_from_var() {
  local var="$1"
  local prefix="$2"
  local value="${!var-}"
  local entry
  local -a kept=() parts=()
  [[ -n "${value}" ]] || return 0
  IFS=':' read -ra parts <<< "${value}"
  for entry in "${parts[@]}"; do
    [[ -n "${entry}" ]] || continue
    [[ "${entry}" == "${prefix}" || "${entry}" == "${prefix}/"* ]] && continue
    kept+=("${entry}")
  done
  if [[ ${#kept[@]} -eq 0 ]]; then
    unset "${var}"
  else
    local IFS=':'
    export "${var}=${kept[*]}"
  fi
}

# colcon 不能 overlay 正在编译的同一工作区；父 shell 若已 source
# install/setup.bash 必须先剥掉，且 colcon 生成脚本在 set -u 下会报
# COLCON_TRACE 未定义，source 前后临时关闭 -u。
drop_workspace_overlay() {
  local prefix="${WORKSPACE}/install"
  unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
  [[ "${AMENT_CURRENT_PREFIX:-}" == "${prefix}" || "${AMENT_CURRENT_PREFIX:-}" == "${prefix}/"* ]] \
    && unset AMENT_CURRENT_PREFIX
  _strip_prefix_from_var PYTHONPATH "${prefix}"
  _strip_prefix_from_var LD_LIBRARY_PATH "${prefix}"
  _strip_prefix_from_var PATH "${prefix}"
}

source_ros() {
  [[ -f "${ROS_SETUP}" ]] || die "未找到 ${ROS_SETUP}（本脚本不安装 ROS 2，需 Humble）"
  drop_workspace_overlay
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  set -u
  [[ "${ROS_DISTRO:-}" == "${REQUIRED_ROS_DISTRO}" ]] \
    || die "source 后 ROS_DISTRO=${ROS_DISTRO:-unset}，需要 ${REQUIRED_ROS_DISTRO}"
}

check_prereq() {
  [[ -d "${WORKSPACE}/src/guide_pulse" ]] || die "不是 guide_pulse 工作区: ${WORKSPACE}"
  [[ -f "${WORKSPACE}/third_party/paho.mqtt.cpp/CMakeLists.txt" ]] \
    || die "缺少 third_party/paho.mqtt.cpp（vendored MQTT 客户端）"
  [[ -d "${WORKSPACE}/tools/deps/paho" ]] \
    || warn "缺少 tools/deps/paho（python paho，mqtt_monitor/后端模拟器需要）"

  # SDK 消息包已内置工程内（来源 sdk_t1-v0.9.4.4，TTS 接口定义以 SDK 为准）
  [[ -f "${WORKSPACE}/src/aimdk_msgs/package.xml" ]] \
    || die "未找到内置 SDK 消息包: src/aimdk_msgs（工程不完整，请同步完整的 guide_pulse 目录）"
  log "SDK 消息包: src/aimdk_msgs（内置）"

  local arch
  arch="$(host_arch)"
  case "${arch}" in
    x86_64|amd64|aarch64|arm64) log "架构 ${arch}" ;;
    *) die "不支持的架构 ${arch}" ;;
  esac

  command -v python3 >/dev/null || die "缺少 python3"
}

ensure_locale() {
  if locale 2>/dev/null | grep -q 'UTF-8'; then return 0; fi
  log "配置 UTF-8 locale"
  as_root apt-get install -y locales >/dev/null
  as_root locale-gen en_US.UTF-8 C.UTF-8 >/dev/null
  as_root update-locale LANG=C.UTF-8 LC_ALL=C.UTF-8
  export LANG=C.UTF-8 LC_ALL=C.UTF-8
}

install_packages() {
  local required=(build-essential cmake nlohmann-json3-dev python3-yaml libcurl4-openssl-dev)
  local optional=(mosquitto)   # 仅集成测试/本地联调需要

  if ! command -v colcon >/dev/null; then
    required+=(ros-humble-colcon-common-extensions)
  fi

  log "安装系统依赖: ${required[*]}"
  as_root apt-get update
  # 变量赋值经 "$@" 展开后不再被识别为赋值，须用 env 前缀
  as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y "${required[@]}"

  for pkg in "${optional[@]}"; do
    log "安装 ${pkg}（可选）"
    as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y "${pkg}" \
      || warn "${pkg} 安装失败，已忽略（仅影响本地集成测试）"
  done
}

maybe_clean_build() {
  if [[ "${CLEAN}" -eq 1 ]]; then
    log "清理 build / install / log"
    rm -rf "${WORKSPACE}/build" "${WORKSPACE}/install" "${WORKSPACE}/log"
    return 0
  fi
  local cache
  cache="$(find "${WORKSPACE}/build" -name CMakeCache.txt -print -quit 2>/dev/null || true)"
  [[ -n "${cache}" ]] || return 0
  local cached current
  cached="$(grep -E '^CMAKE_SYSTEM_PROCESSOR:' "${cache}" | head -n1 | cut -d= -f2 | tr -d '[:space:]' || true)"
  current="$(host_arch)"
  if [[ -n "${cached}" && "${cached}" != "${current}" ]]; then
    warn "build/ 记录架构为 ${cached}，当前是 ${current}，自动清理后重编"
    rm -rf "${WORKSPACE}/build" "${WORKSPACE}/install" "${WORKSPACE}/log"
  fi
}

build_workspace() {
  maybe_clean_build
  source_ros

  # aimdk_msgs 已内置 src/（真实目录，非软链），随工程一起分发；
  # paho 完整性由上面 check_prereq 与 CMakeLists FATAL_ERROR 双重把关

  export MAKEFLAGS="-j${JOBS}"
  export CMAKE_BUILD_PARALLEL_LEVEL="${JOBS}"
  log "colcon 编译（-j${JOBS}，${COLCON_WORKERS} 个包一起编）"
  colcon build \
    --symlink-install \
    --packages-select aimdk_msgs liwei_nav_msgs guide_pulse \
    --parallel-workers "${COLCON_WORKERS}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

  [[ -x "${WORKSPACE}/install/guide_pulse/lib/guide_pulse/guide_bridge" ]] \
    || die "编译产物缺失: guide_bridge"
}

print_next_steps() {
  cat <<EOF

部署完成: ${WORKSPACE}

下一步:
  1. 检查/修改 MQTT 与播报参数: ${CONFIG_YAML}
     （broker_uri / enroll_origin / serial_number / tts_priority_level 等；
       serial_number 需运维在 RobotPolice 侧预登记，见 yaml 注释）
  2. 启动:
       source install/setup.bash
       ros2 launch guide_pulse guide_pulse.launch.py
  3. 验证:
       python3 tools/mqtt_monitor.py            # 实时观察 MQTT 消息
       bash tools/run_integration_test.sh       # 本地自检（不连后端、不出声）
  4. 单元测试:
       source install/setup.bash
       colcon test --packages-select guide_pulse && colcon test-result --verbose

说明:
  - MQTT 客户端 paho.mqtt.cpp 1.6.0 在 third_party/（静态链接，无需系统安装）。
  - python paho 随工程分发于 tools/deps/，无需 pip。
  - 真机运行依赖机器人固件提供的 /aimdk_5Fmsgs/srv/PlayTts 服务。
  - MQTT 凭据由 enroll 动态换取（协议 v0.9.0+），yaml 不再保存密码；
    serial_number 需运维在 RobotPolice 侧预登记。
EOF
}

main() {
  log "工作区: ${WORKSPACE}"
  check_prereq

  set_build_parallelism
  log "并行 -j${JOBS}，${COLCON_WORKERS} 个包一起编"

  if [[ "${BUILD_ONLY}" -eq 1 || "${CLEAN}" -eq 1 ]]; then
    if [[ "${CLEAN}" -eq 1 ]]; then
      log "清理后全量编译（跳过 apt）"
    else
      log "只编译（跳过 apt）"
    fi
  else
    ensure_locale
    install_packages
  fi
  build_workspace
  print_next_steps
}

main "$@"
