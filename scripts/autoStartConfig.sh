#!/usr/bin/env bash
# Install the guide_pulse EM startup entry (and log directories).
#
# Usage:
#   sudo ./scripts/autoStartConfig.sh
#
# Idempotent; does not restart guide_pulse and does NOT touch the process_manager
# YAML — the EM registration must be edited manually per docs/AutoStart.md.
#
# 与 robot_com/scripts/autoStartConfig.sh 的差异：guide_pulse 是单节点轻量应用，
# 不写 systemd 资源 drop-in；如需限流可参考 robot_com 那份的 write_dropin。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"

START_SOURCE="${WORKSPACE}/scripts/start_guide_pulse.sh"
START_TARGET="/robot/software/guide_pulse/bin/start_guide_pulse"
LOG_DIR="/robot/persist/log/guide_pulse"

log() { printf '[autoStartConfig] %s\n' "$*"; }
die() { printf '[autoStartConfig] ERROR: %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'
用法:
  sudo ./scripts/autoStartConfig.sh [GUIDE_PULSE_WS]

参数:
  GUIDE_PULSE_WS  开机脚本使用的工作区（默认=脚本所在工作区，可传参覆盖）

选项:
  -h, --help   显示帮助

功能:
  1. 语法检查 start_guide_pulse.sh
  2. 创建 /robot/software/guide_pulse/bin 与日志目录
  3. 部署 start_guide_pulse（内容拷贝，非软链，改仓库脚本后需重跑）

脚本不修改 t1_v2d0_soc0_config.yaml，也不重启 guide_pulse。
EOF
}

# 默认取脚本所在的工作区（不带参数即可在工程原地部署）；传参可覆盖
DEPLOY_WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    *) DEPLOY_WS="$1"; shift ;;
  esac
done

[[ "$(id -u)" -eq 0 ]] || die "请使用 sudo 运行"
[[ -f "${START_SOURCE}" ]] || die "不存在启动脚本: ${START_SOURCE}"
[[ -f "${DEPLOY_WS}/install/setup.bash" ]] || die "工作区未构建: ${DEPLOY_WS}/install/setup.bash（先 bash scripts/build.sh）"

bash -n "${START_SOURCE}" || die "start_guide_pulse.sh 语法检查失败"

install -d -m 0755 "$(dirname "${START_TARGET}")" "${LOG_DIR}"
install -m 0755 "${START_SOURCE}" "${START_TARGET}"
# 把部署时的工作区烧进副本的默认值：EM/手动运行不传 GUIDE_PULSE_WS 也不会
# 落回源码里的 /opt/guide_pulse 示例路径（传 env 仍可覆盖）
sed -i "s|GUIDE_PULSE_WS:-/opt/guide_pulse}|GUIDE_PULSE_WS:-${DEPLOY_WS}}|" "${START_TARGET}"
bash -n "${START_TARGET}" || die "部署副本语法检查失败"
chown -R run:run "${LOG_DIR}"
log "已部署 ${START_TARGET}（默认工作区已烧录为 ${DEPLOY_WS}）"
log "完成。process_manager 配置请按 docs/AutoStart.md 手动修改。"
