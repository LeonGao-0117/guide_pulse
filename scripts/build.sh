#!/usr/bin/env bash
# ============================================================================
# guide_pulse 一键构建脚本（薄包装：转发到 deploy.sh --build-only）
# 实际逻辑（前置检查 / 按内存自动收紧并行度 / colcon）都在 deploy.sh
# 中维护，避免两份编译逻辑漂移。SDK 消息包 aimdk_msgs 已内置 src/，
# 构建不再依赖外部 SDK 目录。
#
# 用法：bash scripts/build.sh [deploy.sh 的参数，如 --clean]
# 新环境部署（含装依赖）请直接用 ./deploy.sh
# ============================================================================
set -euo pipefail

BUILD_SH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${BUILD_SH_DIR}/deploy.sh" --build-only "$@"
