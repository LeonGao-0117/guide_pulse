// ============================================================================
// 迎宾模块（scene 指令，协议 v0.17.0 + v0.20.0 可选 texts）：
// 确定迎宾播报词 -> 由桥接节点开一轮 PlaylistSession。
// 纯函数实现，无 ROS/MQTT 依赖，独立单测（test_welcome.cpp）。
//
// // ponytail: 迎宾动作（SetMcPresetMotion 招手）预留参数 welcome_motion_id
// //（默认 0 禁用）——动作 ID 因机型而异（文档 examples：T1=1002，另一机型
// //=3001），真机校准后接入；届时需先 SetMcInputSource 注册自定义输入源。
// ============================================================================
#pragma once

#include <string>
#include <vector>

namespace guide_pulse
{

// 迎宾词兜底（配置错误时也要出声，避免哑迎宾）
inline const std::vector<std::string> & builtin_welcome_texts()
{
  static const std::vector<std::string> kTexts{"欢迎光临！我是展厅讲解机器人，很高兴为您服务。"};
  return kTexts;
}

// 生成迎宾播报计划：指令 texts 优先；缺省回落参数默认词；两者皆空回落
// 内置兜底词（防御：welcome_default_texts 被配成空列表）。
// 返回值恒非空（迎宾至少播一条）。
inline std::vector<std::string> make_welcome_texts(
  const std::vector<std::string> & command_texts,
  const std::vector<std::string> & default_texts)
{
  if (!command_texts.empty()) {
    return command_texts;  // 协议 v0.20.0：App 指定的迎宾词，Server 原样转发
  }
  if (!default_texts.empty()) {
    return default_texts;
  }
  return builtin_welcome_texts();
}

}  // namespace guide_pulse
