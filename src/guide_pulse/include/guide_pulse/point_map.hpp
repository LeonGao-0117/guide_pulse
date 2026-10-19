// ============================================================================
// 点位映射表本地存储（v0.25.0 P-039 point_map 指令的 Robot 侧落地；
// v0.28.0 P-042 条目含可选播放文本 text）。
//
// 契约要点：App 是映射表的唯一权威来源，point_map.points 全量下发，
// Robot 整表替换（不做增量合并）；Server 不保存映射表状态；映射如何
// 持久化（内存/落盘、是否跨重启保留）归 Robot 本地实现——本实现选择
// 落盘 JSON 文件（临时文件 + rename 原子替换）到持久化分区，重启后
// 自动加载，App 无需重推；断电不丢（点位映射 + 播放词一起落盘）。
//
// 查询方向：navigate 指令携带的 point_name -> code（本地航点名，如
// 「A」）-> NavExecutor 执行。2026-10-17 联调确认 App 下发的就是
// code 本身（如「A」），故 resolve 以 code 匹配为主路径；name 匹配
// （人话名如「比亚迪ATTO 1」-> A）保留为兜底，两种用法均兼容。
//
// 线程模型：仅桥接节点 worker 线程访问（replace/resolve），load 在
// worker 启动前调用——免锁。纯 C++ 无 ROS 依赖，可独立单测。
// ============================================================================
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "guide_pulse/guide_protocol.hpp"

namespace guide_pulse
{

class PointMapStore
{
public:
  // file_path：落盘文件绝对路径（目录不存在时自动创建）；空 = 只存内存
  //（联调用，重启即失效）
  explicit PointMapStore(std::string file_path);

  // 全量替换：先原子落盘（写 .tmp 后 rename）成功才更新内存——落盘
  // 失败返回 false 且内存保留旧表（与 point_map.result=failed 的
  // 「未生效」语义一致），调用方回 failed/point_map_failed。
  // points 已由协议层校验（1..256 条、code 唯一、标量上限），此处信任入参
  bool replace(const std::vector<PointMapEntry> & points);

  // 启动加载：文件不存在 = 空表（首次部署，等 App 推表，返回 true）；
  // 文件存在但损坏/结构非法 = 空表并返回 false（保留旧文件等 App 重推
  // 覆盖，不删除——便于人工排查）。仅 worker 启动前调用一次
  bool load();

  // point_name -> code 两级解析（均表内权威精确匹配、区分大小写）：
  // 先按 code 匹配（2026-10-17 联调确认 App 下发的就是 code，如「A」），
  // 未命中再按 name 匹配（兜底兼容人话名）；空表/两级均未命中返回
  // nullopt，调用方自行兜底
  std::optional<std::string> resolve(const std::string & point_name) const;

  // 名称关键词 -> code 子串匹配（2026-10-19 现场需求：welcome_prep 语料
  // 的点位部分写名称关键词如「迎宾」，不绑定 code——在 name 中找含
  // 关键词的首个条目取其 code，App 换迎宾点 code 改表即生效）；空
  // 关键词/未命中返回 nullopt（与 resolve 独立，不影响精确解析语义）
  std::optional<std::string> resolve_by_name_keyword(const std::string & keyword) const;

  // code -> 该点位播放文本（v0.28.0 P-042）：条目未配置 text 或 code
  // 未命中返回 nullopt。供 navigate 到达播报（指令未带 texts 时的
  // 优先级来源，见 mqtt_guide_bridge_node 到达分支）
  std::optional<std::string> text_for_code(const std::string & code) const;

  // 当前表内容（日志/测试用）
  const std::vector<PointMapEntry> & points() const {return points_;}

  std::size_t size() const {return points_.size();}

  // 落盘文件路径（日志用）
  const std::string & file_path() const {return file_path_;}

private:
  std::string file_path_;
  std::vector<PointMapEntry> points_;
};

}  // namespace guide_pulse
