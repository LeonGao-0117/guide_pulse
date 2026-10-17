// ============================================================================
// 点位映射表本地存储实现：全量替换 + 原子落盘 + 启动加载 + name->code
// 解析。落盘格式与协议载荷同构（{"spec_version","points":[{code,name}]}），
// 2 格缩进美化（每元素一行），便于人工检查与跨版本兼容。
// ============================================================================
#include "guide_pulse/point_map.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <utility>

#include <nlohmann/json.hpp>

namespace guide_pulse
{

namespace
{

using nlohmann::json;

// UTF-8 码点计数（与 guide_protocol.cpp 同规则：非续字节即码点起点）
std::size_t utf8_scalar_count(const std::string & value)
{
  std::size_t count = 0;
  for (const unsigned char c : value) {
    if ((c & 0xC0U) != 0x80U) {
      ++count;
    }
  }
  return count;
}

// 单条目结构校验（加载路径复用契约限制；解析路径已在协议层校验）
bool valid_entry(const std::string & code, const std::string & name)
{
  const auto code_scalars = utf8_scalar_count(code);
  const auto name_scalars = utf8_scalar_count(name);
  return code_scalars >= 1U && code_scalars <= POINT_CODE_MAX_SCALARS &&
         name_scalars >= 1U && name_scalars <= POINT_NAME_MAX_SCALARS;
}

}  // namespace

PointMapStore::PointMapStore(std::string file_path)
: file_path_(std::move(file_path)) {}

bool PointMapStore::replace(const std::vector<PointMapEntry> & points)
{
  if (!file_path_.empty()) {
    // 原子落盘：目录兜底创建 -> 写同目录 .tmp -> fsync -> rename 替换。
    // rename 同文件系统原子，读端（load）要么看到旧表要么看到完整新表
    const auto parent = std::filesystem::path(file_path_).parent_path();
    std::error_code ec;
    if (!parent.empty() && !std::filesystem::exists(parent, ec)) {
      std::filesystem::create_directories(parent, ec);
      if (ec) {
        return false;  // 目录创建失败：映射未生效，调用方回 failed
      }
    }
    json doc;
    doc["spec_version"] = "1.0";
    json items = json::array();
    for (const auto & entry : points) {
      items.push_back({{"code", entry.code}, {"name", entry.name}});
    }
    doc["points"] = std::move(items);

    const std::string tmp_path = file_path_ + ".tmp";
    {
      std::ofstream out(tmp_path, std::ios::binary | std::ios::trunc);
      if (!out.is_open()) {
        return false;
      }
      // 缩进 2 格美化落盘：每个元素独立一行，便于人工查看与 diff
    out << doc.dump(2) << '\n';
      out.flush();
      if (!out.good()) {
        out.close();
        std::remove(tmp_path.c_str());
        return false;
      }
    }
    std::filesystem::rename(tmp_path, file_path_, ec);
    if (ec) {
      std::remove(tmp_path.c_str());
      return false;
    }
  }
  // 落盘成功（或纯内存模式）：整表替换内存（全量替换语义，不做合并）
  points_ = points;
  return true;
}

bool PointMapStore::load()
{
  points_.clear();
  if (file_path_.empty()) {
    return true;  // 纯内存模式：空表合法
  }
  std::error_code ec;
  if (!std::filesystem::exists(file_path_, ec)) {
    return true;  // 首次部署无文件：空表合法（等 App 推表）
  }
  std::ifstream in(file_path_, std::ios::binary);
  if (!in.is_open()) {
    return false;
  }
  json doc;
  try {
    in >> doc;
  } catch (const json::exception &) {
    return false;  // 文件损坏：空表，保留文件等 App 重推覆盖
  }
  if (!doc.is_object() || !doc.contains("points") || !doc["points"].is_array()) {
    return false;
  }
  std::vector<PointMapEntry> loaded;
  std::set<std::string> seen_codes;
  for (const auto & item : doc["points"]) {
    if (!item.is_object() || !item.contains("code") || !item["code"].is_string() ||
      !item.contains("name") || !item["name"].is_string())
    {
      return false;  // 结构不符：整表作废（防御文件被改动）
    }
    PointMapEntry entry;
    entry.code = item["code"].get<std::string>();
    entry.name = item["name"].get<std::string>();
    if (!valid_entry(entry.code, entry.name) || !seen_codes.insert(entry.code).second) {
      return false;
    }
    loaded.push_back(std::move(entry));
  }
  if (loaded.size() > POINT_MAP_POINTS_MAX) {
    return false;
  }
  points_ = std::move(loaded);
  return true;
}

std::optional<std::string> PointMapStore::resolve(const std::string & point_name) const
{
  // 两级解析，均为表内权威精确匹配（区分大小写，与 Server 侧校验一致）：
  // 1) code 匹配（主路径：2026-10-17 联调确认 App 下发的 point_name
  //    就是 code 本身，如「A」，直接命中对应航点）；
  // 2) name 匹配（兜底兼容：App 界面人话名如「比亚迪ATTO 1」-> 本地
  //    航点名，首个命中——name 重复时取先到者，与 App 展示顺序一致）。
  // code 优先于 name，交叉重复时以 code 命中为准。
  const auto by_code = std::find_if(
    points_.begin(), points_.end(),
    [&point_name](const PointMapEntry & entry) {return entry.code == point_name;});
  if (by_code != points_.end()) {
    return by_code->code;
  }
  const auto by_name = std::find_if(
    points_.begin(), points_.end(),
    [&point_name](const PointMapEntry & entry) {return entry.name == point_name;});
  if (by_name != points_.end()) {
    return by_name->code;
  }
  return std::nullopt;
}

}  // namespace guide_pulse
