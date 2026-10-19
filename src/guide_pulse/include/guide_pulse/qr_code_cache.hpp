// ============================================================================
// 迎宾二维码缓存与下载组件（RobotCar 协议 v0.22.0 / P-036）：
//   scene 下行携带 qr_sha256（Server 端当前二维码内容哈希快照）->
//   与本地单文件缓存比对 -> 命中直接返回缓存图片路径（不下载）；
//   未命中经 HTTP 文件面
//     GET {origin}/v1/robots/{robot_id}/qr-code/content
//   以自身 MQTT 凭据（HTTP Basic，username=robot_id）拉取一次，
//   校验字节 sha256 与快照一致 + PNG/JPEG 魔数 + ≤512KiB 后原子替换
//   本地缓存（文件名内嵌哈希：qr_{sha256}.{png|jpg}，同时淘汰旧哈希文件）。
//
// 失败语义（协议 qr-file-protocol.md）：
//   - 下载失败/校验失败：本次迎宾不显示二维码（不阻塞迎宾动作，
//     不产生上行事件，单次迎宾内不做重试风暴）；
//   - 401：凭据失效（协议建议走 enroll 循环重取后重试一次；当前实现
//     记日志放弃——凭据轮换由 MQTT 重连路径统一驱动，此处不重复 enroll）；
//   - 404：Server 端未设置二维码（App 从未上传或已清除）。
//
// 只依赖 libcurl + OpenSSL + 标准库，不依赖 ROS，便于独立单元测试
//（HTTP 传输层由集成联调覆盖，单测覆盖哈希/魔数/缓存文件操作）。
// ============================================================================
#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "guide_pulse/guide_protocol.hpp"

namespace guide_pulse
{

// 契约：迎宾二维码图片上限 512 KiB（Protocol/contracts/shared/constants.md）
inline constexpr std::size_t QR_IMAGE_MAX_BYTES = 524288;

// 计算数据的 SHA-256 摘要，返回 64 位小写十六进制串（OpenSSL EVP）。
// 输入为任意字节串。失败返回空串（不抛异常）。
std::string sha256_hex(const std::string & data);

// 图片魔数检测：PNG（\x89PNG\r\n\x1a\n）-> "png"、
// JPEG（\xFF\xD8\xFF）-> "jpg"，其余返回空串。
std::string detect_image_type(const std::string & data);

class QrCodeCache
{
public:
  struct Config
  {
    std::string origin;       // Server HTTP 面（与 enroll_origin 同源，如 http://host:8780）
    std::string cache_dir;    // 本地缓存目录（自动创建；跨重启保留可免二次下载）
    int timeout_seconds{5};   // 单次下载（连接+传输）超时
  };

  struct FetchResult
  {
    bool ok{false};
    std::string image_path;   // ok 时的本地图片绝对路径（缓存文件）
    std::string detail;       // 失败原因（日志用，不含密码）
  };

  explicit QrCodeCache(Config config);

  // 缓存命中查询：qr_sha256 对应图片已在本地返回其路径，否则 nullopt。
  // 文件名内嵌哈希，命中即内容可信（写入前已校验过哈希）。
  std::optional<std::string> cached_path(const std::string & qr_sha256) const;

  // 下载 + 校验 + 落缓存（未命中时调用；命中请直接用 cached_path 免下载）。
  // robot_id/password：当次 enroll 的 MQTT 凭据（Basic username=robot_id）。
  // 线程模型：阻塞 HTTP；迎宾流程中由阶段 1 后台准备线程调用（其余
  // 调用方为 worker 线程）。无共享可变状态（curl 句柄每次调用独立），
  // 跨线程调用安全；调用方须先在 worker 上拷贝凭据再传入。
  FetchResult fetch(
    const std::string & robot_id, const std::string & password,
    const std::string & qr_sha256);

  // 校验并写入缓存（原子替换 + 淘汰其它哈希文件）。独立出来便于单测：
  // data 为图片字节，qr_sha256 为期望内容哈希（scene 快照）。
  // 校验项：哈希一致、PNG/JPEG 魔数、大小 ≤512KiB。任一不符不落盘。
  static FetchResult store_to_cache(
    const std::string & cache_dir, const std::string & data,
    const std::string & qr_sha256);

private:
  Config config_;
};

}  // namespace guide_pulse
