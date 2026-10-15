// ============================================================================
// 迎宾二维码缓存与下载组件实现（接口说明见 qr_code_cache.hpp 文件头）
// ============================================================================
#include "guide_pulse/qr_code_cache.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <fstream>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <curl/curl.h>
#include <openssl/evp.h>

namespace guide_pulse
{

namespace
{

// curl 响应体回调：追加到调用方字符串；超过契约上限视为异常响应主动中止
//（正常二维码 ≤512KiB，超限即恶意/错端点响应）
std::size_t append_body(char * ptr, std::size_t size, std::size_t nmemb, void * userdata)
{
  auto * out = static_cast<std::string *>(userdata);
  const std::size_t bytes = size * nmemb;
  if (out->size() + bytes > QR_IMAGE_MAX_BYTES) {
    return 0;  // 中止传输，curl 返回 CURLE_WRITE_ERROR
  }
  out->append(ptr, bytes);
  return bytes;
}

// 16 进制字符表（小写）
inline char hex_digit(unsigned value)
{
  return static_cast<char>(value < 10 ? '0' + value : 'a' + (value - 10));
}

// 缓存文件名：qr_{sha256}.{png|jpg}（哈希内嵌文件名，命中即内容可信）
std::string cache_file_path(
  const std::string & cache_dir, const std::string & qr_sha256,
  const std::string & image_type)
{
  const std::string & dir = cache_dir;
  const std::string base = (dir.back() == '/' ? dir.substr(0, dir.size() - 1U) : dir);
  return base + "/qr_" + qr_sha256 + "." + image_type;
}

// 递归创建目录（mkdir -p 语义）：逐级创建缺失的目录段，已存在视为成功。
// 最终路径为普通文件（同名文件占位）时返回 false。
bool mkdir_p(const std::string & path)
{
  if (path.empty()) {
    errno = EINVAL;
    return false;
  }
  std::string current;
  for (std::size_t i = 0; i < path.size(); ++i) {
    current.push_back(path[i]);
    const bool segment_end = path[i] == '/' || i + 1U == path.size();
    if (!segment_end || current == "/" || current.size() <= 1U) {
      continue;  // 未凑满一段（含前导 "/"）不调 mkdir
    }
    if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
      return false;
    }
  }
  struct stat st{};
  if (::stat(path.c_str(), &st) != 0) {
    return false;  // errno 保留 stat 的失败码
  }
  if (!S_ISDIR(st.st_mode)) {
    errno = ENOTDIR;  // 路径被同名普通文件占用
    return false;
  }
  errno = 0;
  return true;
}

// 删除目录下所有 qr_ 前缀的缓存文件（新缓存写入后淘汰旧哈希）。
// POSIX dirent 直接遍历（板端 Linux，无平台差异）；失败只影响旧文件残留，
// 不影响正确性。
void evict_other_qr_files(const std::string & cache_dir, const std::string & keep_path)
{
  DIR * dir = ::opendir(cache_dir.c_str());
  if (dir == nullptr) {
    return;
  }
  while (const dirent * entry = ::readdir(dir)) {
    const std::string name = entry->d_name;
    if (name.size() > 3U && name.compare(0, 3, "qr_") == 0) {
      const std::string path =
        cache_dir + (cache_dir.back() == '/' ? "" : "/") + name;
      if (path != keep_path) {
        std::remove(path.c_str());
      }
    }
  }
  ::closedir(dir);
}

}  // namespace

std::string sha256_hex(const std::string & data)
{
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_len = 0;
  const EVP_MD * md = EVP_sha256();
  if (EVP_Digest(
      data.data(), data.size(), digest, &digest_len, md, nullptr) != 1 ||
    digest_len != 32U)
  {
    return "";
  }
  std::string hex;
  hex.reserve(64U);
  for (unsigned int i = 0; i < digest_len; ++i) {
    hex.push_back(hex_digit(digest[i] >> 4U));
    hex.push_back(hex_digit(digest[i] & 0x0FU));
  }
  return hex;
}

std::string detect_image_type(const std::string & data)
{
  static const unsigned char kPngMagic[] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
  if (data.size() >= sizeof(kPngMagic) &&
    std::equal(kPngMagic, kPngMagic + sizeof(kPngMagic), data.begin()))
  {
    return "png";
  }
  if (data.size() >= 3U && static_cast<unsigned char>(data[0]) == 0xFF &&
    static_cast<unsigned char>(data[1]) == 0xD8 && static_cast<unsigned char>(data[2]) == 0xFF)
  {
    return "jpg";
  }
  return "";
}

QrCodeCache::QrCodeCache(Config config)
: config_(std::move(config))
{
  while (!config_.origin.empty() && config_.origin.back() == '/') {
    config_.origin.pop_back();
  }
  while (!config_.cache_dir.empty() && config_.cache_dir.back() == '/') {
    config_.cache_dir.pop_back();
  }
  if (config_.timeout_seconds <= 0) {
    config_.timeout_seconds = 5;
  }
}

std::optional<std::string> QrCodeCache::cached_path(const std::string & qr_sha256) const
{
  if (!valid_qr_sha256(qr_sha256) || config_.cache_dir.empty()) {
    return std::nullopt;
  }
  for (const char * type : {"png", "jpg"}) {
    const std::string path = cache_file_path(config_.cache_dir, qr_sha256, type);
    struct stat st{};
    if (::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode) && st.st_size > 0) {
      return path;
    }
  }
  return std::nullopt;
}

QrCodeCache::FetchResult QrCodeCache::store_to_cache(
  const std::string & cache_dir, const std::string & data, const std::string & qr_sha256)
{
  FetchResult result;
  if (!valid_qr_sha256(qr_sha256)) {
    result.detail = "qr_sha256 格式非法（需 64 位小写十六进制）";
    return result;
  }
  if (data.empty() || data.size() > QR_IMAGE_MAX_BYTES) {
    result.detail = "图片大小超限（需 1.." + std::to_string(QR_IMAGE_MAX_BYTES) + " 字节，实际 " +
      std::to_string(data.size()) + "）";
    return result;
  }
  const std::string image_type = detect_image_type(data);
  if (image_type.empty()) {
    result.detail = "图片魔数非 PNG/JPEG，拒绝缓存";
    return result;
  }
  const std::string actual_sha = sha256_hex(data);
  if (actual_sha != qr_sha256) {
    // 协议：校验失败不得写入缓存、不得显示
    result.detail = "内容哈希与 scene 快照不一致（实际 " + actual_sha + "）";
    return result;
  }
  if (cache_dir.empty()) {
    result.detail = "cache_dir 未配置";
    return result;
  }
  // 递归创建缓存目录（mkdir -p 语义）：父目录可能整体缺失（如
  // /tmp/guide_pulse/qr_cache，此前由 ScreenDisplay::show 的 mkdir -p 顺带
  // 创建，但下载发生在 show 之前，目录创建责任归本组件自负）
  if (!mkdir_p(cache_dir)) {
    result.detail = "无法创建缓存目录 " + cache_dir +
      "（errno=" + std::to_string(errno) + "）";
    return result;
  }

  const std::string final_path = cache_file_path(cache_dir, qr_sha256, image_type);
  const std::string tmp_path = final_path + ".tmp";
  {
    std::ofstream file(tmp_path, std::ios::binary | std::ios::trunc);
    if (!file) {
      result.detail = "无法写入临时文件 " + tmp_path;
      return result;
    }
    file.write(data.data(), static_cast<std::streamsize>(data.size()));
    file.flush();
    if (!file) {
      std::remove(tmp_path.c_str());
      result.detail = "写入临时文件中断: " + tmp_path;
      return result;
    }
  }
  // 原子替换：rename 同文件系统内原子，失败回退清理
  if (std::rename(tmp_path.c_str(), final_path.c_str()) != 0) {
    std::remove(tmp_path.c_str());
    result.detail = "缓存文件替换失败: " + final_path;
    return result;
  }
  // 淘汰其它哈希的旧缓存（每机器人只需保留当前一张）
  evict_other_qr_files(cache_dir, final_path);
  result.ok = true;
  result.image_path = final_path;
  return result;
}

QrCodeCache::FetchResult QrCodeCache::fetch(
  const std::string & robot_id, const std::string & password,
  const std::string & qr_sha256)
{
  FetchResult result;
  if (!valid_qr_sha256(qr_sha256)) {
    result.detail = "qr_sha256 格式非法（需 64 位小写十六进制）";
    return result;
  }
  if (config_.origin.empty() || robot_id.empty() || password.empty()) {
    result.detail = "下载配置不完整（origin/凭据缺失）";
    return result;
  }
  const std::string url =
    config_.origin + "/v1/robots/" + robot_id + "/qr-code/content";
  std::string body;

  CURL * curl = curl_easy_init();
  if (curl == nullptr) {
    result.detail = "curl_easy_init 失败";
    return result;
  }
  // HTTP Basic = 自身 MQTT 凭据（协议规定 username 必须等于路径 robot_id）
  const std::string userpwd = robot_id + ":" + password;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_USERPWD, userpwd.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);  // 多线程进程必须关掉信号量定时器
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(config_.timeout_seconds));
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, static_cast<long>(config_.timeout_seconds));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_body);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  // 与 enroll 同一受控内网明文 HTTP；不跟随重定向（防开放重定向带出凭据）
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
  curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");

  const CURLcode rc = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_easy_cleanup(curl);

  if (rc == CURLE_WRITE_ERROR) {
    // append_body 超限主动中止：响应超过 512KiB 上限
    result.detail = "二维码响应超过 512KiB 契约上限";
    return result;
  }
  if (rc != CURLE_OK) {
    result.detail = std::string("二维码下载请求失败: ") + curl_easy_strerror(rc);
    return result;
  }
  if (http_code == 401) {
    // 凭据失效：协议建议走 enroll 循环重取后重试一次；凭据轮换由 MQTT
    // 重连路径统一驱动，这里不重复 enroll（避免与连接线程竞争轮换密码），
    // 本次不显示二维码，下次迎宾/重连后自然恢复
    result.detail = "二维码下载 401：凭据失效（等待 MQTT 重连重新 enroll）";
    return result;
  }
  if (http_code == 404) {
    result.detail = "二维码下载 404：Server 端未设置二维码";
    return result;
  }
  if (http_code != 200) {
    result.detail = "二维码下载失败: HTTP " + std::to_string(http_code);
    return result;
  }
  return store_to_cache(config_.cache_dir, body, qr_sha256);
}

}  // namespace guide_pulse
