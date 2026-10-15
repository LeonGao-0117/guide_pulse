// ============================================================================
// 迎宾二维码缓存/下载组件纯函数单测（无 ROS 依赖）：
// sha256 摘要已知向量、PNG/JPEG 魔数检测、qr_sha256 格式校验、
// 缓存写入/校验拒绝/旧文件淘汰。HTTP 传输层（libcurl + Basic 鉴权）
// 由集成联调覆盖（RobotCar simulate-app/simulate-robot 模拟器）。
// ============================================================================
#include <gtest/gtest.h>

#include <sys/stat.h>

#include <cstdlib>
#include <fstream>
#include <string>

#include "guide_pulse/qr_code_cache.hpp"

namespace
{

using guide_pulse::QrCodeCache;
using guide_pulse::detect_image_type;
using guide_pulse::sha256_hex;

// NIST 已知向量：sha256("abc") / sha256("")
constexpr const char * kShaAbc =
  "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
constexpr const char * kShaEmpty =
  "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";

// 构造最小合法 PNG/JPEG 字节串（魔数 + 少量载荷；缓存校验只看魔数与哈希）
std::string make_png()
{
  return std::string("\x89PNG\r\n\x1a\n", 8) + "fake-png-body";
}

std::string make_jpeg()
{
  return std::string("\xFF\xD8\xFF", 3) + "fake-jpeg-body";
}

// 每用例独立临时目录（mkdtemp），析构递归清理由系统 /tmp 承担
std::string make_temp_dir()
{
  char tmpl[] = "/tmp/qr_cache_test_XXXXXX";
  const char * dir = ::mkdtemp(tmpl);
  return dir == nullptr ? "" : std::string(dir);
}

bool file_exists(const std::string & path)
{
  struct stat st{};
  return ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

TEST(Sha256Hex, KnownVectors)
{
  EXPECT_EQ(sha256_hex("abc"), kShaAbc);
  EXPECT_EQ(sha256_hex(""), kShaEmpty);
  EXPECT_EQ(sha256_hex(make_png()).size(), 64U);
}

TEST(DetectImageType, MagicNumbers)
{
  EXPECT_EQ(detect_image_type(make_png()), "png");
  EXPECT_EQ(detect_image_type(make_jpeg()), "jpg");
  EXPECT_EQ(detect_image_type("GIF89a...."), "");   // GIF 不受支持
  EXPECT_EQ(detect_image_type("BM...."), "");       // BMP 不受支持
  EXPECT_EQ(detect_image_type(""), "");             // 空
  EXPECT_EQ(detect_image_type("\x89PN"), "");       // 截断的 PNG 魔数
}

TEST(ValidQrSha256, FormatRules)
{
  const std::string valid(64, 'a');
  EXPECT_TRUE(guide_pulse::valid_qr_sha256(valid));
  EXPECT_TRUE(guide_pulse::valid_qr_sha256(kShaAbc));
  EXPECT_FALSE(guide_pulse::valid_qr_sha256(std::string(63, 'a')));   // 短
  EXPECT_FALSE(guide_pulse::valid_qr_sha256(std::string(65, 'a')));   // 长
  EXPECT_FALSE(guide_pulse::valid_qr_sha256(std::string(64, 'A')));   // 大写
  EXPECT_FALSE(guide_pulse::valid_qr_sha256(std::string(64, 'g')));   // 非 hex
  EXPECT_FALSE(guide_pulse::valid_qr_sha256(""));                     // 空
}

TEST(QrCodeCacheStore, WritesValidPngAndCaches)
{
  const std::string dir = make_temp_dir();
  ASSERT_FALSE(dir.empty());
  const std::string png = make_png();
  const std::string sha = sha256_hex(png);

  const auto result = QrCodeCache::store_to_cache(dir, png, sha);
  EXPECT_TRUE(result.ok) << result.detail;
  EXPECT_EQ(result.image_path, dir + "/qr_" + sha + ".png");
  EXPECT_TRUE(file_exists(result.image_path));

  // cached_path 命中同一文件
  QrCodeCache cache({"/", dir, 5});
  const auto hit = cache.cached_path(sha);
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(*hit, result.image_path);
  // 未缓存的哈希不命中
  EXPECT_FALSE(cache.cached_path(kShaEmpty).has_value());
}

// 回归：父目录整体缺失时须递归创建（真机故障——单层 mkdir
// 在 /tmp/guide_pulse 缺失时静默失败，导致"无法写入临时文件"）
TEST(QrCodeCacheStore, CreatesMissingParentDirs)
{
  char tmpl[] = "/tmp/qr_cache_test_XXXXXX";
  ASSERT_NE(::mkdtemp(tmpl), nullptr);
  const std::string root = tmpl;
  ::rmdir(root.c_str());  // 整个路径从头不存在
  const std::string nested = root + "/guide_pulse/qr_cache";

  const std::string png = make_png();
  const auto result = QrCodeCache::store_to_cache(nested, png, sha256_hex(png));
  EXPECT_TRUE(result.ok) << result.detail;
  EXPECT_TRUE(file_exists(result.image_path));

  QrCodeCache cache({"/", nested, 5});
  EXPECT_TRUE(cache.cached_path(sha256_hex(png)).has_value());
}

TEST(QrCodeCacheStore, WritesValidJpeg)
{
  const std::string dir = make_temp_dir();
  ASSERT_FALSE(dir.empty());
  const std::string jpeg = make_jpeg();
  const auto result = QrCodeCache::store_to_cache(dir, jpeg, sha256_hex(jpeg));
  EXPECT_TRUE(result.ok) << result.detail;
  EXPECT_EQ(result.image_path, dir + "/qr_" + sha256_hex(jpeg) + ".jpg");
}

TEST(QrCodeCacheStore, RejectsHashMismatch)
{
  const std::string dir = make_temp_dir();
  ASSERT_FALSE(dir.empty());
  // 期望哈希与内容不符：不得写入缓存（协议：校验失败不得显示、不得写入）
  const auto result = QrCodeCache::store_to_cache(dir, make_png(), kShaEmpty);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.detail.find("哈希"), std::string::npos);
  EXPECT_FALSE(file_exists(dir + "/qr_" + std::string(kShaEmpty) + ".png"));
}

TEST(QrCodeCacheStore, RejectsNonImageAndBadSha)
{
  const std::string dir = make_temp_dir();
  ASSERT_FALSE(dir.empty());
  // 非 PNG/JPEG 魔数
  const auto not_image = QrCodeCache::store_to_cache(
    dir, "plain text", sha256_hex("plain text"));
  EXPECT_FALSE(not_image.ok);
  // sha 格式非法
  const auto bad_sha = QrCodeCache::store_to_cache(dir, make_png(), "xyz");
  EXPECT_FALSE(bad_sha.ok);
  // 超过 512KiB 契约上限
  std::string big = make_png();
  big.resize(guide_pulse::QR_IMAGE_MAX_BYTES + 1, 'x');
  const auto too_big = QrCodeCache::store_to_cache(dir, big, sha256_hex(big));
  EXPECT_FALSE(too_big.ok);
}

TEST(QrCodeCacheStore, EvictsOldHashFiles)
{
  const std::string dir = make_temp_dir();
  ASSERT_FALSE(dir.empty());
  // 第一次：存 hash A
  const std::string png_a = make_png();
  const std::string sha_a = sha256_hex(png_a);
  const auto first = QrCodeCache::store_to_cache(dir, png_a, sha_a);
  ASSERT_TRUE(first.ok);

  // 第二次：图片更新，存 hash B -> 旧 hash A 文件被淘汰
  const std::string png_b = make_jpeg();
  const std::string sha_b = sha256_hex(png_b);
  const auto second = QrCodeCache::store_to_cache(dir, png_b, sha_b);
  ASSERT_TRUE(second.ok);

  QrCodeCache cache({"/", dir, 5});
  EXPECT_FALSE(cache.cached_path(sha_a).has_value());  // 旧哈希未命中
  const auto hit = cache.cached_path(sha_b);
  ASSERT_TRUE(hit.has_value());                        // 新哈希命中
  EXPECT_EQ(*hit, second.image_path);
  // 无 .tmp 残留
  EXPECT_FALSE(file_exists(dir + "/qr_" + sha_b + ".jpg.tmp"));
}

}  // namespace
