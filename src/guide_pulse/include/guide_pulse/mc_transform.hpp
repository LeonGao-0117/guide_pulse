// ============================================================================
// 迎宾变形模块：直调厂家运控（SetMcAction/GetMcAction）确保目标形态。
// 两个方向：ensure_biped 人形（迎宾变形/2026-10-18 导航前置——收到导航
// 指令先站起来再走）；ensure_quadruped 四足（2026-10-18 迎宾准备模式：
// 到达迎宾点后变四足、摄像头舵机收平）。
//
// 做法对齐 t1_slam_nav_ws 的 stand_if_down.py：发动作无需信源注册，
// 与导航栈的速度桥并存不冲突；转移路径与判定口径均真机实测校准
//（详见 mc_transform.cpp 内注释）。异步发起、失败只记日志不阻断播报。
// ============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <aimdk_msgs/srv/get_mc_action.hpp>
#include <aimdk_msgs/srv/set_mc_action.hpp>
#include <aimdk_msgs/srv/set_servo.hpp>

namespace guide_pulse
{

// ---- 纯函数（单测覆盖，见 test/test_mc_transform.cpp）----

// 从「当前动作」走到人形的转移路径表：键 = 当前动作名，值 = 依次要发的
// 动作序列（空 = 已是人形只需等待）。表里没有的当前动作一律拒绝不猜
const std::map<std::string, std::vector<std::string>> & mc_path_to_biped();

// 从「当前动作」走到四足的转移路径表（2026-10-18 迎宾准备模式）：
// 键 = 当前动作名，值 = 依次要发的动作序列（空 = 已是四足只需收摄像头）。
// 表里没有的当前动作一律拒绝不猜。人形 -> BIPED_TO_QUADRUPED 发完运控
// 自动落四足行走态（SDK 状态图 B2Q -.-> QBASE）；趴姿扶到四足站立即
// 达标（无需再进行走态）
const std::map<std::string, std::vector<std::string>> & mc_path_to_quadruped();

// 「趴着」的动作集合（路径表键里非人形非站立的那些）
const std::set<std::string> & mc_down_actions();

// 「已是四足形态」的动作集合（四足行走/站立族，ensure_quadruped 的
// 快速通道判定：直接收摄像头不再发动作）
const std::set<std::string> & mc_quadruped_actions();

// CommonState.reason 与形态切换相关失败原因的人话（报裸数字没法排障）
std::string mc_reason_text(int reason);

// ---------------------------------------------------------------------------
// 迎宾变形执行器：scene=welcome 时确保人形形态。至多一个变形线程在途，
// 重复发起被跳过；wait_done() 供调用方「先变形、后人声」的同步点
// ---------------------------------------------------------------------------
class McTransform final
{
public:
  struct Config
  {
    bool enabled{false};          // false = 功能关闭（保持当前姿态直接播报）
    bool stand_from_down{true};   // 趴姿是否一路扶起再变形（迎宾现场有人在
                                  // 机器人跟前，与导航场景不同）
    std::string get_action_service;  // 默认 /aimdk_5Fmsgs/srv/GetMcAction
    std::string set_action_service;  // 默认 /aimdk_5Fmsgs/srv/SetMcAction
    std::string source_name;         // SetMcAction.request.source（非空即可，
                                     // 无需信源注册——stand_if_down 实测）
    int wait_timeout_seconds{60};    // 变形全程时限（含趴姿起立最长五步）
    // 摄像头舵机（头部相机随形态翻转：0 度=四足收平，90 度=双足竖起）。
    // 变形到位后调 SetServo 归位（幂等，已到位也调），失败只告警不阻断
    // 播报：人形竖起 camera_position（迎宾人脸检测可用），四足收平
    // camera_retract_position（2026-10-18 迎宾准备模式：摄像头收进去）
    bool camera_servo_enabled{true};
    int camera_position{90};         // 人形时摄像头舵机角度（0-90）
    int camera_retract_position{0};  // 四足时摄像头舵机角度（0-90，SDK 0=四足位置）
    int camera_speed{500};           // 舵机速度（100-1000 步/秒）
    std::string servo_service;       // 默认 /aimdk_5Fmsgs/srv/SetServo
    // 变形音：要发变形动作前后台 SSH 运控板 paplay 厂家官方变形音（先开口、
    // 立刻发动作，声音与变形叠加；2026-10-16 补齐迎宾直调路径缺的这声）。
    // 背景：固件 v2d0 起 /hal_audio 没了，/aima/hal/audio/playback 无人订阅
    //（PCM 直推死路）；PlayAudioFile 服务在图里但请求无人应答。扬声器走
    // 运控板 PulseAudio（audio_manager 路由日志），官方 wav 也在运控板上
    bool sound_enabled{true};
    std::string sound_path;      // 人形方向（狗->人）wav 绝对路径（默认厂家官方
                                 // /robot/software/aimrt_agent/bin/cfg/t1/audio/transform4to2.wav）
    std::string down_sound_path;  // 四足方向（人->狗）wav 绝对路径（默认同目录
                                  // transform2to4.wav——按 4to2 命名推定，真机
                                  // 校准点：文件不存在仅一条 WARN 不影响变形）
    std::string ssh_target;      // SSH 目标 user@host（默认 run@10.1.1.100）
    std::string ssh_pass;        // SSH 密码（空 = 变形音禁用；复用 display_remote_pass）
  };

  // enabled=true 时创建独立回调组与两个运控客户端（回执由 executor 投递，
  // 变形线程等待——TtsClient 同款模式）
  McTransform(rclcpp::Node::SharedPtr node, Config config);
  ~McTransform();

  // 异步发起「确保人形形态」流程（不阻塞播报；结果只记日志）。
  // source 仅用于日志溯源（如 "迎宾:{operation_id}"）
  void ensure_biped(const std::string & source);

  // 异步发起「确保四足形态」流程（2026-10-18 迎宾准备模式：到达迎宾点
  // 后变四足 + 摄像头收平；结果只记日志不阻断播报）。source 用于日志
  // 溯源（如 "迎宾准备:{point}"）
  void ensure_quadruped(const std::string & source);

  // 变形流程是否未结束（去重判定）
  bool in_flight() const {return transform_in_flight_.load();}

  // 最近一次人形确保流程是否确认「机器人已是双足且运行中」
  //（2026-10-18 现场需求：导航指令必须等双足形态确认后才允许发给
  //  t1_slam_nav_ws）。语义：ensure_biped 发起即清零，流程走到
  //  「已是人形」或「变形完成且 target 运行中」两处成功点才置位；
  //  失败路径（运控不可用/查形态失败/动作被拒/未到位）保持 false。
  //  ensure_quadruped 发起同样清零——变四足成功也不是人形
  bool biped_confirmed() const {return biped_ok_.load();}

  // 最近一次人形确保流程是否**实际发生了形态切换**（走了发变形动作
  // 的路径且成功落位，成功点②）。对照 biped_confirmed 的「已是人形」
  // 零动作路径（成功点①）不置位——2026-10-18 现场需求：变形动作自带
  // 约 180° 转身会把 AMCL 定位带偏，导航/迎宾前置变形**真的变了形**才
  // 需要向 t1_slam_nav_ws 发 {command: set_pose}（用最新停留点重定位
  // 拉回）；本来就是人形则不发。语义：ensure_biped/ensure_quadruped
  // 发起即清零，仅变形线程在成功点②写；四足方向恒不置位（set_pose
  // 要求人形，四足无重定位面）
  bool biped_transformed() const {return biped_switched_.load();}

  // 等待变形流程结束（join）。返回 true = 本次确有线程被收尾。
  // 必然在有限时间返回：内部每层等待都有上限，运控不可达时立即放弃
  bool wait_done();

  // 停止并回收变形线程（等待循环内 rclcpp::ok() 使其在数百毫秒内退出）
  void shutdown();

private:
  // 变形流程线程体（to_biped=true 人形 / false 四足）：查当前动作 ->
  // 按对应路径表逐步发动作等到位 -> 舵机归位。双方向共用一套骨架，
  // 差异点（路径表/目标态/变形动作/摄像头角度/变形音）按方向取
  void run_ensure_form(const std::string & source, bool to_biped);
  // 直调运控原语（变形线程内同步调用）：
  // 查当前动作 -> (动作名, 状态值)；空动作名 = 调用失败（why 给原因）
  std::pair<std::string, int> get_current_action(std::string & why);
  // 发一条动作。空串 = 成功；否则为已带人话的失败原因
  std::string send_action(const std::string & action);
  // 轮询等动作变 expected 且 RUNNING；settle_seconds>0 须连续保持（运控
  // 受理瞬间即 RUNNING 但物理未完，单次判定会被 1008 限频拒——实测）
  std::string wait_for_action(
    const std::string & expected,
    std::chrono::steady_clock::time_point deadline,
    double settle_seconds = 0.0);
  // 竖起摄像头舵机（变形线程内同步调用）。空串 = 成功；否则为失败原因
  std::string raise_camera();
  // 摄像头舵机转到指定角度（人形竖起/四足收平共用；变形线程内同步
  // 调用）。空串 = 成功；否则为失败原因
  std::string set_camera(int position);
  // 后台播变形音（先开口、立刻返回，不挡变形动作；失败只记日志）。
  // 仅在路径含变形动作时调用；至多一个播放线程在途
  void start_transform_chime(const std::string & source, const std::string & path);
  // 播放线程体：SSH 运控板 paplay 官方变形音 wav（阻塞至播完才返回）
  void run_chime(const std::string & source, const std::string & path);

  rclcpp::Node::SharedPtr node_;
  Config config_;
  rclcpp::Client<aimdk_msgs::srv::SetMcAction>::SharedPtr set_action_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_action_client_;
  rclcpp::Client<aimdk_msgs::srv::SetServo>::SharedPtr servo_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::thread transform_thread_;                 // 当前变形调用线程（至多一个）
  std::atomic_bool transform_in_flight_{false};  // 变形流程未结束（去重入口）
  std::atomic_bool biped_ok_{false};             // 最近一次人形确保的结果
                                                 //（见 biped_confirmed；发起清零、
                                                 //  成功点置位，仅变形线程写）
  std::atomic_bool biped_switched_{false};       // 最近一次人形确保是否实际
                                                 // 发生了变形（见 biped_transformed；
                                                 //  发起清零、成功点②置位，仅
                                                 //  变形线程写）
  std::thread chime_thread_;                     // 变形音播放线程（至多一个）
  std::atomic_bool chime_in_flight_{false};      // 播放未结束（收尾判定）
};

}  // namespace guide_pulse
