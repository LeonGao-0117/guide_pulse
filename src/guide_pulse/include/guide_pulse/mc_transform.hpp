// ============================================================================
// 迎宾变形模块：直调厂家运控（SetMcAction/GetMcAction）确保人形形态。
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

namespace guide_pulse
{

// ---- 纯函数（单测覆盖，见 test/test_mc_transform.cpp）----

// 从「当前动作」走到人形的转移路径表：键 = 当前动作名，值 = 依次要发的
// 动作序列（空 = 已是人形只需等待）。表里没有的当前动作一律拒绝不猜
const std::map<std::string, std::vector<std::string>> & mc_path_to_biped();

// 「趴着」的动作集合（路径表键里非人形非站立的那些）
const std::set<std::string> & mc_down_actions();

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
  };

  // enabled=true 时创建独立回调组与两个运控客户端（回执由 executor 投递，
  // 变形线程等待——TtsClient 同款模式）
  McTransform(rclcpp::Node::SharedPtr node, Config config);
  ~McTransform();

  // 异步发起「确保人形形态」流程（不阻塞播报；结果只记日志）。
  // source 仅用于日志溯源（如 "迎宾:{operation_id}"）
  void ensure_biped(const std::string & source);

  // 变形流程是否未结束（去重判定）
  bool in_flight() const {return transform_in_flight_.load();}

  // 等待变形流程结束（join）。返回 true = 本次确有线程被收尾。
  // 必然在有限时间返回：内部每层等待都有上限，运控不可达时立即放弃
  bool wait_done();

  // 停止并回收变形线程（等待循环内 rclcpp::ok() 使其在数百毫秒内退出）
  void shutdown();

private:
  void run_ensure_biped(const std::string & source);  // 变形调用线程体
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

  rclcpp::Node::SharedPtr node_;
  Config config_;
  rclcpp::Client<aimdk_msgs::srv::SetMcAction>::SharedPtr set_action_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMcAction>::SharedPtr get_action_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::thread transform_thread_;                 // 当前变形调用线程（至多一个）
  std::atomic_bool transform_in_flight_{false};  // 变形流程未结束（去重入口）
};

}  // namespace guide_pulse
