// ============================================================================
// 迎宾变形模块实现（接口说明见 mc_transform.hpp 文件头）。
// 转移路径与判定口径对齐 t1_slam_nav_ws 的 stand_if_down.py（真机校准）。
// ============================================================================
#include "guide_pulse/mc_transform.hpp"

#include <optional>
#include <utility>

#include "guide_pulse/service_call.hpp"

namespace guide_pulse
{

namespace
{

// 站立人形的运动模式。变形完成后运控自动落到此动作，勿手动再发
constexpr const char * kMcBipedTarget = "BIPED_LOCOMOTION_WBC";

// 狗 -> 人变形动作（带 180° 转身，与厂家 App 同款）
constexpr const char * kMcToBipedAction = "QUADRUPED_TO_BIPED_ROTATE";

// 四足站立 / 四足行走态（站定没有出边直达变形，要先进行走态）
constexpr const char * kMcStandTarget = "QUADRUPED_STAND_DEFAULT";
constexpr const char * kMcLocomotionTarget = "QUADRUPED_LOCOMOTION_DEFAULT";
constexpr const char * kMcPassive = "PASSIVE_DEFAULT";

// McActionStatus.value：100 = 运行中（0 空闲 / 200 切换中）
constexpr int kMcStatusRunning = 100;
// CommonState.SUCCESS = 1（⚠️ 0 是 UNKNOWN）
constexpr int kMcStateSuccess = 1;
// reason=16 表示「已经是目标状态」，不是失败
constexpr int kMcReasonTargetState = 16;

// 中间动作保持 3s：受理瞬间即 RUNNING 但物理未完，紧发下一条会被
// 运控限频拒；只罚中间步骤，末步无需 dwell
constexpr double kMcActionSettleSeconds = 3.0;

// 上一动作可能还在切换中（status!=100），此时发动作会被拒，先等稳定
constexpr int kMcSettleWaitSeconds = 10;

}  // namespace

// 从「当前动作」走到人形的转移路径（键 = 当前动作，值 = 依次要发的动作，
// 空 = 已是人形只需等待）。逐条抄自 SDK _ACTION_RULES；表外动作一律
// 拒绝不猜——抄错一格 = 机器人做出没预期的动作。
const std::map<std::string, std::vector<std::string>> & mc_path_to_biped()
{
  static const std::map<std::string, std::vector<std::string>> kPath = {
    {kMcBipedTarget, {}},                      // 已是人形运动模式，不动
    {"BIPED_STAND_DEFAULT", {}},               // 人形站定：自动落到 WBC，只等
    {kMcLocomotionTarget, {kMcToBipedAction}},
    {"QUADRUPED_LOCOMOTION_RUN", {kMcToBipedAction}},
    {"QUADRUPED_LOCOMOTION_TERRAIN", {kMcToBipedAction}},
    {kMcStandTarget, {kMcLocomotionTarget, kMcToBipedAction}},
    // ---- 趴姿（stand_from_down=true 才走）----
    {kMcPassive, {kMcStandTarget, kMcLocomotionTarget, kMcToBipedAction}},
    {"QUADRUPED_GET_DOWN_DEFAULT", {kMcStandTarget, kMcLocomotionTarget, kMcToBipedAction}},
    {"QUADRUPED_SIT_DOWN_DEFAULT", {kMcStandTarget, kMcLocomotionTarget, kMcToBipedAction}},
    {"DAMPING_DEFAULT", {kMcPassive, kMcStandTarget, kMcLocomotionTarget, kMcToBipedAction}},
    {"QUAD_LIE_DOWN", {kMcPassive, kMcStandTarget, kMcLocomotionTarget, kMcToBipedAction}},
  };
  return kPath;
}

// 「趴着」的动作集合（路径表键里非人形非站立的那些）
const std::set<std::string> & mc_down_actions()
{
  static const std::set<std::string> kDown = {
    kMcPassive, "DAMPING_DEFAULT", "QUAD_LIE_DOWN",
    "QUADRUPED_GET_DOWN_DEFAULT", "QUADRUPED_SIT_DOWN_DEFAULT",
  };
  return kDown;
}

// CommonState.reason 与形态切换相关失败原因的人话；报裸数字没法排障
std::string mc_reason_text(int reason)
{
  switch (reason) {
    case 4: return "当前形态不支持";
    case 5: return "低电量限制";
    case 6: return "正在充电中（要先离开充电桩）";
    case 9: return "当前模式不支持";
    case 10: return "前方有障碍物";
    case 11: return "后方有障碍物";
    case 12: return "左方有障碍物";
    case 13: return "右方有障碍物";
    case 14: return "上方有障碍物";
    case 15: return "其他任务正在运行";
    case 19: return "需要机器人处于双足站立状态";
    case 22: return "当前姿态下无法进行动作转移";
    case 27: return "机器人正在起立";
    case 30: return "当前状态不满足变形/动作执行条件";
    default: return "";
  }
}

McTransform::McTransform(rclcpp::Node::SharedPtr node, Config config)
: node_(std::move(node)), config_(std::move(config))
{
  if (!config_.enabled) {
    return;
  }
  // 独立回调组：变形线程阻塞在 future 上，服务回执由 executor 线程处理
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto qos = rmw_qos_profile_services_default;
  set_action_client_ = node_->create_client<aimdk_msgs::srv::SetMcAction>(
    config_.set_action_service, qos, callback_group_);
  get_action_client_ = node_->create_client<aimdk_msgs::srv::GetMcAction>(
    config_.get_action_service, qos, callback_group_);
}

McTransform::~McTransform()
{
  shutdown();
}

void McTransform::ensure_biped(const std::string & source)
{
  if (!config_.enabled) {
    return;  // 功能关闭：保持当前姿态直接播报
  }
  if (transform_in_flight_.load()) {
    RCLCPP_WARN(
      node_->get_logger(), "上一次迎宾变形仍在进行，本次跳过变形（%s）", source.c_str());
    return;
  }
  // join 上一线程（逻辑已走完，仅收尾），保证至多一个变形线程
  if (transform_thread_.joinable()) {
    transform_thread_.join();
  }
  transform_in_flight_.store(true);
  transform_thread_ = std::thread([this, source]() {run_ensure_biped(source);});
}

bool McTransform::wait_done()
{
  if (transform_thread_.joinable()) {
    transform_thread_.join();
    return true;
  }
  return false;
}

void McTransform::shutdown()
{
  if (transform_thread_.joinable()) {
    transform_thread_.join();
  }
}

std::pair<std::string, int> McTransform::get_current_action(std::string & why)
{
  auto response = call_with_timeout(
    *get_action_client_,
    std::make_shared<aimdk_msgs::srv::GetMcAction::Request>(), 3);
  if (!response) {
    why = "GetMcAction 无应答（运控没起？）";
    return {"", -1};
  }
  why.clear();
  return {response->info.action_desc, response->info.status.value};
}

std::string McTransform::send_action(const std::string & action)
{
  auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
  // source 留空会被运控以 Invalid source 拒（state=-10002）
  request->header.stamp = node_->now();
  request->source = config_.source_name;
  request->command.action_desc = action;
  auto response = call_with_timeout(*set_action_client_, request, 3);
  if (!response) {
    return "SetMcAction(" + action + ") 无应答（运控没起？）";
  }
  const auto & result = response->response;
  const int code = static_cast<int>(result.header.code);
  const int state = result.state.value;
  const int reason = static_cast<int>(result.state.reason);
  if (reason == kMcReasonTargetState) {
    return "";
  }
  if (code == 0 && state == kMcStateSuccess) {
    return "";
  }
  // 厂家私有负数状态码：运控只把人话写进自己的 mc.log，客户端只回数字
  std::string note = "运控拒绝 " + action +
    "（code=" + std::to_string(code) + " state=" + std::to_string(state) +
    " reason=" + std::to_string(reason) + "）";
  if (state == -1008) {
    note += "：上一动作尚在切换中，等稳定后再发";
  } else if (state == -10001) {
    note += "：成因多样（限频/电量等），详见运控板 mc.log";
  }
  const std::string text = mc_reason_text(reason);
  return text.empty() ? note : note + "：" + text;
}

std::string McTransform::wait_for_action(
  const std::string & expected, std::chrono::steady_clock::time_point deadline,
  double settle_seconds)
{
  // 稳定计时：匹配记起点、掉出去清零——「断续地对过几次」不算稳住
  std::optional<std::chrono::steady_clock::time_point> stable_since;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    std::string why;
    const auto [current, status] = get_current_action(why);
    if (why.empty() && current == expected && status == kMcStatusRunning) {
      const auto now_t = std::chrono::steady_clock::now();
      if (!stable_since.has_value()) {
        stable_since = now_t;
      }
      if (settle_seconds <= 0.0 ||
        std::chrono::duration<double>(now_t - *stable_since).count() >=
        settle_seconds)
      {
        return "";
      }
    } else {
      // 未匹配或瞬时查询失败：清零重计，持续失败由总时限兜底
      stable_since.reset();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return rclcpp::ok() ? ("等 " + expected + " 到位超时") : "收到关停信号，不再等动作到位";
}

void McTransform::run_ensure_biped(const std::string & source)
{
  using Clock = std::chrono::steady_clock;
  // 退出前必清 in_flight（各 return 路径共用）
  const auto finish = [this]() {transform_in_flight_.store(false);};

  if (!set_action_client_->service_is_ready() ||
    !get_action_client_->service_is_ready())
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "迎宾变形运控服务不可用: %s / %s（运控没起？本次保持当前姿态，"
      "播报不受影响）",
      config_.set_action_service.c_str(), config_.get_action_service.c_str());
    finish();
    return;
  }

  std::string why;
  auto [current, status] = get_current_action(why);
  if (!why.empty()) {
    RCLCPP_WARN(
      node_->get_logger(), "迎宾变形查形态失败: %s（保持当前姿态继续播报）: %s",
      why.c_str(), source.c_str());
    finish();
    return;
  }

  // 上一个动作可能还在切换中（action_desc 已变目标名但 status 未稳），
  // 此时发动作会被拒：先等稳定，稳定后用最新动作名查表
  if (status != kMcStatusRunning) {
    RCLCPP_INFO(
      node_->get_logger(),
      "迎宾变形: 当前动作 %s 状态 %d（0=空闲 200=切换中），尚未稳定，"
      "等它稳定（最多 %ds）: %s",
      current.c_str(), status, kMcSettleWaitSeconds, source.c_str());
    const auto settle_deadline =
      Clock::now() + std::chrono::seconds(kMcSettleWaitSeconds);
    while (status != kMcStatusRunning && rclcpp::ok() &&
      Clock::now() < settle_deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::tie(current, status) = get_current_action(why);
      if (!why.empty()) {
        status = -1;  // 瞬时查询失败：重试到上限，别让一次 DDS 抖动掐断流程
      }
    }
    if (status != kMcStatusRunning) {
      const std::string detail = why.empty() ? std::string() : "，" + why;
      RCLCPP_WARN(
        node_->get_logger(),
        "迎宾变形: 当前动作 %s 在 %ds 内未稳定（状态 %d%s），本次不变形，"
        "保持当前姿态继续播报: %s",
        current.c_str(), kMcSettleWaitSeconds, status,
        detail.c_str(), source.c_str());
      finish();
      return;
    }
  }

  // 已是人形：什么都不发（幂等）
  if (current == kMcBipedTarget && status == kMcStatusRunning) {
    RCLCPP_INFO(
      node_->get_logger(), "迎宾变形: 已是 %s 且运行中，无需切换（%s）",
      kMcBipedTarget, source.c_str());
    finish();
    return;
  }
  const bool down = mc_down_actions().count(current) > 0;
  if (down && !config_.stand_from_down) {
    RCLCPP_WARN(
      node_->get_logger(),
      "迎宾变形: 当前是 %s（趴着/被动，也可能正在充电），"
      "welcome_stand_from_down=false 不自动扶起，本次保持原姿态播报（%s）",
      current.c_str(), source.c_str());
    finish();
    return;
  }
  const auto path_it = mc_path_to_biped().find(current);
  if (path_it == mc_path_to_biped().end()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "迎宾变形: 当前动作 %s 没有到人形的已知转移路径，不猜（已知起点见"
      "源码 mc_path_to_biped）。请先用厂家 App 把它切到狗形/人形再迎宾（%s）",
      current.c_str(), source.c_str());
    finish();
    return;
  }
  const auto & path = path_it->second;

  RCLCPP_INFO(
    node_->get_logger(), "迎宾变形: %s -> %s，依次发出 %zu 步%s（%s，机器人会动）",
    current.c_str(), kMcBipedTarget, path.size(),
    down ? "（含起立）" : "", source.c_str());

  // 逐步走完路径，每步：发动作 -> 等到位。总时限覆盖全程；中途关停
  // 不再发下一条，不把机器人留在没人要的中间动作上
  const auto deadline = Clock::now() +
    std::chrono::seconds(config_.wait_timeout_seconds);
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto & step = path[i];
    if (!rclcpp::ok()) {
      RCLCPP_WARN(
        node_->get_logger(), "收到关停信号，剩余动作（%s 起）未发，机器人停在路上: %s",
        step.c_str(), source.c_str());
      finish();
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "迎宾变形: 发动作 %s", step.c_str());
    why = send_action(step);
    if (!why.empty()) {
      RCLCPP_WARN(
        node_->get_logger(), "迎宾变形失败（保持当前姿态继续播报）: %s: %s",
        why.c_str(), source.c_str());
      finish();
      return;
    }
    // 变形动作发完运控自动落到 WBC（等后继而非它本身）；dwell 只罚
    // 中间步骤——末步没有下一条动作，再等是白等
    const bool is_last_step = (i + 1 == path.size());
    const char * expected =
      (step == kMcToBipedAction) ? kMcBipedTarget : step.c_str();
    why = wait_for_action(expected, deadline,
        is_last_step ? 0.0 : kMcActionSettleSeconds);
    if (!why.empty()) {
      RCLCPP_WARN(
        node_->get_logger(), "迎宾变形失败（动作 %s 未到位）: %s: %s",
        step.c_str(), why.c_str(), source.c_str());
      finish();
      return;
    }
  }
  // 路径为空（BIPED_STAND_DEFAULT）时循环不执行，这里统一等落位
  why = wait_for_action(kMcBipedTarget, deadline);
  if (!why.empty()) {
    RCLCPP_WARN(
      node_->get_logger(), "迎宾变形失败（未落到 %s）: %s: %s",
      kMcBipedTarget, why.c_str(), source.c_str());
    finish();
    return;
  }
  RCLCPP_INFO(
    node_->get_logger(), "迎宾变形完成: 已切到 %s 且运行中（%s）",
    kMcBipedTarget, source.c_str());
  finish();
}

}  // namespace guide_pulse
