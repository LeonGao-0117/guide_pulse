# guide_pulse 设计文档 —— 展厅讲解机器人（迎宾 / 导航 / 汽车讲解）

状态：已评审（基线 v0.1.0）。本文是 guide_pulse 工程的唯一设计事实来源，
代码与测试与本文冲突时以本文为准并修代码。

## 1. 需求与指令映射

展厅讲解链路：**APP 经 Server 下发参数到 Robot，Robot 本地执行**。
全部落在 RobotCar 协议 v0.21.0 已有指令上，**零协议变更**：

| 展厅功能 | 协议指令（down/commands） | 回执 | Robot 本地行为 |
| --- | --- | --- | --- |
| 迎宾 | `scene`：`scene="welcome"` + 可选 `texts` | 无 | 播迎宾词一轮（`texts` 缺省用本地默认词） |
| 导航 + 汽车讲解 | `navigate`：`point_name` 必填 + 可选 `texts` | 无 | 本地导航到点位，到达后按序播讲解词一轮 |
| 独立播报 | `tts`：`texts`+`mode`+`interval_seconds` | `tts.result` | 列表单轮播完自行结束（2026-10-14 变更，原为沿用 tts_demo 列表循环播报） |
| 停止 | `tts.stop` | 无 | 停止一切播报并取消进行中的导航（本地策略） |

典型讲解流程：APP 下发 `scene(welcome)` 迎宾 → APP 下发
`navigate("3号展台", [讲解词...])` → Robot 走到展台 → 到达后播报讲解词。

不实现（明确排除）：

- `condition` 拥堵播报（tts_demo 的展厅外功能，不搬运）；
- `relocate` 重定位、`ota.start` 升级（无需求，静默忽略，由其他组件处理）；
- `navigate.result` 等回执（协议明确后置，v1 无导航回执面）。

## 2. 架构

```
┌──────────┐  HTTP(指令)  ┌──────────┐  MQTT(QoS1) robotCar/v1/robots/{robot_id}/down/commands
│   APP    │ ──────────▶ │  Server   │ ─────────────────────────────┐
└──────────┘             └──────────┘                              ▼
                       ▲           ┌─────────────────────────────────────────┐
                       │ presence/ │ guide_pulse（本工程，ROS2 ament, C++17） │
                       │ heartbeat │  ┌──────────────────────────────────┐  │
                       │ tts.result│  │ mqtt_guide_bridge（沿用 tts_demo │  │
                       └───────────│  │ 框架：enroll/MQTT/队列/回执）     │  │
                                    │  └──────┬───────────┬──────────┬───┘  │
                                    │   ┌─────┘           │          │      │
                                    │   ▼                 ▼          ▼      │
                                    │ WelcomeModule   TourModule  tts 播报 │
                                    │ (scene 迎宾)   (navigate)  (单轮)   │
                                    │   │               │          │      │
                                    │   ▼               ▼          ▼      │
                                    │ PlaylistSession（共享列表播报状态机）│
                                    │   │               │                 │
                                    │   ▼               ▼                 │
                                    │ TtsClient      NavExecutor(接口)    │
                                    └───│───────────────│─────────────────┘
                                        ▼               ▼
                          SDK /aimdk_5Fmsgs/srv/PlayTts   MockNavExecutor（默认）
                                                       （真机导航对接点）
```

沿用 tts_demo 的部分（拷贝改造，命名空间 guide_pulse）：

| 模块 | 来源 | 改动 |
| --- | --- | --- |
| `robot_enroll`（SN→enroll→MQTT 凭据） | tts_demo 原样 | 无（协议 v0.9.0/v0.10.0 同一注册面） |
| MQTT 桥接框架（连接线程/遗嘱/心跳/有界队列/幂等去重/回执缓存） | tts_demo | 删 condition 状态机，加 scene/navigate 分发 |
| `TtsClient`（SDK PlayTts 封装 + 最高权重抢占停止） | tts_demo 原样 | 无 |
| 播报列表循环（sequence/shuffle、语音时长估算、失败重试） | tts_demo cycle | 抽成 `PlaylistSession`，加「单轮模式」 |
| 协议层解析/校验/回执构造 | tts_demo tts_protocol | 删 condition，加 scene/navigate 载荷 |
| SDK 消息包 `src/aimdk_msgs` | sdk_t1-v0.9.4.4 | 无（与 tts_demo 内置副本 byte 级一致，已验证） |
| paho.mqtt.cpp vendored | tts_demo third_party | 无 |

## 3. 模块设计（迎宾 / 汽车讲解分开调用）

### 3.1 PlaylistSession（共享底座，纯逻辑，无 ROS 依赖）

一个音频会话状态机，worker 线程驱动（同 tts_demo cycle 模式）：

- 字段：`source`（母本）、`round`（当前轮顺序）、`index`、`round_no`、
  `mode`（sequence/shuffle）、`interval_seconds`、`loop_forever`。
- `loop_forever=true`：tts 指令，播到被替换/tts.stop（契约语义）。
- `loop_forever=false`：迎宾词与讲解词，**一轮播完即自行结束**
  （协议 scene/navigate 无 interval 字段，间隔用本地参数
  `tour_speak_interval_seconds`，默认 2 秒）。
- 单条文本失败按 `tts_retry_seconds` 重试（抢占空闲窗口，同 tts_demo）。

### 3.2 WelcomeModule（迎宾模块）

输入 `scene` 指令；只认 `scene="welcome"`（可扩展枚举的当前唯一值，
未知值记日志忽略）。行为：

1. 确定迎宾词：指令 `texts` 优先，缺省用参数 `welcome_default_texts`；
2. 开一轮 PlaylistSession（替换当前任何播报——音频单会话，见 3.4）；
3. 迎宾动作（SDK `SetMcPresetMotion` 招手）预留参数 `welcome_motion_id`
   （默认 0=禁用）。动作 ID 因机型而异（文档 examples：T1=1002 招手，
   另一机型=3001），真机校准后再启用。// ponytail: 默认只播报，动作待真机校准 ID 后开启

### 3.3 TourModule（汽车讲解模块）

输入 `navigate` 指令。状态机 `idle → navigating → (到达回调) → 讲解(PlaylistSession) → idle`：

1. `goto(point_name)`：经 `NavExecutor` 发起（新导航替换进行中的导航
   ——协议允许本地决定）；
2. 到达（`NavOutcome.ok`）→ 开一轮讲解 PlaylistSession（`texts` 缺省
   不播，记日志——讲解无本地默认词，点位即展台，讲解词必须由 APP 给）；
3. 到达失败 → 记 ERROR 日志（协议无回执面，失败不上报）；
4. `cancel()`：取消导航（`tts.stop`/新 navigate 时调用）。

### 3.4 互斥与替换策略（本地策略，协议不约束，全部归 Robot）

- **音频单会话**：任一新播报（tts / 迎宾词 / 到达讲解）替换当前
  会话并从头开始；同一时刻只有一路 PlaylistSession。
- **播报防抢占（SDK 仲裁侧）**：迎宾/讲解播报用 `SYSTEM_L7+0=700`
  （`tts_priority_level`）。SDK 唤醒响应/交互首句为 `INTERACTION_L6=600`：
  700 > 600，播报期间唤醒与其他 L6 请求被 SDK 仲裁丢弃
  （`is_success=false, priority_rejected`，真机已验证），迎宾词/讲解词
  完整播完不被抢话；`WARNING_L8`（避障预警）/`SAFETY_L10`（安全警报）
  仍可打断（安全优先，合理保留）。若需允许唤醒打断改回
  `tts_priority_level: 6`。注：vendored `TtsPriorityLevel.msg` 常量缺
  `SYSTEM_L7`，真机服务端实测受理 level=7（文档《接口说明.md》有此档）。
- **唤醒抑制（agent 强打断侧，v0.23.0）**：L7 防不住 agent 的
  `WakeUpManager` 强打断——唤醒词触发 `strong_interrupt` 直接对音频会话
  `request_stop`，**绕过 TTS 优先级仲裁**，SPEAKING 态必掐断（真机日志
  证实，与 PlayTts 参数无关）。官方出口 `SetAgentSuppression`
  （`SUPPRESSION_TYPE_WAKEUP=1`，srv 从运控板系统消息包引入 vendored
  aimdk_msgs）。**看门狗续期模式**：TTL 固定短（`suppress_ttl_ms` 默认
  8s），单轮播报进行期间 worker 按 `suppress_renew_interval_ms`
  （默认 5s，须 < TTL）周期续期。**尾部宽限（2026-10-14 重做）**：
  SDK 无播完事件（流式合成下 `estimated_duration` 恒 0 且无播完话题，
  《接口说明.md》PlayTts 节官方建议即字数预估），最后一条受理后按
  两段式估算持续续期 `tail_ms = suppress_tail_lead_ms + 字数估算秒×
  1000 × suppress_tail_scale`（`suppress_tail_max_ms>0` 封顶；默认
  估算×scale（scale 默认 0.8，lead 默认 0 不加；原为预估×2——120 字迎宾词
  宽限 52s，实际 ~30s 播完，播完后唤醒被白禁 20s+；真机复测收紧后仍白禁
  7~8s，2026-10-14 再砍两刀：scale 1.3→1.0 + 收尾刷由云端默认 3s 改 1ms
  立即解除），到点停续期并刷 `suppress_tail_flush_ms` 立即解除；真机校准：
  掐表 2~3 条典型文本拟合 lead 与 `tts_speech_ms_per_char`。
  `tts.stop` 掐断语音即停抑制。tts（type=tts）2026-10-14 起亦为单轮，
  播报期间同样抑制唤醒（原循环模式不抑制）；抑制失败仅告警不阻断播报。
  真机验证：抑制生效期间喊唤醒词，播报完整播完（无 strong_interrupt/
  request_stop，正常 tts_status.done）。
- **导航单任务**：新 `navigate` 替换进行中的导航。
- `tts.stop`：掐断当前语音（SDK 最高权重 1099 抢占）+ 终止播报会话 +
  **取消进行中的导航**（展厅语义：stop = 放弃本次讲解流程；一行可改）。
- `scene` 不打断导航：讲解途中来迎宾，导航继续，到达后讲解词按替换
  规则接管播报。
- **迎宾任务完成语义**：迎宾 TTS 单轮播完即视为任务完成（播报会话结束，
  协议无回执面）。二维码显示独立于播报会话：播完后**继续常驻**，
  直至 ①任意新业务（tts/scene/navigate/tts.stop）到达立即取消
  （新 scene 重入则重新显示并重置超时）或 ②`display_show_timeout_seconds`
  （默认 300s）超时自动恢复默认表情——二者取先到者。
- 幂等：按 `event_id` 去传输重复、按 `operation_id` 防重复副作用
  （同一 operation_id 的 scene/navigate 不重复触发；tts 重发重发同一回执）。

## 4. 导航执行边界（重要）

SDK `sdk_t1-v0.9.4.4` **没有点位导航服务**（已核对全部 srv/action：只有
PlayTts/PlayEmotion/SetMcPresetMotion/SetMcMotion 等；`pnc` 是运控内置
输入源，不对外暴露 goto-point 接口）。协议把「point_name → 物理位置映射
与导航执行」明确划给 Robot 本地。

因此定义窄接口 `NavExecutor`：

```cpp
struct NavOutcome { bool ok; std::string detail; };
class NavExecutor {
  virtual void goto_point(point_name, on_done) = 0;  // 异步；回调线程不限
  virtual void cancel() = 0;
};
```

- 内置实现两个，参数 `nav_executor` 选择：
  - `mock`（默认）：`MockNavExecutor`，`mock_nav_seconds` 秒后回调到达
    （默认 0=立即），`cancel()` 丢弃回调。联调与集成测试用。
  - `waypoint`（真机）：`WaypointNavExecutor`，对接 `/opt/liwei/t1_slam_nav_ws`
    航点巡航节点（节点名 `waypoint_follower`），已实现（含 6 个进程内
    Mock 服务单测）：
    - `/waypoint_follower/go_to_waypoints`：`waypoints=[point_name]`、
      `return_to_quadruped=false`（讲解机器人保持人形）、action 缺省 go；
      `accepted` 只表示「已开始执行」；
    - `/waypoint_follower/get_status`：1s 轮询，终态
      `SUCCEEDED`（到达）/ `FAILED` / `CANCELLED`（失败回调，带 reason）
      停住不回落，判定安全；`awaiting_pose` 时 RUNNING 但未动，继续等；
      轮询兜底超时 600s；
    - `/waypoint_follower/cancel`（Trigger）：取消当前任务就地停。
    - 协议 `point_name` 直接映射航点名（航点文件即本地「点位→坐标」
      映射表，恰好满足契约「映射归 Robot 本地」）。
    - 真机启用：`config/guide_pulse.yaml` 设 `nav_executor: "waypoint"`，
      前置条件：t1_slam_nav_ws 航点巡航（阶段 5）在该机上可用。
    - 服务消息包已拷贝内置 `src/waypoint_nav_msgs`（拷自 t1_slam_nav_ws）。
- 本工程不存地图；航点表归 t1_slam_nav_ws 的航点文件维护。

## 5. 关键参数（config/guide_pulse.yaml）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `topic_prefix` | `robotCar/v1/robots` | RobotCar Server 固定 TopicPrefix（写死在 Server 代码，非部署配置） |
| `broker_uri` / `enroll_origin` / `serial_number(_file)` | 同 tts_demo | enroll 凭 SN 换 MQTT 凭据 |
| `welcome_default_texts` | `["欢迎光临！我是展厅讲解机器人，很高兴为您服务。"]` | scene 未带 texts 时的迎宾词 |
| `tour_speak_interval_seconds` | `2` | 讲解词/迎宾词每两条间隔（scene/navigate 载荷无间隔字段） |
| `welcome_motion_id` | `0`（禁用） | 迎宾预设动作 ID，机型相关，真机校准后启用 |
| `nav_executor` | `mock` | 导航执行器：`mock`（联调/测试）或 `waypoint`（真机，对接 t1_slam_nav_ws 航点服务） |
| `mock_nav_seconds` | `0` | Mock 导航模拟耗时（0=立即到达） |
| `tts_*`（服务名/优先级/停止文本/重试/超时） | 同 tts_demo | SDK PlayTts 调用参数 |

## 6. 测试计划

| 层 | 用例文件 | 覆盖 |
| --- | --- | --- |
| 协议层（纯 C++） | `test_guide_protocol.cpp` | tts/tts.stop/scene/navigate 载荷正反例、信封校验、过期、幂等键提取、回执构造 |
| enroll（纯 C++） | `test_robot_enroll.cpp` | 沿用 tts_demo（响应解析/错误码/SN 校验） |
| TtsClient（进程内 Mock 服务） | `test_tts_client.cpp` | 沿用 tts_demo（字段映射/回执判定/超时） |
| 播报状态机（纯 C++） | `test_playlist.cpp` | sequence/shuffle、单轮结束、无界循环、失败重试推进 |
| 迎宾模块（纯 C++） | `test_welcome.cpp` | texts 优先/默认词回退、未知 scene 值忽略 |
| 讲解模块（纯 C++，FakeNav） | `test_tour.cpp` | start/到达开播/失败不开播/替换/取消 |
| 端到端集成 | `tools/run_integration_test.sh` | enroll Mock + mosquitto + 桥接 + Mock TTS + 后端模拟器发 scene/navigate/tts/tts.stop，断言日志与回执 |

## 7. 明确后置

- 导航执行结果回执、导航进度、navigate.stop（协议后置，需先扩契约）；
- 迎宾动作真机参数校准与输入源注册（`SetMcInputSource`）；
- 多点位连续讲解（App 编排多条 navigate 即可，Robot 不做排队）；
- relocate / ota 指令支持。
