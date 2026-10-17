# guide_pulse 设计文档 —— 展厅讲解机器人（迎宾 / 导航 / 汽车讲解）

状态：已评审（基线 v0.1.0）。本文是 guide_pulse 工程的唯一设计事实来源，
代码与测试与本文冲突时以本文为准并修代码。

## 1. 需求与指令映射

展厅讲解链路：**APP 经 Server 下发参数到 Robot，Robot 本地执行**。
指令面落在 RobotCar 协议 v0.24.0（P-038 起 navigate/scene 有一次性
结果回执）：

| 展厅功能 | 协议指令（down/commands） | 回执 | Robot 本地行为 |
| --- | --- | --- | --- |
| 迎宾 | `scene`：`scene="welcome"` + 可选 `texts` | `scene.result`（v0.24.0） | 播迎宾词一轮（`texts` 缺省用本地默认词）；流程主体完成（变形结束+二维码上屏+会话已立）回 `success`，忙碌回 `failed/busy` |
| 导航 + 汽车讲解 | `navigate`：`point_name` 必填 + 可选 `texts` | `navigate.result`（v0.24.0） | 本地导航到点位，到达回 `success`（讲解词播报与否不影响结果）后按序播讲解词一轮；失败回 `failed/navigation_failed`；被 `navigate.stop`/`tts.stop` 中止回 `failed/aborted`；忙碌回 `failed/busy` |
| 地图重定位 | `relocate`：`point_name` 必填 + 可选 `texts`（v0.21.0，2026-10-17 接入） | `relocate.result` | **不可打断的互斥业务**：`point_name` 经点位映射表解析 -> `set_pose` 重设定位初值（机器人不动）-> 等待导航模块验收（worker 250ms 轮询 `status_json` 的 `set_pose.verified`，真机受理后 ~1.5s 出结果）-> 验收通过回 `success` 并播报（`texts` 缺省播「{point_name}点重定位成功」），验收失败/`relocate_timeout_seconds`（默认 30s）超时回 `failed/localization_failed`；受理被拒当场回 `failed/point_not_found`（航点名不在航点表，available 结构化判定）或 `failed/localization_failed`（其余归并）。等待期占业务互斥：新业务回 `failed/busy` 并播「我正在重定位，请稍后」（迎宾/播报/讲解中正在出声，被拒时不播提示）；`tts.stop` 不中止等待（初值已发，终态由导航模块给出） |
| 独立播报 | `tts`：`texts`+`mode`+`interval_seconds` | `tts.result` | 列表单轮播完自行结束（2026-10-14 变更，原为沿用 tts_demo 列表循环播报） |
| 停止 | `tts.stop` | 无 | 停止一切播报并取消进行中的导航（本地策略，被取消的导航回 `failed/aborted`） |
| 停止导航 | `navigate.stop` | 无 | 只取消在途导航（被取消的导航回 `failed/aborted`；无导航 no-op） |

典型讲解流程：APP 下发 `scene(welcome)` 迎宾 → APP 下发
`navigate("3号展台", [讲解词...])` → Robot 走到展台 → 到达后播报讲解词。

不实现（明确排除）：

- `condition` 拥堵播报（tts_demo 的展厅外功能，不搬运）；
- `ota.start` 升级（无需求，静默忽略，由其他组件处理）。

### 回执要点（v0.24.0 P-038）

- **恰好一条**：每条 navigate/relocate/scene 指令恰好一条最终回执；传输重发
  （同 operation_id 再次到达）重发**同一信封**（同 event_id，receipt 缓存）。
- **回执时机**：受理即终态的（忙碌/过期/载荷非法）当场回；执行类的在
  终态确定时回（到达/失败/中止/迎宾流程完成；relocate 的终态是导航模块
  的**验收结果**——`set_pose.verified`，不是受理应答：受理成功后等待
  验收（真机 ~1.5s），验收通过/失败/超时（`relocate_timeout_seconds`
  默认 30s）才回 `relocate.result` 并释放业务占用）。回执信封发送时
  生成，执行耗时不受指令信封 TTL 约束。
- **reason 归并**：导航失败统一 `navigation_failed`（NavOutcome 无结构化
  分类，不猜 point_not_found/localization_lost）。重定位例外：
  `point_not_found` 由服务端回带的 available 清单结构化判定（点名不在
  合法航点清单 = point_not_found），其余被拒（导航在途之外的状态冲突/
  非站立人形/定位未起）与服务不可用归并 `localization_failed`。
- **本地注入不发回执**：语料触发（`event.local`）的 operation_id 为本地
  生成，Server 不认识（对应不到已下发指令的结果只记日志）。
- **掉线/重启无回执**：进程重启即丢失在途状态，契约允许无回执
  （App 不得把「没收到回执」当失败）。

## 2. 架构

```
┌──────────┐  HTTP(指令)  ┌──────────┐  MQTT(QoS1) robotCar/v1/robots/{robot_id}/down/commands
│   APP    │ ──────────▶ │  Server   │ ─────────────────────────────┐
└──────────┘             └──────────┘                              ▼
                       ▲           ┌─────────────────────────────────────────┐
                       │ presence/ │ guide_pulse（本工程，ROS2 ament, C++17） │
                       │ heartbeat │  ┌──────────────────────────────────┐   │
                       │ tts.result│  │ mqtt_guide_bridge                │   │
                       └───────────│  │ (enroll/MQTT/队列/回执)           │  │
                                    │  └──────┬───────────┬──────────┬───┘  │
                                    │   ┌─────┘           │          │      │
                                    │   ▼                 ▼          ▼      │
                                    │ WelcomeModule   TourModule  tts 播报  │
                                    │ (scene 迎宾)   (navigate)  (单轮)      │
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

1. **迎宾变形**（2026-10-15）：**直调厂家运控服务**（本体固件提供的
   `/aimdk_5Fmsgs/srv/GetMcAction` + `SetMcAction`）确保人形形态——
   做法、转移表与回执判定口径对齐 t1_slam_nav_ws 的 `stand_if_down.py` /
   `mc_velocity_bridge.py`（那边真机实测校准过），但**不依赖
   t1_slam_nav_ws 是否在跑**；发动作不需要信源注册（`source` 非空即可，
   与导航栈速度桥并存不冲突，`stand_if_down.py` 即此先例）。
   行为：已是人形（`BIPED_LOCOMOTION_WBC` + RUNNING）则零动作直接确认；
   狗站立/四足行走态按 SDK 转移表逐步变形（变形动作发完等运控自动落位，
   SDK 明确勿手动再发 WBC）；趴姿由 `welcome_stand_from_down`（默认
   true）决定是否一路扶起再变形——迎宾由后端在有人到场时触发，与远程
   下发航点的场景不同；表外动作（空翻/跳舞等）拒绝不猜。
   **时序（2026-10-15 需求：先变形、后人声；2026-10-16 需求：二维码
   上屏在 TTS 开播之前；2026-10-17 需求：先播引导语再变形）**：迎宾
   到达先播引导语 `welcome_lead_text`（默认「元气满满，小T来迎接我的
   顾客了」，空=禁用）——复用播报会话机制（麦克风静音/唤醒抑制/trace
   前缀），播完以 TtsStatus 终态回流为准、字数估算兜底（`wait_speech_
   done`，BEST_EFFORT 丢事件不卡流程），受理失败直接放行进变形；二
   维码**准备**（下载/转码/推送 ~3.5s，ScreenDisplay 两阶段的阶段 1）
   与引导语播报、变形并行重叠；引导语播完后变形异步发起，变形结束后
   先执行二维码**上屏**（阶段 2：PlayEmotion 常驻循环，毫秒级受理）
   再开 TTS 会话——上屏必须在变形之后：变形动画/变形结束恢复的表情
   会顶掉变形前上屏的内容（PlayEmotion 单通道，新请求即打断），且迎宾
   词提及「扫描我屏幕上显示的二维码」，TTS 开播时屏幕必须已是二维码。
   总时长 = max(变形, 准备) + 上屏受理。变形失败/超时仍照常上屏与
   播报（姿态链路失败不阻断语音）。等待期间新指令在下行队列排队，
   迎宾互斥窗口自开播时起算。防抖三件套（均为真机实测教训）：
   规划前等上一个动作稳定（2003 拒绝，state=-1008）、中间动作 dwell
   3s 连续保持（1008 限频，state=-10001）、回执 SUCCESS=1/已目标态
   reason=16 非失败。消息类型注意：SDK v0.9.4.4 的 `McActionInfo.msg`
   缺 `source` 字段，与运控固件（v0.9.4.8+）DDS 类型不匹配，内置
   `src/aimdk_msgs` 已按固件版补齐该字段；
1b. **播报伴随动作**（2026-10-15）：每条语音（迎宾/讲解/tts）PlayTts
   受理成功时从 `motion_preset_ids` 轮换池取一个 `SetMcPresetMotion`
   预设动作触发（默认 1001 右手挥手打招呼），动作与语音并行、
   `interrupt=true` 打断上一个伴随动作；worker 同步短超时调用（正常
   毫秒级），失败仅 WARN 不重试不阻断播报。独立于迎宾变形开关；
2. 确定迎宾词：指令 `texts` 优先，缺省用参数 `welcome_default_texts`；
3. 迎宾二维码（v0.22.0 P-036，`qr_sha256` 哈希快照驱动）：
   `QrCodeCache` 与本地单文件缓存比对——命中直接取缓存图片（免下载），
   未命中经 Server 文件面（HTTP Basic = 自身 MQTT 凭据）下载一次，
   sha256 + PNG/JPEG 魔数 + ≤512KiB 校验后原子替换缓存（文件名内嵌
   哈希 `qr_{sha256}.{png|jpg}`，旧哈希文件淘汰）；`ScreenDisplay` 将
   图片 ffmpeg 转 848x480 MP4 推运控板常驻显示。`qr_sha256` 省略/
   下载失败/校验失败：本次不显示二维码，不影响迎宾播报，不上报；
4. 开一轮 PlaylistSession（替换当前任何播报——音频单会话，见 3.4）；
5. 迎宾动作（SDK `SetMcPresetMotion` 招手）预留参数 `welcome_motion_id`
   （默认 0=禁用）。动作 ID 因机型而异（文档 examples：T1=1002 招手，
   另一机型=3001），真机校准后再启用。// ponytail: 默认只播报，动作待真机校准 ID 后开启

### 3.3 TourModule（汽车讲解模块）

输入 `navigate` 指令。状态机 `idle → navigating → (到达回调) → 讲解(PlaylistSession) → idle`：

1. `goto(point_name)`：经 `NavExecutor` 发起（新导航替换进行中的导航
   ——协议允许本地决定；本工程单业务模型下在途导航不会被新 navigate
   替换，忙碌互斥先行拒绝）；
2. 到达（`NavOutcome.ok`）→ 回 `navigate.result(success)`（v0.24.0）→
   开一轮讲解 PlaylistSession（`texts` 缺省不播，记日志——讲解无本地
   默认词，点位即展台，讲解词必须由 APP 给）；
3. 到达失败 → 回 `navigate.result(failed, navigation_failed)` 并记
   ERROR 日志（reason 归并，见 §1 回执要点）；
4. `cancel()`：取消导航（`tts.stop`/新 navigate 时调用），取消即对原
   指令回 `failed/aborted`。

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
  1000 × suppress_tail_scale`（`suppress_tail_max_ms>0` 封顶；历史调参：
  原为预估×2——120 字迎宾词宽限 52s，实际 ~30s 播完，播完后唤醒被白禁
  20s+；2026-10-14 砍 scale 1.3→1.0 + 收尾刷改 1ms 立即解除；2026-10-16
  定稿 lead=2500（受理≠出声，云端合成 1~3s）+ scale=1.0 不缩水——0.8 会
  使抑制早于语音结束 1~3s 解除，尾部裸奔被唤醒强打断播报，宁可多禁 3~5s），
  到点停续期并刷 `suppress_tail_flush_ms` 立即解除；真机校准：
  掐表 2~3 条典型文本拟合 lead 与 `tts_speech_ms_per_char`。
  `tts.stop` 掐断语音即停抑制。tts（type=tts）2026-10-14 起亦为单轮，
  播报期间同样抑制唤醒（原循环模式不抑制）；抑制失败仅告警不阻断播报。
  真机验证：抑制生效期间喊唤醒词，播报完整播完（无 strong_interrupt/
  request_stop，正常 tts_status.done）。
- **麦克风硬抑制（2026-10-16）**：唤醒抑制只拦 wuw 唤醒入口；迎宾前
  ~30s 内的云端对话会话（cloud_event）仍开着时，用户语音被当对话轮
  直送云端并触发 `AudioSDK request_stop` 掐断播报——绕过 TTSManager
  优先级仲裁与 WAKEUP 抑制（证据链见 AIMDK 反馈稿
  `aimdk_feedback_2026-10-16_dialog-turn-interrupt.md`）。兜底：
  `mic_mute_enabled`（默认开）单轮播报会话开始时 SSH 执行运控板
  `pactl set-source-mute @DEFAULT_SOURCE@ true`（PulseAudio 默认音源
  静音，不依赖厂商脚本，目标/密码复用 `display_remote_*`），尾部宽限
  到期/会话替换/tts.stop/节点关停时 `false` 恢复——无音频进入即无
  打断之源。恢复失败自动重试一次并 ERROR 提示人工
  `pactl set-source-mute @DEFAULT_SOURCE@ false`。
  **唤醒抑制随之降级为备用（同日）**：静音成功时唤醒抑制整套跳过（不发
  SetAgentSuppression、不刷续期日志——无音频即无抑制对象）；仅 SSH 静音
  失败的会话自动武装原抑制链路（arm+续期+收尾）兜底。
  **终态提前解除（同日）**：尾部宽限的字数估算偏保守（缓存命中实际
  ~19s < 估算 ~24s，另加 lead 与 5s 续期粒度），AgentMonitor 收到本会话
  trace 的 TtsStatus 终态（END/STOP/ERROR，`/interaction/tts_status`
  回流的 item_id = 会话来源前缀+后缀）即经 `notify_tts_terminal` 唤醒
  worker 立即恢复麦克风并解除抑制——实际播完即恢复，估算时序仅兜底
  （BEST_EFFORT 丢事件时按原时序到期）。同一动作**即刻释放播报类
  业务占用**（迎宾 120s 总超时、tts/讲解字数估算窗口均降级为兜底
  超时，连续迎宾不再被固定窗口卡死）。
- **导航单任务**：新 `navigate` 替换进行中的导航。
- **迎宾流程互斥（2026-10-15；2026-10-16 改终态驱动）**：迎宾业务占用
  在播报终态回流时**即刻释放**（事件源与麦克风提前解除同一处：
  AgentMonitor 收到本会话 trace 的 TtsStatus 终态，见 §3.4 唤醒抑制
  「终态提前解除」）。`welcome_flow_timeout_seconds`（默认 120s，须
  覆盖最坏流程 变形 60s + 默认词 ~26s）仅为终态事件丢失（BEST_EFFORT）
  时的**兜底超时**，不是固定互斥窗口（2026-10-16 真机教训：迎宾 ~30s
  播完、终态已回流、麦克风已恢复，固定 120s 窗口仍占用，连续两次迎宾
  第二次被拒 `failed/busy`）。占用期间新的 `scene`/`navigate` 直接
  拒绝，回各自 `*.result(failed, busy)`（v0.24.0，App 可感知稍后重试）。
  `tts`/`tts.stop` 不受限：tts 替换会话或 tts.stop 终止会话均使占用
  立即失效（迎宾音频被打断 = 流程终止）。tts/讲解业务的字数估算窗口
  同理：终态回流即刻释放，窗口仅兜底；导览在途期不涉播报窗口
  （以导航状态为准）。
- `tts.stop`：掐断当前语音（SDK 最高权重 1099 抢占）+ 终止播报会话 +
  **取消进行中的导航**（展厅语义：stop = 放弃本次讲解流程；一行可改）。
- `scene` 不打断导航：讲解途中来迎宾，导航继续，到达后讲解词按替换
  规则接管播报。
- **迎宾任务完成语义**：迎宾 TTS 单轮播完即视为任务完成（播报会话结束；
  协议回执在流程主体完成时已发，见 §1 回执要点）。二维码显示独立于
  播报会话：播完后**继续常驻**，
  直至 ①任意新业务（tts/scene/navigate/tts.stop）到达立即取消
  （新 scene 重入则重新显示并重置超时）或 ②`display_show_timeout_seconds`
  （默认 180s）超时自动恢复默认表情——二者取先到者。
- **语料触发二维码显示（2026-10-16；2026-10-17 改互斥业务）**：
  AgentMonitor 监测 agent TTS 应答话术（trace_id 回查运控板日志原文，
  `agent_monitor.qr_phrases` 语料表，如「收到，这就给您显示二维码，您
  可以扫描二维码留下您的个人信息哦。」）命中 -> 本地注入 `show_qr`
  指令（仅本地类型，与 MQTT 下行同 worker 串行链路）->
  `handle_show_qr` 显示默认二维码图片（`config/QRCode.png`），常驻
  `display_show_timeout_seconds`（默认 180s=3 分钟）。**2026-10-17
  现场需求起为互斥业务**：显示二维码是一个业务，上屏成功即占
  kShowQr 业务 `qr_business_timeout_seconds`（默认 20s）互斥窗口，
  期间其他业务（tts/scene/navigate/relocate）被拒（App 下发 tts 回
  `rejected(business_busy)`——二维码刚上屏不被新业务立即让位）；窗口
  到期自然释放（无播报会话、无终态回流面），二维码常驻不受影响（仍由
  屏幕超时或让位规则收尾）。窗口内复读被拒（business_busy），窗口外
  复读重新上屏并重置计时；冷却（`agent_monitor.cooldown_seconds`）
  按动作独立计时。上屏失败（默认图缺失/转码/推送失败）仅告警不占
  业务（与 tts 受理失败不占用同语义）。
- **语料触发取消二维码显示（2026-10-17）**：AgentMonitor 监测 agent
  TTS 应答话术（`agent_monitor.qr_cancel_phrases` 语料表，如「好的，
  这就取消二维码显示」）命中 -> 本地注入 `cancel_qr` 指令 ->
  `handle_cancel_qr` 撤屏恢复默认表情（未显示 no-op 幂等）。取消
  二维码显示是第二个互斥业务：受理即占 kCancelQr 业务
  `qr_business_timeout_seconds`（默认 20s）与其他业务互斥；例外——
  **可抢占 kShowQr 显示业务窗口**（取消类动作与 tts.stop/navigate.stop
  永远放行同语义：显示保护期内「收到后取消二维码显示」必须生效），
  抢占后业务切换为 kCancelQr 重计 20s。撤屏失败仅告警，业务照占。
- **语料触发重定位（2026-10-16；2026-10-17 改等待验收业务）**：
  AgentMonitor 监测 agent TTS 应答话术（`agent_monitor.relocate_phrases`，
  每条格式「航点名|语料」，如「A|好的，这就在展厅A点开始重定位」，展厅
  A~E 五点各一条；航点名即 t1_slam_nav_ws 航点文件点位名）命中 ->
  本地注入 `relocate` 指令（event.local，与 MQTT 下行同 worker 串行链路）
  -> `NavExecutor::set_pose`：真机经 waypoint_follower 的
  `go_to_waypoints(action=set_pose)` 把定位初值重设到该航点——**机器人
  不动**，前提是人工已把机器人摆到该点位并对准朝向（朝向比位置更要紧，
  见 GoToWaypoints.md）。受理**不是终态**：占用 kRelocate 业务，worker
  250ms 轮询验收（`status_json` 的 `set_pose.verified`，真机 ~1.5s 出
  结果；`relocate_timeout_seconds` 默认 30s 超时兜底），验收通过才播报
  （缺省「{point_name}点重定位成功」）。本地注入不发协议回执（Server
  不认识其 operation_id）。与 MQTT 下行 relocate 同受业务互斥约束
  （见下条）。冷却按「action+航点名」独立计时（五点互不挤占）；mock
  执行器 `mock_relocate_hold_polls` 模拟验收延迟（联调/集成测试用）。
- **重定位互斥与忙碌提示（2026-10-17）**：relocate 是不可打断业务——
  等待验收期间（受理 -> verified/超时）新业务指令（tts/navigate/scene/
  relocate）一律拒绝：tts 回 `rejected(business_busy)`，其余回各自
  `*.result(failed, busy)`；`tts.stop`/`navigate.stop` 不中止等待（初值
  已发，终态由导航模块给出；等待期无会话无导航，两 stop 对它本就 no-op，
  仅业务占用不释放）。拒绝时播忙碌提示「我正在{业务}，请稍后」
  （`busy_hint_relocate_text`/`busy_hint_navigate_text`）：**仅非语音业务
  进行中播**（重定位等待期/导航在途——机器人没在说话，口头告知忙碌）；
  迎宾/播报/讲解中正在出声，插提示反而打断，不播。提示
  `is_interrupted=false` 同档排队，失败仅记日志（拒绝结果已回执，提示
  属本地增强；本地注入指令被拒同样播——现场正在跟机器人说话）。
- 幂等：按 `event_id` 去传输重复、按 `operation_id` 防重复副作用
  （同一 operation_id 的 scene/navigate 不重复触发；tts 重发重发同一回执）。

## 4. 导航执行边界（重要）

SDK `sdk_t1-v0.9.4.4` **没有点位导航服务**（已核对全部 srv/action：只有
PlayTts/PlayEmotion/SetMcPresetMotion/SetMcMotion 等；`pnc` 是运控内置
输入源，不对外暴露 goto-point 接口）。协议把「point_name → 物理位置映射
与导航执行」明确划给 Robot 本地。

因此定义窄接口 `NavExecutor`：

```cpp
struct NavOutcome { bool ok; std::string detail; bool point_not_found; };
class NavExecutor {
  virtual void goto_point(point_name, on_done) = 0;  // 异步；回调线程不限
  virtual void cancel() = 0;
  virtual NavOutcome set_pose(point_name) = 0;       // 重定位受理（同步毫秒级）
  virtual std::optional<bool> poll_set_pose_verified(point_name);  // 验收轮询
};
```

- 内置实现两个，参数 `nav_executor` 选择：
  - `mock`（默认）：`MockNavExecutor`，`mock_nav_seconds` 秒后回调到达
    （默认 0=立即），`cancel()` 丢弃回调。联调与集成测试用。
  - `waypoint`（真机）：`WaypointNavExecutor`，对接 `/opt/liwei/t1_slam_nav_ws`
    航点巡航节点（节点名 `waypoint_follower`），已实现（含进程内 Mock 服务
    单测：导航受理/终态/取消 6 个 + 重定位 set_pose 受理/拒因分类 3 个 +
    验收轮询 status_json 解析 1 个）：
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
| `agent_monitor.relocate_phrases` | `[]` | 语料触发重定位：每条「航点名\|语料」，命中 -> 该航点 set_pose 重定位（空=关闭） |
| `relocate_timeout_seconds` | `30` | 重定位等待导航模块验收的超时（真机 ~1.5s 出结果，此值兜底 status 丢失/服务卡死；超时按 failed/localization_failed 收尾） |
| `busy_hint_relocate_text` | `"我正在重定位，请稍后"` | 业务互斥拒绝提示（重定位等待期播；空=禁用） |
| `busy_hint_navigate_text` | `"我正在导航，请稍后"` | 业务互斥拒绝提示（导航在途播；空=禁用；迎宾/播报/讲解中恒不播——正在出声） |
| `mock_relocate_hold_polls` | `0` | Mock 重定位验收等待窗口（poll 次数，~250ms/次；联调/集成测试用） |
| `tour_speak_interval_seconds` | `2` | 讲解词/迎宾词每两条间隔（scene/navigate 载荷无间隔字段） |
| `welcome_motion_id` | `0`（禁用） | 迎宾预设动作 ID，机型相关，真机校准后启用 |
| `welcome_flow_timeout_seconds` | `120` | 迎宾占用兜底超时：播报终态回流即释放占用，此值仅在终态事件丢失时兜底（须覆盖变形 60s + 默认词 ~26s） |
| `nav_executor` | `mock` | 导航执行器：`mock`（联调/测试）或 `waypoint`（真机，对接 t1_slam_nav_ws 航点服务） |
| `mock_nav_seconds` | `0` | Mock 导航模拟耗时（0=立即到达） |
| `display_*`（服务名/运控板/超时/恢复表情） | 见 yaml | 迎宾二维码显示：PlayEmotion 调用与 MP4 推送运控板 |
| `qr_cache_dir` | `/tmp/guide_pulse/qr_cache` | 迎宾二维码本地缓存目录（文件名内嵌哈希；被清则下次迎宾重新拉取） |
| `qr_download_timeout_seconds` | `5` | 二维码单次下载（连接+传输）超时 |
| `tts_*`（服务名/优先级/停止文本/重试/超时） | 同 tts_demo | SDK PlayTts 调用参数 |

## 6. 测试计划

| 层 | 用例文件 | 覆盖 |
| --- | --- | --- |
| 协议层（纯 C++） | `test_guide_protocol.cpp` | tts/tts.stop/scene/navigate/relocate 载荷正反例、scene.qr_sha256 格式正反例、信封校验、过期、幂等键提取、回执构造（含 relocate.result 枚举） |
| enroll（纯 C++） | `test_robot_enroll.cpp` | 沿用 tts_demo（响应解析/错误码/SN 校验） |
| 二维码缓存（纯 C++） | `test_qr_code_cache.cpp` | sha256 已知向量、PNG/JPEG 魔数、qr_sha256 格式、缓存写入/哈希拒绝/旧文件淘汰 |
| TtsClient（进程内 Mock 服务） | `test_tts_client.cpp` | 沿用 tts_demo（字段映射/回执判定/超时） |
| 播报状态机（纯 C++） | `test_playlist.cpp` | sequence/shuffle、单轮结束、无界循环、失败重试推进 |
| 迎宾模块（纯 C++） | `test_welcome.cpp` | texts 优先/默认词回退、未知 scene 值忽略 |
| 讲解模块（纯 C++，FakeNav） | `test_tour.cpp` | start/到达开播/失败不开播/替换/取消 |
| 端到端集成 | `tools/run_integration_test.sh` | enroll Mock + mosquitto + 桥接 + Mock TTS + Mock 运控（SetMcAction/GetMcAction 状态机，迎宾变形走完整转移序列断言）+ 后端模拟器发 scene/navigate/tts/tts.stop，断言日志与回执 |

## 7. 明确后置

- 导航/场景过程事件（进度、位置上报——契约明确仅最终结果，过程后置）；
  导航失败 reason 细分（point_not_found/localization_lost 归并
  navigation_failed，见 §1 回执要点；relocate 的 point_not_found 已有
  结构化判定，不属归并）；
- 迎宾动作真机参数校准与输入源注册（`SetMcInputSource`）；
- 多点位连续讲解（App 编排多条 navigate 即可，Robot 不做排队）；
- `ota.start` 指令支持（含 ota.result——由未来接手组件负责）。
