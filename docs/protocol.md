# guide_pulse MQTT 协议说明（RobotCar 协议 v0.26.0 子集）

guide_pulse 消费 RobotCar Protocol 正式契约的指令子集，本文只记录
本工程视角的要点与本地策略；完整契约见
`RobotCar/Protocol/contracts/server-robot/mqtt/v1/command-protocol.md`。

## 1. 主题与连接

| 主题（前缀 `robotCar/v1/robots`，Server 固定值） | 方向 | QoS | Retain |
| --- | --- | --- | --- |
| `.../{robot_id}/down/commands` | Server→Robot | 1 | false |
| `.../{robot_id}/up/commands` | Robot→Server（回执 + 语音触发上报） | 1 | false |
| `.../{robot_id}/heartbeat` | Robot→Server | 0 | false |
| `.../{robot_id}/presence` | Robot/遗嘱 | 1 | true |

- Client ID：`guide-pulse-{robot_id}`。
- 凭据：启动凭预登记 SN 调 `POST {enroll_origin}/v1/robots/enroll` 换取
  `robot_id` + MQTT 账号密码（协议 v0.9.0/v0.10.0）；CONNECT 被拒
  （0x86/0x87）自动重新 enroll；密码只在内存。
- presence 遗嘱/心跳/会话语义与 tts_demo 相同（session_id 每次连接前
  重生成 uuidv7；`capabilities=["text"]`）。

## 2. 本工程消费的下行指令

信封 8 字段（`spec_version/event_id/robot_id/direction/type/occurred_at/
expires_at/payload`），未知字段/越窗（`expires_at > occurred_at+2min`）/
未知 type 一律拒绝；过期指令按各自规则处理；按 `event_id` 去传输重复、
按 `operation_id` 防重复副作用。

### 2.1 `tts`（有回执，沿用 tts_demo）

payload：`operation_id` + `texts`(1..20 条，每条 1..2048 字节) + `mode`
(sequence/shuffle) + `interval_seconds`(1..86400)。**2026-10-14 需求变更：
列表单轮播完自行结束，不再无限循环**（原契约语义为循环直到替换/停止，
`tts.stop` 仍可提前掐断）。必须回 `tts.result`（accepted / rejected+reason：
`command_expired`/`invalid_request`/`playback_unavailable`/`business_busy`）；
同一 operation_id 重发同一回执（同 event_id）。

**2026-10-16 单业务占用**：迎宾/导览/播报任一业务进行中，新 `tts`/
`scene`/`navigate` 指令一律拒绝——`tts` 回 `rejected(business_busy)`，
`scene`/`navigate` 回各自 `*.result` 的 `failed(busy)`（v0.24.0 P-038）。
`tts.stop` 永远放行（停止一切并立即释放占用）；`navigate.stop` 永远
放行（取消类指令，只取消在途导航）。业务结束口径：播报类以 TtsStatus
终态回流为准即刻释放（迎宾固定 120s 总超时与 tts/讲解字数估算窗口
仅为终态丢失时的兜底超时，2026-10-16）；导览另含导航在途期
（到达/失败事件即终态）。**2026-10-17 补充**：语料触发的二维码
显示/取消（本地注入 `show_qr`/`cancel_qr`，无协议面）亦为互斥业务，
各占 `qr_business_timeout_seconds`（默认 20s）窗口——窗口内 App 下发
`tts` 同样回 `rejected(business_busy)`（保护二维码刚上屏不被立即
让位）；窗口到期自然释放。

### 2.2 `tts.stop`（无回执）

payload 仅 `operation_id`。本工程行为：最高权重抢占掐断语音 + 终止播报
会话 + **取消进行中的导航**（本地策略：stop = 放弃本次讲解流程）。

### 2.3 `scene`（无回执）——迎宾

```json
{"payload": {"operation_id": "<uuidv7>", "scene": "welcome",
             "texts": ["欢迎光临"],
             "qr_sha256": "9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0b822cd15d6c15b0f00a08"}}
```

- `scene` 必填，当前唯一合法值 `welcome`（可扩展枚举）；未知值记日志忽略。
- `texts` 可选（1..20 条，每条 1..2048）；缺省用 `welcome_default_texts`。
- `qr_sha256` 可选（v0.22.0 P-036，64 位小写十六进制）：Server 下发时该
  机器人当前迎宾二维码图片的**内容哈希快照**（App 经 HTTP 按需上传/替换/
  清除，不随迎宾触发重传）。机器人侧与本地单文件缓存比对（QrCodeCache，
  缓存文件名内嵌哈希）：**一致直接显示缓存图片、不重新下载**；不一致经
  `GET {enroll_origin}/v1/robots/{robot_id}/qr-code/content`（HTTP Basic =
  自身 MQTT 凭据，username=robot_id）拉取一次，校验字节 sha256 与快照
  一致 + PNG/JPEG 魔数 + ≤512KiB 后原子替换缓存再显示（旧哈希文件淘汰）。
  校验失败/下载失败（含 401/404）本次不显示二维码——**不阻塞迎宾动作、
  不产生上行事件、单次迎宾内不重试**。**字段省略（App 未上传或已清除）
  本次不显示二维码**。命中缓存的图片经 ffmpeg 转 848x480 H.264 MP4 推送
  运控板后 `PlayEmotion(loop_count=0)` 常驻显示，5 分钟超时
  （`display_show_timeout_seconds`）或任意新业务（tts/scene/navigate/
  tts.stop）到达即恢复默认表情。
- 行为：开一轮迎宾播报（单轮结束，不循环）+ 二维码常驻显示；不打断导航。
  屏幕显示失败不影响语音播报（仅告警日志）。
- **迎宾互斥（2026-10-15）**：迎宾词播报未完（字数估算窗口）期间，新的
  `scene`/`navigate` 直接拒绝——协议无回执面，本地丢弃记日志，App 需重发；
  `tts`/`tts.stop` 不受限（会终止迎宾流程并解除互斥）。
- 过期直接丢弃（无回执通道，协议规定）。

### 2.4 `navigate`（无回执）——导航 + 汽车讲解

```json
{"payload": {"operation_id": "<uuidv7>", "point_name": "3号展台",
             "texts": ["这台车搭载...", "续航达到..."]}}
```

- `point_name` 必填，1..64 个 Unicode 标量（按码点计）；名称→位置映射
  归 Robot 本地（本工程：归 NavExecutor 实现）。
- `texts` 可选；缺省到达后不播报（记日志）。
- 行为：`NavExecutor` 导航（新 navigate 替换旧导航）→ 到达后按序播一轮
  讲解词（间隔 `tour_speak_interval_seconds`）。
- 过期直接丢弃；同一 `operation_id` 重发不重复触发导航。

### 2.5 `navigate.stop`（无回执）——停止导航（v0.23.0 P-037）

payload 仅 `operation_id`（与 `tts.stop` 同构）。契约：停止当前正在
进行的导航；**不带** `point_name`/`texts`（停止无需知道目标）；当前没有
导航时 no-op。停止方式（立即停车/减速停靠/是否播报）归 Robot 本地。
本工程行为：取消在途导航并丢弃待消费的到达讲解（导览业务终止、释放
占用）；到达后的讲解已在播或其余播报业务**不受影响**（停一切用
`tts.stop`）。过期直接丢弃；同一 `operation_id` 重发不重复执行停止。

### 2.6 `point_map`——点位映射表同步（v0.25.0 P-039 + v0.28.0 P-042 播放词，有回执）

```json
{"payload": {"operation_id": "<uuidv7，Server 铸造的子 ID>",
             "points": [{"code": "A", "name": "比亚迪ætəʊ 1",
                         "text": "这就是比亚迪ætəʊ 1的展车了…"},
                        {"code": "B", "name": "比亚迪ætəʊ 2"}]}}
```

- 家庭级群发：App 是映射表唯一权威来源，Server 只校验转发不保存；
  每台机器人一条下行、各带 Server 铸造的子 `operation_id`（App 父 ID
  不出现在 MQTT 面）。
- `points` 必填 1..256 条 `{code, name, text?}`：`code` 1..32 个 Unicode
  标量、表内唯一（解析层纵深防御，重复回 `rejected/invalid_request`）；
  `name` 1..64 个 Unicode 标量（规则同 `point_name`）；`text` 可选
  （P-042）1..2048 个 Unicode 标量（规则同 `tts.texts` 单条）为该点位
  播放词——缺省 = 该点位无播放词，省略与空串不等价（空串按
  `invalid_request` 拒）。
- **播放词本地策略**（P-042 契约留给 Robot）：navigate **到达**且指令
  未携带 `texts` 时播放该点位 `text`；优先级 = 指令自带 `texts` >
  点位表 `text` > 本地 `navigate_default_texts` 默认讲解词（App 未配
  `text` 时的兜底）；迎宾准备（Y 点）不播。
- **语义是全量替换**：每次下发都用 `points` 整表覆盖旧表（不做增量
  合并）。本工程落盘到 `point_map_file`（临时文件 + rename 原子写；
  部署 yaml 指定持久化分区路径 `/robot/persist/var/guide_pulse/
  point_map.json`——ext4 数据分区，**断电不丢**；点位映射 + 播放词
  同表一起落盘），重启自动加载，App 无需重推。无 `text` 的条目不写
  `text` 键（与旧格式文件逐字节兼容）。
- **不受业务互斥拦截**（2026-10-17 需求）：纯数据同步，无音频/导航/
  形态副作用——迎宾/导览/播报任一业务进行中也照常处理并回 success。
- 落盘失败（磁盘满/路径不可写）：内存保留旧表（本次未生效），回
  `failed/point_map_failed`。
- 回执 `point_map.result`：success（已全量替换生效）/ failed
  (`point_map_failed`) / rejected（`command_expired` / `invalid_request`）。
  过期回 `rejected/command_expired`；同一 `operation_id` 重发重发同一
  回执（同 event_id），不重复替换。
- **点位解析**：此后 `navigate.point_name` 先经映射表 `name -> code`
  解析（如「比亚迪ætəʊ 1」→ 航点 `A`），未命中原样透传作航点名
  （兼容直接下发航点名/未推表的旧用法）。

## 3. 上行回执

`up/commands`（QoS 1，direction=up）：`tts.result`（受理回执，accepted
禁带 reason、rejected 必带 reason 枚举）与 `navigate.result` /
`scene.result` / `point_map.result`（v0.24.0/v0.25.0 一次性最终结果回执：
success 禁 reason / failed 必填 reason 按指令枚举 / rejected 必填
`command_expired`|`invalid_request`）。回执原文缓存重发（同 event_id）。
`tts.stop` / `navigate.stop` 不得发布任何上行事件（契约）。

### 3.1 `navigate.triggered` / `relocate.triggered`——语音触发上报（v0.26.0 P-040）

协议首个不由 App 指令驱动的上行事件。TTS 话术命中
`agent_monitor.navigate_phrases` / `agent_monitor.relocate_phrases` 语料
（语音自主触发）**且重定位成功后**（2026-10-19 现场需求，原 v0.26.0
为「受理开始执行时」）发布恰好一条：

```json
{"payload": {"point_name": "比亚迪ATTO 1", "trigger": "voice"}}
```

- **无 `operation_id`**：自主触发没有下发指令可关联，无幂等诉求
  （`event_id` 已承担传输去重）；payload 恰好 `point_name` + `trigger`。
- `point_name`：映射表显示名（`point_map` 的 name，反查失败用原始
  点位名）；Server 原样转发进 App WebSocket 事件
  `robot.navigation.triggered`（payload `{command, point_name, trigger}`）。
- `trigger`：可扩展枚举，当前唯一值 `voice`。
- **发布时机 = 各自动作的前置重定位成功**（2026-10-19 现场需求：只有
  重定位成功才能发消息给后端）：
  - `relocate.triggered`：`set_pose` 验收通过（终态 `finish_relocate`
    成功分支）后发布；验收失败/超时不发布。
  - `navigate.triggered`：导航出发前**强制重定位**（`set_pose` 最新
    停留点）成功后、导航 goal 发出时发布。语料触发的导航**无论是否
    发生变形都真做一次 set_pose**（首版曾把「已是人形跳过」当成功，
    真机验证未做重定位也发了上报，已修正）；重定位失败则导航照常
    出发但**不发布**（定位未验证的导航不让 App 感知）。迎宾准备
    （`welcome_prep` 语料）同口径。MQTT 下行 `navigate` 不强制
    （维持 2026-10-18「仅变形后补偿性重定位」，其结果经
    `navigate.result` 回执）。
- **配套终态（2026-10-18 现场需求）**：语料触发的导航被 `navigate.stop`
  停止时，也发一条 `navigate.result`（`failed/aborted`），`operation_id`
  用触发时本地生成的 uuid（与 `navigate.triggered` 一样无下发指令可
  关联，Server 侧按孤儿终态处理或与 triggered 事件按时间序对应）——
  后端对语音触发的导览既有「开始」也有「被停止」终态可感知。
- **不发布的场景**：App 下发的 `navigate` / `relocate` 指令（App 已知
  自己触发了什么）；业务互斥 busy 拒绝、点位映射表空/未命中（本地
  策略拒绝，未进入执行流程）；**重定位失败/被拒/超时**（2026-10-19
  需求起属「未成功」，不上报——App 拿到的事件语义从「语音触发了去
  该点」收紧为「语音触发的重定位已成功、动作已在执行」；此前
  2026-10-17「执行故障不撤销上报」的口径就此作废）。

## 4. 停止播报的实现说明

SDK v0.9.4.4 无 StopTts 服务。按《接口说明.md》仲裁矩阵：以
`SAFETY_L10(10)*100 + weight 99 + is_interrupted=true`（权重 1099）发送
SSML 静音段 `<speak><break time="300ms"/></speak>` 立即掐断任意播报。
（沿用 tts_demo 实测结论；勿改纯标点，会卡仲裁通道 30-60 秒。）
