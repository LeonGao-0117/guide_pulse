# guide_pulse MQTT 协议说明（RobotCar 协议 v0.22.0 子集）

guide_pulse 消费 RobotCar Protocol 正式契约的指令子集，本文只记录
本工程视角的要点与本地策略；完整契约见
`RobotCar/Protocol/contracts/server-robot/mqtt/v1/command-protocol.md`。

## 1. 主题与连接

| 主题（前缀 `robotCar/v1/robots`，Server 固定值） | 方向 | QoS | Retain |
| --- | --- | --- | --- |
| `.../{robot_id}/down/commands` | Server→Robot | 1 | false |
| `.../{robot_id}/up/commands` | Robot→Server（仅 `tts.result`） | 1 | false |
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
`command_expired`/`invalid_request`/`playback_unavailable`）；同一
operation_id 重发同一回执（同 event_id）。

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

## 3. 上行回执

仅 `tts.result`（`up/commands`，QoS 1，direction=up）。accepted 禁带
reason；rejected 必带 reason 枚举。回执原文缓存重发（同 event_id）。
`scene` / `navigate` / `tts.stop` 不得发布任何上行事件（契约）。

## 4. 停止播报的实现说明

SDK v0.9.4.4 无 StopTts 服务。按《接口说明.md》仲裁矩阵：以
`SAFETY_L10(10)*100 + weight 99 + is_interrupted=true`（权重 1099）发送
SSML 静音段 `<speak><break time="300ms"/></speak>` 立即掐断任意播报。
（沿用 tts_demo 实测结论；勿改纯标点，会卡仲裁通道 30-60 秒。）
