# guide_pulse MQTT 协议说明（RobotCar 协议 v0.21.0 子集）

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
             "qrcode_png_base64": "iVBORw0KGgo..."}}
```

- `scene` 必填，当前唯一合法值 `welcome`（可扩展枚举）；未知值记日志忽略。
- `texts` 可选（1..20 条，每条 1..2048）；缺省用 `welcome_default_texts`。
- `qrcode_png_base64` 可选（v0.22.0，1..2796210 字节标准 base64，解码后
  ≤2MB PNG）：迎宾二维码图，App 连同迎宾 enable 一起下发。机器人侧转
  848x480 H.264 MP4 推送运控板后 `PlayEmotion(loop_count=0)` 常驻显示，
  5 分钟超时（`display_show_timeout_seconds`）或任意新业务
  （tts/scene/navigate/tts.stop）到达即恢复默认表情。缺省播放运控板
  预置 MP4（`display_remote_dir/QRCode.mp4`，运维预部署）。
- 行为：开一轮迎宾播报（单轮结束，不循环）+ 二维码常驻显示；不打断导航。
  屏幕显示失败不影响语音播报（仅告警日志）。
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
