# 原厂 Agent（运控板）TTS/动作 触发点与记录点调研结论

> 调研日期：2026-10-15。目标：找到原厂唤醒触发动作和 TTS 播报的记录点/触发点，
> 作为 guide_pulse 的输入源，衔接自有代码流程。
>
> **归档说明（2026-10-15）**：本文档原为 agent_bridge_runtime 调研工程的
> README，该工程已合并进本工程（`AgentMonitor` 组件 +
> `src/aimdk_msgs/agent/` 消息），原目录已删除。本文保留调研结论与
> 运控板日志回溯方法备查。

## 一、核心结论

原厂语音链路全部在**运控板**（`run@10.1.1.100`，出厂密码 `1`）上的
`aima-agent-app` 进程（即 DDS 域里的 `/aimrt_agent_node`）。
该板与 Jetson 同处一个 FastDDS 域（多播自动发现），
**Jetson 上可以直接订阅 agent 发布的全部话题，无需运控板做任何改动。**
消息运行时已拷贝到本目录并验证可用（见第四节）。

## 二、触发点：Jetson 可直接订阅的 DDS 话题

均由运控板 `aimrt_agent_node` 发布。QoS：BEST_EFFORT + TRANSIENT_LOCAL。

| 话题 | 消息类型 | 用途（衔接价值） |
| --- | --- | --- |
| `/aima/agent/interaction_state` | `AgentConversationState` | **唤醒/对话状态机**：IDLE(0)→LISTENING(1)→THINKING(2)→SPEAKING(3)。唤醒即变 LISTENING，子状态 VOICE_ACTIVE(2)/INTERRUPTED(3) |
| `/interaction/tts_status` | `TtsStatus` | **TTS 播报事件**：BEGIN(1)/PLAYING(2)/END(3)/STOP(4)/ERROR(5)，含 `text` 话术原文、`trace_id`、`domain`、优先级 |
| `/aima/agent/asr_result` | `AsrResult` | 用户说的话（识别文本、is_final、置信度、语言） |
| `/aima/agent/greeting_state` | `GreetingState` | 迎宾触发事件（face_id、name） |
| `/aima/agent_show_text` | `AgentShowText` | 流式字幕文本（START/CONTENT/END/SINGLE），与屏幕显示同源 |
| `/aima/agent/semantic_intent` | `SemanticIntent` | 语义意图（确认/取消等 type+value） |
| `/agent/task_operation` | `TaskOperation` | agent 任务操作（创建/完成等，msg_content 为 JSON） |
| `/agent/process_audio_output` | `ProcessedAudioOutput` | TTS 合成后的音频流（VAD 状态 + audio_data） |

动作类（运控板 task_engine/rc 发布）：

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/aima/mc/body_pose` | `McBodyPose` | 身体姿态/动作指令 |
| `/aima/interaction/state` | `InteractionState` | 动作执行状态（McPlayerState、McActionInfo 等） |
| `/aima/motion/intelligent_task_state` | `IntelligentMotionTaskState` | 智能动作任务状态 |

已验证的消息定义见本文件同目录 `msg_defs/`（从运控板导出的 .msg 原文）。

## 三、记录点：运控板日志（历史话术/动作回溯）

SSH 访问：`SSHPASS=1 sshpass -e ssh run@10.1.1.100`

- **agent 日志**：`/robot/persist/log/agent/agent.log`（含轮转 agent.log_1）
  - TTS 话术：`grep -a 'TTSManager received request' ... | grep -o 'text=.*domain'`
  - TTS 状态：`TTSManager dispatching status ... text="..."`
  - 唤醒：`wakeup_manager.cpp`、`iflytek_engine_wrapper.cpp [Wakeup]`、`wakeup_doa_*`
  - 交互状态机：`publishInteractionState, main_state: kIdle/kListening/...`
  - 动作技能：`skill_manager.cpp RebuildSkillCacheLocked`（wave_hands=1、shake_hands=2、
    laugh=26、proud=25、raise_hand_both=34、hands_down_both=37 等）
  - 注意：日志含二进制字符，grep 需加 `-a`
- **TTS 音频缓存**：`/robot/data/var/agent/tts_cache/*.opus`（按话术文本+MD5 命名）
- **讯飞引擎日志**：`/robot/persist/log/agent/iflytek_log/`
- 其他相关：`/robot/persist/log/{audio_manager,lumina,mc,rc,scene_engine,task_engine}/`

## 四、Jetson 侧运行时（本目录，已验证）

运控板 `/robot/software/aimrt_pulsecore/` 的 aimdk_msgs（201 个消息，aarch64+Py3.10+Humble，
与 Jetson 完全兼容）已拷贝到：

```
/opt/liwei/agent_bridge_runtime/
├── lib/        # libaimdk_msgs__*.so（8 个 typesupport 库）
├── python/     # aimdk_msgs Python 包（201 个消息）
├── test_agent_topics.py   # 长期订阅演示脚本
├── verify_e2e.py          # 端到端验证脚本（已通过）
└── README.md   # 本文档
```

使用方法：

```bash
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/opt/liwei/agent_bridge_runtime/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/liwei/agent_bridge_runtime/python:$PYTHONPATH
python3 /opt/liwei/agent_bridge_runtime/test_agent_topics.py
```

**2026-10-15 10:56 实测记录**（`verify_e2e.py`，通过 PlayTts 服务触发）：

```
[对话状态] main_state=3(SPEAKING)
[TTS] tts_status=1(BEGIN)  trace_id=liwei_test_1792032989710_...
[TTS] tts_status=2(PLAYING)
[TTS] tts_status=4(STOP)
[对话状态] main_state=0(IDLE)
```

说明：经 PlayTts 服务触发的播报，TtsStatus.text 为空（文本在请求侧已知）；
agent 自身产生的话术（唤醒回复、交互应答）TtsStatus.text 携带完整话术原文。

注意：`AsrResult/GreetingState/TaskOperation` 的 .msg 原文在
`/robot/software/scene_engine/share/aimdk_msgs/msg/`（运控板），
本运行时包（aimrt_pulsecore 版）未包含这三个消息的 Python 类；
如需订阅它们，可从运控板 scene_engine 对应 Python 包补充，或以 .msg 重建。

## 五、衔接建议

1. **输入源选择**：
   - 只想知道"机器人正在说话/说了什么"：订阅 `/interaction/tts_status`（话术+状态）。
   - 想知道"被唤醒了"：订阅 `/aima/agent/interaction_state`（main_state 变 LISTENING 即唤醒，
     回 IDLE 即一轮结束）。
   - 想知道"用户说了什么"：订阅 `/aima/agent/asr_result`。
2. **guide_bridge 集成**：在 guide_pulse 里加一个 ROS2 sub 节点，把上述事件桥接到
   现有 MQTT 流程（如占用检测：agent SPEAKING 期间抑制自有播报，避免撞音）。
3. **主动触发**：`/aimdk_5Fmsgs/srv/PlayTts` 服务（SDK 正式接口，已验证可用）。

## 六、2026-10-15 11:03 唤醒实测记录（真机验证通过）

用户实机唤醒一轮对话（话术"腾势D9是辆好车"），Jetson 订阅与运控板日志完全对上：

| 时刻 | Jetson DDS 订阅收到 | 运控板 agent.log 对应记录 |
| --- | --- | --- |
| 11:03:46 | `main_state=LISTENING`（唤醒） | `[Wakeup] wake keyword=yuan2 zai3 yuan2 zai3 doa=321` / `Wakeup detected` |
| 11:03:49-53 | `sub=VOICE_ACTIVE → THINKING` | ASR/对话处理 |
| 11:03:53 | `SPEAKING + TTS BEGIN/PLAYING` trace=`event_y9MqLg0ei8IhKJ3RBQcXU_tts_1_61GB...` | `TTSManager received request ... text=腾势D9是辆好车 domain=agent` |
| 11:03:56 | `TTS STOP → LISTENING` | `dispatching status ... status=4 text="腾势D9是辆好车"` |

### 关键结论（修正此前推测）

1. **TtsStatus.text 恒为空**：原厂 `tts_action_impl.cpp:191 ReceiveAndPublishTTSStatus: text=`
   发布到 DDS 前就没填话术文本（内部 tts_manager 有，转发时丢弃）。
2. **trace_id 是关联键**：DDS 事件里的 trace_id（如 `event_xxx_tts_1_61GB...`）与运控板日志
   `TTSManager received request item_id=... text=...` 的 item_id 完全一致，
   可实现「DDS 实时事件 → 按 trace_id 查日志取话术」的近实时衔接。
3. **/aima/agent_show_text 本轮未发布**（全程零消息），字幕话题在该场景不可用。
4. 唤醒词（拼音 yuan2 zai3 yuan2 zai3）、声源方位 doa、对话轮次均可在日志回溯。
