# guide_pulse —— 展厅讲解机器人工程（迎宾 / 导航 / 汽车讲解）

APP 经 Server（RobotCar 协议 v0.25.0）下发参数到 Robot，Robot 本地执行
**迎宾**（`scene`）、**导航+汽车讲解**（`navigate`）、**播报**（`tts`，
单轮播完）与**点位映射表同步**（`point_map`，全量替换落盘）。框架沿用 tts_demo 工程（enroll 注册 / MQTT 服务 / TTS 播报），SDK 消息包
（`src/aimdk_msgs`，拷贝自 sdk_t1-v0.9.4.4）与接口文档
（`docs/sdk/接口说明.md`）已内置本工程独立编译。

```
┌──────────┐  HTTP(指令)  ┌──────────┐  MQTT(QoS1) down/commands   ┌─────────────────┐
│   APP    │ ──────────▶ │  Server   │ ─────────────────────────▶ │ guide_bridge     │
└──────────┘             └──────────┘                             │ (本工程, C++17)  │
                           presence/heartbeat/*.result ◀─────── │                 │
                                                                   │ scene=welcome →│→ 迎宾词一轮(PlayTts)
                                                                   │ navigate      →│→ NavExecutor→到达→讲解词一轮
                                                                   │ tts / tts.stop→│→ 列表单轮/停止
                                                                   └─────────────────┘
```

## 1. 功能与指令映射（零协议变更，详见 docs/design.md）

| 展厅功能 | 协议指令 | 回执 | Robot 本地行为 |
| --- | --- | --- | --- |
| 迎宾 | `scene`（`scene="welcome"` + 可选 `texts`） | `scene.result`（v0.24.0） | 先播引导语 `welcome_lead_text`（默认「元气满满，小T来迎接我的顾客了」，2026-10-17 需求，空=禁用）→ **播完再变形**（直调运控 SetMcAction/GetMcAction，已是人形零动作；不依赖 t1_slam_nav_ws）→ 变形完成后才播迎宾词一轮（texts 缺省用 `welcome_default_texts`；二维码准备与引导语/变形并行，上屏在变形后 TTS 前） |
| 导航 + 汽车讲解 | `navigate`（`point_name` + 可选 `texts`） | `navigate.result`（v0.24.0） | 导航到点位，到达后按序播讲解词一轮（到达回 `success`；失败/中止/忙碌回 `failed+reason`） |
| 地图重定位 | `relocate`（`point_name` + 可选 `texts`，2026-10-17 接入） | `relocate.result` | **不可打断的互斥业务**：`point_name` 经映射表解析 -> `set_pose` goal 重设定位初值（机器人不动）-> 等导航模块验收（goal result（验收完才回），真机 ~1.5s；`relocate_timeout_seconds` 默认 30s 超时兜底）-> 验收通过回 `success` 并播报（缺省「{点位名}点重定位成功」）；等待期新业务回 `failed/busy` 并播「我正在重定位，请稍后」（迎宾/播报/讲解中被拒不播——正在出声）；受理被拒当场回 `point_not_found`/`localization_failed`（兼语料触发本地注入，不回执） |
| 播报 | `tts`（`texts`+`mode`+`interval_seconds`） | `tts.result` | 列表单轮播完自行结束（2026-10-14 变更，原为循环播到替换/停止） |
| 停止 | `tts.stop` | 无 | 停一切播报并取消导航 |
| 取消导航 | `navigate.stop`（v0.23.0） | 无 | 只取消当前导航（无导航 no-op，不影响讲解/播报） |
| 点位映射表同步 | `point_map`（v0.25.0 P-039） | `point_map.result` | App 唯一权威：`points`（1..256 条 `{code,name}`）**全量替换**本地映射并落盘（`point_map_file`，重启自动加载）；纯数据同步**不受业务互斥拦截**；落盘失败回 `failed/point_map_failed`。此后 `navigate`/`relocate` 的 `point_name` 经映射表 `name->code` 解析（如「比亚迪ATTO 1」→航点 `A`），未命中原样透传 |

本地策略：音频单会话（新播报替换旧会话）；导航单任务（新 navigate 替换旧导航）；
tts/迎宾/讲解均单轮播完自行结束（2026-10-14 起 tts 不再循环）；迎宾词播报未完
期间新 scene/navigate 直接拒绝（2026-10-15，无回执面本地丢弃）。SDK 无点位导航服务——`NavExecutor` 为真机对接点
（默认 Mock；真机实现对接 `/opt/liwei/t1_slam_nav_ws` 的
`/liwei_navigation/navigate` action，见 docs/design.md §4）。

## 2. 目录结构

```
guide_pulse/
├── third_party/paho.mqtt.cpp      # vendored MQTT C++（静态链接，拷自 tts_demo）
├── src/
│   ├── aimdk_msgs/                # SDK 消息包（拷自 sdk_t1-v0.9.4.4）
│   └── guide_pulse/               # 机器人侧实现（ament_cmake, C++17）
│       ├── include/guide_pulse/
│       │   ├── guide_protocol.hpp     # MQTT 协议层（tts/tts.stop/scene/navigate）
│       │   ├── robot_enroll.hpp       # enroll 注册面客户端（沿用 tts_demo）
│       │   ├── tts_client.hpp         # SDK PlayTts 封装 + 最高权重抢占停止
│       │   ├── playlist.hpp           # 播报状态机（单轮/循环，纯逻辑）
│       │   ├── welcome.hpp            # 迎宾模块（纯函数）
│       │   ├── tour.hpp               # 讲解模块状态机（导航→到达→讲解）
│       │   ├── nav_executor.hpp       # 导航执行器接口 + Mock（真机对接点）
│       │   └── mqtt_guide_bridge_node.hpp
│       ├── src/… / test/…             # 实现 + gtest 单元测试（136 用例）
│       ├── config/guide_pulse.yaml    # 节点参数
│       └── launch/guide_pulse.launch.py
├── scripts/                        # build/deploy/start/autoStart（沿用 tts_demo 模式）
├── tools/                          # backend_simulator / enroll_mock / mqtt_monitor
│                                   # / run_integration_test.sh / deps(paho python)
└── docs/
    ├── design.md                   # 设计文档（先于代码评审定稿）
    ├── protocol.md                 # 协议子集与本地策略
    ├── testing.md                  # 测试说明
    └── sdk/接口说明.md              # SDK 接口文档副本
```

## 3. 快速开始

### 3.1 构建

```bash
cd /opt/liwei/guide_pulse
bash scripts/build.sh          # 本机已具备 ROS Humble；新环境用 scripts/deploy.sh
```

### 3.2 本地联调（无真机）

```bash
# 终端1：本地 mosquitto + 终端2：enroll 注册面 Mock
mosquitto -v -p 11883
python3 tools/enroll_mock_server.py

# 终端3：桥接节点 + Mock SDK TTS 服务
source install/setup.bash
ros2 launch guide_pulse guide_pulse.launch.py \
  broker_uri:=tcp://127.0.0.1:11883 enroll_origin:=http://127.0.0.1:18080 \
  serial_number:=SIM-local-01 mock_tts:=true mock_nav_seconds:=2

# 终端4：模拟后端（APP 经 Server 下发）
python3 tools/backend_simulator.py --broker tcp://127.0.0.1:11883 welcome            # 迎宾（默认词）
python3 tools/backend_simulator.py --broker tcp://127.0.0.1:11883 \
  navigate 3号展台 --text "这是我们的主打车型" --text "续航一千公里"                    # 导航+讲解
python3 tools/backend_simulator.py --broker tcp://127.0.0.1:11883 stop               # 停止
python3 tools/backend_simulator.py --broker tcp://127.0.0.1:11883 nav-stop           # 只取消导航
python3 tools/mqtt_monitor.py --broker tcp://127.0.0.1:11883                         # 实时观察 MQTT
```

### 3.3 真机部署

与 tts_demo 相同：SN 自动获取（参数 > 环境变量 `GUIDE_PULSE_SERIAL_NUMBER`
> `/robot/info/sn` > 开机脚本 SSH 运控板）；运维预登记 SN
（`RobotCar/Server`：`scripts/robot-admin.sh register <SN> guide_pulse`）；
确认 `config/guide_pulse.yaml` 的 `broker_uri`/`enroll_origin`；真机固件提供
`/aimdk_5Fmsgs/srv/PlayTts`（勿开 mock_tts）。开机自启见 tts_demo
`docs/AutoStart.md` 同款步骤（`scripts/autoStartConfig.sh` 已随本工程提供，
进程名 `guide_bridge`）。真机导航待 t1_slam_nav_ws 航点巡航就绪后按
`docs/design.md` §4 接入 `WaypointNavExecutor`。

## 4. 测试

```bash
colcon test --packages-select guide_pulse && colcon test-result --verbose  # 单元测试
bash tools/run_integration_test.sh                                        # 端到端集成测试
```

覆盖：协议解析正反例、enroll、TtsClient（进程内 Mock PlayTts）、播报状态机、
迎宾/讲解模块（FakeNav）、真机导航执行器（进程内 Mock 航点服务，含
set_pose 重定位/point_not_found 分类）、迎宾单轮、导航到达讲解、tts.stop、
navigate.stop（取消导航/no-op）、导航替换、relocate（成功播报回执/导航在途
busy 拒绝）、过期指令、幂等回执。详见 `docs/testing.md`。

## 5. 已知边界

- 导航执行器二选一（参数 `nav_executor`）：`mock`（默认，联调/测试）或
  `waypoint`（真机，对接 t1_slam_nav_ws `/liwei_navigation/navigate`
  action，`point_name` 直接映射航点名，详见 `docs/design.md` §4）。真机启用
  前置条件：t1_slam_nav_ws 导航栈（liwei_navigation）在该机上可用。
- 迎宾变形（2026-10-15）：迎宾时直调厂家运控（`SetMcAction`/
  `GetMcAction`，本体固件提供）确保人形——狗站立/行走态按 SDK 转移表
  逐步变形，已是人形零动作，趴姿默认一路扶起（`welcome_stand_from_down`
  可关）。**不依赖 t1_slam_nav_ws**（与导航栈速度桥并存安全，发动作
  无需信源注册）。变形在引导语（`welcome_lead_text`，2026-10-17）播完
  之后发起、与二维码准备并行，失败不阻断语音。无真机联调
  用 `scripts/mock_mc_service.py`（集成测试已覆盖完整转移序列）。
  已知取舍：~~变形音与头顶摄像头伸出跟随的是导航栈（速度桥）的行为，
  迎宾直调路径不附带这两项~~ 摄像头伸出已于 2026-10-15 跟进（变形完成
  后 `SetServo` 竖起）；变形音已于 2026-10-16 跟进——发变形动作前后台
  SSH 运控板 `paplay` 官方 `transform4to2.wav`（`welcome_transform_sound`
  可关；凭据复用 `display_remote_*`，失败只记日志）。
  出声路径选型（真机排查）：固件 v2d0 起 `/hal_audio` 节点没了，
  `/aima/hal/audio/playback` 全域无订阅者——导航栈推 PCM 的旧经验已
  失效；`PlayAudioFile` 服务在图里但请求无人应答。扬声器走运控板
  PulseAudio（audio_manager 路由日志），官方 wav 也在运控板上，SSH
  paplay 直接播。
- 迎宾动作（招手）预留参数 `welcome_motion_id`（默认 0 禁用）：动作 ID 因
  机型而异，真机校准后启用。
- `ota.start` 等指令静默忽略（由其他组件处理；`relocate` 已于 2026-10-17
  接入：MQTT 下行重定位 -> set_pose -> 播报「{点位名}点重定位成功」->
  `relocate.result` 回执）。
- 停止播报依赖 SDK 仲裁抢占（SAFETY_L10+99 权重 1099 静音段抢占，tts_demo
  实测结论）；`estimated_duration` SDK 固定返回 0，语音时长按 ~220ms/字估算。
