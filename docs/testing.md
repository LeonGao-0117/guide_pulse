# guide_pulse 测试说明

三层测试，全部无需真机：

## 1. 单元测试（gtest，`colcon test`）

```bash
cd /opt/liwei/guide_pulse
bash scripts/build.sh
colcon test --packages-select guide_pulse && colcon test-result --verbose
```

| 可执行 | 覆盖 |
| --- | --- |
| `guide_protocol_test` | 四类指令载荷正反例、信封严格校验、过期判定、幂等键提取、tts.result 构造 |
| `robot_enroll_test` | enroll 响应解析、错误码映射、SN 格式校验 |
| `tts_client_test` | 进程内 Mock PlayTts：请求字段映射、回执判定、超时分类 |
| `playlist_test` | 播报状态机：sequence/shuffle 轮次、单轮自行结束、无界循环、重试不推进 |
| `welcome_test` | 迎宾词来源（指令 texts 优先/参数默认）、未知 scene 值忽略 |
| `tour_test` | 讲解状态机（FakeNavExecutor）：到达开播、到达失败不开播、新导航替换、取消 |
| `waypoint_nav_executor_test` | 真机导航执行器（进程内 Mock 航点服务）：受理/到达/失败/拒绝/状态转移/取消 |

协议层/enroll/迎宾/讲解为纯 C++（无 ROS 依赖），编译为独立测试目标。

## 2. 端到端集成测试（`tools/run_integration_test.sh`）

自带全部依赖，产线外可跑：

```
enroll_mock_server(注册面 Mock, :18080) ← guide_bridge(enroll 换凭据)
backend_simulator → mosquitto(:1884) → guide_bridge → mock_tts_service(SDK Mock)
```

用例（断言桥接日志 / Mock TTS 播报日志 / backend_simulator 收到的回执）：

1. **tts 基线**：下发 tts → Mock TTS 收到第一条 → 收到 `tts.result(accepted)`；
2. **迎宾**：下发 `scene(welcome)` 无 texts → Mock TTS 收到默认迎宾词一轮
   （出现后不再重复——单轮语义）；带 texts → 播指令词；
3. **汽车讲解**：下发 `navigate("3号展台", [讲解词])` → 日志出现
   `Mock导航到达` → Mock TTS 依次收到讲解词 → 播完不循环；
4. **停止**：讲解播报中下发 `tts.stop` → 播报终止（无回执）；
5. **导航替换**：navigate A 进行中再发 navigate B → 日志出现替换、只到达 B；
6. **过期指令**：`--expires` 已过期的 navigate → 被丢弃且不导航。

```bash
bash tools/run_integration_test.sh
```

## 3. 本地联调（手动）

与 tts_demo 相同的四步（详见 README §快速开始）：

```bash
mosquitto -v -p 11883
python3 tools/enroll_mock_server.py
ros2 launch guide_pulse guide_pulse.launch.py \
  broker_uri:=tcp://127.0.0.1:11883 enroll_origin:=http://127.0.0.1:18080 \
  serial_number:=SIM-local-01 mock_tts:=true
python3 tools/backend_simulator.py --broker tcp://127.0.0.1:11883 welcome
python3 tools/backend_simulator.py --broker tcp://127.0.0.1:11883 \
  navigate 3号展台 --text "这是我们的主打车型" --text "续航一千公里"
```
