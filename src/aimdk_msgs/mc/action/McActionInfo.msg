# 当前运行的Action
McAction current_action

# 触发源（与 SetMcAction.request.source 同义）
# 注意：SDK v0.9.4.4 头文件缺此字段，但运控固件（v0.9.4.8+，与
# t1_slam_nav_ws 导航栈实测通信的版本）带它——缺字段会导致 GetMcAction
# 的 DDS 类型不匹配（服务永远不可达）。字段位置按固件版实测顺序
#（current_action 之后、action_desc 之前），见 docs/design.md 迎宾变形节
string source

# 描述
string action_desc

# Action状态
McActionStatus status

# action 切换是否就绪, false 未准备就绪, true 切换就绪
bool switch_ready
