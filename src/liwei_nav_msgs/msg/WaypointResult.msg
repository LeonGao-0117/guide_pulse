# 一个任务的最终执行结果，发在航点巡航节点的 `~/result` 话题上。
#
# 每个**受理了的**请求（Navigate action 的 goal：go / cancel / charge / set_pose，
# `~/goto` 话题，`~/start` 与 `~/cancel` 两个 Trigger）真正做完时发**恰好一条**，
# 用 `task_id` 与受理时拿到的编号对上。被拒的请求没有编号，也不会有结果。
#
# 什么时候算「做完」：
#   go       走完（带 return_to_quadruped 时等变形也做完）、失败、被取消、或被新任务取代
#   cancel   Nav2 确认取消（带 return_to_quadruped 时等停稳、记停留点、变形都做完）
#   charge   趴下（或中途哪一步失败）
#   set_pose 应答时就已经有结果，这里再发一条，便于只订话题的调用方统一处理
#
# 话题是 reliable + transient_local、保留最近 10 条：先调服务、后订话题也收得到。

# 受理时在应答（Command.task_id / Trigger.message）里给出的编号，从 1 开始。
uint32 task_id

# go / cancel / charge / set_pose
string action

# 只有 `~/goto` 话题下发的任务才有：那条命令里带的 token。
string token

# **请求的事情全部做成了**才是 true：
#   go       state=SUCCEEDED、没有跳过的航点、要求变狗形时也变成了
#   cancel   Nav2 确认已取消、要求变狗形时也变成了
#   charge   已趴下
#   set_pose 重定位验收通过
bool success

# 结束时巡航节点的状态：SUCCEEDED / FAILED / CANCELLED / IDLE …
# go 的导航结果看它；success=false 而 state=SUCCEEDED 说明导航成了、但跳过了
# 航点或者没变成狗形，看 skipped / park。
string state

# 人可读的结果说明，失败时写了原因。
string message

# 这个任务的航点名序列（cancel / charge 为空）。
string[] plan

# 没到达就被跳过的航点（on_failure=skip）。
string[] skipped

# 变狗形 / 趴下这一步的结果；空 = 这个任务没要求。
#   done 已变狗形   failed 变形失败   skipped 没做（通道关着 / 桥没起 / 没走成）
#   charged 已趴下  lie_failed 已变狗形但没趴下
string park
string park_message
