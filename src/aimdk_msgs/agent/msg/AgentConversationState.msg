# -----------------------------------------------------------------------------------
# Agent 向外发布的对话状态
# -----------------------------------------------------------------------------------

# agent 所处的对话状态
AgentConversationMainState main_state

# main_state=LISTENING 时的子状态；其他主态下为 DEFAULT
AgentConversationSubState sub_state

# 当前用户输入/对话事件ID
string event_id

# 当前播报实例ID；当前实现使用TTS/A2G的 item_id 映射
string trace_id
