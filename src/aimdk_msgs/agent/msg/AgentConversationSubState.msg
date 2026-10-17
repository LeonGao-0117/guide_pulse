# -----------------------------------------------------------------------------------
# Agent 对话子状态枚举（main_state=LISTENING 时生效）
# -----------------------------------------------------------------------------------

uint32 value

# 子状态不适用（main_state ≠ LISTENING 时的占位值）
uint32 DEFAULT = 0

# 听取中但无语音输入（main_state = LISTENING 时生效）
uint32 IDLE = 1

# 语音激活
uint32 VOICE_ACTIVE = 2

# 被打断
uint32 INTERRUPTED = 3
