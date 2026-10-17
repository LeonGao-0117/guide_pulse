# -----------------------------------------------------------------------------------
# ASR 语音识别结果
# topic: /aima/agent/asr_result
#
# 用于通知外部当前 ASR 语音识别结果（文本转写）。
# 与 SemanticIntent 的区别：本 topic 仅发布原始识别文本，不含语义解析，
# 订阅方不需要语义意图时只订本 topic 即可。
# -----------------------------------------------------------------------------------

MessageHeader header

# 识别文本
string text

# 事件ID，用于关联对话上下文
string event_id

# 是否为最终结果（false 表示中间/部分结果）
bool is_final

# 置信度 [0.0, 1.0]
float32 confidence

# 语言标签，如 "zh"、"en"
string language
