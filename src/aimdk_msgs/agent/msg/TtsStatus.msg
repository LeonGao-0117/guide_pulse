# 播报文本
string text

# 最终优先级系数
uint32 priority

# 与语音对话的event_id相匹配（例如event_id为001时，trace_id为001_count_4，标识当前对话中的第四句TTS播报），播报方如果传了这个trace_id，则返回对应的trace_id
string trace_id

# 播报状态
TtsStatusType tts_status

 # 标识调用方来源
string domain

# 仅当TTSStatusType为TTSStatusType_Error时有效，当前错误与业务方无关，暂不定义具体错误码
string error_message
