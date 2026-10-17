uint8 value

uint8 TTS_CONFIG_STATUS_TYPE_UNKNOWN = 0

# 播报开始
uint8 TTS_STATUS_TYPE_BEGIN = 1      

# 播报中        
uint8 TTS_STATUS_TYPE_PLAYING = 2

# 播报结束          
uint8 TTS_STATUE_TYPE_END = 3

# 暂停播报/取消播报/中断播报           
uint8 TTS_STATUS_TYPE_STOP = 4

# 播报失败     
uint8 TTS_STATUS_TYPE_ERROR = 5

# 在播报队列中，尚未开始播报         
uint8 TTS_STATUS_TYPE_INQUE = 6

# 播报队列无此文本，也不在播报    
uint8 TTS_STATUS_TYPE_NOT_INQUE = 7