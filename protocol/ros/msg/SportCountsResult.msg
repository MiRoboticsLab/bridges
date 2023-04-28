uint8 ALGO_OPEN = 0                     #算法开
uint8 COUNT_COMPLETE_CLOSE = 1       #计数完成，算法关
uint8 TIMEOUT_CLOSE = 2              #超时，算法关
uint8 REQUEST_CLOSE = 3              #请求算法关闭，算法关

int32 SPORT_SQUAT= 1             #深蹲
int32 SPORT_HIGHKNEES = 2        #高抬腿
int32 SPORT_SITUP = 3            #仰卧起坐
int32 SPORT_PRESSUP = 4          #俯卧撑
int32 SPORT_PLANK = 5            #平板支撑
int32 SPORT_JUMPJACK = 6         #开合跳

uint8 algo_switch          #算法开关
int32 sport_type           #运动类型
int32 counts               #运动个数
int32 duration             #运动时长