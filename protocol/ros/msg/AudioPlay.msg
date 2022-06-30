
string module_name  # 模块名
uint16 play_id      # 播放ID

# play_id具体取值，每个ID对应具体的语音内容
uint8 PID_WIFI_OFFLINE = 1             # 没有连接网络/掉线提示
uint8 PID_WIFI_AUTO_CONNECTION = 2     # 正在自动连接网络提示
uint8 PID_WIFI_MANUAL_CONNECTION = 3   # 需要手动连接网络提示
uint8 PID_WIFI_COMMUNICATION = 4       # 连接到网络提示
uint8 PID_BATTERY_CAPICITY_LOW = 101   # 电量低于10%
uint8 PID_BATTERY_IN_CHARGING = 102    # 现在开始充电
uint8 PID_FACE_ENTRY_START = 103       # 人脸录入开始，蹲在面前不要动
uint8 PID_BATTERY_CHARGE_COMPELETE = 104 # 充电完成了 
uint8 PID_AI_PLEASE_ENABLE = 124       # 请登录app账号以启用小爱在线功能
uint8 PID_AI_ENABLE_SUCCESS = 125      # 小爱在线功能启动
uint8 PID_AI_SERVICE_EXPIRED = 127     # 语音登录已过期
