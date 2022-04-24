
string module_name  # 模块名
uint16 play_id      # 播放ID

# play_id具体取值，每个ID对应具体的语音内容
uint8 PID_WIFI_OFFLINE = 1             # 没有连接网络/掉线提示
uint8 PID_WIFI_AUTO_CONNECTION = 2     # 正在自动连接网络提示
uint8 PID_WIFI_MANUAL_CONNECTION = 3   # 需要手动连接网络提示
uint8 PID_WIFI_COMMUNICATION = 4       # 连接到网络提示