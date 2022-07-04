
string module_name  # 模块名
uint16 play_id      # 播放ID

# play_id具体取值，每个ID对应具体的语音内容
uint8 PID_WIFI_ENTER_CONNECTION_MODE_0 = 1  # 进入连接模式
uint8 PID_WIFI_ENTER_CONNECTION_MODE_1 = 2  # 开启连接模式
uint8 PID_WIFI_WAIT_FOR_SCAN_CODE_0 = 3     # 请出示WIFI二维码辅助连接
uint8 PID_WIFI_SCAN_CODE_SUCCEEDED_0 = 4    # 扫码成功，正在连接，请稍后
uint8 PID_WIFI_CONNECTION_SUCCEEDED_0 = 5   # 目标WIFI连接成功，可以访问互联网
uint8 PID_WIFI_CONNECTION_SUCCEEDED_1 = 6   # 目标WIFI连接成功，但不可以访问互联网
uint8 PID_WIFI_CONNECTION_FAILED_0 = 7      # 目标WIFI连接失败，请核对WIFI名称
uint8 PID_WIFI_CONNECTION_FAILED_1 = 8      # 目标WIFI连接失败，请核对WIFI密码
uint8 PID_WIFI_CONNECTION_FAILED_2 = 9      # 目标WIFI连接失败，其他错误（超时或中断）
uint8 PID_WIFI_EXIT_CONNECTION_MODE_0 = 10   # 退出连接模式
uint8 PID_WIFI_OFFLINE = 11                 # 当前WIFI已掉线
uint8 PID_WIFI_COMMUNICATION = 12           # 正常通信中
uint8 PID_BATTERY_CAPICITY_LOW = 101   # 电量低于10%
uint8 PID_BATTERY_IN_CHARGING = 102    # 现在开始充电
uint8 PID_FACE_ENTRY_START = 103       # 人脸录入开始，蹲在面前不要动
uint8 PID_BATTERY_CHARGE_COMPELETE = 104 # 充电完成了 
uint8 PID_AI_PLEASE_ENABLE = 124       # 请登录app账号以启用小爱在线功能
uint8 PID_AI_ENABLE_SUCCESS = 125      # 小爱在线功能启动
uint8 PID_AI_SERVICE_EXPIRED = 127     # 语音登录已过期
