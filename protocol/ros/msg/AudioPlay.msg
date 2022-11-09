
string module_name  # 模块名
uint16 play_id      # 播放ID

# play_id具体取值，每个ID对应具体的语音内容
uint16 PID_WIFI_ENTER_CONNECTION_MODE_0 = 1  # 进入连接模式
uint16 PID_WIFI_ENTER_CONNECTION_MODE_1 = 2  # 开启连接模式
uint16 PID_WIFI_WAIT_FOR_SCAN_CODE_0 = 3     # 请出示WIFI二维码辅助连接
uint16 PID_WIFI_SCAN_CODE_SUCCEEDED_0 = 4    # 扫码成功，正在连接，请稍后
uint16 PID_WIFI_CONNECTION_SUCCEEDED_0 = 5   # 目标WIFI连接成功，可以访问互联网
uint16 PID_WIFI_CONNECTION_SUCCEEDED_1 = 6   # 目标WIFI连接成功，但不可以访问互联网
uint16 PID_WIFI_CONNECTION_FAILED_0 = 7      # 目标WIFI连接失败，请核对WIFI名称
uint16 PID_WIFI_CONNECTION_FAILED_1 = 8      # 目标WIFI连接失败，请核对WIFI密码
uint16 PID_WIFI_CONNECTION_FAILED_2 = 9      # 目标WIFI连接失败，其他错误（超时或中断）
uint16 PID_WIFI_EXIT_CONNECTION_MODE_0 = 10   # 退出连接模式
uint16 PID_WIFI_OFFLINE = 11                 # 当前WIFI已掉线
uint16 PID_WIFI_COMMUNICATION = 12           # 正常通信中
uint16 PID_WIFI_SCAN_CODE_IP_ERROR = 13        # 二维码提供方IP错误
uint16 PID_WIFI_SCAN_CODE_INFO_ERROR = 14      # 二维码信息错误，未检出到wifi信息
uint16 PID_WIFI_REQUEST_OPEN_CAMERA_SUCCESS = 15    # 请求打开摄像头成功
uint16 PID_WIFI_REQUEST_OPEN_CAMERA_FAIL = 16       # 请求打开摄像头失败
uint16 PID_WIFI_REQUEST_CLOSE_CAMERA_SUCCESS = 17   # 请求关闭摄像头成功
uint16 PID_WIFI_REQUEST_CLOSE_CAMERA_FAIL = 18      # 请求关闭摄像头失败
uint16 PID_FACE_ENTRY_ADD_FACE = 21            # 开始录入人脸，请正对摄像头，且勿遮挡人脸，保持稳定
uint16 PID_FACE_ENTRY_CANCLE_ADD_FACE = 22     # 取消录入人脸
uint16 PID_FACE_ENTRY_CONFIRM_LAST_FACE = 23   # 确认录入人脸
uint16 PID_FACE_ENTRY_UPDATE_FACE_ID = 24      # 更新录入人脸
uint16 PID_FACE_ENTRY_DELETE_FACE = 25         # 删除录入人脸
uint16 PID_FACE_ENTRY_GET_ALL_FACES = 26       # 获取录入人脸信息
uint16 PID_FACE_ENTRY_FIX_POSE  = 27           # 请正对摄像头
uint16 PID_FACE_ENTRY_FIX_POSE_LEFT = 28       # 请向左扭头
uint16 PID_FACE_ENTRY_FIX_POSE_RIGHT = 29      # 请向右扭头
uint16 PID_FACE_ENTRY_FIX_POSE_UP = 30         # 请向上抬头
uint16 PID_FACE_ENTRY_FIX_POSE_DOWN = 31       # 请向下低头
uint16 PID_FACE_ENTRY_FIX_DISTANCE_CLOSE = 32  # 请距狗头近一点
uint16 PID_FACE_ENTRY_FIX_DISTANCE_NEAR = 33   # 请距狗头远一点
uint16 PID_FACE_ENTRY_FIX_STABLE = 34          # 请保持稳定
uint16 PID_FACE_ENTRY_MUTIPLE_FACES = 35       # 检测到多个人脸
uint16 PID_FACE_ENTRY_NONE_FACES = 36          # 没有检测到人脸
uint16 PID_FACE_ENTRY_TIMEOUT = 37             # 录入超时，请重新录入
uint16 PID_FACE_ENTRY_FINISH = 38              # 录入成功
uint16 PID_FACE_RECOGNITION_REQUEST = 39       # 开始人脸识别，请正对摄像头，且勿遮挡人脸
uint16 PID_FACE_DEGREE_HEAD_TILT = 40          # 请不要歪头
uint16 PID_FACE_RECGONITION_FINISH = 41        # 人脸识别成功
uint16 PID_FACE_RECGONITION_TIMEOUT = 42       # 人脸识别已经超时，请重新尝试
uint16 PID_FACE_ALREADY_EXIST = 43             # 人脸已存在，请不要录入同一张脸
uint16 PID_BATTERY_CAPICITY_LOW = 101   # 电量低于10%
uint16 PID_BATTERY_IN_CHARGING = 102    # 现在开始充电
uint16 PID_FACE_ENTRY_START = 103       # 人脸录入开始，蹲在面前不要动
uint16 PID_BATTERY_CHARGE_COMPELETE = 104 # 充电完成了
uint16 PID_AI_PLEASE_ENABLE = 124       # 请登录app账号以启用小爱在线功能
uint16 PID_AI_ENABLE_SUCCESS = 125      # 小爱在线功能启动
uint16 PID_AI_SERVICE_EXPIRED = 127     # 语音登录已过期
uint16 PID_TEST_HARDWARE_AUDIO = 3000          # 当前为语音硬件测试
uint16 PID_TEST_STAGE_ONE = 3001               # 第一阶段测试结束。请查看测试结果并记录后，进行第二阶段测试
uint16 PID_TEST_STAGE_THREE = 3003             # 第三阶段测试结束，请查看测试结果并记录
uint16 PID_SOUND_EFFECT_READY = 6000           # 整机就绪音效
uint16 PID_STOP_AUDIO_PLAY = 9999              # 打断当前正在播放的语音
