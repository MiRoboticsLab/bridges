# 该消息用于表征模块所在设备的wifi信息的接口约束

bool is_connected  # 是否连接了网络，取值范围{true, false}
                   # 仅当[is_connected=true]时，下述字段有效
string ip          # 设备ip地址
string ssid        # 设备wifi名字
uint8 strength     # 设备wifi信号强度，取值范围[0,100]，0：无信号，100：信号最强