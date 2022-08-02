int32        motion_id
int32        cmd_type
int32        cmd_source   
int32        value
float32[]    vel_des      # x y yaw speed  m/s
float32[]    rpy_des      # roll pitch yaw  rad
float32[]    pos_des      # x y z  m/s
float32[]    acc_des      # acc for jump m^2/s
float32[]    ctrl_point   # pose ctrl point  m
float32[]    foot_pose    # front/back foot pose x,y,z  m
float32[]    step_height  # step height when trot  m

# cmd_type 取值定义
int32 SERVO_START = 0
int32 SERVO_DATA = 1
int32 SERVO_END = 2

# cmd_source 取值定义
int32 APP = 0
int32 AUDIO = 1
int32 VIS = 2