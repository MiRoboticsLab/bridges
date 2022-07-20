int8         mode
int8         gait_id
int8         contact
int8         life_count   
float32[]    vel_des      # x y yaw speed  m/s
float32[]    rpy_des      # roll pitch yaw  rad
float32[]    pos_des      # x y z  m/s
float32[]    acc_des      # acc for jump m^2/s
float32[]    ctrl_point   # pose ctrl point  m
float32[]    foot_pose    # front/back foot pose x,y,z  m
float32[]    step_height  # step height when trot  m
int32        value
int32        duration