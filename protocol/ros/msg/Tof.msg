
# This message contains tof

std_msgs/Header header # Header timestamp should be acquisition time of tof

uint8 LEFT_FRONT=0
uint8 RIGHT_FRONT=1
uint8 LEFT_BACK=2
uint8 RIGHT_BACK=3
int32 TOF_DATA_NUM=64
float32 SCALE_FACTOR=0.02

uint8 tof_postion              

float32[] data             