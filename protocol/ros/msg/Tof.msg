
# This message is  used to describe single tof 

std_msgs/Header header # Header timestamp should be acquisition time of tof

uint8 LEFT_FRONT=0 #  serial number of tof
uint8 RIGHT_FRONT=1
uint8 LEFT_BACK=2
uint8 RIGHT_BACK=3
int32 TOF_DATA_NUM=64 # number of tof data 
float32 SCALE_FACTOR=0.001 #factor of tof data 
bool data_available # Is tof sensor data available?
uint8 tof_postion   #location of tof sensor           
float32[] data      # tof data Unit: m    