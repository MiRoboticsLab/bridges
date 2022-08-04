
# This message is  used to describe single tof payload
# Send frequency: 10
# The effective distance currently set by TOF is 150-660mm, less than 150mm, the displayed value is 0; 
# greater than 660mm, the displayed value is the maximum value of 660mm;
# At present, there is a tof sensor on each of the four legs of the dog, which is used to Obtain the elevation information 

std_msgs/Header header # Header timestamp should be acquisition time of tof data

uint8 LEFT_HEAD=0  #  the macro definition of tof serial number 
uint8 RIGHT_HEAD=1 #  Used to set tof position
uint8 LEFT_REAR=2
uint8 RIGHT_REAR=3

uint8 HEAD=4
uint8 REAR=5

int32 TOF_DATA_NUM=64 # number of tof data 

float32 SCALE_FACTOR=0.001 # factor of tof data used to compute real distance

bool data_available  # A flag that tof data is available

uint8 tof_position   # location of tof sensor 
                     # (LEFT_HEAD, RIGHT_HEAD, LEFT_REAR,RIGHT_REAR) [enum]

float32[] data      # tof data , Unit: m    
float32[] intensity      # tof data intensity     