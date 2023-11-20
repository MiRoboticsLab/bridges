int32       motion_id
int8        contact
int8        order_process_bar  #  order process, 1 == 1 %
int8        switch_status      
int8        ori_error
int16       footpos_error
int32[]     motor_error

# switch_status macro value
int8 NORMAL = 0
int8 TRANSITIONING = 1
int8 ESTOP = 2
int8 EDAMP = 3
int8 LIFTED = 4
int8 BAN_TRANS = 5
int8 OVER_HEAT = 6
int8 LOW_BAT = 7
int8 ORI_ERR = 8
int8 FOOTPOS_ERR = 9
int8 STAND_STUCK = 10
int8 MOTOR_OVER_HEAT = 11
int8 MOTOR_OVER_CURR = 12
int8 MOTOR_ERR = 13
int8 CHARGING = 14