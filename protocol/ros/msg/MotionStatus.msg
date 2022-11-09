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