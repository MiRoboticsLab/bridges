int32       motion_id
int8        contact
int8        order_process_bar  #  order process, 1 == 1 %
int8        switch_status      
int8        ori_error
int16       footpos_error
int32[]     motor_error

# switch_status macro value
int8 kNORMAL = 0
int8 kTRANSITIONING = 1
int8 kESTOP = 2
int8 kEDAMP = 3
int8 kBAN_TRANS = 4