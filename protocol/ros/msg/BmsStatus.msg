std_msgs/Header header

uint16 batt_volt
int16 batt_curr
uint8 batt_soc
int16 batt_temp
int16 power_adapter_temp
int16 wireless_charging_temp
uint8 batt_st
uint8 bms_state_one
uint8 bms_state_two
int8  batt_health
int16 batt_loop_number

bool power_normal
bool power_wired_charging
bool power_finished_charging
bool power_motor_shutdown
bool power_soft_shutdown
bool power_wp_place
bool power_wp_charging
bool power_expower_supply

bool charge_over_current                                    # 充电过流保护：0-nomal，1-error
bool discharge_over_current                                 # 放电过流保护：0-nomal，1-error
bool cell_over_voltage                                      # 电池过压保护：0-nomal，1-error
bool cell_under_voltage                                     # 电池欠压保护：0-nomal，1-error
bool cell_volt_abnormal                                     # 电池电压异常：0-nomal，1-error
bool mos_over_temp                                          # MOS过温保护：0-nomal，1-error
bool discharge_short                                        # 放电短路保护：0-nomal，1-error
bool fuse                                                   # 保险丝断开保护：0-nomal，1-error
bool discharge_over_tmp                                     # 放电过温保护：0-nomal，1-error
bool discharge_under_tmp                                    # 放电欠温保护：0-nomal，1-error
bool charge_over_temp                                       # 充电过温保护：0-nomal，1-error
bool charge_under_temp                                      # 充电欠温保护：0-nomal，1-error
bool charge_mos_state                                       # 充电MOS状态：0-on，1-off
bool discharge_mos_state                                    # 放电MOS状态: 0-on，1-off
bool chg_mos_fault                                          # 充电MOS故障: 0-nomal，1-error
bool dsg_mos_fault                                          # 放电MOS故障：0-nomal，1-error