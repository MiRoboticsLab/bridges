std_msgs/Header header

uint16 batt_volt
int16 batt_curr
uint8 batt_soc
int16 batt_temp
int16 power_adapter_temp       # 2023.02.15新增
int16 wireless_charging_temp   # 2023.02.15新增
uint8 batt_st
uint8 bms_state_one;           # 2023.02.16新增
uint8 bms_state_two;           # 2023.02.16新增
uint8 key_val                  # 2023.02.17删除
uint8 disable_charge           # 2023.02.17删除
uint8 power_supply             # 2023.02.17删除
uint8 buzze                    # 2023.02.17删除
uint8 status                   # 2023.02.17删除
int8  batt_health
int16 batt_loop_number
int8  powerboard_status        # 2023.02.17删除

bool power_normal
bool power_wired_charging
bool power_finished_charging
bool power_motor_shutdown
bool power_soft_shutdown
bool power_wp_place
bool power_wp_charging
bool power_expower_supply