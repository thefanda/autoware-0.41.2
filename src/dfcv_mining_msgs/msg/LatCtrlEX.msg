std_msgs/Header header

uint8 lat_ctrl_eps1ctl_adu_kb  #转向机信号，无实意
float64 lat_ctrl_lateral_acceleration  #转向机信号，无实意
float64 lat_ctrl_target_steer_angle  #目标方向盘转角
uint8 lat_ctrl_control_demand  #转向机控制使能
uint8 lat_ctrl_control_mode_angle_demand  #转向机角度控制模式使能
int64 time_pub
int64 time_meas
int8 channel_id
int32 can_id
uint8[8] data   #发送到CAN总线上的原始数据

