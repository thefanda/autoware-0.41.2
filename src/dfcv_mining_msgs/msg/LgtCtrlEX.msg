std_msgs/Header header

uint8 lgt_ctrl_creep_complete
uint8 lgt_ctrl_control_mode_manage
uint8 lgt_ctrl_lift_complete
float64 lgt_ctrl_acceleration_calculate
uint8 lgt_ctrl_digns_pcan_tm_err   #pcan诊断错误信号
uint8 lgt_ctrl_digns_ccan_tm_err   #ccan诊断错误信号
uint8 lgt_ctrl_digns_xbr_err    #XBR诊断错误信号
uint8 lgt_ctrl_digns_epb_err   #EPB诊断错误信号
uint8 lgt_ctrl_digns_tcu_err   #TCU诊断错误信号

int64 time_pub
int64 time_meas
int8 channel_id
int32 can_id
uint8[8] data
