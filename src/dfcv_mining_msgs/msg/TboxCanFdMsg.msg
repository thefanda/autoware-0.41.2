#TboxCanFdEx.msg

std_msgs/Header header

uint8 driving_mode_set

uint8 move_packets_num
uint8 move_packets_no
uint64 move_task_no
uint8 move_task_type
uint8 move_task_operation
uint16 move_limit_speed
float64 move_start_x
float64 move_start_y
float64 move_start_z
float64 move_end_x
float64 move_end_y
float64 move_end_z

uint8 move_id_packets_num
uint8 move_id_packets_no
uint64 move_id_task_no
uint8 move_id_task_type
uint16 move_id_path_id_length
uint32[] move_id_pass_path_id

uint64 up_ctl_task_no
uint8 up_ctl_task_type
uint8 up_ctl_task_operation

uint64 joint_task_no
uint8 joint_tasktype
uint8 joint_slow_guide
uint16 joint_joint_point
uint8 joint_joint_method

uint16 step_switch_task_no
uint8 step_switch_tasktype
uint8 step_switch_task_operation

uint16 step_move_task_no
uint8 step_move_tasktype
uint16 step_move_length

bool has_t_box_ad_mode_set
bool has_t_box_move
bool has_t_box_joint
bool has_t_box_drop
bool has_t_box_step_switch
bool has_t_box_step_move
bool plat_ctrl



