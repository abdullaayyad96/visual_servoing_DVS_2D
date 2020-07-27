std_msgs/Header header

# Grip force 10,50,100 percent
string condition

# object material plastic or metal
string object

# Detector type Harris or Canny
string detector

# Frame in millisec 5,10,15
uint16 frame_ms

# F/T  sensor data
geometry_msgs/WrenchStamped ft_raw_data
geometry_msgs/WrenchStamped f_max_data
geometry_msgs/WrenchStamped t_max_data
baxter_core_msgs/EndEffectorState state_data


# Gripper sensor data 0 (closed) -100 (opened)
float32 gripper_percent

# raw noise
uint16 sample_start
float32 raw_max
float32 raw_max_P
float32 raw_max_N
float32 flat_raw_max
float32 flat_raw_max_P
float32 flat_raw_max_N
float32 edge_raw_max
float32 edge_raw_max_P
float32 edge_raw_max_N
float32 corner_raw_max
float32 corner_raw_max_P
float32 corner_raw_max_N
uint16 sample_stop

# Gripper sensor data 0 (closed) -100 (opened)
uint16 Gripper_start
string Gripper_status 
float32 Gripper_percent
float32 Gripper_force
float32 GR_count
float32 GR_count_raw_frame
float32 GR_count_raw_positive
float32 GR_count_raw_negative
float32 GR_count_flat_frame
float32 GR_count_flat_positive
float32 GR_count_flat_negative
float32 GR_count_edge_frame
float32 GR_count_edge_positive
float32 GR_count_edge_negative
float32 GR_count_corner_frame
float32 GR_count_corner_positive
float32 GR_count_corner_negative
uint16 Gripper_stop
float32 GR_duration_SS



# slippage data for millisec frame
uint16 slip_monitor
uint16 slip_start
float32 slip_count
float32 slip_count_total

float32 slip_count_raw_frame
float32 slip_count_raw_positive
float32 slip_count_raw_negative
float32 slip_count_flat_frame
float32 slip_count_flat_positive
float32 slip_count_flat_negative
float32 slip_count_edge_frame
float32 slip_count_edge_positive
float32 slip_count_edge_negative
float32 slip_count_corner_frame
float32 slip_count_corner_positive
float32 slip_count_corner_negative

float32 slip_total_events_noise
float32 slip_total_events_edge
float32 slip_total_events_corner
uint16 slip_stop
float32 slip_duration_SS
uint16 Exp_count

