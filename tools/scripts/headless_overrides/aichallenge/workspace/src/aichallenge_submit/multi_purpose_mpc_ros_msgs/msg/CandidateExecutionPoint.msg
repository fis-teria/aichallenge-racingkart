# Bounded, header-free copy of every field that is actually delivered in an
# autoware_auto_planning_msgs/TrajectoryPoint. CandidateExecutionRequest owns
# the common plan stamp and frame.
builtin_interfaces/Duration time_from_start
float64 position_x_m
float64 position_y_m
float64 position_z_m
float64 orientation_x
float64 orientation_y
float64 orientation_z
float64 orientation_w
float32 longitudinal_velocity_mps
float32 lateral_velocity_mps
float32 acceleration_mps2
float32 heading_rate_rps
float32 front_wheel_angle_rad
float32 rear_wheel_angle_rad
