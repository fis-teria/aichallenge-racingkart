# One bounded, controller-neutral sample from the controller's shadow rollout.
# All fields use SI units. This is evidence only and grants no motion authority.
float32 elapsed_time_sec
float32 progress_m
geometry_msgs/Pose pose
float32 speed_mps
float32 raw_steering_tire_angle_rad
float32 bounded_steering_tire_angle_rad
float32 raw_steering_tire_rotation_rate_radps
float32 bounded_steering_tire_rotation_rate_radps
