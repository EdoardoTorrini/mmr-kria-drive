# Standard ROS 2 message header
std_msgs/Header header

# Actual Vehicle speed. The field represents the combination of speed values given by GNSS, RPM and WHEEL sensors (m/s)
float64 actual_speed

# Target vehicle speed. The field represents the desired velocity at the next iteration (m/s)
float64 target_speed

# Actual Status of the actuators. The field represents the actual status of the actuators
ActuatorsStatus actual_actuators_state

# Actual Status of the actuators. The field represents the desired status for the actuators at the next iteration
ActuatorsStatus target_actuators_state

# Selected mission identifier
uint8 mission_id

# Actual Gear
uint8 actual_gear

# Actual vehicle pose (translation + orientation)
geometry_msgs/Pose actual_pose