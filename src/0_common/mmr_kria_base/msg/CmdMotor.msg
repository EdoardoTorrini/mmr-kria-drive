# Standard ROS 2 message header
std_msgs/Header header

# set the motor in enable
bool enable

# make the homing mode in the motor
bool homing

# target torque for the brake - leave empty if the cmd is for steer or clutch
float64 brake_torque

# target position for the steer and clutch - leave empty if the cmd is for brake
float64 wheel_angle

# target status for the clutch - leave empty if the cmd is for brake and steer
bool disengaged

bool disable

