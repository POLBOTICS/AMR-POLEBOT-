# Robot overall status message

std_msgs/Header header

# Operational mode
uint8 MODE_IDLE     = 0
uint8 MODE_MANUAL   = 1
uint8 MODE_AUTO     = 2
uint8 MODE_CHARGING = 3
uint8 MODE_ERROR    = 4
uint8 mode

# Battery
float32 battery_voltage
float32 battery_percentage
bool is_charging

# Pose summary
geometry_msgs/Pose2D pose_2d

# Navigation status
bool is_navigating
string current_goal_id

# System health
bool lidar_ok
bool imu_ok
bool motor_ok
bool camera_ok
