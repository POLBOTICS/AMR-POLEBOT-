# SensorData aggregated message

std_msgs/Header header

# IMU simplified
float64 roll
float64 pitch
float64 yaw

# Range sensors (ultrasonic, if any)
float32[] range_readings
string[]  range_frame_ids

# Ambient environment
float32 ambient_temperature
float32 ambient_humidity
