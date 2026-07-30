# Motor status message
# Published by the motor driver hardware interface

std_msgs/Header header

# Left/Right wheel velocities (rad/s)
float64 left_velocity
float64 right_velocity

# Left/Right wheel encoder ticks
int32 left_encoder
int32 right_encoder

# Motor current draw (Ampere)
float32 left_current
float32 right_current

# Motor temperature (Celsius)
float32 left_temperature
float32 right_temperature

# Error flags
bool left_fault
bool right_fault
