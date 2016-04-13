# QuadPositionDerived Message
# Position, velocity and acceleration information of the quad

# is the body found by the mocap system
bool found_body

# Give the position of the quad in the Qualisys frame
float64 x
float64 y
float64 z
float64 pitch
float64 roll
float64 yaw

# Velocity
float64 x_vel
float64 y_vel
float64 z_vel
float64 pitch_vel
float64 roll_vel
float64 yaw_vel

# Acceleration
float64 x_acc
float64 y_acc
float64 z_acc
float64 pitch_acc
float64 roll_acc
float64 yaw_acc

# Time difference between 2 measurement
float64 time_diff
