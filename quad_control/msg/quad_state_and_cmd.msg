# is the body found by the mocap system
bool found_body

#-------------------------#
# time
float64 time

#-------------------------#
# State of the Quad

# linear position
float64 x
float64 y
float64 z

# linear velocity
float64 vx
float64 vy
float64 vz

# angular position
float64 roll
float64 pitch
float64 yaw

#-------------------------#
# Desired trajectory: this will need to be comunicated to the GUI

# linear position
float64 xd
float64 yd
float64 zd

# linear velocity
float64 vxd
float64 vyd
float64 vzd

# angular position
float64 roll_d
float64 pitch_d
float64 yaw_d


#-------------------------#
# Commands/Channels

float64 cmd_1
float64 cmd_2
float64 cmd_3
float64 cmd_4
float64 cmd_5
float64 cmd_6
float64 cmd_7
float64 cmd_8
