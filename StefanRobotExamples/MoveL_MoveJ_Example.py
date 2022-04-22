import copy

import rtde_control, rtde_receive
import numpy as np
import math

# Setup robot with robot IP address
rtde_c = rtde_control.RTDEControlInterface("192.168.1.20")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.20")

# Positions
home_J = [-298.22, -66.90, -113.00, -89.74, 90.11, -24.63]
position1_J = [-267.76, -90.89, -106.61, -72.35, 89.95, 5.21]
position2_J = [-242.44, -48.15, -133.21, -88.66, 89.90, -59.50]


# Speeds and accelerations - Joint space
speed_J_fast = 0.7
acc_J_fast = 1
# Speeds and accelerations - Cartesian space
speed_L_fast = 0.5
acc_L_fast = 1


# Move robot between positions in Joint space

# You can define position, speed and acceleration
print("1")
rtde_c.moveJ(np.deg2rad(home_J), speed_J_fast, acc_J_fast)
# You can define only position, default speed and acceleration will be applied
print("2")
rtde_c.moveJ(np.deg2rad(position1_J))
# You can define position and speed, default acceleration will be applied
print("3")
rtde_c.moveJ(np.deg2rad(position2_J), speed_J_fast)
# You can define whether movement will be blocking or non-blocking
# Default is False - blocking movement, True is non-blocking movement
print("2")
rtde_c.moveJ(np.deg2rad(position1_J), speed_J_fast, acc_J_fast, False)

"""

# Get current position in Cartesian space
temp_L = rtde_r.getActualTCPPose()
# Move robot along Y axis for 100 mm, and along Z axis for 50 mm
temp_L[1] += 0.1
temp_L [2] += 0.05
rtde_c.moveL(temp_L, speed_J_fast, acc_L_fast)
# Move robot along along Z axis for 50 mm, non-blocking movement
# Default is False - blocking movement, True is non-blocking movement
temp_L [2] += 0.05
rtde_c.moveL(temp_L, speed_J_fast, acc_L_fast, True)
"""

Joint_positions = rtde_r.getActualQ()
print('Joint positions are: ', Joint_positions, end = '\n')
displacement = copy.copy(Joint_positions)

displacement[5] -= math.pi/2
print(Joint_positions)

rtde_c.moveJ(displacement)




# Stop the RTDE control script
rtde_c.stopScript()