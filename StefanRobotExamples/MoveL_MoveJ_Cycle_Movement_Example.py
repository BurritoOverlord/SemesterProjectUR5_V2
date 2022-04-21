import rtde_control, rtde_receive
import numpy as np
import time

# Setup robot with robot IP address
rtde_c = rtde_control.RTDEControlInterface("192.168.1.20")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.20")

# Positions
home_J = [-298.22, -66.90, -113.00, -89.74, 90.11, -24.63]
position1_J = [-267.76, -90.89, -106.61, -72.35, 89.95, 5.21]


# Speeds and accelerations - Joint space
speed_J_fast = 0.7
acc_J_fast = 1
# Speeds and accelerations - Cartesian space
speed_L_fast = 0.5
acc_L_fast = 1

# Move robot between positions in Joint space

# You can define position, speed and acceleration
rtde_c.moveJ(np.deg2rad(home_J), speed_J_fast, acc_J_fast)
# You can define only position, default speed and acceleration will be applied
rtde_c.moveJ(np.deg2rad(position1_J))

# Get current position in Cartesian space
center_L = rtde_r.getActualTCPPose()

# Copy current position to other variables
position_1 = np.copy(center_L)
position_2 = np.copy(center_L)
position_3 = np.copy(center_L)
position_4 = np.copy(center_L)
position_5 = np.copy(center_L)
position_6 = np.copy(center_L)

# Increment variable
increment = 0.05
# Increment other variables
position_1 [0] -= increment
position_2 [0] += increment
position_4 [1] -= increment
position_3 [1] += increment
position_5 [2] -= increment
position_6 [2] += increment

# Cycle between these positions as blocking movement
for i in range (1, 3):
    rtde_c.moveL(center_L, speed_L_fast)
    rtde_c.moveL(position_1, speed_L_fast)
    rtde_c.moveL(position_2, speed_L_fast)
    rtde_c.moveL(position_3, speed_L_fast)
    rtde_c.moveL(position_4, speed_L_fast)
    rtde_c.moveL(position_5, speed_L_fast)
    rtde_c.moveL(position_6, speed_L_fast)

# Cycle between these positions as non-blocking movement
# If there is no time.sleep, robot will move directly from center_L to position_6
# time.sleep is essentially time required for robot to move between different positions
for i in range (1, 3):
    rtde_c.moveL(center_L, speed_L_fast, True)
    time.sleep(0.2)
    rtde_c.moveL(position_1, speed_L_fast, True)
    time.sleep(0.2)
    rtde_c.moveL(position_2, speed_L_fast, True)
    time.sleep(0.2)
    rtde_c.moveL(position_3, speed_L_fast, True)
    time.sleep(0.2)
    rtde_c.moveL(position_4, speed_L_fast, True)
    time.sleep(0.2)
    rtde_c.moveL(position_5, speed_L_fast, True)
    time.sleep(0.2)
    rtde_c.moveL(position_6, speed_L_fast, True)

# Stop the RTDE control script
rtde_c.stopScript()