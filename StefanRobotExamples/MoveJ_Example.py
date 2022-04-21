import rtde_control
import numpy as np

# Setup robot with robot IP address
rtde_c = rtde_control.RTDEControlInterface("192.168.1.20")

# Positions
home_J = [-298.22, -66.90, -113.00, -89.74, 90.11, -24.63]
position1_J = [-267.76, -90.89, -106.61, -72.35, 89.95, 5.21]
position2_J = [-294.59, -75.76, -110.12, -83.88, 90.08, 68.98]

# Speeds and accelerations
speed_J_fast = 0.7
acc_J_fast = 1

# Move robot between positions

# You can define position, speed and acceleration
rtde_c.moveJ(np.deg2rad(home_J), speed_J_fast, acc_J_fast)
# You can define only position, default speed and acceleration will be applied
rtde_c.moveJ(np.deg2rad(position1_J))
# You can define position and speed, default acceleration will be applied
rtde_c.moveJ(np.deg2rad(position2_J), speed_J_fast)
# You can define whether movement will be blocking or non-blocking
# Default is False - blocking movement, True is non-blocking movement
rtde_c.moveJ(np.deg2rad(home_J), speed_J_fast, acc_J_fast, True)

# Stop the RTDE control script
rtde_c.stopScript()