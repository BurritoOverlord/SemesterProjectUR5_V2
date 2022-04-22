import math

import rtde_receive

# Setup robot with robot IP address
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.20")

Joint_positions = rtde_r.getActualQ()
print ('Joint positions are: ', Joint_positions, end = '\n')
print(Joint_positions[5])

Joint_positions[5] -= math.pi/2
print(Joint_positions)

# rtde_c.moveJ(np.deg2rad(center_J), VELOCITY, ACCELERATION)

Cartesian_positions = rtde_r.getActualTCPPose()
print ('Cartesian positions are: ', Cartesian_positions, end = '\n')


