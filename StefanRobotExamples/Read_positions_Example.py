import math
import rtde_control
import rtde_receive
import numpy as np

# Setup robot with robot IP address
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.20")
# Setup robot with robot IP address
rtde_c = rtde_control.RTDEControlInterface("192.168.1.20")

Joint_positions = rtde_r.getActualQ()
print ('Joint positions are: ', Joint_positions, end = '\n')
print(Joint_positions[5])

Joint_positions[5] -= math.pi/2
print(Joint_positions)

test = []
# rtde_c.moveJ(np.deg2rad(center_J), VELOCITY, ACCELERATION)

Cartesian_positions = rtde_r.getActualTCPPose()
print ('Cartesian positions are: ', Cartesian_positions, end = '\n')

cup_mix_pos_J = [-291.43, -85.40, -124.83, -59.52, 90.05, -18.35]

pouring_start_J = [-297.91, -126.84, -45.16, -97.71, 89.99, -118.10]

print("test-0")
for i in range(len(pouring_start_J )):
    test.append(cup_mix_pos_J[i]-pouring_start_J[i])
print(test)

rtde_c.moveJ(np.deg2rad(pouring_start_J))
pouring_1_J = [-282.91, -112.46, -78.05, -67.75, 44.47, -103.61]

print("test-1")
for i in range(len(pouring_start_J )):
    test[i]= pouring_start_J[i]-pouring_1_J[i]
print(test)


rtde_c.moveJ(np.deg2rad(pouring_1_J))
pouring_2_J = [-265.66, -111.86, -95.02, -56.94, -0.05, -103.62]

print("test-2")
for i in range(len(pouring_start_J )):
    test[i]= pouring_1_J[i]-pouring_2_J[i]
print(test)
rtde_c.moveJ(np.deg2rad(pouring_2_J))
pouring_3_J = [-259.60, -118.37, -107.38, -41.84, -31.20, -96.06]

print("test-3")
for i in range(len(pouring_start_J )):
    test[i]= pouring_2_J[i]-pouring_3_J[i]
print(test)

rtde_c.moveJ(np.deg2rad(pouring_3_J))
pouring_4_J = [-259.19, -121.37, -109.93, -37.13, -45.32, -95.55]

print("test-4")
for i in range(len(pouring_start_J )):
    test[i]= pouring_3_J[i]-pouring_4_J[i]
print(test)
rtde_c.moveJ(np.deg2rad(pouring_4_J))

cup_mix_pos_J = [-291.43, -85.40, -124.83, -59.52, 90.05, -18.35]
rtde_c.moveJ(np.deg2rad(cup_mix_pos_J ))