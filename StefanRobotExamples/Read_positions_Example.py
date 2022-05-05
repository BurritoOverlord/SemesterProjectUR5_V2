import copy
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


"""
test_0 = [6.480000000000018, 41.44, -79.67, 38.18999999999999, 0.060000000000002274, 99.75]
test-1 = [-15.0, -14.38000000000001, 32.89, -29.959999999999994, 45.519999999999996, -14.489999999999995]
test-2 = [-17.25, -0.5999999999999943, 16.97, -10.810000000000002, 44.519999999999996, 0.010000000000005116]
test-3 = [-6.060000000000002, 6.510000000000005, 12.36, -15.099999999999994, 31.15, -7.560000000000002]
test-4 = [-0.410000000000025, 3.0, 2.5500000000000114, -4.710000000000001, 14.120000000000001, -0.5100000000000051]
"""

test = []
# rtde_c.moveJ(np.deg2rad(center_J), VELOCITY, ACCELERATION)

Cartesian_positions = rtde_r.getActualTCPPose()
Cartesian_positionsS = copy.copy(Cartesian_positions)
print ('Cartesian positions are: ', Cartesian_positions, end = '\n')

cup_mix_pos_J = [-291.43, -85.40, -124.83, -59.52, 90.05, -18.35]
pouring_start_J = [-297.91, -126.84, -45.16, -97.71, 89.99, -118.10]

rtde_c.moveJ(np.deg2rad(pouring_start_J))

Cartesian_positions = rtde_r.getActualTCPPose()
Cartesian_positions0 = copy.copy(Cartesian_positions)

print("test-0")
for i in range(len(Cartesian_positions0)):
    test.append(Cartesian_positionsS[i]-Cartesian_positions0[i])
print(test)

pouring_1_J = [-282.91, -112.46, -78.05, -67.75, 44.47, -103.61]

rtde_c.moveJ(np.deg2rad(pouring_1_J))

Cartesian_positions = rtde_r.getActualTCPPose()
Cartesian_positions1 = copy.copy(Cartesian_positions)

print("test-1")
for i in range(len(Cartesian_positions1)):
    test[i]= Cartesian_positions0[i]-Cartesian_positions1[i]
print(test)



pouring_2_J = [-265.66, -111.86, -95.02, -56.94, -0.05, -103.62]

rtde_c.moveJ(np.deg2rad(pouring_2_J))

Cartesian_positions = rtde_r.getActualTCPPose()
Cartesian_positions2 = copy.copy(Cartesian_positions)

print("test-2")
for i in range(len(Cartesian_positions2)):
    test[i]= Cartesian_positions2[i]-Cartesian_positions1[i]
print(test)

pouring_3_J = [-259.60, -118.37, -107.38, -41.84, -31.20, -96.06]

rtde_c.moveJ(np.deg2rad(pouring_3_J))

Cartesian_positions = rtde_r.getActualTCPPose()
Cartesian_positions3 = copy.copy(Cartesian_positions)

print("test-3")
for i in range(len(Cartesian_positions2)):
    test[i] = Cartesian_positions3[i]-Cartesian_positions2[i]
print(test)



pouring_4_J = [-259.19, -121.37, -109.93, -37.13, -45.32, -95.55]

rtde_c.moveJ(np.deg2rad(pouring_4_J))

Cartesian_positions = rtde_r.getActualTCPPose()
Cartesian_positions4 = copy.copy(Cartesian_positions)

print("test-4")
for i in range(len(Cartesian_positions2)):
    test[i] = Cartesian_positions4[i]-Cartesian_positions3[i]
print(test)

cup_mix_pos_J = [-291.43, -85.40, -124.83, -59.52, 90.05, -18.35]
