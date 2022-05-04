import copy

import numpy as np
import math
import rtde_control
import rtde_receive
from comms_wrapper import *

#my
import PathPlanning.geometry as geometry
"SETTINGS AND VARIABLES ________________________________________________________________"

# Setup robot with robot IP address
rtde_c = rtde_control.RTDEControlInterface("192.168.1.20")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.20")

# Setup Gripper
arduino = Arduino(descriptiveDeviceName="ARD", portName="COM5", baudrate=115200)
# Connects with the Arduino Nano and does the handshake to start void loop()
arduino.connect_and_handshake()

ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

# Gripper Offset parameter - To change if we have new gripper
grip_dz = 168.4405 / 1000  # meters
grip_dy = 0.12  # meters

grab_height_offset = 0.075 #grab height offset to cup from origin_L

# best if I get it from Joint Position
origin_L = [0.305, 0.506, 0.31504431455331383+2*grab_height_offset, -3.13713023885791, 0.08284771453405795,
            -0.009878696005977336]


pouring_1_J = [-15.0, -14.38000000000001, 32.89, -29.959999999999994, 45.519999999999996, -14.489999999999995]
"""
test-1
[-15.0, -14.38000000000001, 32.89, -29.959999999999994, 45.519999999999996, -14.489999999999995]
test-2
[-17.25, -0.5999999999999943, 16.97, -10.810000000000002, 44.519999999999996, 0.010000000000005116]
test-3
[-6.060000000000002, 6.510000000000005, 12.36, -15.099999999999994, 31.15, -7.560000000000002]
test-4
[-0.410000000000025, 3.0, 2.5500000000000114, -4.710000000000001, 14.120000000000001, -0.5100000000000051]

cup_mix_pos_J = [-291.43, -85.40, -124.83, -59.52, 90.05, -18.35]
"""


def initialize_robot():
    # initialise robot with URBasic
    home_J = [-298.22, -66.90, -113.00, -89.74, 90.11, -24.63]
    # rtde_c.moveJ(np.deg2rad(home_J), VELOCITY, ACCELERATION)

    rtde_c.moveJ(np.deg2rad(home_J), VELOCITY, ACCELERATION)

    time.sleep(1)  # just a short wait to make sure everything is initialised

    # test gripper
    print("Testing Gripper")
    arduino.send_message("grab")
    time.sleep(3)
    arduino.send_message("release")
    time.sleep(3)


def grab_cup(cX, cY, angle):

    temp_L = copy.copy(origin_L)

    #####################################################################
    # Go to Desired Position
    #print("Go to desired position")

    pX = copy.copy(cX)
    pY = copy.copy(cY) - grip_dy

    pX, pY = geometry.rotate(cX, cY, pX, pY, angle)


    temp_L[0] = pX
    temp_L[1] = pY

    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    #####################################################################
    # Rotate Wrist Correspondingly


    # Get current Joint position in Joint space
    center_J = rtde_r.getActualQ()
    #Get desired angle
    rot = geometry.get_angle((pX, pY), (cX , cY))
    print(rot)

    print(center_J[5])

    center_J[5] -= rot
    rtde_c.moveJ(center_J)

    Cartesian_positions = rtde_r.getActualTCPPose()
    temp_L = copy.copy(Cartesian_positions)
    # go down and grab and go back up
    temp_L[2] = origin_L[2] - 3 * grab_height_offset #was calculated by hand
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)
    arduino.send_message("grab")
    time.sleep(7)

    # go down and grab and go back up
    temp_L[2] = origin_L[2]
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    return

def place_cup(cX, cY):
    temp_L = copy.copy(origin_L)

    #print("place Cup")
    temp_L[0] = cX + grip_dy
    temp_L[1] = cY

    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    
    # Get current Joint position in Joint space
    center_J = rtde_r.getActualQ()
    #have a 90 degree rotation
    if center_J[5] > 0:
        center_J[5] -= math.pi/2
    else:
        center_J[5] += 3* math.pi/2

    rtde_c.moveJ(center_J)

    # go down and grab and go back up
    temp_L = rtde_r.getActualTCPPose()
    temp_L[2] = origin_L[2] - 3 * grab_height_offset
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)
    arduino.send_message("release")
    time.sleep(7)

    # go down and grab and go back up
    temp_L[2] = origin_L[2]
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    return


def place_cupP(cX, cY):
    temp_L = copy.copy(origin_L)

    #print("place Cup")
    temp_L[0] = cX
    temp_L[1] = cY - grip_dy

    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    # go down and grab and go back up
    temp_L = rtde_r.getActualTCPPose()
    temp_L[2] = origin_L[2] - 3 * grab_height_offset
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)
    arduino.send_message("release")
    time.sleep(7)

    # go down and grab and go back up
    temp_L[2] = origin_L[2]
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    return


def grab_cupP(cX, cY, angle):

    temp_L = copy.copy(origin_L)

    #####################################################################
    # Go to Desired Position
    #print("Go to desired position")

    pX = copy.copy(cX)
    pY = copy.copy(cY) - grip_dy

    pX, pY = geometry.rotate(cX, cY, pX, pY, angle)


    temp_L[0] = pX
    temp_L[1] = pY

    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    #####################################################################
    # Rotate Wrist Correspondingly
    # Get current Joint position in Joint space
    center_J = rtde_r.getActualQ()
    #Get desired angle
    rot = angle
    print(rot)

    print(center_J[5])

    center_J[5] -= rot
    rtde_c.moveJ(center_J)

    Cartesian_positions = rtde_r.getActualTCPPose()
    temp_L = copy.copy(Cartesian_positions)
    # go down and grab and go back up
    temp_L[2] = origin_L[2] - 3 * grab_height_offset #was calculated by hand
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)
    arduino.send_message("grab")
    time.sleep(7)

    # go down and grab and go back up
    temp_L[2] = origin_L[2]
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    return

def pouring(cX, cY):
    temp_L = copy.copy(origin_L)

    temp_L[0] = cX + grip_dy
    temp_L[1] = cY

    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)
    Joint_positions = rtde_r.getActualQ()
    temp_L = copy.copy(Joint_positions)

    for i in range(len(temp_L)):
        temp_L[i] += pouring_1_J
    rtde_c.moveJ(np.deg2rad(pouring_1_J))
    return

def stop_script():
    # Stop the RTDE control script
    rtde_c.stopScript()
    return