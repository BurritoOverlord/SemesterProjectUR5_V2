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

pouring_dx = 0.08

grab_height_offset = 0.075 #grab height offset to cup from origin_L

# best if I get it from Joint Position
origin_L = [0.305, 0.506, 0.31504431455331383+2*grab_height_offset, -3.13713023885791, 0.08284771453405795,
            -0.009878696005977336]


pouring_1_L = [0.1329460444039362, 0.001992027271668584, 0.028232181258689593, 0.4172834491949249, 0.5468231534718957, -0.7511273037641358]
pouring_2_L = [-0.1646497520298684, 0.017626478234580634, -0.056484304018509124, -0.7349117459413628, -0.3580076965193224, 0.4108333718214877]
pouring_3_L = [-0.07186949898256362, -0.023014226767514745, -0.08788628407032015, -0.352989202528292, -0.4261227544800278, 0.33786829085844206]
pouring_4_L = [-0.014978186806280838, -0.010535790024139513, -0.02171575440950893, -0.17231163199225163, -0.20942507249648468, 0.07274833908588763]


"""
test-0
[-3.7720707644628426e-06, -4.971880862347788e-06, 2.472595297670077e-05, 4.4505749226431135e-05, -8.457623627222688e-06, 6.690946141478353e-05]
test-1
[0.1329460444039362, 0.001992027271668584, 0.028232181258689593, 0.4172834491949249, 0.5468231534718957, -0.7511273037641358]
test-2
[-0.1646497520298684, 0.017626478234580634, -0.056484304018509124, -0.7349117459413628, -0.3580076965193224, 0.4108333718214877]
test-3
[-0.07186949898256362, -0.023014226767514745, -0.08788628407032015, -0.352989202528292, -0.4261227544800278, 0.33786829085844206]
test-4
[-0.014978186806280838, -0.010535790024139513, -0.02171575440950893, -0.17231163199225163, -0.20942507249648468, 0.07274833908588763]
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

    # print("place Cup")
    temp_L[0] = cX + grip_dy + pouring_dx
    temp_L[1] = cY

    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    # Get current Joint position in Joint space
    center_J = rtde_r.getActualQ()
    # have a 90 degree rotation
    if center_J[5] > 0:
        center_J[5] -= math.pi / 2
    else:
        center_J[5] += 3 * math.pi / 2

    rtde_c.moveJ(center_J)

    Cartesian_positions = rtde_r.getActualTCPPose()
    temp_L = copy.copy(Cartesian_positions)

    #Move to Pouring Position 1
    for i in range(len(temp_L)):
        temp_L[i] -= pouring_1_L[i]
    print(temp_L)
    rtde_c.moveL(temp_L)

    #Move to Pouring Position 2
    for i in range(len(temp_L)):
        temp_L[i] += pouring_2_L[i]
    print(temp_L)
    rtde_c.moveL(temp_L)

    #Move to Pouring Position 3
    for i in range(len(temp_L)):
        temp_L[i] += pouring_3_L[i]
    print(temp_L)
    rtde_c.moveL(temp_L)

    #Move to Pouring Position 4
    for i in range(len(temp_L)):
        temp_L[i] += pouring_4_L[i]
    print(temp_L)
    rtde_c.moveL(temp_L)
    return

def stop_script():
    # Stop the RTDE control script
    rtde_c.stopScript()
    return