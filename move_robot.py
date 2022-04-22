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
origin_L = [0.305, 0.506, 0.31504431455331383, -3.13713023885791, 0.08284771453405795,
            -0.009878696005977336]


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
    print("Go to desired position")

    pX = copy.copy(cX)
    pY = copy.copy(cY) - grip_dy

    print(pY)

    pX, pY = geometry.rotate(cX, cY, pX, pY, angle)

    print(cY)
    print(pY)

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
    print("rotation")

    center_J[5] -= rot
    rtde_c.moveJ(center_J)

    print("go down")
    Cartesian_positions = rtde_r.getActualTCPPose()
    temp_L = copy.copy(Cartesian_positions)
    # go down and grab and go back up
    temp_L[2] = origin_L[2] - grab_height_offset
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)
    arduino.send_message("grab")
    time.sleep(7)

    print("go up")
    # go down and grab and go back up
    temp_L[2] = origin_L[2] + 2*grab_height_offset
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)



    return

def place_cup(cX, cY):
    temp_L = copy.copy(origin_L)

    print("place Cup")
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


    print("go down")
    # go down and grab and go back up
    temp_L = rtde_r.getActualTCPPose()
    temp_L[2] = origin_L[2] - grab_height_offset
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)
    arduino.send_message("release")
    time.sleep(7)

    print("go up")
    # go down and grab and go back up
    temp_L[2] = origin_L[2]
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)


    return

def stop_script():
    # Stop the RTDE control script
    rtde_c.stopScript()
    return