import numpy as np
import math
import rtde_control
import rtde_receive
from comms_wrapper import *
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
grip_dy = 96.58142 / 1000  # meters

grab_height_offset = 0.075 #grab height offset to cup from origin_L

# best if I get it from Joint Position
origin_L = [0.305, 0.491, 0.31504431455331383, -3.13713023885791, 0.08284771453405795,
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


def grab_cup(cX, cY):
    temp_L = copy.copy(origin_L)

    print("Grabbing Cup")
    temp_L[0] = cX
    print(temp_L[0])
    temp_L[1] = cY - grip_dy
    print(temp_L[1])

    print("move1")
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    print("go down")
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
    print(temp_L[0])
    temp_L[1] = cY
    print(temp_L[1])

    print("move1")
    rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

    
    # Get current Joint position in Joint space
    center_J = rtde_r.getActualQ()
    #have a 90 degree rotation
    center_J[5] -= math.pi/2

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
