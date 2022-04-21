import numpy as np
import rtde_control
import rtde_receive
from comms_wrapper import *

# Setup robot with robot IP address
rtde_c = rtde_control.RTDEControlInterface("192.168.1.20")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.20")


# Setup Gripper
arduino = Arduino(descriptiveDeviceName="ARD", portName="COM5", baudrate=115200)
# Connects with the Arduino Nano and does the handshake to start void loop()
arduino.connect_and_handshake()
time.sleep(0.2)

# test gripper
arduino.send_message("grab")
time.sleep(5)
#arduino.send_message("release")
