import numpy as np
import rtde_control
import rtde_receive
import cv2
import cv2.aruco as aruco
import imutils
from imutils.video import VideoStream
from comms_wrapper import *

"SETTINGS AND VARIABLES ________________________________________________________________"

# Total number of Cups to be detected ( The marker ID's have to start at 4 and go incrementally
nb_Cups = 4

# Setup robot with robot IP address
rtde_c = rtde_control.RTDEControlInterface("192.168.1.20")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.20")

# Setup Gripper
arduino = Arduino(descriptiveDeviceName="ARD", portName="COM5", baudrate=115200)
# Connects with the Arduino Nano and does the handshake to start void loop()
arduino.connect_and_handshake()

# The Joint position the robot starts at
robot_start_positionJ = [-298.22, -66.90, -113.00, -89.74, 90.11, -24.63]

ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

video_resolution = (640, 480)  # resolution the video capture will be resized to, smaller sizes can speed up detection

# Gripper Offset parameter - To change if we have new gripper
grip_dz = 168.4405/1000  # meters
grip_dy = 96.58142/1000  # meters

# Environment width and height
r_width = 290
r_height = 172

# Calibration Parameters
calibrationMatrix = np.load('Cam_Calibration/calibration_matrix.npy')
distortionCoefficient = np.load('Cam_Calibration/distortion_coefficients.npy')

vs = VideoStream(src= 1 ,
                 resolution=video_resolution,
                 framerate = 13,
                 meter_mode = "backlit",
                 exposure_mode ="auto",
                 shutter_speed = 8900,
                 exposure_compensation = 2,
                 rotation = 0).start()
time.sleep(0.2)

"""FUNCTIONS _____________________________________________________________________________"""

def findArUcoMarkers(frame, markerSize=4, totalMarkers=50, draw=True):

    #start by converting to gray scale
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.getPredefinedDictionary(key)
    arucoParam = aruco.DetectorParameters_create()
    bbox, ids, rejected = aruco.detectMarkers(img_gray, arucoDict, parameters=arucoParam,
                                              cameraMatrix=calibrationMatrix, distCoeff=distortionCoefficient)

    # Store the "center coordinates" of all marker in detected objects
    # in order from the upper left in the clockwise direction.
    detected_markers = list()  # Storage destination of center coordinates for all detected markers with their ID
    field_corners = np.empty((4, 2))  # detect the corner values of the "play field" ->marker ID: 0-3
    # verify *at least* one ArUco marker was detected
    if len(bbox) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(bbox, ids):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            if markerID == 0:
                field_corners[markerID] = bottomRight
            if markerID == 1:
                field_corners[markerID] = bottomLeft
            if markerID == 2:
                field_corners[markerID] = topLeft
            if markerID == 3:
                field_corners[markerID] = topRight

            detected_markers.append(dict(id=markerID, cx=cX, cy=cY, pose=topLeft))

            """
            if draw:
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            """

    if draw:
        # Draw bbox
        aruco.drawDetectedMarkers(frame, bbox, ids)

    return [detected_markers, field_corners]

def getRealCoordinates(frame, field_corners, detected_markers, p_width=video_resolution[0],
                       p_height=video_resolution[1]):

    #resize frame into the "playing field" coordinate space
    #width, height = (500, 500)  # Image size after transformation in pixels
    field_corners_vect = np.float32([field_corners[0],field_corners[1],field_corners[2],field_corners[3]])
    true_coordinates = np.float32([[0, 0], [p_width, 0], [p_width, p_height], [0, p_height]])
    trans_mat = cv2.getPerspectiveTransform(field_corners_vect, true_coordinates)

    #show playing field
    img_trans = cv2.warpPerspective(frame, trans_mat, (p_width, p_height))
    cv2.imshow("new frame", img_trans)

    #print(detected_markers)
    ##########################################################################q##################
    ############################################################################################
    detected_true_coordinates = np.empty((len(detected_markers),3)) #create matrix of coordinates
    for i, obj in enumerate(detected_markers, 0):
        # apply the transform matrix to each vector of center coordinates
        detected_true_coordinates[i] = np.dot(trans_mat, (obj["cx"], obj["cy"], 1))
        #change back the values in the detected markers list to have the real coordinates
        obj["cx"] = detected_true_coordinates[i][0] * r_width / p_width
        obj["cy"] = detected_true_coordinates[i][1] * r_height / p_height

        #Print real coordinates to calibrate the offset manually
        """
        if obj["id"] > 3:
            print(f'■ REAL COORDINATES: Detected markerID{obj["id"]:>3.0f} Center position X={obj["cx"]:>3.0f}mm'
                  f' Y={obj["cy"]:>3.0f}mm ')
        """
    return detected_markers

def move_to_markers(detected_markers):


    # Specify which marker will induce a movement and store the coordinates in a table
    nb_objects = len(detected_markers) - 4  # - the four corner markers
    print(nb_objects)

    id_table = np.empty(nb_objects)
    move_table = np.empty((nb_objects, 2)) #here we only store the values of 4 ArucoMarkers
    temp_L = copy.copy(origin_L)

    #For each specific marker ID do move the robot
    for i, obj in enumerate(detected_markers, 0):
        if obj["id"] > 3:
            print(f'■ REAL COORDINATES: Detected markerID{obj["id"]:>3.0f} Center position'
                  f'X={obj["cx"]:>3.0f}mm Y={obj["cy"]:>3.0f}mm ')
            x = obj["id"] - 4
            id_table[x] = True
            move_table[x][0] = obj["cx"]/ 1000
            move_table[x][1] = obj["cy"]/ 1000

    print(id_table)
    if nb_objects == nb_Cups:
        print("All markers detected, will begin displacement")
        print(move_table)
        rtde_c.moveL(origin_L, VELOCITY, ACCELERATION)
        print(origin_L)

        # go to marker 5
        temp_L[0] = origin_L[0] - move_table[1][0]
        temp_L[1] = origin_L[1] + move_table[1][1] - grip_dy
        rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)
        print("marker 5 move:")
        print(origin_L)
        print(temp_L)

        # go down and grab and go back up
        temp_L[2] = origin_L[2] - 0.075
        rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)
        time.sleep(5)
        arduino.send_message("grab")
        time.sleep(5)

        # go to marker 4
        temp_L[2] = origin_L[2]
        rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)

        temp_L[0] = origin_L[0] - move_table[0][0]
        temp_L[1] = origin_L[1] + move_table[0][1] - grip_dy
        rtde_c.moveL(temp_L, VELOCITY, ACCELERATION)
        print("marker 4 move:")
        print(origin_L)
        print(temp_L)

        arduino.send_message("release")

    else:
        print("error, too many or not all markers detected")


"""ArUco TRACKING LOOP ____________________________________________________________________"""


# initialise robot with URBasic
home_J = [-298.22, -66.90, -113.00, -89.74, 90.11, -24.63]
#rtde_c.moveJ(np.deg2rad(home_J), VELOCITY, ACCELERATION)

#best if I get it from Joint Position
origin_L = [0.33, 0.491, 0.31504431455331383, -3.13713023885791, 0.08284771453405795,
            -0.009878696005977336]


rtde_c.moveJ(np.deg2rad(home_J), VELOCITY, ACCELERATION)


time.sleep(1)  # just a short wait to make sure everything is initialised


try:
    print("starting loop")

    #test gripper
    arduino.send_message("grab")
    time.sleep(5)
    arduino.send_message("release")

    while True:

        frame = vs.read()
        frame = imutils.resize(frame, width=video_resolution[0], height=video_resolution[1])

        detected_markers, field_corners = findArUcoMarkers(frame)  # detected markers are in pixels
        detected_markers = getRealCoordinates(frame, field_corners, detected_markers)  # convert to real values

        cv2.imshow('RobotCamera', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vs.stream.release()
    cv2.destroyAllWindows()

    #move robot to markers
    move_to_markers(detected_markers)

    print("exiting loop")

except KeyboardInterrupt:
    print("closing robot connection")
    vs.stream.release()
    cv2.destroyAllWindows()
except:
    print("closing robot connection")
    print("Check Camera Connection")
    vs.stream.release()
    cv2.destroyAllWindows()