import move_robot
import PathPlanning.geometry as geometry
import math

# misc. hyperparameters
rMax = 0.12 # maximum radius to check if cups obstruct the gripper (usually go for 1.2 times the gripper dy
dCups = 0.2
xLoc = -0.10
yLoc = 0.235


def simple_pouring(listCups):
    listCups_center = []
    listCups_edge = []
    listCups_moved = []
    listCups_endlocations = []

    completed = False

    # initialize list tables
    for i, obj in enumerate(listCups, 0):
        listCups_center.append(obj["center"])
        listCups_edge.append(obj["edge"])
        listCups_moved.append(False)
        listCups_endlocations.append([xLoc, yLoc+(i+1)*dCups])

    # cycle through all cups and check if they can and should be moved

    for i in range(len(listCups_center)):

        #check if there is any obstruction
        nb_obst = check_obstruction(i, listCups_center)

        # Case 1: No Obstruction
        if nb_obst == 0:
            # Grab cup with fixed angle
            angle = 0
            move_robot.grab_cup(listCups_center[i][0], listCups_center[i][1], angle)

            # Place cup on the designated areas with good orientations
            move_robot.place_cup(listCups_endlocations[i][0],listCups_endlocations[i][1])
            #change the center position in the list

        #if nb_obst == 2:
            ##calculate the angle for which the gripper needs to grab the cup properly

    return


def check_obstruction(i, listCups_center):
    obstruction = 0

    for j in range(len(listCups_center)):
        if i != j:
            dist = geometry.calculate_distance(listCups_center[i], listCups_center[j])
            if dist < rMax:
                obstruction += 1
        else:
            continue

    print("Detected number of obstructions")
    print(obstruction)
    return obstruction
