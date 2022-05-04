import copy

import move_robot
import PathPlanning.geometry as geometry
import math

# misc. hyperparameters
rMax = 0.156 + 0.05  # maximum radius to check if cups obstruct the gripper (usually go for 1.3 times the gripper dy
dCups = 0.19 # distance between each cup in the end position
xLoc = -0.11
yLoc = 0.235
rCups = 0.055 * 1.5


def simple_pouring(listCups):
    listCups_center = []
    listCups_edge = []
    listCups_alpha = []
    listCups_moved = []
    listCups_endlocations = []

    # initialize list tables
    for i, obj in enumerate(listCups, 0):
        listCups_center.append(obj["center"])
        listCups_edge.append(obj["edge"])
        alpha = geometry.get_angle(listCups_center[i], listCups_edge[i])
        listCups_alpha.append(alpha)
        listCups_moved.append(False)
        listCups_endlocations.append([xLoc, yLoc+(i)*dCups])

    position_cups(listCups_moved, listCups_center, listCups_alpha, listCups_endlocations)

    print(listCups_alpha)
    pouring_cups(listCups_moved, listCups_center, listCups_alpha, listCups_endlocations)



    return


def position_cups(listCups_moved, listCups_center,listCups_alpha, listCups_endlocations):
    # cycle through all cups and check if they can and should be moved
    while not (check(listCups_moved)):
        for i in range(len(listCups_center)):
            # check if there is any obstruction
            print(i)
            nb_obst, id_obst = check_obstruction(i, listCups_center)
            print(nb_obst)

            # Case 1: No Obstruction
            if nb_obst == 0:
                print("No Detected Obstruction")
                # Grab cup with fixed robot wrist angle
                angle = 0
                move_robot.grab_cup(listCups_center[i][0], listCups_center[i][1], angle)
                move_robot.place_cup(listCups_endlocations[i][0], listCups_endlocations[i][1])

                # change the center position in the list & update alpha
                listCups_center[i] = listCups_endlocations[i]
                listCups_moved[i] = True
                listCups_alpha[i] -= math.pi/2
                print("alpha")
                print(listCups_alpha)



            # Case 2: 1 Obstruction
            if nb_obst == 1:
                print("1 Detected Obstruction")
                # calculate the angle for which the gripper needs to grab the cup properly
                # Here we just make it grab at the opposite of the other cup
                angle = geometry.get_angle(listCups_center[i], listCups_center[id_obst[0]])
                move_robot.grab_cup(listCups_center[i][0], listCups_center[i][1], angle)
                move_robot.place_cup(listCups_endlocations[i][0], listCups_endlocations[i][1])

                listCups_center[i] = listCups_endlocations[i]
                listCups_moved[i] = True
                listCups_alpha[i] -= math.pi / 2 + angle

            # Case 3: 2 Obstructions
            if nb_obst == 2:
                print("2 Detected Obstruction")
                # new point
                tmpPoint = geometry.calculate_midpoint(listCups_center[id_obst[0]], listCups_center[id_obst[1]])
                print(tmpPoint)
                dist = geometry.calculate_distance(listCups_center[i], tmpPoint)

                if dist > rCups:
                    angle = geometry.get_angle(listCups_center[i], tmpPoint)
                    move_robot.grab_cup(listCups_center[i][0], listCups_center[i][1], angle)
                    move_robot.place_cup(listCups_endlocations[i][0], listCups_endlocations[i][1])

                    listCups_center[i] = listCups_endlocations[i]
                    listCups_moved[i] = True
                    listCups_alpha[i] -= math.pi / 2 + angle

        print(listCups_moved)
    return

def pouring_cups(listCups_moved, listCups_center,listCups_alpha, listCups_endlocations):

    #Pour cup 1 into cup 2
    print("Pouring cups")
    x = 0
    angle = math.pi/2
    move_robot.grab_cup(listCups_center[x][0], listCups_center[x][1], angle)
    move_robot.place_cupP(0.2, 0.5)
    listCups_alpha[x] += angle
    #angle that wrist is supposed to take
    print(listCups_alpha[x])

    move_robot.grab_cupP(0.2, 0.5, listCups_alpha[x])
    move_robot.pouring(listCups_endlocations[1][0],listCups_endlocations[1][1])
    return

def check_obstruction(i, listCups_center):
    obstruction = 0
    obstruction_id = []
    obstruction_dist = []
    for j in range(len(listCups_center)):
        if i != j:
            dist = geometry.calculate_distance(listCups_center[i], listCups_center[j])
            print("distance")
            print(dist)
            if dist < rMax:
                obstruction += 1
                obstruction_id.append(j)
                obstruction_dist.append(dist)
        else:
            continue

    print()

    #Make obstruction_id list in descending order based on the distance
    obstruction_order = copy.copy(obstruction_dist)
    obstruction_order.sort(reverse=True)
    for j in range(len(obstruction_order)):
        for k in range(len(obstruction_order)):
            if obstruction_dist[j] == obstruction_order[k]:
                obstruction_dist[j] = k

    obstruction_id = [obstruction_id[j] for j in obstruction_dist]

    return obstruction, obstruction_id

def check(list):
    for item in list:
        if item == False:
            return False
    return True