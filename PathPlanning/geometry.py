import numpy as np
import matplotlib.pyplot as plt
import math


#for readability purpose
x=0
y=1


# %% GEOMETRICAL FUNCTIONS

def angle3Pts(a, b, c):
    npa = np.array(a)
    npb = np.array(b)
    npc = np.array(c)

    vBA = npa - npb
    vBC = npc - npb

    angle = np.degrees(np.arccos(np.dot(vBA, vBC) / (np.linalg.norm(vBA) * np.linalg.norm(vBC))))

    return angle

def pts2Vect(ptA, ptB):
    return np.array(ptB) - np.array(ptA)

def segmentExtension(ptA, ptB, dist):

    vAB = pts2Vect(ptA, ptB)
    vAC = dist * vAB / np.linalg.norm(vAB)
    ptC = np.array(ptA) + vAC

    return ptC




# determine if the orthogonal projection of a querry point on line d belongs to segment ab of line d
# if the orthogonal projection belongs to segment ab, calculate distance between querry point and orthogonal projection
def orthoProj(ptA, ptB, querryPt):
    # compute vector AB (segment) and AP (vector between point A and the querry point)
    vAB = pts2Vect(ptA, ptB)
    vAP = pts2Vect(ptA, querryPt)

    if np.linalg.norm(vAB) == 0:
        # ptA = ptB the segment AB is in fact a point
        return np.linalg.norm(vAP)

    # compute orthogonal projection ptH of querryPt on d
    vAH = vAB * np.dot(vAP, vAB) / np.linalg.norm(vAB) ** 2
    ptH = np.array(ptA) + vAH

    # let line(t) the parametrized form of line d, compute parameter tH so that line(t=tH) = ptH
    # because it is parametrized using vAB as a defining vector, tH must be between 0 and 1 for ptH to belong to AB
    if ptB[x] != ptA[x]:
        # vAB is not vertical, we can use x coordinates to determine tH
        tH = (ptH[x] - ptA[x]) / (ptB[x] - ptA[x])

    elif ptB[y] != ptA[y]:
        # vAB is not horizontal, we can use y coordinates to determine tH
        tH = (ptH[y] - ptA[y]) / (ptB[y] - ptA[y])

    if tH > 0 and tH < 1:
        # projected point H belongs to segment AB
        dist = np.linalg.norm(ptH - querryPt)
        return dist
    else:
        # projected point H does not belong to segment AB, the distance is not useful
        return np.nan


"""Angle between vector and Y axis"""
def get_angle(ptA, ptB):

    vAB = pts2Vect(ptA, ptB)
    vOY = pts2Vect((0,0), (0,1))
    unit_vAB = vAB / np.linalg.norm(vAB)
    unit_vOY = vOY / np.linalg.norm(vOY)
    dot_product = np.dot(unit_vAB, unit_vOY)
    alpha = np.arccos(dot_product)

    c = np.cross(ptB,ptA)
    if c > 0:
        alpha += math.pi

    return alpha

def calculate_distance(centerA, centerB):

    print(centerA)
    distance = math.sqrt(((centerA[0]-centerB[0])**2)+((centerA[1]-centerB[1])**2))
    print(distance)

    return distance

def calculate_midpoint(ptA, ptB):
    ptC = ((ptA[0] + ptB[0]) / 2, (ptA[1] + ptB[1]) / 2)
    return ptC

def rotate(cX, cY, pX, pY, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """

    qX = cX + math.cos(angle) * (pX - cX) - math.sin(angle) * (pY - cY)
    qY = cY + math.sin(angle) * (pX - cX) + math.cos(angle) * (pY - cY)
    return qX, qY

def get_angleDiff(angleC, angleR):
    dif = angleR - angleC
    return dif