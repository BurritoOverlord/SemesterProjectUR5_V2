import matplotlib.pyplot as plt

#my modules
from PathPlanning import geometry

#misc. hyperparameters
rCups = 0.055 # Cup radius in cm
rMax = 30
dMtCe = 0.02 # distance from marker center to cup edge in cm

# best if I get it from Joint Position
origin_L = [0.33, 0.491, 0.31504431455331383, -3.13713023885791, 0.08284771453405795,
            -0.009878696005977336]

#misc. Terrain Field to be defined at the beginning
xMax = 0.5
yMax = 1
xMin = -0.2

def move_to_markers(detected_markers):

    # Create list for Cups
    listCups_center = []
    listCups_edge = []
    listMarker_center = []
    listMarker_edge = []

    # loop over the detected markers to calculate the cup center points and angle
    # For each specific marker ID do move the robot
    for i, obj in enumerate(detected_markers, 0):

        #ignore the reference ArUco markers and store all important values in cupList
        #convert all values to meters
        if obj["id"] > 3:

            # Get cup edge en center from ArUco markers - in Meters!
            mX = origin_L[0] + obj["cx"] / 1000
            mY = origin_L[1] + obj["cy"] / 1000

            tX = origin_L[0] + obj["ex"] / 1000
            tY = origin_L[1] + obj["ey"] / 1000

            dist1 = rCups + dMtCe
            dist2 = 2 * rCups + dMtCe

            [cX, cY] = geometry.segmentExtension((mX, mY), (tX, tY), dist1)
            [eX, eY] = geometry.segmentExtension((mX, mY), (tX, tY), dist2)

            listMarker_center.append([mX, mY])
            listMarker_edge.append([tX, tY])
            listCups_center.append([cX, cY])
            listCups_edge.append([eX, eY])


    cup_layout(listCups_center, listCups_edge, listMarker_center, listMarker_edge)

    return

def cup_layout(cupCList, cupEList, markerCList, markerEList):

    figure, ax = plt.subplots()

    # change default range so that new circles will work
    ax.set_xlim((xMin, xMax))
    ax.set_ylim((0, yMax))

    #plot circles
    for Pos in cupCList :
        circle = plt.Circle(Pos, rCups, color='red', ec = 'black')
        plt.gcf().gca().add_patch(circle)

    # plot cup edge dots
    for Pos in cupEList :
        circle = plt.Circle(Pos, rCups/5, color='blue', ec = 'black')
        plt.gcf().gca().add_patch(circle)

    for Pos in markerCList :
        circle = plt.Circle(Pos, rCups/10, color='black', ec = 'black')
        plt.gcf().gca().add_patch(circle)

    for Pos in markerEList :
        circle = plt.Circle(Pos, rCups/10, color='green', ec = 'black')
        plt.gcf().gca().add_patch(circle)

    plt.title('Cup Layout')
    ax.set_aspect(1)
    plt.show()

    return
