import numpy as np
import cv2
import glob

cb_width = 8
cb_height = 6
cb_square_size = 40

#termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#prepare object points, like (0,0,0), (1,0,0), (2,0,0) ... (6,5,0)
cb_3D_points = np.zeros((cb_width * cb_height, 3), np.float32)
cb_3D_points[:,:2] = np.mgrid[0:cb_width, 0:cb_height].T.reshape(-1,2) * cb_square_size

#Arrays to store object points and image points from all the images
list_cb_3d_points = []
list_cb_2d_img_points = []

list_images = glob.glob('calibration_checkerboard/*.jpg')

for frame_name in list_images:
    img = cv2.imread(frame_name)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

    #If found, add object points, image points (after refining them)
    if ret == True:
        list_cb_3d_points.append(cb_3D_points)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        list_cb_2d_img_points.append(corners2)

        #draw and display the corners
        cv2.drawChessboardCorners(img, (cb_width,cb_height), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(list_cb_3d_points, list_cb_2d_img_points, gray.shape[::-1], None, None)

print("Calibration Matrix")
print(mtx)
print("Distortion: ", dist)

with open('cam_calibration.npy','wb') as f:
    np.save(f, mtx)
    np.save(f, dist)