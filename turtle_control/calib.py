import numpy as np
import cv2
import glob

chess_size = (10,7)
sq_size = 0.023

objp = np.zeros((chess_size[0]*chess_size[1],3), np.float32)
objp[:,:2] = np.mgrid[0:chess_size[0],0:chess_size[1]].T.reshape(-1,2)
objp = objp * sq_size

objpoints = []
imgpoints = []

img = glob.glob('calib_*.jpg')

for fname in img:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    ret, corners = cv2.findChessboardCorners(gray, chess_size , None)
    
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        cv2.waitKey(200)

cv2.destroyAllWindows()

ret, camera_matrix , dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera Matrix:\n", camera_matrix)
print("Distortion Coeffs:\n", dist_coeffs)

np.savez("calibration_data.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
