import numpy as np
from dt_apriltags import Detector
import cv2
import glob

image_path = "calib_images/workspace.png"
tagFamily = "tagStandard41h12"

tagsize = 0.05
square_size = tagsize# - 2*(tagsize/9)
tag_id = 0

tagCorners = np.array([
    [-square_size/2, square_size/2, 0],
    [square_size/2, square_size/2, 0],
    [square_size/2, -square_size/2, 0],
    [-square_size/2, -square_size/2, 0]], dtype=np.float32)

#load intrinsics
data = np.load('config/camera_calib.npz')
K = data['camera_matrix']
dist = data['dist_coeffs']

img = cv2.imread(image_path)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

at_detector = Detector(
    families=tagFamily,
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=True,
    decode_sharpening=0.25,
    debug=False,
)

detection = at_detector.detect(gray)

imgpoints = detection[tag_id].corners
objpoints = tagCorners

retval, rvec, tvec = cv2.solvePnP(objpoints, imgpoints, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)

print("pose found:", retval)
if retval:
    print("pose found for", image_path, "tag:", detection[tag_id].tag_id)
print(rvec.ravel())
print(tvec.ravel())

axis = np.float32([
    [0.02, 0, 0],   # X axis (red)
    [0, 0.02, 0],   # Y axis (green)
    [0, 0, -0.02]   # Z axis (blue)
])

imgpts, _ = cv2.projectPoints(axis, rvec, tvec, K, dist)

corner = tuple(detection[tag_id].corners.mean(axis=0).astype(int))  # approximate tag center
imgpts = np.int32(imgpts).reshape(-1, 2)
img = cv2.line(img, corner, tuple(imgpts[0]), (0, 0, 255), 3)  # X - red
img = cv2.line(img, corner, tuple(imgpts[1]), (0, 255, 0), 3)  # Y - green
img = cv2.line(img, corner, tuple(imgpts[2]), (255, 0, 0), 3)  # Z - blue

cv2.imshow("Image", img)
cv2.waitKey(0)

proj_points, _ = cv2.projectPoints(objpoints, rvec, tvec, K, dist)
proj_points = proj_points.reshape(-1, 2)
img_points = imgpoints.reshape(-1, 2)
error = cv2.norm(imgpoints.astype(np.float32), proj_points.astype(np.float32), cv2.NORM_L2) / len(proj_points)
print("Reprojection error (pixels):", error)