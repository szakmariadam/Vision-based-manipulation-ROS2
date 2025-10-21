import numpy as np
from dt_apriltags import Detector
import cv2

image_path = "calib_images/workspace.png"
tagFamily = "tagStandard41h12"

tagsize = 0.05
square_size = 0.6
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

imgpoints = np.array(
    [
        detection[2].center,
        detection[3].center,
        detection[0].center,
        detection[1].center
    ])

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

proj_points, _ = cv2.projectPoints(objpoints, rvec, tvec, K, dist)
proj_points = proj_points.reshape(-1, 2)
img_points = imgpoints.reshape(-1, 2)
error = cv2.norm(imgpoints.astype(np.float32), proj_points.astype(np.float32), cv2.NORM_L2) / len(proj_points)
print("Reprojection error (pixels):", error)