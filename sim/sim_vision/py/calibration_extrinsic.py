import numpy as np
from dt_apriltags import Detector
import cv2
import glob

image_path = "calib_images/workspace.png"
tagFamily = "tagStandard41h12"

tagsize = 0.05
square_size = tagsize - 2*(tagsize/9)

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

imgpoints = detection[0].corners
objpoints = tagCorners

retval, rvec, tvec = cv2.solvePnP(objpoints, imgpoints, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)

print("pose found:", retval)
print(rvec)
print(tvec)

