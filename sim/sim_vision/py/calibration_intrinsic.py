import numpy as np
from dt_apriltags import Detector
import cv2
import glob

#apriltag layout
tagFamily = "tagStandard41h12"
tagPositions = np.array([
    [-0.3, -0.3, 0],
    [-0.3, 0.3, 0],
    [0.3, 0.3, 0],
    [0.3, -0.3, 0]], dtype=np.float32)

objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

at_detector = Detector(
    families=tagFamily,
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=True,
    decode_sharpening=0.25,
    debug=False,
)

images = glob.glob("calib_images/*.png")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    detections = at_detector.detect(gray)

    imgcenters = np.array([[0, 0], [0, 0], [0, 0], [0, 0]], dtype=np.float32)
    i=0
    for det in detections:
        imgcenters[i] = det.center
        i+=1

    objpoints.append(tagPositions)
    imgpoints.append(imgcenters)

h, w = gray.shape[:2]
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, (w, h), None, None
)

print("\nCalibration successful:", ret)
print("\nCamera matrix:\n", camera_matrix)
print("\nDistortion coefficients:\n", dist_coeffs.ravel())

#save
np.savez("config//camera_calib.npz", 
         camera_matrix=camera_matrix, 
         dist_coeffs=dist_coeffs, 
         rvecs=rvecs, 
         tvecs=tvecs)