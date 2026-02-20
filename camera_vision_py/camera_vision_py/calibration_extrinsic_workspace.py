import numpy as np
from dt_apriltags import Detector
import cv2

image_path = "camera_vision_py/calib_images/workspace_center.png"
tagFamily = "tagStandard41h12"

square_size = 0.6
tag_size = (0.05/9)*5.12

tagCorners = np.array([
    [-square_size/2, square_size/2, 0],
    [-square_size/2 - tag_size/2, square_size/2 - tag_size/2, 0],
    [-square_size/2 + tag_size/2, square_size/2 - tag_size/2, 0],
    [-square_size/2 + tag_size/2, square_size/2 + tag_size/2, 0],
    [-square_size/2 - tag_size/2, square_size/2 + tag_size/2, 0],

    [square_size/2, square_size/2, 0],
    [square_size/2 - tag_size/2, square_size/2 - tag_size/2, 0],
    [square_size/2 + tag_size/2, square_size/2 - tag_size/2, 0],
    [square_size/2 + tag_size/2, square_size/2 + tag_size/2, 0],
    [square_size/2 - tag_size/2, square_size/2 + tag_size/2, 0],

    [square_size/2, -square_size/2, 0],
    [square_size/2 - tag_size/2, -square_size/2 - tag_size/2, 0],
    [square_size/2 + tag_size/2, -square_size/2 - tag_size/2, 0],
    [square_size/2 + tag_size/2, -square_size/2 + tag_size/2, 0],
    [square_size/2 - tag_size/2, -square_size/2 + tag_size/2, 0],

    [-square_size/2, -square_size/2, 0],
    [-square_size/2 - tag_size/2, -square_size/2 - tag_size/2, 0],
    [-square_size/2 + tag_size/2, -square_size/2 - tag_size/2, 0],
    [-square_size/2 + tag_size/2, -square_size/2 + tag_size/2, 0],
    [-square_size/2 - tag_size/2, -square_size/2 + tag_size/2, 0]
        ], dtype=np.float32)

#load intrinsics
data = np.load('camera_vision_py/config/camera_calib.npz')
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

imgpoints = np.array([

        detection[2].center,
        detection[2].corners[0],
        detection[2].corners[1],
        detection[2].corners[2],
        detection[2].corners[3],

        detection[3].center,
        detection[3].corners[0],
        detection[3].corners[1],
        detection[3].corners[2],
        detection[3].corners[3],

        detection[0].center,
        detection[0].corners[0],
        detection[0].corners[1],
        detection[0].corners[2],
        detection[0].corners[3],

        detection[1].center,
        detection[1].corners[0],
        detection[1].corners[1],
        detection[1].corners[2],
        detection[1].corners[3],
    ])

objpoints = tagCorners

retval, rvec_workspace, tvec_workspace = cv2.solvePnP(objpoints, imgpoints, K, dist, flags=cv2.SOLVEPNP_IPPE)

print("pose found:", retval)
if retval:
    print("pose found for", image_path)
print(rvec_workspace.ravel())
print(tvec_workspace.ravel())

axis = np.float32([
    [0.02, 0, 0],   # X axis (red)
    [0, 0.02, 0],   # Y axis (green)
    [0, 0, -0.02]   # Z axis (blue)
])

proj_points, _ = cv2.projectPoints(objpoints, rvec_workspace, tvec_workspace, K, dist)
proj_points = proj_points.reshape(-1, 2)
img_points = imgpoints.reshape(-1, 2)
error = cv2.norm(imgpoints.astype(np.float32), proj_points.astype(np.float32), cv2.NORM_L2) / len(proj_points)
print("Reprojection error (pixels):", error)

# Convert rvec to rotation matrix
R_workspace2cam, _ = cv2.Rodrigues(rvec_workspace)

# Invert the rotation
R_cam2workspace = R_workspace2cam.T

# Invert the translation
tvec_cam = -R_cam2workspace @ tvec_workspace

# Convert rotation matrix back to rvec
rvec_cam, _ = cv2.Rodrigues(R_cam2workspace)

print("Camera pose in board frame:")
print("rvec_cam:", rvec_cam.ravel())
print("tvec_cam:", tvec_cam.ravel())

#visualization
imgpts, _ = cv2.projectPoints(axis, rvec_workspace, tvec_workspace, K, dist)
imgpts = np.int32(imgpts).reshape(-1, 2)
center = (639,290)

img = cv2.line(img, center, tuple(imgpts[0]), (0, 0, 255), 1)  # X - red
img = cv2.line(img, center, tuple(imgpts[1]), (0, 255, 0), 1)  # Y - green
img = cv2.line(img, center, tuple(imgpts[2]), (255, 0, 0), 1)  # Z - blue

cv2.imshow("Image", img)
cv2.waitKey(0)