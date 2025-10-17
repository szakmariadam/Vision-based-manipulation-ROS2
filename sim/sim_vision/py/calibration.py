import numpy as np
from dt_apriltags import Detector
import cv2
import glob

#apriltag layout
tagFamily = "tagStandard41h12"
tagPositions = np.array([[-0.3, -0.3, 0], [-0.3, 0.3, 0], [0.3, 0.3, 0], [0.3, -0.3, 0]], dtype=np.float32)
'''tagPositions = np.append(tagPositions, [[-0.3, -0.3, 0]])     #tagID=0
tagPositions = np.append(tagPositions, [[-0.3, 0.3, 0]])      #tagID=1
tagPositions = np.append(tagPositions, [[0.3, 0.3, 0]])       #tagID=2
tagPositions = np.append(tagPositions, [[0.3, -0.3, 0]])      #tagID=3'''

objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

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
    

    '''for det in detections:
        corners = np.rint(det.corners).astype(int)
        cv2.polylines(img, [corners], True, (0, 255, 0), 2)
        cv2.putText(img, str(det.tag_id), tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.imshow("Detections", img)
    cv2.waitKey(100)'''

#objpoints = [np.array(objpoints, dtype=np.float32)]
#imgpoints = [np.array(imgpoints, dtype=np.float32)]

h, w = gray.shape[:2]
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, (w, h), None, None
)

print("\nCalibration successful:", ret)
print("\nCamera matrix:\n", camera_matrix)
print("\nDistortion coefficients:\n", dist_coeffs.ravel())