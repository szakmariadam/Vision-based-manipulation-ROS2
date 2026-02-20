from dt_apriltags import Detector
import cv2

imgPath = "camera_vision_py/calib_images/workspace.png"
image = cv2.imread(imgPath, cv2.IMREAD_GRAYSCALE)
image_color = cv2.imread(imgPath)

at_detector = Detector(families="tagStandard41h12")

results = at_detector.detect(image)

for r in results:
	# extract the bounding box (x, y)-coordinates for the AprilTag
	# and convert each of the (x, y)-coordinate pairs to integers
	(ptA, ptB, ptC, ptD) = r.corners
	ptB = (int(ptB[0]), int(ptB[1]))
	ptC = (int(ptC[0]), int(ptC[1]))
	ptD = (int(ptD[0]), int(ptD[1]))
	ptA = (int(ptA[0]), int(ptA[1]))
	# draw the bounding box of the AprilTag detection
	cv2.line(image_color, ptA, ptB, (0, 255, 0), 1)
	cv2.line(image_color, ptB, ptC, (0, 255, 0), 1)
	cv2.line(image_color, ptC, ptD, (0, 255, 0), 1)
	cv2.line(image_color, ptD, ptA, (0, 255, 0), 1)
	# draw the center (x, y)-coordinates of the AprilTag
	(cX, cY) = (int(r.center[0]), int(r.center[1]))
	cv2.circle(image_color, (cX, cY), 5, (0, 0, 255), -1)
	# draw the tag family on the image
	tagFamily = r.tag_family.decode("utf-8")
	cv2.putText(image_color, tagFamily, (ptA[0], ptA[1] - 15),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

print(results)

cv2.imshow("Image", image_color)
cv2.waitKey(0)

