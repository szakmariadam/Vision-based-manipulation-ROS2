from ultralytics import YOLO
import cv2

# Load a COCO-pretrained YOLO26n model
model = YOLO("yolo26n.pt")

# Run inference with the YOLO26n model on the 'bus.jpg' image
results = model("camera_vision_py/calib_images/bottle.png")

img = results[0].plot()

for r in results:
    boxes = r.boxes
    for box in boxes:
        print("Class:", int(box.cls))
        print("Confidence:", float(box.conf))
        print("Coordinates:", box.xyxy)

cv2.imshow("result", img)
cv2.waitKey(0)
cv2.destroyAllWindows()