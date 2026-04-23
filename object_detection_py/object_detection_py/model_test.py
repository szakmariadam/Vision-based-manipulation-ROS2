from ultralytics import YOLO
import cv2

# Load a COCO-pretrained YOLO26n model
model = YOLO("yolo26n.pt")

# Run inference with the YOLO26n model on the 'bus.jpg' image
results = model.predict("camera_vision_py/calib_images/bottle.png", verbose=True)

img = results[0].plot()



classes = results[0].boxes.cls.cpu().numpy().astype(int)

names = [results[0].names[i] for i in classes]
print(names)

bb_positions = results[0].boxes.xyxy.cpu().numpy()

print(bb_positions)

cv2.imshow("result", img)
cv2.waitKey(0)
cv2.destroyAllWindows()