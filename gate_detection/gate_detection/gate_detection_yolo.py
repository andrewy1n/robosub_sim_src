from ultralytics import YOLO
import numpy as np
import os
import cv2
from ament_index_python.packages import get_package_share_directory

# Import saved model and demo image
package_dir = get_package_share_directory('gate_detection')
model_path = os.path.join(package_dir, 'config', 'gate_yolo.pt')
model = YOLO(model_path)

def find_gate_center(img, test=True):
    # set point size
    radius = 5
    color = (0, 255, 0)  # Green
    colorC = (0, 0, 255) # Red
    thickness = -1  # Solid circle

    # Get and display bounding boxes using YOLO
    results = model(img)

    # Check if we have at least two detections
    if len(results[0].boxes) < 2:
        return None

    box1 = list(map(int, results[0].boxes.xyxy[0]))
    box2 = list(map(int, results[0].boxes.xyxy[1]))

    x1, y1, _, _ = box1 # left bounding box
    _, _, x4, y4 = box2 # right bounding box

    temp_img = img.copy()
    temp_img = cv2.rectangle(temp_img, box1[:2], box1[2:], (0,0,255))
    temp_img = cv2.rectangle(temp_img, box2[:2], box2[2:], (0,0,255))

    cv2.circle(temp_img, (x1, y1), radius, color, thickness)  # top-left
    cv2.circle(temp_img, (x4, y4), radius, color, thickness)  # bottom-right

    centerX, centerY = (x1 + x4) // 2, (y1 + y4) // 2
    cv2.circle(temp_img, (centerX, centerY), radius, colorC, thickness)  # center of gate

    if test:
        cv2.imshow('gate image', temp_img)
        cv2.waitKey(0)

    return (centerX, centerY)

def contains_gate(img, conf_threshold: float = 0.5) -> bool:
    """
    Returns True if the YOLO model detects at least one gate
    in `img` with confidence >= conf_threshold.
    """
    results = model(img)

    boxes = results[0].boxes

    # If no boxes were detected at all
    if boxes is None or len(boxes) == 0:
        return False

    confidences = boxes.conf.cpu().numpy()

    return bool(np.any(confidences >= conf_threshold))