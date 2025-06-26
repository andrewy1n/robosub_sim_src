import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# Try to import ultralytics
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False

class SimCameraCV(Node):
    def __init__(self):
        super().__init__('sim_camera_cv')
        self.subscription = self.create_subscription(
            Image,
            '/robosub/camera/simulated_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        
        # Initialize YOLO model
        self.model = None
        self.model_loaded = False
        
        # Define class names for gate_yolo_sim model
        self.class_names = ['leftPipe', 'rightPipe', 'centerPipe', 'divider']
        
        # Define colors for each class
        self.class_colors = {
            'leftPipe': (0, 255, 0),      # Green
            'rightPipe': (255, 0, 0),     # Blue
            'centerPipe': (0, 0, 255),    # Red
            'divider': (255, 255, 0)      # Cyan
        }
        
        if ULTRALYTICS_AVAILABLE:
            try:
                # Load the YOLO model
                package_dir = get_package_share_directory('camera_test')
                model_path = os.path.join(package_dir, 'config', 'gate_yolo_sim.pt')
                self.model = YOLO(model_path)
                self.model_loaded = True
                self.get_logger().info(f"YOLO model loaded successfully from: {model_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO model: {e}")
                self.model_loaded = False
        else:
            self.get_logger().error("Ultralytics not available - install ultralytics to use YOLO detection")

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run detection if model is loaded
            if self.model_loaded:
                frame = self.detect_and_display_gates(frame)
            else:
                # Add a placeholder message on the frame
                cv2.putText(frame, "YOLO Detection Disabled", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                if not ULTRALYTICS_AVAILABLE:
                    cv2.putText(frame, "Ultralytics not available", (10, 70), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                else:
                    cv2.putText(frame, "Model loading failed", (10, 70), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Display the frame
            cv2.imshow('Camera Feed with YOLO Detections', frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def detect_and_display_gates(self, frame):
        try:
            if not self.model_loaded or self.model is None:
                return frame
                
            # Run YOLO inference
            results = self.model(frame, imgsz=640, conf=0.25)
            
            # Process detections
            for result in results:
                boxes = result.boxes
                if boxes is not None and len(boxes) > 0:
                    for i, box in enumerate(boxes):
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        
                        # Get confidence score
                        confidence = float(box.conf[0])
                        
                        # Get class name
                        class_id = int(box.cls[0]) if box.cls is not None else 0
                        if class_id < len(self.class_names):
                            class_name = self.class_names[class_id]
                        else:
                            class_name = f"Unknown_{class_id}"
                        
                        # Get color for this class
                        color = self.class_colors.get(class_name, (255, 255, 255))  # White for unknown
                        
                        # Draw bounding box
                        thickness = 2
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                        
                        # Draw label with confidence
                        label = f"{class_name}: {confidence:.2f}"
                        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                        cv2.rectangle(frame, (x1, y1 - label_size[1] - 10), (x1 + label_size[0], y1), color, -1)
                        cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                        
                        # Draw center point
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        cv2.circle(frame, (center_x, center_y), 5, (255, 255, 255), -1)  # White center point
                        
                        self.get_logger().info(f"Detected {class_name} at ({center_x}, {center_y}) with confidence {confidence:.2f}")
                else:
                    self.get_logger().debug("No pipes or dividers detected in this frame")
                    
        except Exception as e:
            self.get_logger().error(f"YOLO detection failed: {e}")
            
        return frame

def main(args=None):
    rclpy.init(args=args)
    node = SimCameraCV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
