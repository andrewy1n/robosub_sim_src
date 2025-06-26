import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
from datetime import datetime

class DatasetCaptureNode(Node):
    def __init__(self):
        super().__init__('dataset_capture_node')
        self.subscription = self.create_subscription(
            Image,
            '/robosub/camera/simulated_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        
        # Dataset configuration
        self.dataset_dir = os.path.expanduser('~/gate_dataset')
        self.images_dir = os.path.join(self.dataset_dir, 'images')
        self.labels_dir = os.path.join(self.dataset_dir, 'labels')
        
        # Create directories
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.labels_dir, exist_ok=True)
        
        # Capture settings
        self.capture_interval = 2.0  # seconds between captures
        self.last_capture_time = 0
        self.image_counter = 0
        
        # Display settings
        self.show_preview = True
        self.preview_window = 'Dataset Capture Preview'
        
        # Class mapping for YOLO
        self.classes = ['vertical_pipe', 'horizontal_pipe']
        self.class_to_id = {cls: i for i, cls in enumerate(self.classes)}
        
        # Save class mapping
        self.save_class_mapping()
        
        self.get_logger().info(f"Dataset capture node initialized")
        self.get_logger().info(f"Images will be saved to: {self.images_dir}")
        self.get_logger().info(f"Labels will be saved to: {self.labels_dir}")
        self.get_logger().info(f"Classes: {self.classes}")
        self.get_logger().info("Press 'c' to capture manually, 'q' to quit, 's' to toggle auto-capture")

    def save_class_mapping(self):
        classes_file = os.path.join(self.dataset_dir, 'classes.txt')
        with open(classes_file, 'w') as f:
            for cls in self.classes:
                f.write(f"{cls}\n")
        self.get_logger().info(f"Class mapping saved to: {classes_file}")

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Check if it's time for auto-capture
            current_time = time.time()
            if current_time - self.last_capture_time >= self.capture_interval:
                self.capture_image(frame)
                self.last_capture_time = current_time
            
            # Show preview with capture info
            if self.show_preview:
                self.show_preview_frame(frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Quitting dataset capture...")
                rclpy.shutdown()
            elif key == ord('c'):
                self.capture_image(frame)
            elif key == ord('s'):
                self.toggle_auto_capture()
            
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

    def capture_image(self, frame):
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            image_filename = f"gate_{timestamp}_{self.image_counter:04d}.jpg"
            label_filename = f"gate_{timestamp}_{self.image_counter:04d}.txt"
            
            image_path = os.path.join(self.images_dir, image_filename)
            label_path = os.path.join(self.labels_dir, label_filename)
            
            # Save image
            cv2.imwrite(image_path, frame)
            
            # Create empty label file (you'll annotate this later)
            with open(label_path, 'w') as f:
                # Empty file - you'll add annotations manually or with annotation tool
                pass
            
            self.image_counter += 1
            self.get_logger().info(f"Captured image {self.image_counter}: {image_filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to capture image: {e}")

    def show_preview_frame(self, frame):
        # Create a copy for display
        display_frame = frame.copy()
        
        # Add capture info overlay
        info_text = [
            f"Dataset Capture - Image {self.image_counter}",
            f"Auto-capture: {'ON' if self.capture_interval > 0 else 'OFF'}",
            f"Interval: {self.capture_interval}s",
            "",
            "Controls:",
            "c - Capture manually",
            "s - Toggle auto-capture",
            "q - Quit"
        ]
        
        # Draw info text
        y_offset = 30
        for i, text in enumerate(info_text):
            color = (0, 255, 0) if i < 3 else (255, 255, 255)
            cv2.putText(display_frame, text, (10, y_offset + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Add capture indicator
        if time.time() - self.last_capture_time < 0.5:
            cv2.putText(display_frame, "CAPTURING...", (display_frame.shape[1]//2 - 100, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        
        cv2.imshow(self.preview_window, display_frame)

    def toggle_auto_capture(self):
        if self.capture_interval > 0:
            self.capture_interval = 0
            self.get_logger().info("Auto-capture disabled")
        else:
            self.capture_interval = 2.0
            self.get_logger().info("Auto-capture enabled (2s interval)")

def main(args=None):
    rclpy.init(args=args)
    node = DatasetCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main() 