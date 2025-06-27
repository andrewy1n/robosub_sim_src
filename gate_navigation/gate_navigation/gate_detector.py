#!/usr/bin/env python3

import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
from typing import Dict, List, Optional, Any


class GateDetector:
    def __init__(self):
        """Initialize the gate detector with YOLO model"""
        self.model = None
        self.model_loaded = False
        
        # Class names for gate_yolo_sim model
        self.class_names = ['leftPipe', 'rightPipe', 'centerPipe', 'divider']
        
        # Colors for visualization
        self.class_colors = {
            'leftPipe': (0, 255, 0),      # Green
            'rightPipe': (255, 0, 0),     # Blue
            'centerPipe': (0, 0, 255),    # Red
            'divider': (255, 255, 0)      # Cyan
        }
        
        self.load_model()
    
    def load_model(self):
        """Load the YOLO model"""
        try:
            # Load the YOLO model from camera_test package
            package_dir = get_package_share_directory('camera_test')
            model_path = os.path.join(package_dir, 'config', 'gate_yolo_sim.pt')
            
            if os.path.exists(model_path):
                self.model = YOLO(model_path)
                self.model_loaded = True
                print(f"YOLO model loaded successfully from: {model_path}")
            else:
                print(f"Model file not found at: {model_path}")
                self.model_loaded = False
                
        except Exception as e:
            print(f"Failed to load YOLO model: {e}")
            self.model_loaded = False
    
    def detect_gates(self, frame, confidence_threshold=0.5):
        """
        Detect gates in the given frame
        
        Args:
            frame: Input image frame (numpy array)
            confidence_threshold: Minimum confidence for detections
            
        Returns:
            List of detections with bbox, class, and confidence
        """
        if not self.model_loaded or self.model is None:
            return []
        
        try:
            # Run YOLO inference
            results = self.model(frame, conf=confidence_threshold)
            
            detections = []
            
            # Process detections
            for result in results:
                boxes = result.boxes
                if boxes is not None and len(boxes) > 0:
                    for box in boxes:
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
                        
                        # Add to detections
                        detection = {
                            'bbox': [x1, y1, x2, y2],
                            'class': class_name,
                            'confidence': confidence,
                            'center': [(x1 + x2) // 2, (y1 + y2) // 2]
                        }
                        
                        detections.append(detection)
            
            return detections
            
        except Exception as e:
            print(f"YOLO detection failed: {e}")
            return []
    
    def get_gate_components(self, detections):
        """
        Extract gate components from detections
        
        Args:
            detections: List of detections from detect_gates()
            
        Returns:
            Dictionary with gate components
        """
        gate_components: Dict[str, Optional[Dict[str, Any]]] = {
            'left_pipe': None,
            'right_pipe': None,
            'center_pipe': None,
            'divider': None
        }
        
        for detection in detections:
            class_name = detection['class']
            bbox = detection['bbox']
            confidence = detection['confidence']
            
            detection_info = {
                'bbox': bbox,
                'confidence': confidence,
                'center': detection['center']
            }
            
            if class_name == 'leftPipe':
                gate_components['left_pipe'] = detection_info
            elif class_name == 'rightPipe':
                gate_components['right_pipe'] = detection_info
            elif class_name == 'centerPipe':
                gate_components['center_pipe'] = detection_info
            elif class_name == 'divider':
                gate_components['divider'] = detection_info
        
        return gate_components
    
    def calculate_gate_center(self, left_pipe, right_pipe):
        """
        Calculate the center of the gate from left and right pipes
        
        Args:
            left_pipe: Detection info for left pipe
            right_pipe: Detection info for right pipe
            
        Returns:
            Gate center coordinates [x, y]
        """
        if left_pipe is None or right_pipe is None:
            return None
        
        left_center = left_pipe['center']
        right_center = right_pipe['center']
        
        gate_center = [
            (left_center[0] + right_center[0]) // 2,
            (left_center[1] + right_center[1]) // 2
        ]
        
        return gate_center
    
    def calculate_gate_width(self, left_pipe, right_pipe):
        """
        Calculate the width of the gate
        
        Args:
            left_pipe: Detection info for left pipe
            right_pipe: Detection info for right pipe
            
        Returns:
            Gate width in pixels
        """
        if left_pipe is None or right_pipe is None:
            return None
        
        left_center = left_pipe['center']
        right_center = right_pipe['center']
        
        width = np.linalg.norm(np.array(right_center) - np.array(left_center))
        return width
    
    def visualize_detections(self, frame, detections):
        """
        Draw detections on the frame for visualization
        
        Args:
            frame: Input image frame
            detections: List of detections
            
        Returns:
            Frame with detections drawn
        """
        vis_frame = frame.copy()
        
        for detection in detections:
            bbox = detection['bbox']
            class_name = detection['class']
            confidence = detection['confidence']
            
            x1, y1, x2, y2 = bbox
            
            # Get color for this class
            color = self.class_colors.get(class_name, (255, 255, 255))
            
            # Draw bounding box
            cv2.rectangle(vis_frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(vis_frame, (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), color, -1)
            cv2.putText(vis_frame, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
            # Draw center point
            center = detection['center']
            cv2.circle(vis_frame, (center[0], center[1]), 5, (255, 255, 255), -1)
        
        return vis_frame 