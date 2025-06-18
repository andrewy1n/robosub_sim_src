import os

from ament_index_python import get_package_share_directory
from .gate_detection_yolo import find_gate_center
from .stereo_vision import triangulate
import numpy as np

# Load calibration data (intrinsic matrix and distortion coefficients of both cameras, rotation and translation matrices between cameras)
package_dir = get_package_share_directory('gate_detection')
data = np.load(os.path.join(package_dir, 'config', 'calibration_data.npz'))

kL = data["kL"] # intrinsic matrix of left camera
dL = data["dL"] # distortion coefficients of left cameras
kR = data["kR"] # intrinsic matrix of right camera
dR = data["dR"] # distortion coefficients of right cameras
R = data["R"] # rotation matrix between cameras
T = data["T"] # rotation matrix between cameras

def find_gate_distance(imageLeft, imageRight) -> int:
    centerL = find_gate_center(imageLeft, test=False)
    centerR = find_gate_center(imageRight, test=False)

    dist = triangulate(kL, kR, R, T, centerL, centerR, dL, dR)

    return dist