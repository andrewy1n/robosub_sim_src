import cv2
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
import os   

# camera coordinate system, relative to left camera:
# +x: rightward
# +y: downward
# +z: forward

# Load calibration data (intrinsic matrix and distortion coefficients of both cameras, rotation and translation matrices between cameras)
package_dir = get_package_share_directory('gate_detection')
data = np.load(os.path.join(package_dir, 'config', 'calibration_data.npz'))

kL = data["kL"] # intrinsic matrix of left camera
dL = data["dL"] # distortion coefficients of left cameras
kR = data["kR"] # intrinsic matrix of right camera
dR = data["dR"] # distortion coefficients of right cameras
R = data["R"] # rotation matrix between cameras
T = data["T"] # rotation matrix between cameras

image_size = (1920, 1080) # (w, h)

'''
calculate distance using direct linear transformation (DLT)
input:
    * projection matrix P1
    * point in image 1 point1
    * projection matrix P2
    * point in image 2 point2
output:
    * triangulated 3D point in Euclidean coordinates
'''
def DLT(P1, P2, point1, point2):
        # flatten points as needed (from tuple or list)
        point1 = np.array(point1)
        point2 = np.array(point2)
 
        A = [point1[1]*P1[2,:] - P1[1,:],
             P1[0,:] - point1[0]*P1[2,:],
             point2[1]*P2[2,:] - P2[1,:],
             P2[0,:] - point2[0]*P2[2,:]
            ]
        A = np.array(A).reshape((4,4))
 
        B = A.transpose() @ A
        from scipy import linalg
        U, s, Vh = linalg.svd(B, full_matrices = False)
 
        return Vh[3,0:3]/Vh[3,3]

'''
given following matrices and points, triangulate the distance:
input:
    * intrinsic matrix K of left camera
    * intrinsic matrix K of right camera
    * rotation matrix R between cameras
    * translation matrix T between cameras
    * pair of (x, y) coordinates from images, to calculate distance
'''
def triangulate(mtxL, mtxR, R, T, pointL, pointR, dL, dR):
    npL = np.array([pointL], dtype=np.float32)
    npR = np.array([pointR], dtype=np.float32)

    # projection matrices for left camera
    RT1 = np.concatenate([np.eye(3), [[0],[0],[0]]], axis = -1)
    PL = mtxL @ RT1
 
    # projection matrices for right camera
    RT2 = np.concatenate([R, T], axis = -1)
    PR = mtxR @ RT2 #projection matrix for C2

    # # call DLT on points
    points3Ddlt = []
    points3Ddlt.append(DLT(PL, PR, npL[0], npR[0]))
    points3Ddlt = np.array(points3Ddlt)

    # undistort points
    undistortL = cv2.undistortPoints(npL.reshape(-1, 1, 2), mtxL, dL)
    undistortR = cv2.undistortPoints(npR.reshape(-1, 1, 2), mtxR, dR)

    # triangulate undistorted points
    # pointUndistortedTriangulated = cv2.triangulatePoints(PL, PR, undistortL.T, undistortR.T)
    # points3D_ = (pointUndistortedTriangulated[:3] / pointUndistortedTriangulated[3]).T

    # triangulate using original, undistorted points
    pointTriangulated = cv2.triangulatePoints(PL, PR, npL.T, npR.T)
    points3D = (pointTriangulated[:3] / pointTriangulated[3]).T

    # for testing purposes: print out distances calculated with each method
    # print("Undistorted distance")
    # print(points3D_)
    # print("Distance calculated")
    # print(points3D)
    # print(points3Ddlt)

    return round(points3D[0][2], 3)

'''
print out image, alongside point (x, y) as red dot
'''
def plotImage(img, x, y, imgPath=None):
    image = mpimg.imread(img)
    width, height = image.shape[:2]

    # plt.imshow(image)
    # plt.scatter(x, y, color='red', s=10)

    # set image dimensions
    dpi = 100
    figsize = (width / dpi, height / dpi)
    fig = plt.figure(figsize=figsize, dpi=dpi)

    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(image)
    ax.scatter([x], [y], color='red', s=30)
    ax.axis('off')

    if not imgPath:
        plt.show()
        return
    # save image as file
    plt.axis('off')
    plt.savefig(imgPath, dpi=dpi, bbox_inches='tight', pad_inches=0)
    plt.show()

# test data for 0.5m
pointL0_5m = (1040, 315)
pointR0_5m = (940, 343)

# test data for 1m
pointL1m = (1070, 435)
pointR1m = (1065, 465)

# plotImage('./testImages/it0.5_imgCam2.jpg', pointR0_5m[0], pointR0_5m[1], './testImages/it0.5_imgCam2_annotated.jpg')
# plotImage('./testImages/it0.5_imgCam0.jpg', pointL0_5m[0], pointL0_5m[1], './testImages/it0.5_imgCam0_annotated.jpg')

# print("Testing images 0.5m away:")
# triangulate(kL, kR, R, T, pointL0_5m, pointR0_5m, dL, dR)

# plotImage('./testImages/it1_imgCam2.jpg', pointR1m[0], pointR1m[1], './testImages/it1_imgCam2_annotated.jpg')
# plotImage('./testImages/it1_imgCam0.jpg', pointL1m[0], pointL1m[1], './testImages/it1_imgCam0_annotated.jpg')

# print()

# print("Testing images 1m away:")
# triangulate(kL, kR, R, T, pointL1m, pointR1m, dL, dR)
