from ament_index_python.packages import get_package_share_directory
SHARE = get_package_share_directory('project')+'/'

import pickle
import cv2
from cv2 import aruco


class VS:
    CALIBRATION_DATA_FILE = SHARE + 'calibration_data.pickle'
    def __init__(self, width, height, FOV=120):
        self.camera_width = width
        self.camera_height = height
        self.FOV = FOV

        with open(self.CALIBRATION_DATA_FILE, 'rb') as f:
            self.mtx, self.dist, self.rvecs, self.tvecs = pickle.load(f)
        
    def draw_pose(self, image, corners, ids, size):
        for i in range(len(ids)):
            rvecs, tvecs, objp = aruco.estimatePoseSingleMarkers(corners, size, self.mtx, self.dist)
            cv2.drawFrameAxes(image, self.mtx, self.dist, rvecs[i], tvecs[i], size * 1.5, 2)

        return image
        # cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        
        # if(estimatePose) {
        # for(unsigned int i = 0; i < ids.size(); i++)
        # cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
        # }