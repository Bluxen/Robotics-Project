from ament_index_python.packages import get_package_share_directory
SHARE = get_package_share_directory('project')+'/'

import pickle
import cv2
import numpy as np

from cv_bridge import CvBridge
from .calibration_data import load_calibration

def mktransform(tvec, rvec):
    R, _ = cv2.Rodrigues(rvec)
    Rt = np.hstack((R, tvec.T))
    return np.vstack((Rt, np.array((0, 0, 0, 1))))

class VS():
    def __init__(self, FOV=120, logger=None):
        self.FOV = FOV
        self.logger=logger
        self.data = load_calibration()
        
    # def get_corners(self, rvecs, tvecs, objp):
    #     corners = []
    #     for rvec, tvec in zip(rvecs, tvecs):
    #         T = mktransform(tvec, rvec)
    #         corners.append([T @ (*obj, 0) for obj, in objp])
    #     return corners
    
    # def target_features(self, objp):
    #     l = np.min(self.data.size)*2
    #     c = np.array(self.data.size)/2
    #     return [c + (obj[:2]*l) for obj, in objp]

    # @property
    # def xy_to_uv(self):
    #     _, pw, ph = self.data.pp
    #     w, h      = self.data.size
    #     return np.matrix([
    #         [1/pw, 0,    w/2],
    #         [0,    1/ph, h/2],
    #         [0,    0,    1  ]
    #     ])
    
    # def interaction(self, feature, measured):
    #     f         = self.data.f
    #     _, pw, ph = self.data.pp
    #     udiff     = measured[0] - (self.data.size[0] / 2)
    #     vdiff     = measured[1] - (self.data.size[1] / 2)
    #     Z         = feature[2]
    #     return np.matrix([
    #         [-f/(pw*Z), 0, +udiff/Z, -(udiff*vdiff*ph)/f, -f-((udiff*udiff*pw)/f), -vdiff],
    #         [0, -f/(ph*Z), -vdiff/Z, +f+((vdiff*vdiff*pw)/f), +(udiff*vdiff*pw)/f, +udiff]
    #     ])

    # def vel_to_target(self, fot, mf, df):
    #     # for measured, desired, feature in zip(mf, df, fot):
    #     #     s = desired - measured
    #     #     L = self.interaction(feature, measured)

    #     # Select the first three features
    #     fot = fot[:3]
    #     mf = mf[:3]
    #     df = df[:3]

    #     # Compute Interaction Matrix
    #     L = np.vstack([self.interaction(feature, measured) for feature, measured in zip(fot, mf)])
        
    #     # Compute uv velocity
    #     s = np.hstack([desired - measured for desired, measured in zip(df, mf)])

    #     Li = np.linalg.inv(L)
    #     v = (Li @ s) * 0.1
    #     return v.tolist()

    def log(self, str):
        if self.logger is not None:
            self.logger.info(f"{str}")