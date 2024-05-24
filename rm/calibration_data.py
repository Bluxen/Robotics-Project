from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
import cv2
import pickle
import numpy as np
import math
SHARE = get_package_share_directory('project')+'/'

@dataclass
class CalibrationData:
    mtx     = None
    dist    = None
    size    = (640, 360)    # Image size in pixels (TODO currently hardcoded to (640, 360))
    _f      = None          # Focal length
    _fovx   = None          # FOV x in degrees
    _fovy   = None          # FOV y in degrees
    _pp     = None          # Principal point
    _ar     = None          # Pixel aspect ratio

    def _setMtxValues(self):
        self._fovx, self._fovy, self._f, self._pp, self._ar = \
            cv2.calibrationMatrixValues(
            self.mtx,           # Camera matrix
            self.size,          # Image size in pixels
            3.70, 2.60    # Aperture size in m
            )

    @property
    def f(self):
        if self._f is None and self.mtx is not None: self._setMtxValues()
        return self._f
    
    @property
    def fov(self):
        if (self._fovx is None or self._fovy is None) and self.mtx is not None: self._setMtxValues()
        return self._fovx, self._fovy
    
    @property
    def pp(self):
        """ Get the camera's principal point, along with pixel width and height """
        if self._pp is None and self.mtx is not None: self._setMtxValues()
        if self._pp is None: return None, None, None
        return self._pp, (self._pp[0] * 2) / self.size[0], (self._pp[1] * 2) / self.size[1]

@dataclass
class OptimalCalibrationData(CalibrationData):
    size    = (640, 360)    # Image size in pixels (TODO currently hardcoded to (640, 360))
    dist    = None
    _fovx   = 120.              # FOV x in degrees
    _fovy   = None              # FOV y in degrees
    _pp     = None              # Principal point
    _ar     = None              # Pixel aspect ratio
    _f      = size[0] / math.tan(math.radians(_fovx) / 2) # Focal length
    mtx     = np.matrix([
        [_f, 0., size[0]/2],
        [0., _f, size[1]/2],
        [0., 0., 1.]
    ], dtype=float)

CALIBRATION_DATA_FILE = SHARE + 'calibration_data.pickle'
def load_calibration():
    # data = CalibrationData()
    data = OptimalCalibrationData()
    # with open(CALIBRATION_DATA_FILE, 'rb') as f:
    #     data.mtx, data.dist, _, _ = pickle.load(f)
    return data