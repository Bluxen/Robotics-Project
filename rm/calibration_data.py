from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
import pickle
SHARE = get_package_share_directory('project')+'/'

class CalibrationData:
    mtx = None
    dist = None

CALIBRATION_DATA_FILE = SHARE + 'calibration_data.pickle'
def load_calibration():
    data = CalibrationData()
    with open(CALIBRATION_DATA_FILE, 'rb') as f:
        data.mtx, data.dist, _, _ = pickle.load(f)
    return data