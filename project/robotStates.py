from enum import Enum

class Rstates(Enum):
    INITIAL=0               # state at the beginning
    SEARCHING=1             # state for searching the aruco markers
    ALIGNING=2              # state for aligning with the aruco marker
    MOVING_FORWARD=3        # state for moving closer to the obstacle
    PREPARING_TO_GRAB= 4    # state for preparing to grab the object   