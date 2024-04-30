
import scipy.linalg
import pyquaternion
import numpy as np

class Pose:

    def __init__(self, R, pos):
        self.R = R
        self.pos = pos

        assert type(R) is np.ndarray
        assert type(pos) is np.ndarray
        assert pos.shape[1] == 1

