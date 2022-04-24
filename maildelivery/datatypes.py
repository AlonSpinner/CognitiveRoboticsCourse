from dataclasses import dataclass
import numpy as np
import gtsam

@dataclass(frozen = True, order = True)
class landmark:
    id : int
    xy : np.ndarray((2))
    type : str

@dataclass(frozen = True, order = True)
class beacon:
    id : int
    xy : np.ndarray((2))
    r : float

def pose2ToNumpy(pose2: gtsam.Pose2):
    return np.array([pose2.x(),pose2.y(),pose2.theta()])
