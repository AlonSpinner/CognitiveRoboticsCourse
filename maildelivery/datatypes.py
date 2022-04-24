from dataclasses import dataclass
import numpy as np
import gtsam

@dataclass(frozen = True, order = True)
class landmark:
    id : int
    xy : np.ndarray((2))
    type : str

    def angle(lm1,lm2): #akeen to lm2 - lm1
        dy = lm2.xy[1]-lm1.xy[1]
        dx = lm2.xy[0]-lm1.xy[0]
        return np.arctan2(dy,dx)

@dataclass(frozen = True, order = True)
class beacon:
    id : int
    xy : np.ndarray((2))
    r : float

@dataclass(frozen = False, order = True)
class package:
    id : int
    location : int #landmark or robot == 1000
    timeLeft : float

@dataclass()
class mvnormal:
    dim = int
    mu = np.ndarray
    sigma = np.ndarray