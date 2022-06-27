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

    def distance(lm1,lm2):
        dy = lm2.xy[1]-lm1.xy[1]
        dx = lm2.xy[0]-lm1.xy[0]
        return (dx**2 + dy**2)**0.5

@dataclass(frozen = True, order = True)
class beacon:
    id : int
    xy : np.ndarray((2))
    r : float

@dataclass(frozen = False, order = True)
class package:
    id : int
    owner : int #landmark id  or robot == 1000 + robot id
    goal : int 
    deliverytime : float

@dataclass()
class mvnormal:
    dim = int
    mu = np.ndarray
    sigma = np.ndarray

#----------------------------- COMMANDS
@dataclass(frozen = True)
class cmd:
    pass

@dataclass(frozen = True)
class goto(cmd):
    lm : landmark

@dataclass(frozen = True)
class move(cmd):
    odom : gtsam.Pose2

@dataclass(frozen = True)
class pickup(cmd):
    p : package
    lm : landmark

@dataclass(frozen = True)
class drop(cmd):
    p : package
    lm : landmark





