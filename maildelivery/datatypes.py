from dataclasses import dataclass
import numpy as np
import gtsam

ROBOT_INDEX_SHIFT = 1000

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

@dataclass(frozen = False, order = True)
class package:
    id : int
    owner : int #landmark id  or robot == ROBOT_INDEX_SHIFT + robot id
    goal : int 
    deliverytime : float

#----------------------------- COMMANDS
@dataclass(frozen = True)
class cmd:
    robot_id : int

@dataclass(frozen = True)
class move(cmd):
    robot_id : int
    lm_from_xy : np.ndarray
    lm_to_xy : np.ndarray
    time_start : float = 0
    time_end : float = 0

@dataclass(frozen = True)
class pickup(cmd):
    robot_id : int
    p_id : package
    lm_xy : np.ndarray
    time_start : float = 0
    time_end : float = 0

@dataclass(frozen = True)
class drop(cmd):
    robot_id : int
    p_id : package
    lm_xy : np.ndarray
    time_start : float = 0
    time_end : float = 0





