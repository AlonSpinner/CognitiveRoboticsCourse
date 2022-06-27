from dataclasses import dataclass
import numpy as np
import gtsam
#----------------------------- COMMANDS
@dataclass(frozen = True)
class cmd:
    robot_id : int #to be overwritten

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
    p_id : int
    lm_xy : np.ndarray
    time_start : float = 0
    time_end : float = 0

@dataclass(frozen = True)
class drop(cmd):
    robot_id : int
    p_id : int
    lm_xy : np.ndarray
    time_start : float = 0
    time_end : float = 0





