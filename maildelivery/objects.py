from dataclasses import dataclass
import gtsam
from maildelivery.datatypes import action, move, pickup, drop
import maildelivery.plotting as plotting
import numpy as np

CONTROL_THETA_THRESHOLD = np.radians(0.001)
CONTROL_DIST_THRESHOLD = 0.001
REACH_DELTA = 0.01

class robot:
    def __init__(self,pose0, id) -> None:
        self.pose : gtsam.Pose2 = pose0
        self.id : int = id
        self.max_forward : float = 1.0
        self.max_rotate : float = np.pi/4
        self.last_landmark : int = 0
        self.goal_landmark : int = 0

    def sense(self): #gps like sensor
        return self.pose.translation()

    def act(self, a : action):
        if a.robot_id != self.id:
            raise('command given to wrong robot')
        if type(a) == move:
            self.motion_control(a)
            if np.linalg.norm(self.sense() - a.lm_to_xy) < REACH_DELTA:
                return True
            else:
                return False
        elif type(a) == pickup:
            if np.linalg.norm(self.sense() - a.lm_xy) < REACH_DELTA:
                return True
            else:
                return False
        elif type(a) == drop:
            if np.linalg.norm(self.sense() - a.lm_xy) < REACH_DELTA:
                return True
            else:
                return False

    def motion_control(self, action : move):
        e_theta = self.pose.bearing(action.lm_to_xy).theta()
        if abs(e_theta) > CONTROL_THETA_THRESHOLD:
            u = np.sign(e_theta)*min(abs(e_theta),self.max_rotate)
            self.pose = self.pose.compose((gtsam.Pose2(0,0,u)))

        e_dist = self.pose.range(action.lm_to_xy)
        if e_dist > CONTROL_DIST_THRESHOLD:
            u = min(e_dist,self.max_forward)
            self.pose = self.pose.compose((gtsam.Pose2(u,0,0)))

    def plot(self,ax):
        return(plotting.plot_robot(ax,self.pose))

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


    
        

