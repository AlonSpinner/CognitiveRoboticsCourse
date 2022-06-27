import gtsam
from dataclasses import dataclass
from maildelivery.world import enviorment, landmark, package
import matplotlib.pyplot as plt
import numpy as np

CONTROL_THETA_THRESHOLD = np.radians(0.001)
CONTROL_DIST_THRESHOLD = 0.001
REACH_DELTA = 0.01

@dataclass(frozen = True)
class action:
    robot_id : int #to be overwritten

@dataclass(frozen = True)
class move(action):
    robot_id : int
    lm_from : landmark
    lm_to : landmark
    time_start : float = 0
    time_end : float = 0

@dataclass(frozen = True)
class pickup(action):
    robot_id : int
    p : package
    lm: landmark
    time_start : float = 0
    time_end : float = 0

@dataclass(frozen = True)
class drop(action):
    robot_id : int
    p : package
    lm: landmark
    time_start : float = 0
    time_end : float = 0

class robot:
    def __init__(self,pose0, id) -> None:
        self.pose : gtsam.Pose2 = pose0
        self.id : int = id
        self.max_forward : float = 0.25
        self.max_rotate : float = np.pi/4
        self.last_landmark : int = 0
        self.goal_landmark : int = 0

    def sense(self): #gps like sensor
        return self.pose.translation()

    def act(self, a : action, env : enviorment): #perform action on self or enviorment
        if a.robot_id != self.id:
            raise('command given to wrong robot')
        if type(a) is move:
            self.motion_control(a)
            if np.linalg.norm(self.sense() - a.lm_to.xy) < REACH_DELTA:
                return True
            else:
                return False
        elif type(a) is pickup:
            if np.linalg.norm(self.sense() - a.lm.xy) < REACH_DELTA:
                env.packages[a.p.id].owner = self.id #put robot as owner of package
                return True
            else:
                return False
        elif type(a) is drop:
            if np.linalg.norm(self.sense() - a.lm.xy) < REACH_DELTA:
                env.packages[a.p.id].owner = a.lm.id #put the landmark as owner of package
                return True
            else:
                return False

    def motion_control(self, action : move):
        e_theta = self.pose.bearing(action.lm_to_xy).theta()
        if abs(e_theta) > CONTROL_THETA_THRESHOLD:
            u = np.sign(e_theta)*min(abs(e_theta),self.max_rotate)
            self.pose = self.pose.compose((gtsam.Pose2(0,0,u)))
            return

        e_dist = self.pose.range(action.lm_to_xy)
        if e_dist > CONTROL_DIST_THRESHOLD:
            u = min(e_dist,self.max_forward)
            self.pose = self.pose.compose((gtsam.Pose2(u,0,0)))
            return

    def plot(self,ax):
        return(plot_robot(ax,self.pose))

#---------------------------------------------------------------------------
#--------------------------------PLOTTING FUNCTIONS-------------------------
#---------------------------------------------------------------------------

def plot_robot(ax , pose : gtsam.Pose2, scale = 20, color = 'b'):
        u = np.cos(pose.theta())
        v = np.sin(pose.theta())
        graphics_quiver = ax.quiver(pose.x(),pose.y(),u,v, color = color, scale = scale, width = 0.02)
        graphics_circle = ax.add_patch(plt.Circle((pose.x(),pose.y()),0.1,fill = False, color = 'b'))
        return [graphics_quiver,graphics_circle]





    
        

