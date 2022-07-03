import gtsam
from dataclasses import dataclass
from maildelivery.world import enviorment, location, package
import matplotlib.pyplot as plt
import numpy as np

CONTROL_THETA_THRESHOLD = np.radians(0.001)
CONTROL_DIST_THRESHOLD = 0.001
REACH_DELTA = 0.01

@dataclass(frozen = True)
class action:
    robot_id : int #to be overwritten

class wait(action):
    robot_id : int

@dataclass(frozen = True)
class move(action):
    robot_id : int
    loc_from : location
    loc_to : location
    time_start : float = 0
    time_end : float = 0

@dataclass(frozen = True)
class pickup(action):
    robot_id : int
    p : package
    loc: location
    time_start : float = 0
    time_end : float = 0

@dataclass(frozen = True)
class drop(action):
    robot_id : int
    p : package
    loc: location
    time_start : float = 0
    time_end : float = 0

class robot:
    def __init__(self,pose0, id) -> None:
        self.pose : gtsam.Pose2 = pose0
        self.id : int = id
        self.max_forward : float = 0.25
        self.max_rotate : float = np.pi #np.pi/4
        self.last_location : int = 0
        self.goal_location : int = 0
        self.owned_packages : list[package] = []
        self.graphics : list = []

    def sense(self): #gps like sensor
        return self.pose.translation()

    def act(self, a : action, env : enviorment): #perform action on self or enviorment
        if a.robot_id != self.id:
            raise('command given to wrong robot')
        if type(a) is move:
            self.motion_control(a)
            for p in self.owned_packages:
                p.xy = self.pose.translation()

            if np.linalg.norm(self.sense() - a.loc_to.xy) < REACH_DELTA:
                return True
            else:
                return False
        elif type(a) is pickup:
            if np.linalg.norm(self.sense() - a.loc.xy) < REACH_DELTA:
                env.packages[a.p.id].owner = self.id #put robot as owner of package
                env.packages[a.p.id].owner_type = 'robot'
                self.owned_packages.append(a.p)
                return True
            else:
                return False
        elif type(a) is drop:
            if np.linalg.norm(self.sense() - a.loc.xy) < REACH_DELTA:
                env.packages[a.p.id].owner = a.loc.id #put the landmark as owner of package
                env.packages[a.p.id].owner_type = 'landmark'
                self.owned_packages.remove(a.p)
                return True
            else:
                return False
        elif type(a) is wait:
            return True

    def motion_control(self, action : move):
        e_theta = self.pose.bearing(action.loc_to.xy).theta()
        if abs(e_theta) > CONTROL_THETA_THRESHOLD:
            u = np.sign(e_theta)*min(abs(e_theta),self.max_rotate)
            self.pose = self.pose.compose((gtsam.Pose2(0,0,u)))
            return

        e_dist = self.pose.range(action.loc_to.xy)
        if e_dist > CONTROL_DIST_THRESHOLD:
            u = min(e_dist,self.max_forward)
            self.pose = self.pose.compose((gtsam.Pose2(u,0,0)))
            return

    def plot(self,ax):
        if self.graphics is not None:
            [g.remove() for g in self.graphics]
        self.graphics = plot_robot(ax,self)

#---------------------------------------------------------------------------
#--------------------------------PLOTTING FUNCTIONS-------------------------
#---------------------------------------------------------------------------

def plot_robot(ax , r : robot, scale = 20, color = 'b'):
        pose = r.pose
        u = np.cos(pose.theta())
        v = np.sin(pose.theta())
        graphics_quiver = ax.quiver(pose.x(),pose.y(),u,v, color = color, scale = scale, width = 0.02)
        graphics_circle = ax.add_patch(plt.Circle((pose.x(),pose.y()),0.1,fill = False, color = 'b'))
        return [graphics_quiver,graphics_circle]





    
        

