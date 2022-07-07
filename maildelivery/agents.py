import gtsam
from dataclasses import dataclass
from maildelivery.world import enviorment, location, package
import matplotlib.pyplot as plt
import numpy as np

CONTROL_THETA_THRESHOLD = np.radians(0.001)
CONTROL_DIST_THRESHOLD = 0.001
REACH_DELTA = 0.001

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
        self.max_forward : float = 0.001
        self.max_rotate : float = np.pi #np.pi/4
        self.last_location : int = 0
        self.goal_location : int = 0
        self.owned_packages : list[package] = []
        self.max_charge : int = 100
        self.charge : int = 100
        self.graphics : list = []
        self.graphics_deadcharge : list = []
        self.f_dist2charge  = lambda dist: 2 * dist #some default function

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
            if np.linalg.norm(self.sense() - a.loc.xy) < REACH_DELTA and \
                np.linalg.norm(self.sense() - a.p.xy) < REACH_DELTA: #due to multirobot we can be at location before package arives
                env.packages[a.p.id].owner = self.id #put robot as owner of package
                env.packages[a.p.id].owner_type = 'robot'
                self.owned_packages.append(a.p)
                print(f'robot {self.id} picked up package {a.p.id} from location {a.loc.id}')
                return True
            else:
                return False
        elif type(a) is drop:
            if np.linalg.norm(self.sense() - a.loc.xy) < REACH_DELTA:
                env.packages[a.p.id].owner = a.loc.id #put the landmark as owner of package
                env.packages[a.p.id].owner_type = 'location'
                env.packages[a.p.id].xy = a.loc.xy
                self.owned_packages.remove(a.p)
                print(f'robot {self.id} dropped package {a.p.id} at location {a.loc.id}')
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

            if self.charge >= abs(u):
                self.pose = self.pose.compose((gtsam.Pose2(u,0,0)))
                self.charge = self.charge - self.f_dist2charge(abs(u))
            return

    def plot(self,ax):
        if self.graphics is not None:
            [g.remove() for g in self.graphics]
        self.graphics = plot_robot(ax,self)

    def plot_deadcharge(self,ax):
        self.graphics_deadcharge = plot_robot_deadcharge(ax,self)

@dataclass(frozen = True)
class chase(action):
    robot_id : int #drone id here
    chased_robot : robot

@dataclass(frozen = True)
class charge(action):
    robot_id : int #drone id here
    chased_robot : robot

class drone:
    def __init__(self, pose0, id) -> None:
        self.pose : gtsam.Pose2 = pose0
        self.id : int = id
        self.max_forward : float = 0.25
        self.max_rotate : float = np.pi #np.pi/4
        self.batteries : int = 3
        self.graphics : list = []
        self.width : float = 0.02

    def sense(self): #gps like sensor
        return self.pose.translation()

    def act(self, a : action): #perform action on self or enviorment
        if a.robot_id != self.id:
            raise('command given to wrong robot')
        if type(a) is chase:
            self.motion_control(a)
            if np.linalg.norm(self.sense() - a.chased_robot.pose.translation()) < REACH_DELTA:
                return True
            else:
                return False

        elif type(a) is charge:
            if np.linalg.norm(self.sense() - a.chased_robot.pose.translation()) < REACH_DELTA:
                if self.batteries > 0:
                    a.chased_robot.charge = a.chased_robot.max_charge
                    self.batteries = self.batteries - 1
                return True
            else:
                return False

    def motion_control(self, action : chase):
        e_theta = self.pose.bearing(action.chased_robot.translation()).theta()
        if abs(e_theta) > CONTROL_THETA_THRESHOLD:
            u = np.sign(e_theta)*min(abs(e_theta),self.max_rotate)
            self.pose = self.pose.compose((gtsam.Pose2(0,0,u)))
            return

        e_dist = self.pose.range(action.chased_robot.translation())
        if e_dist > CONTROL_DIST_THRESHOLD:
            u = min(e_dist,self.max_forward)
            self.pose = self.pose.compose((gtsam.Pose2(u,0,0)))
            return

    def plot(self,ax):
        if self.graphics is not None:
            [g.remove() for g in self.graphics]
        self.graphics = plot_drone(ax,self)

#---------------------------------------------------------------------------
#--------------------------------PLOTTING FUNCTIONS-------------------------
#---------------------------------------------------------------------------

def plot_robot(ax , r : robot, scale = 20, color = 'b'):
        TEXT_OFFSET = 0.04
        RADIUS = 0.1
        pose = r.pose
        u = np.cos(pose.theta())
        v = np.sin(pose.theta())
        graphics_quiver = ax.quiver(pose.x(),pose.y(),u,v, color = color, scale = scale, width = 0.02)
        graphics_circle = ax.add_patch(plt.Circle((pose.x(),pose.y()),RADIUS,fill = False, color = color))
        graphics_txt = ax.text(r.pose.translation()[0]-2*TEXT_OFFSET,r.pose.translation()[1],r.id, color = color)
        return [graphics_quiver,graphics_circle,graphics_txt]

def plot_robot_deadcharge(ax, r : robot, scale = 20, color = 'r'):
    pose = r.pose
    g1 = ax.add_patch(plt.Circle((pose.x(),pose.y()),0.15,fill = False, color = color))
    g2 = ax.add_patch(plt.Circle((pose.x(),pose.y()),0.25,fill = False, color = color))
    return [g1,g2]

def plot_drone(ax, d : drone, scale = 20, color = 'r'):
        pose = d.pose
        
        p_ego = d.width * np.array([[-1,-1],
                             [1,-1],
                             [1,1],
                             [-1,1]])

        graphics = []
        for p in p_ego:
            graphics.append(ax.add_patch(plt.Circle(pose.transformFrom(p),d.width,fill = True, color = color)))
        return graphics

        





    
        

