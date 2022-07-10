from dataclasses import dataclass
from maildelivery.world import enviorment, location, package
from maildelivery.geometry import pose2
import matplotlib.pyplot as plt
import numpy as np

CONTROL_THETA_THRESHOLD = np.radians(0.01)
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

@dataclass(frozen = True)
class chargeup(action):
    robot_id : int
    loc: location
    time_start : float = 0
    time_end : float = 0

class robot:
    def __init__(self,pose0, id, dt) -> None:
        self.dt = dt
        self.pose : pose2 = pose0
        self.id : int = id
        self.velocity = 1.0 #[m/s]
        self.max_rotate : float = np.pi #np.pi/4
        self.last_location : int = 0
        self.goal_location : int = 0
        self.owned_packages : list[package] = []
        self.max_charge : int = 100
        self.charge : int = 100
        self.f_dist2charge  = lambda dist: 2 * dist #some default function
        self.f_charge2time = lambda missing_charge: missing_charge/100
        self.graphics : list = []
        self.graphics_deadcharge : list = []
       

    def sense(self): #gps like sensor
        return self.pose.t()

    def act(self, a : action, env : enviorment): #perform action on self or enviorment
        if a.robot_id != self.id:
            raise('command given to wrong robot')
        if type(a) is move:
            self.motion_control(a)
            for p in self.owned_packages:
                p.xy = self.pose.t()

            if np.linalg.norm(self.pose.transformTo(a.loc_to.xy)) < REACH_DELTA:
                return True
            else:
                return False
        elif type(a) is pickup:
            if np.linalg.norm(self.pose.transformTo(a.loc.xy)) < REACH_DELTA and \
                np.linalg.norm(self.pose.transformTo(a.p.xy)) < REACH_DELTA: #due to multirobot we can be at location before package arives
                env.packages[a.p.id].owner = self.id #put robot as owner of package
                env.packages[a.p.id].owner_type = 'robot'
                self.owned_packages.append(a.p)
                print(f'robot {self.id} picked up package {a.p.id} from location {a.loc.id}')
                return True
            else:
                return False
        elif type(a) is drop:
            if np.linalg.norm(self.pose.transformTo(a.loc.xy)) < REACH_DELTA:
                env.packages[a.p.id].owner = a.loc.id #put the landmark as owner of package
                env.packages[a.p.id].owner_type = 'location'
                env.packages[a.p.id].xy = a.loc.xy
                self.owned_packages.remove(a.p)
                print(f'robot {self.id} dropped package {a.p.id} at location {a.loc.id}')
                return True
            else:
                return False

        elif type(a) is chargeup:
            if np.linalg.norm(self.pose.transformTo(a.loc.xy)) < REACH_DELTA:
                self.charge = self.max_charge #need to figure this out later
                return True
            else:
                return False

        elif type(a) is wait:
            return True

    def motion_control(self, action : move):
        e_theta = self.pose.bearing(action.loc_to.xy)
        if abs(e_theta) > CONTROL_THETA_THRESHOLD:
            u = np.sign(e_theta)*min(abs(e_theta),self.max_rotate)
            self.pose = self.pose + pose2(0,0,u)
            return

        e_dist = self.pose.range(action.loc_to.xy)
        if e_dist > CONTROL_DIST_THRESHOLD:
            u = min(e_dist,self.velocity * self.dt)

            if self.charge >= self.f_dist2charge(abs(u)):
                self.pose = self.pose + pose2(u,0,0)
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
        self.pose : pose2 = pose0
        self.id : int = id
        self.max_forward : float = 0.25
        self.max_rotate : float = np.pi #np.pi/4
        self.batteries : int = 3
        self.graphics : list = []
        self.width : float = 0.05

    def sense(self): #gps like sensor
        return self.pose.t()

    def act(self, a : action): #perform action on self or enviorment
        if a.robot_id != self.id:
            raise('command given to wrong robot')
        if type(a) is chase:
            self.motion_control(a)
            if np.linalg.norm(self.sense() - a.chased_robot.pose.t()) < REACH_DELTA:
                return True
            else:
                return False

        elif type(a) is charge:
            if np.linalg.norm(self.sense() - a.chased_robot.pose.t()) < REACH_DELTA:
                if self.batteries > 0:
                    a.chased_robot.charge = a.chased_robot.max_charge
                    self.batteries = self.batteries - 1
                return True
            else:
                return False

    def motion_control(self, action : chase):
        e_theta = self.pose.bearing(action.chased_robot.t())
        if abs(e_theta) > CONTROL_THETA_THRESHOLD:
            u = np.sign(e_theta)*min(abs(e_theta),self.max_rotate)
            self.pose = self.pose + pose2(0,0,u)
            return

        e_dist = self.pose.range(action.chased_robot.t())
        if e_dist > CONTROL_DIST_THRESHOLD:
            u = min(e_dist,self.max_forward)
            self.pose = self.pose + pose2(u,0,0)
            return

    def plot(self,ax):
        if self.graphics is not None:
            [g.remove() for g in self.graphics]
        self.graphics = plot_drone(ax,self)

#---------------------------------------------------------------------------
#--------------------------------PLOTTING FUNCTIONS-------------------------
#---------------------------------------------------------------------------

def plot_robot(ax : plt.Axes , r : robot, scale = 20, color = 'b', textcolor = 'magenta'):
        pose = r.pose
        u = np.cos(pose.theta)
        v = np.sin(pose.theta)
        graphics_quiver = ax.quiver(pose.x,pose.y,u,v, color = color, scale = scale, width = 0.02)
        graphics_circle = ax.scatter(pose.x, pose.y, marker = 'o', c = 'none',\
             s = 700, edgecolors = color)
        graphics_txt = ax.text(pose.x,pose.y,f"{r.id}    ", color = color, horizontalalignment = 'right', verticalalignment = 'center')
        return [graphics_quiver,graphics_circle,graphics_txt]

def plot_robot_deadcharge(ax, r : robot, scale = 20, color = 'r'):
    pose = r.pose
    g1 = ax.scatter(pose.x, pose.y, marker = 'o', c = 'none',\
             s = 800, edgecolors = color)
    g2 = ax.scatter(pose.x, pose.y, marker = 'o', c = 'none',\
             s = 1200, edgecolors = color)
    return [g1,g2]

def plot_drone(ax, d : drone, scale = 20, color = 'r'):
        pose = d.pose
        
        p_ego = d.width * np.array([[-1,-1],
                             [1,-1],
                             [1,1],
                             [-1,1]])

        graphics = []
        for p in p_ego:
            pdraw = pose.transformFrom(p)
            graphics.append(ax.scatter(pdraw[0],pdraw[1],marker = 'o', c = 'red', s = 100))
        return graphics

        





    
        

