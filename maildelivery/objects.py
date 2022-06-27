from dataclasses import dataclass
import gtsam
from maildelivery.datatypes import cmd, move, pickup, drop
from maildelivery.objects import robot
import maildelivery.plotting as plotting
import numpy as np

CONTROL_THETA_THRESHOLD = np.radians(0.001)
CONTROL_DIST_THRESHOLD = 0.001

class robot:
    def __init__(self,pose0, id) -> None:
        self.pose : gtsam.Pose2 = pose0
        self.max_forward : float = 1.0
        self.max_rotate : float = np.pi/4
        self.id : int = id
        self.last_landmark : int = 0
        self.goal_landmark : int = 0


    def control(self, _cmd : move):
        #given r and goto command, produce proper move command
        e_theta = self.pose.bearing(_cmd.lm_to_xy).theta()
        if abs(e_theta) > CONTROL_THETA_THRESHOLD:
            u = np.sign(e_theta)*min(abs(e_theta),self.max_rotate)
            self.pose = self.pose.compose((gtsam.Pose2(0,0,u)))

        e_dist = self.pose.range(_cmd.lm_to_xy)
        if e_dist > CONTROL_DIST_THRESHOLD:
            u = min(e_dist,self.max_forward)
            self.pose = self.pose.compose((gtsam.Pose2(u,0,0)))

    def act(self, _cmd : cmd):
        if _cmd.robot_id != self.id:
            raise('command given to wrong robot')
        if type(_cmd) == move:
            return self.move(_cmd)
        elif type(_cmd) == pickup:
            return self.pickup(_cmd)
        elif type(_cmd) == drop:
            return self.drop(_cmd)


    def pickup(self, cmd: pickup):
        lm = cmd.lm
        p = cmd.p
        if np.linalg.norm(lm.xy-self.pose.translation()) < self.reachDelta:
            if p.location != 1000: #robot holds package
                p.location  = 1000
                success = True
        else:
            success = False      
        return success

    def drop(self, cmd: drop):
        lm = cmd.lm
        p = cmd.p
        if np.linalg.norm(lm.xy-self.pose.translation()) < self.reachDelta:
            if p.location == 1000: #robot holds package
                p.location  = lm.id
                success = True
        else:
            success = False
        return success

    def move(self, cmd : move):
        self.pose = self.pose.compose(cmd.odom)
        
        success = True
        return success

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


    
        

