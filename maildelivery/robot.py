import gtsam
from maildelivery.datatypes import landmark, package, mvnormal
from maildelivery.datatypes import goto, move, pickup, drop
import maildelivery.plotting as plotting
import numpy as np

class robot:
    def __init__(self,pose0) -> None:
        self.pose : gtsam.Pose2 = pose0
        self.max_forward = 1.0
        self.max_rotate = np.pi/4
        self.reachDelta = 0.001

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

    
        

