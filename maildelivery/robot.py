import gtsam
from maildelivery.datatypes import landmark, package, mvnormal
from maildelivery.datatypes import move, pickup, drop
import maildelivery.plotting as plotting
import numpy as np

class robot:
    def __init__(self,pose0) -> None:
        self.pose : gtsam.Pose2 = pose0
        self.reachDelta = 1.0 
        self.max_forward = 1.0
        self.max_rotate = np.pi/4
    
    def move(self, cmd : move):
        self.pose = self.pose.compose(cmd.odom)
        
        success = True
        return success

    def pickup(self, cmd: pickup):
        lm = cmd.lm
        p = cmd.p
        if np.linalg.norm(lm.xy-self.pose.translation()) < self.reachDelta:
            if p.location != 1000:
                p.location  = 1000
                success = True
        else:
            success = False      
        return success

    def drop(self, cmd: drop):
        lm = cmd.lm
        p = cmd.p
        if np.linalg.norm(lm.xy-self.pose.translation()) < self.reachDelta:
            if p.location == 1000:
                p.location  = lm.id
                success = True
        else:
            success = False
        return success

    def plot(self,ax):
        return(plotting.plot_robot(ax,self.pose))

    
        

