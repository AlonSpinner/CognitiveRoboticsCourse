import gtsam
from maildelivery.datatypes import landmark, package, mvnormal
import maildelivery.plotting as plotting
import numpy as np

class robot:
    def __init__(self,pose0) -> None:
        self.pose : gtsam.Pose2 = pose0
        self.packages : list[int] #package indcies which robot holds
        self.reachDelta = 1.0 
    
    def move(self, odom: gtsam.Pose2):
        self.pose = self.pose.compose(odom)
        
        success = True
        return success

    def reach(self, lm: landmark):
        #set x,y of pose to be the same as landmark
        if np.linalg.norm(lm.xy-self.pose.translation()) < self.reachDelta:
            success = True    
            self.pose = gtsam.Pose2(lm.xy[0],lm.xy[1],self.pose.theta())
        else:
            success = False
        return success

    def pickup(self, p : package):
        success = True
        return success

    def drop(self, p : package):
        success = True
        return success

    def plot(self,ax):
        return(plotting.plot_robot(ax,self.pose))

    
        

