import maildelivery.plotting as plotting
from maildelivery.objects import robot, landmark, package
from maildelivery.datatypes import move, pickup, drop
import matplotlib.pyplot as plt
import numpy as np

REACH_DELTA = 0.001

class enviorment:    
    def __init__(self,beacons,landmarks,connectivityList, packages = None):
        #instance attributes
        self.landmarks : list[landmark]  = sorted(landmarks)
        self.connectivityList : list[(int,int)]  = connectivityList
        self.packages : list[package] = packages

    def plot(self,ax : plt.Axes = None):
        if ax == None:
            fig , ax = plotting.spawnWorld()

        for b in self.beacons:
            plotting.plot_beacon(ax,b)
        for lm in self.landmarks:
            if lm.type == "house":
                plotting.plot_house(ax,lm)
            elif lm.type == "dock":
                plotting.plot_dock(ax,lm)
            else:
                plotting.plot_intersection(ax,lm)    
        for c in self.connectivityList:
            lm1 = self.landmarks[c[0]]
            lm2 = self.landmarks[c[1]]
            plotting.plot_road(ax,lm1,lm2)
        return ax

    def find_adjacent(self,lm : landmark):
        adjacent = []
        for c in self.connectivityList:
            if lm.id in c:
                i = 1 - c.index(lm.id)
                adjacent.append(c[i])
        return adjacent

    def has_reached(r: robot, _cmd : move):
        if np.linalg.norm(r.pose.range(_cmd.lm_to_xy)) < REACH_DELTA:
            return True
        else:
            return False

    def can_pickup(r : robot, _cmd : pickup):
        if np.linalg.norm(r.pose.range(_cmd.lm_xy)) < REACH_DELTA:
            return True
        else:
            return False

    def can_drop(r : robot, _cmd : drop):
        if np.linalg.norm(r.pose.range(_cmd.lm_xy)) < REACH_DELTA:
            return True
        else:
            return False





    

    
