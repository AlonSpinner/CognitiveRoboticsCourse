from dataclasses import dataclass
import maildelivery.plotting as plotting
import matplotlib.pyplot as plt
import numpy as np

class enviorment:    
    def __init__(self,landmarks,connectivityList, packages = None):
        #instance attributes
        self.landmarks : list[landmark]  = sorted(landmarks)
        self.connectivityList : list[(int,int)]  = connectivityList
        self.packages : list[package] = packages

    def plot(self,ax : plt.Axes = None):
        if ax == None:
            fig , ax = plotting.spawnWorld()
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

@dataclass(frozen = True, order = True) #ordered so they can be sorted!
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

@dataclass(frozen = False, order = True) #NOT FROZEN
class package:
    id : int
    owner : int #landmark id  or robot == ROBOT_INDEX_SHIFT + robot id
    goal : int 
    deliverytime : float
    xy : np.ndarray((2))





    

    
